// Maniobras del robot real: tramo recto del seguidor de paredes, tramos por celdas, giros y marcha atras.
#include "comms/telemetry.h"
#include "config.h"
#include "control/motion_progress.h"
#include "robot/robot_hw.h"

namespace {

float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

int medianOf(int* v, int n) {
  for (int a = 1; a < n; ++a)
    for (int b = a; b > 0 && v[b] < v[b - 1]; --b) {
      const int t = v[b];
      v[b] = v[b - 1];
      v[b - 1] = t;
    }
  return n >= 2 ? v[n / 2] : (n == 1 ? v[0] : -1);
}

raton::MotionReason reasonFor(bool aborted, bool complete) {
  return aborted ? raton::MotionReason::Aborted
                 : (complete ? raton::MotionReason::Complete : raton::MotionReason::Interrupted);
}

}  // namespace

raton::ParamsMovimiento RobotHW::motionParams() {
  raton::ParamsMovimiento p;
  p.sideCrashMm = cfg::SIDE_CRASH_MM;
  p.skewLimitDeg = cfg::SKEW_LIMIT_DEG;
  p.rubMs = cfg::RUB_MS;
  p.maxRubs = cfg::MAX_RUBS;
  p.pushCheckMm = cfg::PUSH_CHECK_MM;
  p.pushMinCloseMm = cfg::PUSH_MIN_CLOSE_MM;
  p.stallMs = cfg::STALL_MS;
  p.accel = cfg::ACCEL;
  p.minSpeed = cfg::MIN_SPEED;
  return p;
}

float RobotHW::wheelDistance() const {
  return 0.5f * (encoders_.left() + encoders_.right()) * cfg::MM_PER_COUNT;
}

void RobotHW::stopMotors() {
  lastPeriodUs_ = 0;  // la pausa entre maniobras no cuenta como retraso del control
  motors_.brake();
  delay(60);
  motors_.coast();
  lastSpeed_ = 0;
  idle(cfg::SETTLE_MS);
}

// ---------------------------------------------------------------- lecturas

int RobotHW::frontMm() {
  const int f = lasers_.mm(Lasers::FRONT);
  if (f >= 0) {
    frontMemMm_ = f;
    frontMemDist_ = odoDist_;
    return f;
  }
  if (frontMemMm_ >= 0 && frontMemMm_ < cfg::FRONT_MEMORY_MM) {
    const float predicted = frontMemMm_ - (odoDist_ - frontMemDist_);
    if (predicted < cfg::FRONT_MEMORY_MM) return predicted > 0 ? static_cast<int>(predicted) : 0;
  }
  frontMemMm_ = -1;
  return -1;
}

Walls RobotHW::readWalls() {
  lasers_.waitFresh(2);
  auto wall = [](int mm, int threshold) { return mm >= 0 && mm < threshold; };
  return Walls{wall(sideMm(Lasers::LEFT), tuning_.sideWall), wall(frontMm(), cfg::FRONT_WALL_MM),
               wall(sideMm(Lasers::RIGHT), tuning_.sideWall)};
}

Walls RobotHW::senseCell() {
  int best[3] = {-1, -1, -1};
  for (int k = 0; k < 5; ++k) {
    lasers_.waitFresh(1);
    for (int i = 0; i < 3; ++i) {
      const int v = lasers_.mm(static_cast<Lasers::Id>(i));
      if (v >= 0 && (best[i] < 0 || v < best[i])) best[i] = v;
    }
  }
  const int threshold = geom_.cellFrontWall();
  auto wall = [&](int i) {
    if (best[i] >= 0) return best[i] < threshold;
    // Ninguna lectura: si la ultima valida era muy corta y fue hace menos de 4 cm de recorrido, esta
    // pegado a la pared (el VL53L0X no mide tan cerca). En un hueco de verdad fue hace casi una celda.
    const Lasers::Id id = static_cast<Lasers::Id>(i);
    return lasers_.lastValidMm(id) < cfg::SIDE_CLOSE_MM && fabsf(odoDist_ - lastValidDist_[i]) < 40.0f;
  };
  return Walls{wall(Lasers::LEFT), wall(Lasers::FRONT), wall(Lasers::RIGHT)};
}

bool RobotHW::cellSides(int i, bool& left, bool& right) const {
  if (i < 0 || i >= numCellSides_) return false;
  left = cellSideL_[i];
  right = cellSideR_[i];
  return true;
}

bool RobotHW::lateralOffset(float& offset, float& rate, float& conf) const {
  const int l = sideMm(Lasers::LEFT);
  const int r = sideMm(Lasers::RIGHT);
  const bool hasL = l >= 0 && l < tuning_.sideWall;
  const bool hasR = r >= 0 && r < tuning_.sideWall;
  // Con una sola pared: centrado = la mitad del hueco que deja el raton en el pasillo medido.
  const float center = 0.5f * (geom_.corridorMm() - cfg::ROBOT_WIDTH_MM);
  const float sideSin = geom_.sideSin();
  const float cl = lasers_.confidence(Lasers::LEFT), cr = lasers_.confidence(Lasers::RIGHT);
  if (hasL && hasR) {
    offset = (l - r) * 0.5f;
    rate = (lasers_.rate(Lasers::LEFT) - lasers_.rate(Lasers::RIGHT)) * 0.5f * sideSin;
    conf = fminf(cl, cr);
  } else if (hasL) {
    offset = l - center;
    rate = lasers_.rate(Lasers::LEFT) * sideSin;
    conf = cl;
  } else if (hasR) {
    offset = center - r;
    rate = -lasers_.rate(Lasers::RIGHT) * sideSin;
    conf = cr;
  } else {
    return false;
  }
  return true;
}

// ---------------------------------------------------------------- control

void RobotHW::controlStep(float v, float speed, float& integral, float dt, bool useWalls) {
  const float err = v - speed;
  integral = clampf(integral + err * dt, -50.0f, 50.0f);
  // Los N20 no se mueven por debajo de cierta potencia: en marcha siempre llevan PWM_MIN_RUN y sobre
  // eso se suma lo que pide la velocidad. Sin esto avanzaba a tirones.
  float base = 0;
  if (v > 0) {
    base = cfg::PWM_MIN_RUN + cfg::KFF_SPEED * v + cfg::KP_SPEED * err + cfg::KI_SPEED * integral;
    if (speed < 20.0f && base < cfg::PWM_MIN_MOVE) base = cfg::PWM_MIN_MOVE;  // parado: empujon para arrancar
    if (base < cfg::PWM_MIN_RUN) base = cfg::PWM_MIN_RUN;
  }

  // Direccion en capas:
  //  - con paredes: desviacion lateral con ganancias escaladas por la confianza de los laseres;
  //  - sin paredes (o laseres poco fiables): mantener el rumbo con el giroscopio (KP_YAW).
  // En ambos casos el giroscopio amortigua (KD_YAW). Positivo = girar a la izquierda.
  float steer;
  float offset, rate, conf;
  const bool walls = useWalls && lateralOffset(offset, rate, conf) && conf > 0.2f;
  if (walls) {
    if (fabsf(offset) <= cfg::LATERAL_DEADBAND_MM) {
      offset = 0;
    } else {
      offset -= offset > 0 ? cfg::LATERAL_DEADBAND_MM : -cfg::LATERAL_DEADBAND_MM;
    }
  }
  if (tuning_.steerMode == 1) {
    // PID directo sobre la desviacion (KP_STEER, KD_STEER), escalado por la confianza.
    if (walls) {
      steer = conf * (tuning_.kpSteer * offset + tuning_.kdSteer * rate * dt) - cfg::KD_YAW * gyro_.rate();
      headingRef_ = heading_;
    } else {
      steer = cfg::KP_YAW * (headingRef_ - heading_) - cfg::KD_YAW * gyro_.rate();
    }
  } else {
    // Cascada: la desviacion fija el rumbo objetivo (con menos fuerza si los laseres son poco fiables)
    // y el PID del giroscopio lo sigue.
    float target = 0;
    if (walls) {
      target = clampf(conf * tuning_.kpWall * offset, -cfg::MAX_WALL_CORRECTION_DEG, cfg::MAX_WALL_CORRECTION_DEG);
    }
    yawTarget_ += cfg::YAW_TARGET_SMOOTH * (target - yawTarget_);
    steer = cfg::KP_YAW * (yawTarget_ - heading_) - cfg::KD_YAW * gyro_.rate();
  }
  steer = clampf(steer, -cfg::MAX_STEER, cfg::MAX_STEER);
  motors_.set(base - steer, base + steer);
}

// ---------------------------------------------------------------- tramos por celdas

int RobotHW::runCells(int cells, float vmax, bool* unexpected, raton::CellObserver* observer) {
  if (unexpected) *unexpected = false;
  if (cells <= 0) return 0;
  // Si el giro anterior ya adelanto parte de la celda, se descuenta.
  const float already = pendingAdvance_;
  pendingAdvance_ = 0;
  const float cellMm = geom_.cellMm();
  float target = fmaxf(cells * cellMm - already, 0.0f);
  const float dt = cfg::CONTROL_PERIOD_US * 1e-6f;

  setState("CELDA");
  // Tras un giro el filtro del frontal aun guarda la pared que veia antes de girar: se espera a tener
  // lecturas nuevas en el rumbo actual antes de arrancar (si no, el tramo acababa en 0 celdas).
  frontMemMm_ = -1;
  lasers_.waitFresh(8);  // ~240 ms
  encoders_.reset();
  resetHeadingToGrid();
  yawTarget_ = 0;
  headingRef_ = 0;

  float v = 0;      // velocidad de consigna
  float lastPos = 0;
  float speed = 0;  // velocidad medida filtrada
  float integral = 0;
  float pos = 0;
  uint32_t next = micros();
  numCellSides_ = 0;
  bool stuck = false;
  const uint32_t runStart = millis();
  // Lecturas laterales en la ventana de la celda que toca: se decide con la MEDIANA (una lectura
  // fantasma suelta no crea una pared). Se guarda una por cada lectura nueva del laser.
  int winL[24], winR[24], nWinL = 0, nWinR = 0;
  uint32_t frontSample = 0;
  int frontPrev = -1, frontConfirmed = -1;
  uint32_t winSample = 0;
  bool windowSeen = false;
  // Avance medido con la odometria total (no se pone a cero al apartarse y enderezarse a mitad de tramo).
  const float startDist = odoDist_;
  // Orquestador de movimiento: decide en cada ciclo frenado y seguridad (roce, empuje, atasco).
  raton::OrquestadorMovimiento motion(motionParams());
  motion.empezarTramo(millis());

  while (!aborted_) {
    waitPeriod(next);

    pos = odoDist_ - startDist;
    if (numCellSides_ < cells && numCellSides_ < kMaxCellSides) {
      const float at = (numCellSides_ + 1) * cellMm - already - geom_.sideLookAhead();
      if (pos >= at - kSideWindowMm) {
        const int l = sideMm(Lasers::LEFT), r = sideMm(Lasers::RIGHT);
        if (laserSamples_ != winSample) {  // solo lecturas nuevas
          winSample = laserSamples_;
          if (l >= 0 && nWinL < 24) winL[nWinL++] = l;
          if (r >= 0 && nWinR < 24) winR[nWinR++] = r;
        }
        windowSeen = true;
      }
      if (windowSeen && pos >= at + kSideWindowMm) {
        const int mL = medianOf(winL, nWinL), mR = medianOf(winR, nWinR);
        cellSideL_[numCellSides_] = mL >= 0 && mL < geom_.cellSideWall();
        cellSideR_[numCellSides_] = mR >= 0 && mR < geom_.cellSideWall();
        numCellSides_++;
        nWinL = nWinR = 0;
        windowSeen = false;
        // Exploracion en marcha: el explorador decide con las paredes recien leidas si sigue recto o se
        // para en esta celda (para girar). Asi no se para en cada celda.
        if (observer && !observer->onCellSides(numCellSides_ - 1, cellSideL_[numCellSides_ - 1],
                                               cellSideR_[numCellSides_ - 1], frontMm())) {
          const float stopAt = numCellSides_ * cellMm - already;
          if (stopAt < target) target = fmaxf(stopAt, pos);
          observer = nullptr;  // ya decidido: el resto del tramo es frenar
        }
      }
    }
    speed = cfg::SPEED_FILTER * speed + (1.0f - cfg::SPEED_FILTER) * (pos - lastPos) / dt;
    lastPos = pos;
    lastSpeed_ = speed;
    updateHeading();
    // Golpe seco (acelerometro) yendo en marcha: pegado.
    const bool bumped = speed > cfg::BUMP_MIN_SPEED && millis() - runStart > cfg::BUMP_IGNORE_MS &&
                        gyro_.bumpG() > cfg::BUMP_G;
    // Frontal confirmado: solo cuenta una lectura que coincide (±30 mm) con la anterior nueva.
    const int frontNow = frontMm();
    if (laserSamples_ != frontSample) {
      frontSample = laserSamples_;
      frontConfirmed = (frontNow >= 0 && frontPrev >= 0 && abs(frontNow - frontPrev) <= 30) ? frontNow
                       : (frontNow < 0 ? -1 : frontConfirmed);
      frontPrev = frontNow;
    }
    raton::EntradaMovimiento in;
    in.ms = millis();
    in.pos = pos;
    in.speed = speed;
    in.remaining = target - pos;
    in.front = frontConfirmed;
    in.stopMm = geom_.frontTurnMm();
    // Distancia a la que "hay pared delante": en un pasillo ancho la parada queda mas lejos.
    in.wallNearMm = max(cfg::FRONT_WALL_MM, geom_.frontTurnMm() + static_cast<int>(0.75f * cellMm));
    in.cellMm = cellMm;
    in.sideL = sideMm(Lasers::LEFT);
    in.sideR = sideMm(Lasers::RIGHT);
    in.skewDeg = gridOn_ ? gridError() : 0.0f;
    const raton::DecisionMovimiento dm = motion.decidir(in);
    if (bumped || dm.seguridad == raton::Seguridad::Pegado) {
      stuck = true;
      if (unexpected) *unexpected = true;
      break;
    }
    if (dm.seguridad == raton::Seguridad::Roce) {
      // Roce o torcido sostenido: frena, se aparta, se endereza hacia la cuadricula y sigue el tramo.
      char line[56];
      snprintf(line, sizeof(line), "ROCE izq=%d der=%d torcido=%d -> se endereza", in.sideL, in.sideR,
               static_cast<int>(in.skewDeg));
      telemetry::println(line);
      stopMotors();
      backUp(cfg::RUB_BACK_MM);
      if (gridOn_) rotate(-gridError());
      if (aborted_) break;
      setState("CELDA");
      encoders_.reset();
      resetHeadingToGrid();
      yawTarget_ = 0;
      integral = 0;
      v = 0;
      speed = 0;
      // Lo leido antes de apartarse (y torcido) no vale para la celda que tocaba: se vuelve a leer.
      nWinL = nWinR = 0;
      windowSeen = false;
      pos = odoDist_ - startDist;
      lastPos = pos;
      motion.trasRecuperar(millis(), pos);
      continue;
    }
    if (dm.paredInesperada && unexpected) *unexpected = true;
    const float remaining = dm.remaining;
    if (remaining <= 1.0f) break;

    // Perfil trapezoidal: acelera hasta vmax y frena para llegar justo.
    v = fminf(fminf(v + cfg::ACCEL * dt, vmax), dm.vmaxFrenado);
    controlStep(fmaxf(v, cfg::MIN_SPEED), speed, integral, dt);
  }
  stopMotors();
  if (stuck && !aborted_) {
    setState("PEGADO");
    char line[48];
    snprintf(line, sizeof(line), "PEGADO pos=%d frente=%d", static_cast<int>(pos), frontMm());
    telemetry::println(line);
    backUp(cfg::UNSTICK_BACK_MM);  // apartarse recto de la pared antes de decidir
    pos = odoDist_ - startDist;
  }
  const auto progress = raton::motionProgress(pos, already, cellMm, cells, false);
  const int moved = progress.cells;
  // Donde queda el centro del raton respecto al de la celda: con pared delante lo dice el laser; si no,
  // lo que se paso o le falto por encoders. Lo usan el giro y la marcha atras siguientes.
  lasers_.waitFresh(1);
  const int f = lasers_.mm(Lasers::FRONT);
  if (f >= 0 && f < cfg::FRONT_WALL_MM) {
    pendingAdvance_ = 0.5f * geom_.corridorMm() - (f - geom_.deadzoneMm()) - cfg::FRONT_SENSOR_AHEAD_MM;
  } else {
    pendingAdvance_ = progress.offset;
  }
  const bool complete = !aborted_ && moved >= cells && !(unexpected && *unexpected);
  recordMotion("cells", cells * cellMm, {complete, pos, gridError(), reasonFor(aborted_, complete)});
  return moved < cells ? moved : cells;
}

// ---------------------------------------------------------------- seguidor de paredes

RobotHW::DriveEvent RobotHW::drive(float maxMm, int watchSide) {
  const float dt = cfg::CONTROL_PERIOD_US * 1e-6f;

  setState(watchSide != 0 ? "RECTA" : "ESQUINA");
  encoders_.reset();
  resetHeading();

  float v = 0;
  float lastPos = 0;
  float speed = 0;
  float integral = 0;
  float openSince = -1;   // posicion donde se vio abierto el lado vigilado
  float openSinceL = -1;  // con kWatchBoth, uno por lado
  float openSinceR = -1;
  int lastOkL = 100;  // ultima lectura valida de cada lateral
  int lastOkR = 100;
  float ignoreSide = 0;
  if (watchSide != 0) {
    ignoreSide = ignoreSideMm_;
    ignoreSideMm_ = 0;
  }
  yawTarget_ = 0;
  headingRef_ = 0;
  uint32_t movingAt = millis();  // ultima vez que se movia: si se atasca, se trata como pared
  const uint32_t startMs = millis();
  uint32_t fastAt = 0;  // ultima vez que iba en marcha de verdad (para detectar un frenazo por golpe)
  DriveEvent ev = DriveEvent::Distance;
  uint32_t next = micros();
  const int& sideWall = tuning_.sideWall;  // ajustable en marcha (orden w)

  while (true) {
    if (aborted_) {
      ev = DriveEvent::Aborted;
      break;
    }
    waitPeriod(next);

    const float pos = wheelDistance();
    speed = cfg::SPEED_FILTER * speed + (1.0f - cfg::SPEED_FILTER) * (pos - lastPos) / dt;
    lastPos = pos;
    lastSpeed_ = speed;
    updateHeading();

    if (speed > 20.0f) movingAt = millis();
    if (millis() - movingAt > cfg::STALL_MS) {
      ev = DriveEvent::FrontWall;
      break;
    }
    // Golpe: pico del acelerometro estando en marcha y frenazo de golpe (el tiron al arrancar no cuenta).
    if (speed > cfg::BUMP_MIN_SPEED) fastAt = millis();
    const bool suddenStop = millis() - fastAt < 150 && speed < 0.5f * cfg::BUMP_MIN_SPEED;
    if (millis() - startMs > cfg::BUMP_IGNORE_MS && suddenStop && gyro_.bumpG() > cfg::BUMP_G) {
      ev = DriveEvent::FrontWall;
      setState("CHOQUE");
      break;
    }

    if (goalSet_ && hypotf(odoX_ - goalX_mm_, odoY_ - goalY_mm_) < cfg::GOAL_RADIUS_MM) {
      goalReached_ = true;
      break;
    }
    float remaining = maxMm - pos;
    if (remaining <= 1.0f) break;
    const int front = frontMm();
    const int left = sideMm(Lasers::LEFT);
    const int right = sideMm(Lasers::RIGHT);
    // Rozando una pared lateral: se para para enderezarse hacia el otro lado. Si los dos laterales
    // estan cerca a la vez es un pasillo estrecho, no un roce: solo se centra.
    const bool leftClose = left >= 0 && left < cfg::SIDE_CRASH_MM;
    const bool rightClose = right >= 0 && right < cfg::SIDE_CRASH_MM;
    const bool narrow = left >= 0 && right >= 0 && abs(left - right) < cfg::SIDE_CRASH_MARGIN_MM;
    if (leftClose && !narrow && (right < 0 || left < right)) {
      ev = DriveEvent::CrashLeft;
      setState("ROCE");
      break;
    }
    if (rightClose && !narrow && (left < 0 || right < left)) {
      ev = DriveEvent::CrashRight;
      setState("ROCE");
      break;
    }
    // Con pared delante cerca, los laterales pueden estar viendo esa misma pared o la esquina:
    // no se usan para centrar ni para buscar huecos.
    const bool frontNear = front >= 0 && front < cfg::FRONT_WALL_MM;
    if (front >= 0) {
      const float toWall = static_cast<float>(front - geom_.frontTurnMm());
      if (toWall <= 0) {
        ev = DriveEvent::FrontWall;
        break;
      }
      remaining = fminf(remaining, toWall);
    }

    // Con pared a los dos lados se va midiendo el ancho del pasillo (adapta el raton a la pista).
    if (!frontNear && left >= 0 && right >= 0 && left <= sideWall && right <= sideWall) {
      geom_.blendCorridor(left + right + cfg::ROBOT_WIDTH_MM);
    }

    // Hueco en el lado vigilado: se confirma tras OPEN_CONFIRM_MM para ignorar lecturas sueltas.
    // Con kWatchBoth: hueco a cualquiera de los dos lados (borde de la pared que termina).
    const bool sidesReady = pos >= ignoreSide;
    if (left >= 0) lastOkL = left;
    if (right >= 0) lastOkR = right;
    // Sin lectura justo despues de una lectura muy corta: demasiado cerca para el laser, no un hueco.
    const int seenL = (left < 0 && lastOkL < cfg::SIDE_CLOSE_MM) ? lastOkL : left;
    const int seenR = (right < 0 && lastOkR < cfg::SIDE_CLOSE_MM) ? lastOkR : right;
    if (watchSide == kWatchBoth && !frontNear && sidesReady) {
      auto check = [&](int mm, float& since) {
        if (mm >= 0 && mm <= sideWall) {
          since = -1;
          return false;
        }
        if (since < 0) since = pos;
        return pos - since >= cfg::OPEN_CONFIRM_MM;
      };
      const bool l = check(seenL, openSinceL);
      const bool r = check(seenR, openSinceR);
      if (l || r) {
        ev = DriveEvent::SideOpen;
        break;
      }
    } else if (watchSide != 0 && watchSide != kWatchBoth && !frontNear && sidesReady) {
      const int side = watchSide > 0 ? seenL : seenR;
      if (side >= 0 && side <= sideWall) {
        openSince = -1;
      } else if (openSince < 0) {
        openSince = pos;
      } else if (pos - openSince >= cfg::OPEN_CONFIRM_MM) {
        ev = DriveEvent::SideOpen;
        break;
      }
    }

    const float brakeSpeed = sqrtf(2.0f * cfg::ACCEL * remaining);
    // La velocidad baja si los laseres son poco fiables.
    const float confAll = fminf(lasers_.confidence(Lasers::FRONT),
                                fmaxf(lasers_.confidence(Lasers::LEFT), lasers_.confidence(Lasers::RIGHT)));
    const float confScale = cfg::CONF_MIN_SPEED + (1.0f - cfg::CONF_MIN_SPEED) * confAll;
    const float vmax = (frontNear ? fminf(tuning_.speed, cfg::SLOW_NEAR_WALL_SPEED) : tuning_.speed) * confScale;
    v = fminf(fminf(v + cfg::ACCEL * dt, vmax), fmaxf(brakeSpeed, cfg::MIN_SPEED));
    controlStep(v, speed, integral, dt, !frontNear);
  }
  stopMotors();
  const float actual = wheelDistance();
  recordMotion("drive", maxMm, {ev == DriveEvent::Distance, actual, heading_,
                                ev == DriveEvent::Aborted ? raton::MotionReason::Aborted
                                : (ev == DriveEvent::Distance ? raton::MotionReason::Complete
                                                              : raton::MotionReason::Interrupted)});
  return ev;
}

// ---------------------------------------------------------------- giros

void RobotHW::turnCorner(float degrees, bool trackCells) {
  // Giro en el sitio (el raton cabe girando en el centro de la celda). Con cuadricula, hasta el rumbo
  // exacto de la pista, asi el error de cada giro no se acumula.
  rotate(gridOn_ ? gridTurnError(degrees) : degrees);
  // Girando sobre el eje (centro del cuerpo) el raton sigue centrado en la celda.
  if (trackCells) pendingAdvance_ = 0;
}

float RobotHW::rotateOnce(float degrees) {
  setState("GIRO");
  frontMemMm_ = -1;  // al girar, la pared recordada ya no esta delante
  gyro_.update();
  gyro_.resetYaw();
  const uint32_t t0 = millis();
  uint32_t next = micros();
  int settled = 0;
  float err = degrees;

  while (!aborted_ && millis() - t0 < cfg::TURN_TIMEOUT_MS) {
    waitPeriod(next);
    err = degrees - gyro_.yaw();
    const float rate = gyro_.rate();
    // Dentro de tolerancia y casi quieto: hecho. Se decide aqui, antes de frenar.
    if (fabsf(err) < cfg::TURN_TOLERANCE_DEG) {
      motors_.brake();
      if (fabsf(rate) < 40.0f && ++settled >= 3) break;
      continue;
    }
    settled = 0;
    const int sign = err >= 0 ? 1 : -1;
    // PD: frena solo al acercarse (la derivada resta cuando ya gira deprisa hacia el objetivo).
    float pwm = clampf(cfg::KP_TURN * err - cfg::KD_TURN * rate, -cfg::TURN_PWM_MAX, cfg::TURN_PWM_MAX);
    if (fabsf(rate) < 20.0f) {
      // Parado. Lejos del objetivo: empujon minimo para arrancar. Cerca (< TURN_ACCEPT_DEG) y parado, no
      // se insiste (insistir a todo o nada lo hacia bailar sobre el objetivo hasta agotar el tiempo).
      if (fabsf(err) < cfg::TURN_ACCEPT_DEG && millis() - t0 > 300) break;
      if (fabsf(pwm) < cfg::TURN_PWM_MIN) pwm = sign * cfg::TURN_PWM_MIN;
    }
    if (cfg::TURN_IN_PLACE) {
      motors_.set(-pwm, pwm);  // sobre si mismo: lo que menos espacio ocupa
    } else if (pwm > 0) {
      motors_.set(0, pwm);
    } else {
      motors_.set(-pwm, 0);
    }
  }
  stopMotors();
  return degrees - gyro_.yaw();
}

void RobotHW::rotate(float degrees) {
  // Giro y, si queda torcido mas de TURN_RETRY_DEG, otro intento corto con lo que falta: un error que se
  // arrastra al tramo siguiente es lo que lo hacia avanzar en diagonal rozando la pared.
  float err = rotateOnce(degrees);
  for (int attempt = 1; attempt < cfg::TURN_ATTEMPTS && !aborted_ && fabsf(err) > cfg::TURN_RETRY_DEG; ++attempt) {
    err = rotateOnce(err);
  }
  const bool failed = !aborted_ && fabsf(err) >= cfg::TURN_FAIL_DEG;
  if (failed) {
    aborted_ = true;
    setState("GIRO_FALLO");
    telemetry::println("ERROR: giro incompleto; parado para no perder la posicion");
  }
  recordMotion("rotate", degrees, {!failed && !aborted_, 0, err,
                                   failed ? raton::MotionReason::TurnError : raton::MotionReason::Complete});
}

// ---------------------------------------------------------------- marcha atras

int RobotHW::backOut(float maxMm) {
  setState("SALIDA");
  encoders_.reset();
  resetHeading();
  const uint32_t t0 = millis();
  uint32_t next = micros();
  float sinceL = -1;
  float sinceR = -1;
  int found = 0;
  float foundAt = 0;
  while (!aborted_ && millis() - t0 < 5000) {
    waitPeriod(next);
    const float back = -wheelDistance();
    if (back >= maxMm) break;
    if (found == 0) {
      auto check = [&](int mm, float& since) {
        if (mm >= 0 && mm <= tuning_.sideWall) {
          since = -1;
          return false;
        }
        if (since < 0) since = back;
        return back - since >= cfg::OPEN_CONFIRM_MM;
      };
      if (check(sideMm(Lasers::LEFT), sinceL)) found = 1;
      else if (check(sideMm(Lasers::RIGHT), sinceR)) found = -1;
      if (found) foundAt = back;
    } else if (back - foundAt >= 0.5f * geom_.corridorMm()) {
      break;  // el laser lateral ya esta en el centro del hueco
    }
    updateHeading();
    const float steer = cfg::KP_YAW * -heading_ - cfg::KD_YAW * gyro_.rate();
    motors_.set(-cfg::BACKUP_PWM - steer, -cfg::BACKUP_PWM + steer);
  }
  stopMotors();
  recordMotion("back_out", -maxMm, {!aborted_ && found != 0, wheelDistance(), heading_, reasonFor(aborted_, found != 0)});
  return found;
}

void RobotHW::backUp(float mm) {
  setState("ATRAS");
  encoders_.reset();
  resetHeading();
  const uint32_t t0 = millis();
  uint32_t next = micros();
  while (!aborted_ && millis() - t0 < 1000) {
    waitPeriod(next);
    if (wheelDistance() <= -mm) break;
    updateHeading();
    const float steer = cfg::KP_YAW * -heading_ - cfg::KD_YAW * gyro_.rate();
    motors_.set(-cfg::BACKUP_PWM - steer, -cfg::BACKUP_PWM + steer);
  }
  stopMotors();
  const float actual = wheelDistance();
  const bool complete = !aborted_ && actual <= -mm + 1;
  recordMotion("back", -mm, {complete, actual, heading_,
                             complete ? raton::MotionReason::Complete
                                      : (aborted_ ? raton::MotionReason::Aborted : raton::MotionReason::Interrupted)});
}

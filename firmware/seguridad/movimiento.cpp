#include "seguridad/robot_hw.h"
#include "comunicacion/telemetry.h"
#include "config.h"
#include "odometria/motion_progress.h"

namespace {
float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
}

namespace {
raton::MotionReason reasonFor(bool aborted, bool complete) {
  return aborted ? raton::MotionReason::Aborted
                 : (complete ? raton::MotionReason::Complete : raton::MotionReason::Interrupted);
}

}

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

void RobotHW::stopMotors() {
  lastPeriodUs_ = 0;  // la pausa entre maniobras no cuenta como retraso del control
  motors_.brake();
  idle(20);  // freno activo, sensores y Bluetooth siguen atendidos
  lastSpeed_ = 0;
  idle(cfg::SETTLE_MS);  // mantener freno hasta la siguiente orden de movimiento
}

// ---------------------------------------------------------------- lecturas

int RobotHW::runCells(int cells, float vmax, bool* unexpected, raton::CellObserver* observer) {
  if (unexpected) *unexpected = false;
  if (cells <= 0) return 0;
  if (!adaptiveStarted_) {
    adaptiveSpeed_.caution(cfg::ADAPT_CAUTION_MM);
    adaptiveStarted_ = true;
  }
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
  if (postTurnFresh_) waitFresh(cfg::POST_TURN_SAMPLES);
  postTurnFresh_ = false;
  encoders_.reset();
  resetHeadingToGrid();
  yawTarget_ = 0;
  headingRef_ = 0;

  float v = 0;      // velocidad de consigna
  float lastPos = 0;
  float speed = 0;  // velocidad medida filtrada
  float integral = 0;
  float pos = 0;
  float speedPos = 0;
  bool speedKnown[kMaxCellSides];
  for (int i = 0; i < kMaxCellSides; ++i) speedKnown[i] = !observer || observer->knownCell(i);
  raton::SpeedPolicy speedPolicy;
  speedPolicy.minimumRatio = cfg::ADAPT_MIN_RATIO;
  speedPolicy.unknownRatio = cfg::ADAPT_UNKNOWN_RATIO;
  speedPolicy.headingDeg = cfg::ADAPT_HEADING_DEG;
  speedPolicy.frontMm = cfg::ADAPT_FRONT_MM;
  uint32_t next = micros();
  numCellSides_ = 0;
  bool stuck = false;
  const uint32_t runStart = millis();
  // Lecturas laterales en la ventana de la celda que toca: se decide con la MEDIANA (una lectura
  // fantasma suelta no crea una pared). Se guarda una por cada lectura nueva del laser.
  raton::CellWindow window;
  uint32_t frontSample = 0;
  int frontPrev = -1, frontConfirmed = -1;
  // Avance medido con la odometria total (no se pone a cero al apartarse y enderezarse a mitad de tramo).
  const float startDist = odoDist_;
  // Orquestador de movimiento: decide en cada ciclo frenado y seguridad (roce, empuje, atasco).
  raton::OrquestadorMovimiento motion(motionParams());
  motion.empezarTramo(millis());

  while (!aborted_) {
    waitPeriod(next);

    pos = odoDist_ - startDist;
    observeCellSides(pos, cells, cellMm, already, target, observer, window);
    speed = cfg::SPEED_FILTER * speed + (1.0f - cfg::SPEED_FILTER) * (pos - lastPos) / dt;
    lastPos = pos;
    lastSpeed_ = speed;
    updateHeading();
    // Golpe seco (acelerometro) yendo en marcha: pegado.
    const bool bumped = speed > cfg::BUMP_MIN_SPEED && millis() - runStart > cfg::BUMP_IGNORE_MS &&
                        gyro_.bumpG() > cfg::BUMP_G;
    // Frontal confirmado: solo cuenta una lectura que coincide (±30 mm) con la anterior nueva.
    const int frontNow = frontMm();
    confirmCellFront(frontNow, frontSample, frontPrev, frontConfirmed);
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
      window.nWinL = window.nWinR = 0;
      window.windowSeen = false;
      pos = odoDist_ - startDist;
      lastPos = pos;
      motion.trasRecuperar(millis(), pos);
      adaptiveSpeed_.caution(cfg::ADAPT_CAUTION_MM);
      speedPos = pos;
      continue;
    }
    if (dm.paredInesperada && unexpected) *unexpected = true;
    const float remaining = dm.remaining;
    if (remaining <= 1.0f) break;

    // Perfil trapezoidal: acelera hasta vmax y frena para llegar justo.
    const float confidence = fminf(lasers_.confidence(Lasers::FRONT),
        fmaxf(lasers_.confidence(Lasers::LEFT), lasers_.confidence(Lasers::RIGHT)));
    const int speedCell = static_cast<int>((pos + already) / cellMm);
    const bool known = speedCell >= 0 && speedCell < kMaxCellSides && speedKnown[speedCell];
    const float cap = adaptiveSpeed_.step(speedPolicy, vmax, confidence, in.skewDeg, known,
        frontNow >= 0 ? fmaxf(0, frontNow - in.stopMm) : -1, pos - speedPos, dt, cfg::ACCEL);
    speedPos = pos;
    v = fminf(fminf(v + cfg::ACCEL * dt, cap), dm.vmaxFrenado);
    controlStep(fmaxf(v, cfg::MIN_SPEED), speed, integral, dt);
  }
  stopMotors();
  cachedCellFront_ = !stuck;
  cachedCellFrontMm_ = frontConfirmed;
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
  waitFresh(1);
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


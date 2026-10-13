#include "robot_hw.h"

#include "config.h"
#include "telemetry.h"

namespace {

float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

}  // namespace

RobotHW::RobotHW(Motors& motors, Encoders& encoders, Lasers& lasers, Gyro& gyro, Display& display)
    : motors_(motors), encoders_(encoders), lasers_(lasers), gyro_(gyro), display_(display) {}

void RobotHW::waitPeriod(uint32_t& next) {
  const uint32_t now = micros();
  if (lastPeriodUs_ != 0 && now - lastPeriodUs_ > maxLagUs_) maxLagUs_ = now - lastPeriodUs_;
  // Si el ciclo se retraso mucho, no intentar recuperar ciclos perdidos de golpe.
  if (static_cast<int32_t>(now - next) > static_cast<int32_t>(4 * cfg::CONTROL_PERIOD_US)) next = now;
  while (static_cast<int32_t>(micros() - next) < 0) {
    lasers_.update();
    gyro_.update();
  }
  telemetry::poll();
  handleInput();
  updateOdometry();
  sendTelemetry();
  lastPeriodUs_ = micros();
  next += cfg::CONTROL_PERIOD_US;
}

void RobotHW::resetOdometry() {
  odoX_ = 0;
  odoY_ = 0;
  odoYaw0_ = gyro_.totalYaw();
  odoLastA_ = 0;
  odoL_ = encoders_.leftTotal();
  odoR_ = encoders_.rightTotal();
}

void RobotHW::updateOdometry() {
  const long l = encoders_.leftTotal();
  const long r = encoders_.rightTotal();
  const float ds = 0.5f * ((l - odoL_) + (r - odoR_)) * cfg::MM_PER_COUNT;
  odoL_ = l;
  odoR_ = r;
  odoDist_ += ds;
  // El rumbo sale del giroscopio (no patina); 0 = mirando hacia +Y al empezar. Se integra con el
  // rumbo medio del paso para que los giros en marcha no deformen el mapa.
  const float a = (gyro_.totalYaw() - odoYaw0_) / 57.2958f;
  const float am = 0.5f * (a + odoLastA_);
  odoLastA_ = a;
  odoX_ -= ds * sinf(am);
  odoY_ += ds * cosf(am);
}

void RobotHW::idle(uint32_t ms) {
  const uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    lasers_.update();
    gyro_.update(true);
    telemetry::poll();
    handleInput();
    updateOdometry();
    sendTelemetry();
  }
}

void RobotHW::stopMotors() {
  lastPeriodUs_ = 0;  // la pausa entre maniobras no cuenta como retraso del control
  motors_.brake();
  delay(60);
  motors_.coast();
  lastSpeed_ = 0;
  idle(cfg::SETTLE_MS);
}

void RobotHW::resetHeading() {
  gyro_.update();
  gyro_.resetYaw();
  heading_ = 0;
  lastGyroYaw_ = 0;
}

void RobotHW::updateHeading() {
  // Filtro complementario: el giroscopio manda a corto plazo; los encoders (sin deriva, pero con
  // patinazos) corrigen poco a poco. Requiere encoders_.reset() junto con resetHeading().
  const float g = gyro_.yaw();
  heading_ += g - lastGyroYaw_;
  lastGyroYaw_ = g;
  const float diff = (encoders_.right() - encoders_.left()) * cfg::MM_PER_COUNT;
  const float encYaw = diff / trackWidth_ * 57.2958f;
  heading_ += cfg::ENC_YAW_WEIGHT * (encYaw - heading_);
}

void RobotHW::handleInput() {
  auto feed = [this](int c) {
    if (c < 0) return;
    if (c == 'x' || c == 'X') {
      aborted_ = true;
      cmdLen_ = 0;
      return;
    }
    if (c == '\n' || c == '\r') {
      if (cmdLen_ > 0) {
        cmd_[cmdLen_] = 0;
        runCommand(cmd_);
      }
      cmdLen_ = 0;
    } else if (cmdLen_ < static_cast<int>(sizeof(cmd_)) - 1) {
      cmd_[cmdLen_++] = static_cast<char>(c);
    }
  };
  while (Serial.available()) feed(Serial.read());
  for (int c = telemetry::read(); c >= 0; c = telemetry::read()) feed(c);
}

void RobotHW::runCommand(const char* cmd) {
  char reply[64];
  const float value = atof(cmd + 1);
  switch (cmd[0]) {
    case 'v':
      speed_ = clampf(value, 30.0f, cfg::SPEED_RUN_SPEED);
      break;
    case 'k':
      kpWall_ = clampf(value, 0.0f, 1.0f);
      break;
    case 's':
      startRequested_ = true;
      break;
    case 'G': {
      // G<x>,<y>: celda meta para el modo laberinto
      const char* comma = strchr(cmd, ',');
      if (comma) {
        goalX_ = atoi(cmd + 1);
        goalY_ = atoi(comma + 1);
      }
      break;
    }
    case 'w':
      sideWall_ = static_cast<int>(clampf(value, 40.0f, 400.0f));
      break;
    case 'z':
      deadzone_ = static_cast<int>(clampf(value, 0.0f, 150.0f));
      break;
    case 'r':
      lasers_.setMaxMm(static_cast<int>(clampf(value, 50.0f, 2000.0f)));
      break;
    case 'g':
      lasers_.setSignalLimit(clampf(value, 0.05f, 5.0f));
      break;
    case 'm':
      if (value >= 1 && value <= 5) requestedMode_ = static_cast<int>(value);
      break;
    case '?':
      break;
    default:
      snprintf(reply, sizeof(reply), "? s x m<1-5> v k r g w z G<x>,<y> ?");
      telemetry::println(reply);
      Serial.println(reply);
      return;
  }
  snprintf(reply, sizeof(reply), "v=%d k=%d/1000 r=%d g=%d/100 w=%d z=%d meta=%d,%d", static_cast<int>(speed_),
           static_cast<int>(kpWall_ * 1000), lasers_.maxMm(), static_cast<int>(lasers_.signalLimit() * 100),
           sideWall_, deadzone_, goalX_, goalY_);
  telemetry::println(reply);
  Serial.println(reply);
}

void RobotHW::sendTelemetry() {
  if (millis() - lastTelemetry_ < cfg::TELEMETRY_PERIOD_MS) return;
  lastTelemetry_ = millis();
  char line[128];
  const int a = static_cast<int>(gyro_.totalYaw() - odoYaw0_);
  // q = lecturas SIN filtrar (para el mapa: el filtro mantiene valores viejos cuando el laser no mide).
  snprintf(line, sizeof(line), "%s I%d F%d D%d v%d r%d p%d x%d y%d a%d t%d q%d,%d,%d", state_,
           lasers_.mm(Lasers::LEFT), lasers_.mm(Lasers::FRONT), lasers_.mm(Lasers::RIGHT),
           static_cast<int>(lastSpeed_), static_cast<int>(heading_), static_cast<int>(corridorMm_),
           static_cast<int>(odoX_), static_cast<int>(odoY_), a, static_cast<int>(maxLagUs_ / 1000),
           lasers_.rawMm(Lasers::LEFT), lasers_.rawMm(Lasers::FRONT), lasers_.rawMm(Lasers::RIGHT));
  maxLagUs_ = 0;
  telemetry::println(line);
}

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

raton::Walls RobotHW::readWalls() {
  lasers_.waitFresh(2);
  auto wall = [](int mm, int threshold) { return mm >= 0 && mm < threshold; };
  return raton::Walls{wall(lasers_.mm(Lasers::LEFT), sideWall_),
                      wall(frontMm(), cfg::FRONT_WALL_MM),
                      wall(lasers_.mm(Lasers::RIGHT), sideWall_)};
}

bool RobotHW::lateralOffset(float& offset) const {
  const int l = lasers_.mm(Lasers::LEFT);
  const int r = lasers_.mm(Lasers::RIGHT);
  const bool hasL = l >= 0 && l < sideWall_;
  const bool hasR = r >= 0 && r < sideWall_;
  // Con una sola pared: centrado = la mitad del hueco que deja el raton en el pasillo medido.
  const float center = 0.5f * (corridorMm_ - cfg::ROBOT_WIDTH_MM);
  if (hasL && hasR) {
    offset = (l - r) * 0.5f;
  } else if (hasL) {
    offset = l - center;
  } else if (hasR) {
    offset = center - r;
  } else {
    return false;
  }
  return true;
}

void RobotHW::moveForward(int cells) {
  if (cells <= 0) return;
  const float vmax = mode_ == raton::RunMode::Speed ? cfg::SPEED_RUN_SPEED : speed_;
  const float target = cells * cfg::CELL_MM;
  const float dt = cfg::CONTROL_PERIOD_US * 1e-6f;

  setState("CELDA");
  encoders_.reset();
  resetHeading();
  yawTarget_ = 0;

  float v = 0;          // velocidad de consigna
  float lastPos = 0;
  float speed = 0;      // velocidad medida filtrada
  float integral = 0;
  uint32_t movingAt = millis();  // si se atasca (empujando una pared), deja de empujar
  uint32_t next = micros();

  while (!aborted_) {
    waitPeriod(next);

    const float pos = 0.5f * (encoders_.left() + encoders_.right()) * cfg::MM_PER_COUNT;
    speed = cfg::SPEED_FILTER * speed + (1.0f - cfg::SPEED_FILTER) * (pos - lastPos) / dt;
    lastPos = pos;
    lastSpeed_ = speed;
    updateHeading();
    if (speed > 20.0f) movingAt = millis();
    if (millis() - movingAt > cfg::STALL_MS) {
      setState("ATASCO");
      break;
    }

    float remaining = target - pos;
    // En la ultima celda, si hay pared delante, frena por el laser: corrige el error acumulado.
    const int front = frontMm();
    if (remaining < cfg::CELL_MM && front >= 0 && front < cfg::FRONT_WALL_MM + cfg::CELL_MM / 2) {
      remaining = static_cast<float>(front - (deadzone_ + cfg::FRONT_MARGIN_MM));
    }
    if (remaining <= 1.0f) break;

    // Perfil trapezoidal: acelera hasta vmax y frena para llegar justo.
    const float brakeSpeed = sqrtf(2.0f * cfg::ACCEL * remaining);
    v = fminf(fminf(v + cfg::ACCEL * dt, vmax), fmaxf(brakeSpeed, cfg::MIN_SPEED));

    controlStep(v, speed, integral, dt);
  }
  stopMotors();
}

void RobotHW::controlStep(float v, float speed, float& integral, float dt, bool useWalls) {
  const float err = v - speed;
  integral = clampf(integral + err * dt, -50.0f, 50.0f);
  // Los N20 no se mueven por debajo de cierta potencia: en marcha siempre llevan PWM_MIN_RUN y sobre
  // eso se suma lo que pide la velocidad. Sin esto avanzaba a tirones (arranca, se para, arranca...).
  float base = 0;
  if (v > 0) {
    base = cfg::PWM_MIN_RUN + cfg::KFF_SPEED * v + cfg::KP_SPEED * err + cfg::KI_SPEED * integral;
    if (speed < 20.0f && base < cfg::PWM_MIN_MOVE) base = cfg::PWM_MIN_MOVE;  // parado: empujon para arrancar
    if (base < cfg::PWM_MIN_RUN) base = cfg::PWM_MIN_RUN;
  }

  // Rumbo: mantener yaw = 0, desviado un poco para volver al centro del pasillo. Zona muerta y
  // suavizado para que no haga zigzag entre las paredes.
  float target = 0;
  float offset;
  if (useWalls && lateralOffset(offset) && fabsf(offset) > cfg::LATERAL_DEADBAND_MM) {
    offset -= offset > 0 ? cfg::LATERAL_DEADBAND_MM : -cfg::LATERAL_DEADBAND_MM;
    target = clampf(kpWall_ * offset, -cfg::MAX_WALL_CORRECTION_DEG, cfg::MAX_WALL_CORRECTION_DEG);
  }
  yawTarget_ += cfg::YAW_TARGET_SMOOTH * (target - yawTarget_);
  const float steer = clampf(cfg::KP_YAW * (yawTarget_ - heading_) - cfg::KD_YAW * gyro_.rate(),
                             -cfg::MAX_STEER, cfg::MAX_STEER);

  motors_.set(base - steer, base + steer);
}

RobotHW::DriveEvent RobotHW::drive(float maxMm, int watchSide) {
  const float dt = cfg::CONTROL_PERIOD_US * 1e-6f;

  setState(watchSide != 0 ? "RECTA" : "ESQUINA");
  encoders_.reset();
  resetHeading();

  float v = 0;
  float lastPos = 0;
  float speed = 0;
  float integral = 0;
  float openSince = -1;  // posicion donde se vio abierto el lado vigilado
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
  uint32_t movingAt = millis();  // ultima vez que se movia: si se atasca, se trata como pared
  const uint32_t startMs = millis();
  uint32_t fastAt = 0;  // ultima vez que iba en marcha de verdad (para detectar un frenazo por golpe)
  DriveEvent ev = DriveEvent::Distance;
  uint32_t next = micros();

  while (true) {
    if (aborted_) {
      ev = DriveEvent::Aborted;
      break;
    }
    waitPeriod(next);

    const float pos = 0.5f * (encoders_.left() + encoders_.right()) * cfg::MM_PER_COUNT;
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

    float remaining = maxMm - pos;
    if (remaining <= 1.0f) break;
    const int front = frontMm();
    const int left = lasers_.mm(Lasers::LEFT);
    const int right = lasers_.mm(Lasers::RIGHT);
    // Rozando una pared lateral: se para para enderezarse hacia el otro lado. Si los dos laterales
    // estan cerca a la vez es un pasillo estrecho, no un roce: solo se centra (si no, se enderezaria
    // a un lado y al otro sin fin).
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
      const float toWall = static_cast<float>(front - frontTurnMm());
      if (toWall <= 0) {
        ev = DriveEvent::FrontWall;
        break;
      }
      remaining = fminf(remaining, toWall);
    }

    // Con pared a los dos lados se va midiendo el ancho del pasillo (adapta el raton a la pista).
    if (!frontNear && left >= 0 && right >= 0 && left <= sideWall_ && right <= sideWall_) {
      const float width = left + right + cfg::ROBOT_WIDTH_MM;
      corridorMm_ = clampf(0.98f * corridorMm_ + 0.02f * width, 80.0f, 400.0f);
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
        if (mm >= 0 && mm <= sideWall_) {
          since = -1;
          return false;
        }
        if (since < 0) since = pos;
        return pos - since >= cfg::OPEN_CONFIRM_MM;
      };
      const bool l = check(seenL, openSinceL);
      const bool r = check(seenR, openSinceR);
      if (l || r) {
        openSide_ = l ? 1 : -1;
        ev = DriveEvent::SideOpen;
        break;
      }
    } else if (watchSide != 0 && watchSide != kWatchBoth && !frontNear && sidesReady) {
      const int side = watchSide > 0 ? seenL : seenR;
      if (side >= 0 && side <= sideWall_) {
        openSince = -1;
      } else if (openSince < 0) {
        openSince = pos;
      } else if (pos - openSince >= cfg::OPEN_CONFIRM_MM) {
        ev = DriveEvent::SideOpen;
        break;
      }
    }

    const float brakeSpeed = sqrtf(2.0f * cfg::ACCEL * remaining);
    const float vmax = frontNear ? fminf(speed_, cfg::SLOW_NEAR_WALL_SPEED) : speed_;
    v = fminf(fminf(v + cfg::ACCEL * dt, vmax), fmaxf(brakeSpeed, cfg::MIN_SPEED));
    controlStep(v, speed, integral, dt, !frontNear);
  }
  stopMotors();
  return ev;
}

void RobotHW::selfCheck(char* msg) {
  setState("REVISION");
  // 1) Motores: la orden "girar a la izquierda" tiene que dar yaw positivo.
  gyro_.update();
  gyro_.resetYaw();
  encoders_.reset();
  uint32_t t0 = millis();
  motors_.set(-cfg::TURN_PWM_MAX, cfg::TURN_PWM_MAX);
  while (millis() - t0 < 250) gyro_.update();
  motors_.brake();
  t0 = millis();
  while (millis() - t0 < 200) gyro_.update();
  motors_.coast();
  const float yaw = gyro_.yaw();
  const bool motorsSwapped = yaw < -3.0f;
  if (motorsSwapped) motors_.setSwapped(true);
  // Distancia entre ruedas: lo que avanza una rueda menos la otra, entre el angulo girado.
  if (!motorsSwapped && yaw > 10.0f) {
    const float diff = (encoders_.right() - encoders_.left()) * cfg::MM_PER_COUNT;
    const float tw = diff / (yaw / 57.2958f);
    if (tw > 40.0f && tw < 160.0f) trackWidth_ = tw;
  }
  rotate(-yaw);  // volver al rumbo inicial (ya con los motores bien)

  // 2) Laseres: tras girar 90 grados a la izquierda, lo que estaba delante queda a la derecha.
  lasers_.waitFresh(3);
  const int f0 = lasers_.mm(Lasers::FRONT);
  bool lasersSwapped = false;
  if (f0 > 0 && f0 < 1000) {
    rotate(90.0f);
    lasers_.waitFresh(3);
    const int l = lasers_.mm(Lasers::LEFT);
    const int r = lasers_.mm(Lasers::RIGHT);
    rotate(-90.0f);
    const int dl = l < 0 ? 10000 : abs(l - f0);
    const int dr = r < 0 ? 10000 : abs(r - f0);
    if (dl + 40 < dr) {
      lasersSwapped = true;
      lasers_.setSwapped(true);
    }
  }
  snprintf(msg, 22, "Mot %s Las %s", motorsSwapped ? "CRUZ" : "ok", lasersSwapped ? "CRUZ" : "ok");
}

void RobotHW::turnCorner(float degrees) {
  if (!cfg::TURN_TWO_STAGE || fabsf(fabsf(degrees) - 90.0f) > 1.0f) {
    rotate(degrees);
    return;
  }
  const float sign = degrees > 0 ? 1.0f : -1.0f;
  rotate(sign * cfg::TURN_SPLIT_DEG);
  drive(cfg::TURN_SPLIT_ADVANCE_MM, 0);
  rotate(sign * (90.0f - cfg::TURN_SPLIT_DEG));
}

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
    const float back = -0.5f * (encoders_.left() + encoders_.right()) * cfg::MM_PER_COUNT;
    if (back >= maxMm) break;
    if (found == 0) {
      auto check = [&](int mm, float& since) {
        if (mm >= 0 && mm <= sideWall_) {
          since = -1;
          return false;
        }
        if (since < 0) since = back;
        return back - since >= cfg::OPEN_CONFIRM_MM;
      };
      if (check(lasers_.mm(Lasers::LEFT), sinceL)) found = 1;
      else if (check(lasers_.mm(Lasers::RIGHT), sinceR)) found = -1;
      if (found) foundAt = back;
    } else if (back - foundAt >= 0.5f * corridorMm_) {
      break;  // el laser lateral ya esta en el centro del hueco
    }
    updateHeading();
    const float steer = cfg::KP_YAW * -heading_ - cfg::KD_YAW * gyro_.rate();
    motors_.set(-cfg::BACKUP_PWM - steer, -cfg::BACKUP_PWM + steer);
  }
  stopMotors();
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
    const float pos = 0.5f * (encoders_.left() + encoders_.right()) * cfg::MM_PER_COUNT;
    if (pos <= -mm) break;
    updateHeading();
    const float steer = cfg::KP_YAW * -heading_ - cfg::KD_YAW * gyro_.rate();
    motors_.set(-cfg::BACKUP_PWM - steer, -cfg::BACKUP_PWM + steer);
  }
  stopMotors();
}

void RobotHW::rotate(float degrees) {
  setState("GIRO");
  frontMemMm_ = -1;  // al girar, la pared recordada ya no esta delante
  gyro_.update();
  gyro_.resetYaw();
  const uint32_t t0 = millis();
  uint32_t next = micros();
  int settled = 0;
  int reversals = 0;
  int lastSign = degrees >= 0 ? 1 : -1;

  while (!aborted_ && millis() - t0 < cfg::TURN_TIMEOUT_MS) {
    waitPeriod(next);
    const float err = degrees - gyro_.yaw();
    const float rate = gyro_.rate();
    // Dentro de tolerancia: frenar y dar el giro por bueno en cuanto se quede casi quieto.
    // Tras pasarse dos veces de un lado a otro, tambien se acepta (evita bailar sobre el objetivo).
    if (fabsf(err) < cfg::TURN_TOLERANCE_DEG || reversals >= 2) {
      motors_.brake();
      if (fabsf(rate) < 30.0f && ++settled > 3) break;
      continue;
    }
    settled = 0;
    const int sign = err >= 0 ? 1 : -1;
    if (sign != lastSign) {
      reversals++;
      lastSign = sign;
    }
    float pwm = clampf(cfg::KP_TURN * err - cfg::KD_TURN * rate, -cfg::TURN_PWM_MAX, cfg::TURN_PWM_MAX);
    // El minimo solo hace falta para arrancar: girando ya, se deja bajar y frena sin pasarse.
    if (fabsf(rate) < 30.0f && fabsf(pwm) < cfg::TURN_PWM_MIN) pwm = sign * cfg::TURN_PWM_MIN;
    if (cfg::TURN_IN_PLACE) {
      motors_.set(-pwm, pwm);  // sobre si mismo: lo que menos espacio ocupa
    } else if (pwm > 0) {
      // Pivote sobre una rueda: a la izquierda se para la izquierda y empuja la derecha.
      motors_.set(0, pwm);
    } else {
      motors_.set(-pwm, 0);
    }
  }
  stopMotors();
}

void RobotHW::onStep(const raton::Maze& maze, const raton::FloodFill& flood, raton::Cell pos, raton::Dir heading) {
  (void)maze;
  char l1[22];
  char l2[22];
  char l3[22];
  snprintf(l1, sizeof(l1), "Celda %d,%d  %c", pos.x, pos.y, raton::dirChar(heading));
  snprintf(l2, sizeof(l2), "Dist meta: %u", static_cast<unsigned>(flood.distance(pos)));
  snprintf(l3, sizeof(l3), "I%4d F%4d D%4d", lasers_.mm(Lasers::LEFT), lasers_.mm(Lasers::FRONT),
           lasers_.mm(Lasers::RIGHT));
  display_.show(mode_ == raton::RunMode::Speed ? "CARRERA" : "EXPLORANDO", l1, l2, l3);
  Serial.print("celda ");
  Serial.print(pos.x);
  Serial.print(',');
  Serial.print(pos.y);
  Serial.print(" rumbo ");
  Serial.print(raton::dirChar(heading));
  Serial.print(" dist ");
  Serial.println(flood.distance(pos));
}

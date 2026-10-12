#include "robot_hw.h"

#include "config.h"

namespace {

float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

}  // namespace

RobotHW::RobotHW(Motors& motors, Encoders& encoders, Lasers& lasers, Gyro& gyro, Display& display)
    : motors_(motors), encoders_(encoders), lasers_(lasers), gyro_(gyro), display_(display) {}

void RobotHW::waitPeriod(uint32_t& next) {
  while (static_cast<int32_t>(micros() - next) < 0) {
    lasers_.update();
    gyro_.update();
  }
  // 'x' por el Monitor Serie detiene el raton
  if (Serial.available() && Serial.read() == 'x') aborted_ = true;
  next += cfg::CONTROL_PERIOD_US;
}

raton::Walls RobotHW::readWalls() {
  lasers_.waitFresh(2);
  auto wall = [](int mm, int threshold) { return mm >= 0 && mm < threshold; };
  return raton::Walls{wall(lasers_.mm(Lasers::LEFT), cfg::SIDE_WALL_MM),
                      wall(lasers_.mm(Lasers::FRONT), cfg::FRONT_WALL_MM),
                      wall(lasers_.mm(Lasers::RIGHT), cfg::SIDE_WALL_MM)};
}

bool RobotHW::lateralOffset(float& offset) const {
  const int l = lasers_.mm(Lasers::LEFT);
  const int r = lasers_.mm(Lasers::RIGHT);
  const bool hasL = l >= 0 && l < cfg::SIDE_WALL_MM;
  const bool hasR = r >= 0 && r < cfg::SIDE_WALL_MM;
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
  const float vmax = mode_ == raton::RunMode::Speed ? cfg::SPEED_RUN_SPEED : cfg::SEARCH_SPEED;
  const float target = cells * cfg::CELL_MM;
  const float dt = cfg::CONTROL_PERIOD_US * 1e-6f;

  encoders_.reset();
  gyro_.update();
  gyro_.resetYaw();

  float v = 0;          // velocidad de consigna
  float lastPos = 0;
  float speed = 0;      // velocidad medida filtrada
  float integral = 0;
  uint32_t next = micros();

  while (!aborted_) {
    waitPeriod(next);

    const float pos = 0.5f * (encoders_.left() + encoders_.right()) * cfg::MM_PER_COUNT;
    speed = 0.7f * speed + 0.3f * (pos - lastPos) / dt;
    lastPos = pos;

    float remaining = target - pos;
    // En la ultima celda, si hay pared delante, frena por el laser: corrige el error acumulado.
    const int front = lasers_.mm(Lasers::FRONT);
    if (remaining < cfg::CELL_MM && front >= 0 && front < cfg::FRONT_WALL_MM + cfg::CELL_MM / 2) {
      remaining = static_cast<float>(front - cfg::FRONT_STOP_MM);
    }
    if (remaining <= 1.0f) break;

    // Perfil trapezoidal: acelera hasta vmax y frena para llegar justo.
    const float brakeSpeed = sqrtf(2.0f * cfg::ACCEL * remaining);
    v = fminf(fminf(v + cfg::ACCEL * dt, vmax), fmaxf(brakeSpeed, cfg::MIN_SPEED));

    controlStep(v, speed, integral, dt);
  }
  motors_.brake();
  delay(60);
  motors_.coast();
}

void RobotHW::controlStep(float v, float speed, float& integral, float dt, bool useWalls) {
  const float err = v - speed;
  integral = clampf(integral + err * dt, -50.0f, 50.0f);
  float base = cfg::KFF_SPEED * v + cfg::KP_SPEED * err + cfg::KI_SPEED * integral;
  // Solo para arrancar: parado, por debajo de PWM_MIN_MOVE el motor no se mueve. Ya en marcha se
  // deja bajar para poder ir despacio.
  if (v > 0 && speed < 20.0f && base < cfg::PWM_MIN_MOVE) base = cfg::PWM_MIN_MOVE;

  // Rumbo: mantener yaw = 0, desviado un poco para volver al centro del pasillo. Zona muerta y
  // suavizado para que no haga zigzag entre las paredes.
  float target = 0;
  float offset;
  if (useWalls && lateralOffset(offset) && fabsf(offset) > cfg::LATERAL_DEADBAND_MM) {
    offset -= offset > 0 ? cfg::LATERAL_DEADBAND_MM : -cfg::LATERAL_DEADBAND_MM;
    target = clampf(cfg::KP_WALL * offset, -cfg::MAX_WALL_CORRECTION_DEG, cfg::MAX_WALL_CORRECTION_DEG);
  }
  yawTarget_ += cfg::YAW_TARGET_SMOOTH * (target - yawTarget_);
  const float steer = clampf(cfg::KP_YAW * (yawTarget_ - gyro_.yaw()) - cfg::KD_YAW * gyro_.rate(),
                             -cfg::MAX_STEER, cfg::MAX_STEER);

  motors_.set(base - steer, base + steer);
}

RobotHW::DriveEvent RobotHW::drive(float maxMm, int watchSide) {
  const float dt = cfg::CONTROL_PERIOD_US * 1e-6f;

  encoders_.reset();
  gyro_.update();
  gyro_.resetYaw();

  float v = 0;
  float lastPos = 0;
  float speed = 0;
  float integral = 0;
  float openSince = -1;  // posicion donde se vio abierto el lado vigilado
  yawTarget_ = 0;
  uint32_t movingAt = millis();  // ultima vez que se movia: si se atasca, se trata como pared
  DriveEvent ev = DriveEvent::Distance;
  uint32_t next = micros();

  while (true) {
    if (aborted_) {
      ev = DriveEvent::Aborted;
      break;
    }
    waitPeriod(next);

    const float pos = 0.5f * (encoders_.left() + encoders_.right()) * cfg::MM_PER_COUNT;
    speed = 0.7f * speed + 0.3f * (pos - lastPos) / dt;
    lastPos = pos;

    if (speed > 20.0f) movingAt = millis();
    if (millis() - movingAt > cfg::STALL_MS) {
      ev = DriveEvent::FrontWall;
      break;
    }

    float remaining = maxMm - pos;
    if (remaining <= 1.0f) break;
    const int front = lasers_.mm(Lasers::FRONT);
    const int left = lasers_.mm(Lasers::LEFT);
    const int right = lasers_.mm(Lasers::RIGHT);
    // Rozando una pared lateral: se para para enderezarse hacia el otro lado.
    if (left >= 0 && left < cfg::SIDE_CRASH_MM) {
      ev = DriveEvent::CrashLeft;
      break;
    }
    if (right >= 0 && right < cfg::SIDE_CRASH_MM) {
      ev = DriveEvent::CrashRight;
      break;
    }
    // Con pared delante cerca, los laterales pueden estar viendo esa misma pared o la esquina:
    // no se usan para centrar ni para buscar huecos.
    const bool frontNear = front >= 0 && front < cfg::FRONT_WALL_MM;
    if (front >= 0) {
      const float toWall = static_cast<float>(front - cfg::FRONT_TURN_MM);
      if (toWall <= 0) {
        ev = DriveEvent::FrontWall;
        break;
      }
      remaining = fminf(remaining, toWall);
    }

    // Con pared a los dos lados se va midiendo el ancho del pasillo (adapta el raton a la pista).
    if (!frontNear && left >= 0 && right >= 0 && left <= cfg::SIDE_OPEN_MM && right <= cfg::SIDE_OPEN_MM) {
      const float width = left + right + cfg::ROBOT_WIDTH_MM;
      corridorMm_ = clampf(0.98f * corridorMm_ + 0.02f * width, 80.0f, 400.0f);
    }

    // Hueco en el lado vigilado: se confirma tras OPEN_CONFIRM_MM para ignorar lecturas sueltas.
    if (watchSide != 0 && !frontNear) {
      const int side = watchSide > 0 ? left : right;
      if (side >= 0 && side <= cfg::SIDE_OPEN_MM) {
        openSince = -1;
      } else if (openSince < 0) {
        openSince = pos;
      } else if (pos - openSince >= cfg::OPEN_CONFIRM_MM) {
        ev = DriveEvent::SideOpen;
        break;
      }
    }

    const float brakeSpeed = sqrtf(2.0f * cfg::ACCEL * remaining);
    v = fminf(fminf(v + cfg::ACCEL * dt, cfg::WALL_FOLLOW_SPEED), fmaxf(brakeSpeed, cfg::MIN_SPEED));
    controlStep(v, speed, integral, dt, !frontNear);
  }
  motors_.brake();
  delay(60);
  motors_.coast();
  return ev;
}

void RobotHW::selfCheck(char* msg) {
  // 1) Motores: la orden "girar a la izquierda" tiene que dar yaw positivo.
  gyro_.update();
  gyro_.resetYaw();
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

void RobotHW::backUp(float mm) {
  encoders_.reset();
  gyro_.update();
  gyro_.resetYaw();
  const uint32_t t0 = millis();
  uint32_t next = micros();
  while (!aborted_ && millis() - t0 < 1000) {
    waitPeriod(next);
    const float pos = 0.5f * (encoders_.left() + encoders_.right()) * cfg::MM_PER_COUNT;
    if (pos <= -mm) break;
    const float steer = cfg::KP_YAW * -gyro_.yaw() - cfg::KD_YAW * gyro_.rate();
    motors_.set(-cfg::BACKUP_PWM - steer, -cfg::BACKUP_PWM + steer);
  }
  motors_.brake();
  delay(60);
  motors_.coast();
}

void RobotHW::rotate(float degrees) {
  gyro_.update();
  gyro_.resetYaw();
  const uint32_t t0 = millis();
  uint32_t next = micros();
  int settled = 0;

  while (!aborted_ && millis() - t0 < cfg::TURN_TIMEOUT_MS) {
    waitPeriod(next);
    const float err = degrees - gyro_.yaw();
    if (fabsf(err) < cfg::TURN_TOLERANCE_DEG && fabsf(gyro_.rate()) < 20.0f) {
      if (++settled > 10) break;
      motors_.brake();
      continue;
    }
    settled = 0;
    float pwm = clampf(cfg::KP_TURN * err - cfg::KD_TURN * gyro_.rate(), -cfg::TURN_PWM_MAX, cfg::TURN_PWM_MAX);
    if (fabsf(pwm) < cfg::TURN_PWM_MIN) pwm = pwm < 0 ? -cfg::TURN_PWM_MIN : cfg::TURN_PWM_MIN;
    if (cfg::TURN_IN_PLACE) {
      motors_.set(-pwm, pwm);  // sobre si mismo: lo que menos espacio ocupa
    } else if (pwm > 0) {
      // Pivote sobre una rueda: a la izquierda se para la izquierda y empuja la derecha.
      motors_.set(0, pwm);
    } else {
      motors_.set(-pwm, 0);
    }
  }
  motors_.brake();
  delay(60);
  motors_.coast();
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

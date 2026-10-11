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
  if (hasL && hasR) {
    offset = (l - r) * 0.5f;
  } else if (hasL) {
    offset = static_cast<float>(l - cfg::SIDE_CENTER_MM);
  } else if (hasR) {
    offset = static_cast<float>(cfg::SIDE_CENTER_MM - r);
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

    const float err = v - speed;
    integral = clampf(integral + err * dt, -50.0f, 50.0f);
    const float base = cfg::KFF_SPEED * v + cfg::KP_SPEED * err + cfg::KI_SPEED * integral;

    // Rumbo: mantener yaw = 0, desviado un poco para volver al centro del pasillo.
    float yawTarget = 0;
    float offset;
    if (lateralOffset(offset)) {
      yawTarget = clampf(cfg::KP_WALL * offset, -cfg::MAX_WALL_CORRECTION_DEG, cfg::MAX_WALL_CORRECTION_DEG);
    }
    const float steer = cfg::KP_YAW * (yawTarget - gyro_.yaw()) - cfg::KD_YAW * gyro_.rate();

    motors_.set(base - steer, base + steer);
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
    if (fabsf(pwm) < cfg::PWM_MIN_MOVE) pwm = pwm < 0 ? -cfg::PWM_MIN_MOVE : cfg::PWM_MIN_MOVE;
    // Angulo positivo = izquierda: rueda izquierda atras, derecha adelante.
    motors_.set(-pwm, pwm);
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

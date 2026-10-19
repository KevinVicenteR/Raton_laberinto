#include "giros/perfil.h"
#include "seguridad/robot_hw.h"
#include "comunicacion/telemetry.h"
#include "config.h"
#include "odometria/motion_progress.h"

namespace {
float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
}

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
      if (fabsf(rate) < 8.0f && ++settled >= 3) break;
      continue;
    }
    settled = 0;
    const int sign = err >= 0 ? 1 : -1;
    // Perfil angular: la distancia de frenado crece con el cuadrado de la tasa.
    // Se reserva un grado para absorber latencia/inercia antes de la tolerancia.
    const float desired = raton::angularTarget(sign, err, cfg::TURN_RATE_DPS, cfg::TURN_ACCEL_DPS2);
    float pwm = clampf(desired / 900.0f + sign * cfg::PWM_MIN_RUN * fminf(1.0f, fabsf(err) / 6.0f) + cfg::KD_TURN * (desired - rate),
                       -cfg::TURN_PWM_MAX, cfg::TURN_PWM_MAX);
    // Unicamente arranque inicial lejos del objetivo: nunca empujones al afinar.
    if (millis() - t0 < 200 && fabsf(err) > 12.0f && fabsf(rate) < 20.0f && fabsf(pwm) < cfg::TURN_PWM_MIN)
      pwm = sign * cfg::TURN_PWM_MIN;
    if (cfg::TURN_IN_PLACE) {
      motors_.set(-pwm, pwm);  // sobre si mismo: lo que menos espacio ocupa
    } else if (pwm > 0) {
      motors_.set(0, pwm);
    } else {
      motors_.set(-pwm, 0);
    }
  }
  stopMotors();
  cachedCellFront_ = false;
  lasers_.invalidateFilters();
  postTurnFresh_ = true;
  adaptiveSpeed_.caution(cfg::ADAPT_CAUTION_MM);
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


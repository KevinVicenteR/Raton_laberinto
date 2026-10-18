#include "seguridad/robot_hw.h"
#include "velocidad/control.h"
#include "config.h"

float RobotHW::speedOutput(float v, float speed, float& integral, float dt) {
  const raton::VelocityParams p{cfg::PWM_MIN_RUN, cfg::KFF_SPEED, cfg::KP_SPEED, cfg::KI_SPEED, cfg::PWM_MIN_MOVE, cfg::PWM_MAX, cfg::MAX_STEER};
  return raton::speedOutput(v, speed, integral, dt, p);
}

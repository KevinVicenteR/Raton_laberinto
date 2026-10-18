#include "seguridad/robot_hw.h"
#include "comunicacion/telemetry.h"
#include "config.h"
#include "odometria/motion_progress.h"

float RobotHW::wheelDistance() const {
  return 0.5f * (encoders_.left() + encoders_.right()) * cfg::MM_PER_COUNT;
}


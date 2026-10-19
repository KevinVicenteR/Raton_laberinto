#include "centrado/control.h"
#include "seguridad/robot_hw.h"
#include "comunicacion/telemetry.h"
#include "config.h"
#include "odometria/motion_progress.h"

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
  const float base = speedOutput(v, speed, integral, dt);

  float offset, rate, conf;
  const bool walls = useWalls && lateralOffset(offset, rate, conf) && conf > 0.2f;
  const raton::SteeringParams p{cfg::LATERAL_DEADBAND_MM, cfg::KD_YAW, cfg::KP_YAW, cfg::WALL_RATE_DAMP, cfg::MAX_WALL_CORRECTION_DEG, cfg::YAW_TARGET_SMOOTH, cfg::MAX_STEER};
  const float steer = raton::steering(v, dt, walls, walls ? offset : 0, walls ? rate : 0, walls ? conf : 0,
      tuning_.steerMode, tuning_.kpSteer, tuning_.kdSteer, tuning_.kpWall, heading_, gyro_.rate(),
      headingRef_, yawTarget_, p);
  motors_.set(base - steer, base + steer);
}

// ---------------------------------------------------------------- tramos por celdas


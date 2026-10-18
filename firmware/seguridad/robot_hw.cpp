#include "seguridad/robot_hw.h"

#include "comunicacion/telemetry.h"
#include "config.h"

namespace {

raton::GeometryParams geometryParams() {
  raton::GeometryParams p;
  p.sideSensorAheadMm = cfg::SIDE_SENSOR_AHEAD_MM;
  p.sideSensorLateralMm = cfg::SIDE_SENSOR_LATERAL_MM;
  p.frontSensorAheadMm = cfg::FRONT_SENSOR_AHEAD_MM;
  p.frontMarginMm = cfg::FRONT_MARGIN_MM;
  p.bodyWidthMm = cfg::BODY_WIDTH_MM;
  p.wallMm = cfg::WALL_MM;
  p.sideWallFactor = cfg::SIDE_WALL_FACTOR;
  return p;
}

}  // namespace

RobotHW::RobotHW(Motors& motors, Encoders& encoders, Lasers& lasers, Gyro& gyro, Display& display)
    : motors_(motors), encoders_(encoders), lasers_(lasers), gyro_(gyro), display_(display), commands_(*this),
      geom_(geometryParams(), cfg::DEFAULT_CORRIDOR_MM, cfg::CELL_MM, cfg::SIDE_SENSOR_ANGLE_DEG,
            cfg::FRONT_DEADZONE_MM) {}

// ---------------------------------------------------------------- ciclo de control

void RobotHW::waitPeriod(uint32_t& next) {
  const uint32_t now = micros();
  if (lastPeriodUs_ != 0 && now - lastPeriodUs_ > maxLagUs_) maxLagUs_ = now - lastPeriodUs_;
  // Si el ciclo se retraso mucho, no intentar recuperar ciclos perdidos de golpe.
  if (static_cast<int32_t>(now - next) > static_cast<int32_t>(4 * cfg::CONTROL_PERIOD_US)) next = now;
  while (static_cast<int32_t>(micros() - next) < 0) {
    lasers_.update();
    gyro_.update();
  }
  service();
  laserSamples_ = lasers_.sampleCount(Lasers::LEFT) + lasers_.sampleCount(Lasers::FRONT) +
                  lasers_.sampleCount(Lasers::RIGHT);
  sendTelemetry();
  lastPeriodUs_ = micros();
  next += cfg::CONTROL_PERIOD_US;
}

void RobotHW::service() {
  telemetry::poll();
  commands_.poll();
  updateOdometry();
}

void RobotHW::idle(uint32_t ms) {
  const uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    lasers_.update();
    gyro_.update(true);
    service();
    sendTelemetry();
  }
}


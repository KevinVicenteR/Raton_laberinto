#include "seguridad/robot_hw.h"
#include "comunicacion/telemetry.h"
#include "config.h"
#include "odometria/motion_progress.h"

bool RobotHW::takeStart() {
  const bool s = requests_.start;
  requests_.start = false;
  return s;
}

int RobotHW::takeMode() {
  const int m = requests_.mode;
  requests_.mode = 0;
  return m;
}

// ---------------------------------------------------------------- odometria y memoria del modo 5

void RobotHW::recordMotion(const char* phase, float requested, const raton::MotionResult& result) {
  char line[384];
  snprintf(line, sizeof(line),
           "MOTION t=%lu phase=%s req=%d actual=%d error_cdeg=%d completed=%d reason=%d enc=%ld,%ld raw=%d,%d,%d "
           "filtered=%d,%d,%d age=%lu,%lu,%lu conf=%d,%d,%d",
           static_cast<unsigned long>(millis()), phase, static_cast<int>(requested), static_cast<int>(result.actualMm),
           static_cast<int>(result.angularErrorDeg * 100), result.completed, static_cast<int>(result.reason),
           encoders_.leftTotal(), encoders_.rightTotal(), lasers_.acquiredMm(Lasers::LEFT),
           lasers_.acquiredMm(Lasers::FRONT), lasers_.acquiredMm(Lasers::RIGHT), lasers_.mm(Lasers::LEFT),
           lasers_.mm(Lasers::FRONT), lasers_.mm(Lasers::RIGHT),
           static_cast<unsigned long>(lasers_.lastValidAgeMs(Lasers::LEFT)),
           static_cast<unsigned long>(lasers_.lastValidAgeMs(Lasers::FRONT)),
           static_cast<unsigned long>(lasers_.lastValidAgeMs(Lasers::RIGHT)),
           static_cast<int>(100 * lasers_.confidence(Lasers::LEFT)), static_cast<int>(100 * lasers_.confidence(Lasers::FRONT)),
           static_cast<int>(100 * lasers_.confidence(Lasers::RIGHT)));
  telemetry::println(line);
}

void RobotHW::sendTelemetry() {
  if (millis() - lastTelemetry_ < cfg::TELEMETRY_PERIOD_MS) return;
  lastTelemetry_ = millis();
  if (millis() - lastObservationTelemetry_ >= cfg::OBS_PERIOD_MS) {
    lastObservationTelemetry_ = millis();
    char sample[256];
    snprintf(sample, sizeof(sample),
             "OBS t=%lu raw=%d,%d,%d filtered=%d,%d,%d age=%lu,%lu,%lu conf=%d,%d,%d enc=%ld,%ld yaw=%d phase=%s drops=%lu",
             static_cast<unsigned long>(millis()), lasers_.acquiredMm(Lasers::LEFT), lasers_.acquiredMm(Lasers::FRONT),
             lasers_.acquiredMm(Lasers::RIGHT), lasers_.mm(Lasers::LEFT), lasers_.mm(Lasers::FRONT),
             lasers_.mm(Lasers::RIGHT), static_cast<unsigned long>(lasers_.lastValidAgeMs(Lasers::LEFT)),
             static_cast<unsigned long>(lasers_.lastValidAgeMs(Lasers::FRONT)),
             static_cast<unsigned long>(lasers_.lastValidAgeMs(Lasers::RIGHT)),
             static_cast<int>(100 * lasers_.confidence(Lasers::LEFT)),
             static_cast<int>(100 * lasers_.confidence(Lasers::FRONT)),
             static_cast<int>(100 * lasers_.confidence(Lasers::RIGHT)), encoders_.leftTotal(), encoders_.rightTotal(),
             static_cast<int>(gyro_.totalYaw() * 100), state_, static_cast<unsigned long>(telemetry::droppedLines()));
    telemetry::printStatus(sample);
  }
  char line[160];
  const int a = static_cast<int>(gyro_.totalYaw() - odoYaw0_);
  // q = lecturas SIN filtrar (para el mapa: el filtro mantiene valores viejos cuando el laser no mide).
  snprintf(line, sizeof(line), "%s I%d F%d D%d v%d r%d p%d x%d y%d a%d t%d q%d,%d,%d c%d,%d,%d", state_,
           lasers_.mm(Lasers::LEFT), lasers_.mm(Lasers::FRONT), lasers_.mm(Lasers::RIGHT), static_cast<int>(lastSpeed_),
           static_cast<int>(heading_), static_cast<int>(geom_.corridorMm()), static_cast<int>(odoX_),
           static_cast<int>(odoY_), a, static_cast<int>(maxLagUs_ / 1000), lasers_.rawMm(Lasers::LEFT),
           lasers_.rawMm(Lasers::FRONT), lasers_.rawMm(Lasers::RIGHT),
           static_cast<int>(lasers_.confidence(Lasers::LEFT) * 100), static_cast<int>(lasers_.confidence(Lasers::FRONT) * 100),
           static_cast<int>(lasers_.confidence(Lasers::RIGHT) * 100));
  maxLagUs_ = 0;
  telemetry::printStatus(line);
}

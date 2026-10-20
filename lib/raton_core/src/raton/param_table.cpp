#include "raton/param_table.h"

#include <string.h>

namespace raton {

namespace {

struct Entry {
  ParamInfo info;
  float Params::*f;
  int Params::*i;
};

#define PF(n, lo, hi) {{#n, false, lo, hi}, &Params::n, nullptr}
#define PI(n, lo, hi) {{#n, true, lo, hi}, nullptr, &Params::n}

const Entry kTable[] = {
    PF(cellMm, 100, 400), PF(wallMm, 0, 40), PI(mazeW, 1, 32), PI(mazeH, 1, 32), PI(startX, 0, 31),
    PI(startY, 0, 31), PI(startDir, 0, 3), PI(goalX, -1, 31), PI(goalY, -1, 31), PI(goalW, 1, 8), PI(goalH, 1, 8),
    PF(wheelDiamMm, 10, 80), PF(countsPerRev, 10, 100000), PF(trackMm, 30, 150), PF(frontAheadMm, -20, 80),
    PF(diagAheadMm, -20, 80), PF(diagLateralMm, 0, 60), PF(diagMountDeg, 20, 70), PF(sensorFovDeg, 0, 40), PI(frontOffsetMm, -60, 60),
    PI(leftOffsetMm, -60, 60), PI(rightOffsetMm, -60, 60), PF(sensorLatencyS, 0, 0.1f), PI(sensorMaxMm, 100, 2000),
    PF(kS, 0, 0.5f), PF(kV, 0.0002f, 0.02f), PF(motorTauS, 0, 0.3f), PF(maxDuty, 0.2f, 1), PF(kpFwd, 0, 0.2f),
    PF(kdFwd, 0, 0.02f), PF(kiFwd, 0, 1), PF(kpRot, 0, 0.2f), PF(kdRot, 0, 0.02f), PF(kiRot, 0, 1),
    PF(kSteer, 0, 2), PF(steerMaxDeg, 0, 20), PF(steerRateGain, 0, 1), PF(searchSpeed, 50, 1500),
    PF(runSpeed, 50, 2500), PF(turnSpeed, 50, 1500), PF(accel, 200, 20000), PF(decel, 200, 20000),
    PF(rotSpeed, 60, 2000), PF(rotAccel, 200, 30000), PF(smoothRadiusMm, 20, 120), PF(sideWallRatio, 1, 3),
    PF(frontWallRatio, 0.1f, 0.9f), PI(frontStopMm, -1, 300), PF(exploreBudgetS, 0, 600), PF(areaLimitS, 2, 29),
    PI(areaRadius, 0, 4), PF(stallS, 0.05f, 3), PF(backoffMm, 0, 100), PI(learnGyro, 0, 1), PI(learnMotor, 0, 1), PF(gyroScale, 0.9f, 1.1f),
    PF(bodyFrontMm, 0, 100), PF(bodyRearMm, 0, 100), PF(bodyWidthMm, 30, 100), PF(sensorHeightMm, 5, 80),
    PF(sensorTiltDeg, -20, 20), PF(wallHeightMm, 20, 100), PF(floorReflect, 0, 1), PF(leftTiltDeg, -10, 10),
    PF(frontTiltDeg, -10, 10), PF(rightTiltDeg, -10, 10), PI(learnVertical, 0, 1),
};

const int kCount = sizeof(kTable) / sizeof(kTable[0]);

}  // namespace

int paramCount() { return kCount; }
const ParamInfo& paramInfo(int i) { return kTable[i].info; }

float getParam(const Params& p, int i) {
  const Entry& e = kTable[i];
  return e.f ? p.*(e.f) : static_cast<float>(p.*(e.i));
}

int findParam(const char* name) {
  for (int i = 0; i < kCount; ++i)
    if (strcmp(kTable[i].info.name, name) == 0) return i;
  return -1;
}

bool setParam(Params& p, const char* name, float value) {
  const int i = findParam(name);
  if (i < 0) return false;
  const Entry& e = kTable[i];
  if (!(value >= e.info.min && value <= e.info.max)) return false;
  if (e.f) p.*(e.f) = value;
  else p.*(e.i) = static_cast<int>(value < 0 ? value - 0.5f : value + 0.5f);
  return true;
}

}  // namespace raton

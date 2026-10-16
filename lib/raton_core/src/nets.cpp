#include "nets.h"

#include <math.h>

namespace raton {

namespace {
float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
}  // namespace

void controlFeatures(const RiskFrame h[3], float speed, float yawRate, float headingError, float remaining,
                     float baseTarget, bool reverse, float out[kControlInputs]) {
  int k = 0;
  for (int t = 0; t < 3; ++t) {
    for (int i = 0; i < 3; ++i) {
      const SensorObservation& s = h[t].sensor[i];
      out[k++] = s.valid ? clampf(s.mm, 0, 300) / 300 : 0;
      out[k++] = s.valid ? 1.f : 0.f;
      out[k++] = clampf(s.confidence, 0, 1);
      out[k++] = clampf(s.ageMs, 0, 1000) / 1000;
    }
  }
  out[k++] = speed / 300;
  out[k++] = yawRate / 180;
  out[k++] = headingError / 90;
  out[k++] = clampf(remaining / 155, -1, 3);
  out[k++] = baseTarget / kControlYawMax;
  out[k++] = reverse ? 0.f : 1.f;
  out[k++] = reverse ? 1.f : 0.f;
}

void controlOutputs(const float out[2], float& yawDeg, float& speedScale) {
  yawDeg = tanhf(out[0]) * kControlYawMax;
  speedScale = 0.5f * (1 + kControlVMin) + tanhf(out[1]) * 0.5f * (1 - kControlVMin);
}

void logicFeatures(const LogicDir dirs[4], int distHere, float seenFraction, float goalForward, float goalRight,
                   int heading, float out[kLogicInputs]) {
  int k = 0;
  for (int i = 0; i < 4; ++i) {
    const LogicDir& d = dirs[i];
    out[k++] = d.open ? 1.f : 0.f;
    out[k++] = d.known ? 1.f : 0.f;
    out[k++] = d.seen ? 1.f : 0.f;
    out[k++] = d.allowed ? 1.f : 0.f;
    out[k++] = clampf((d.open ? d.dist : 999) / 60.f, 0, 1.5f);
    out[k++] = clampf(d.visits / 4.f, 0, 1);
    out[k++] = d.goalCos;
  }
  out[k++] = clampf(distHere / 60.f, 0, 1.5f);
  out[k++] = seenFraction;
  out[k++] = goalForward;
  out[k++] = goalRight;
  for (int i = 0; i < 4; ++i) out[k++] = heading == i ? 1.f : 0.f;
}

}  // namespace raton

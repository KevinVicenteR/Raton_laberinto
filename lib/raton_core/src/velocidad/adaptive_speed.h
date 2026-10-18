// Consigna continua a partir de observaciones; sin acceso a la pose real ni al mapa oculto.
#pragma once
#include <algorithm>
#include <cmath>

namespace raton {
struct SpeedPolicy {
  float minimumRatio = 0.6f;
  float unknownRatio = 0.9f;
  float headingDeg = 5.0f;
  float frontMm = 120.0f;
};

class AdaptiveSpeed {
 public:
  void caution(float distance) { cautionLeft_ = std::max(cautionLeft_, distance); }
  float cautionLeft() const { return cautionLeft_; }
  static float target(const SpeedPolicy& p, float maximum, float confidence, float heading,
                      bool known, float frontClearance, bool cautious) {
    const auto unit = [](float x) { return std::max(0.0f, std::min(1.0f, x)); };
    float trust = std::min(unit(confidence), unit(1 - std::fabs(heading) / p.headingDeg));
    if (frontClearance >= 0) trust = std::min(trust, unit(frontClearance / p.frontMm));
    if (cautious) trust = 0;
    float ratio = p.minimumRatio + (1 - p.minimumRatio) * trust;
    if (!known) ratio = std::min(ratio, p.unknownRatio);
    return maximum * ratio;
  }
  float step(const SpeedPolicy& p, float maximum, float confidence, float heading, bool known,
             float frontClearance, float distance, float dt, float accel) {
    cautionLeft_ = std::max(0.0f, cautionLeft_ - std::max(0.0f, distance));
    const float wanted = target(p, maximum, confidence, heading, known, frontClearance, cautionLeft_ > 0);
    if (cap_ < 0) cap_ = wanted;
    const float change = accel * dt;
    cap_ += std::max(-change, std::min(change, wanted - cap_));
    return std::min(maximum, cap_);
  }
 private:
  float cautionLeft_ = 0;
  float cap_ = -1;
};
}  // namespace raton

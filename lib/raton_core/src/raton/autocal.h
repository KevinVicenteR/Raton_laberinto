// Aprendizaje acotado: sin maniobras extra fuera de las ordenes de navegacion.
#pragma once
#include <math.h>

namespace raton {
class GyroBiasWindow {
 public:
  void add(float z) {
    if (!isfinite(z)) { invalid_ = true; return; }
    ++n_;
    const float d = z - mean_;
    mean_ += d / n_;
    m2_ += d * (z - mean_);
  }
  bool ready() const { return !invalid_ && n_ >= 25 && fabsf(mean_) < 10 && m2_ / (n_ - 1) < 0.1225f; }
  float bias() const { return mean_; }
 private:
  int n_ = 0;
  bool invalid_ = false;
  float mean_ = 0, m2_ = 0;
};

class Breakaway {
 public:
  void reset() { wait_ = ramp_ = moving_ = 0; }
  // extra conserva solamente el apoyo confirmado por movimiento posterior.
  float update(float cmd, float speed, float dt, bool learn, float& extra) {
    const int sign = cmd > 0 ? 1 : -1;
    if (sign != sign_) reset();
    sign_ = sign;
    if (fabsf(cmd) < 25 || cmd * speed < -25) { reset(); return 0; }
    if (fabsf(speed) < 5) {
      wait_ += dt;
      moving_ = 0;
      if (learn && wait_ > 0.08f) {
        ramp_ += 0.8f * dt;
        if (ramp_ + extra > 0.22f) ramp_ = 0.22f - extra;
      }
    } else if (cmd * speed > 20 * fabsf(cmd)) {
      moving_ += dt;
      if (moving_ > 0.04f) {
        if (learn && wait_ > 0.12f && ramp_ > 0) extra += 0.5f * ramp_;
        reset();
      }
    }
    float fade = 1 - fabsf(speed) / 20;
    if (fade < 0) fade = 0;
    return (cmd > 0 ? 1 : -1) * (extra + ramp_) * fade;
  }
 private:
  int sign_ = 0;
  float wait_ = 0, ramp_ = 0, moving_ = 0;
};
} // namespace raton

#pragma once
#include <math.h>

namespace raton {
inline float angularTarget(int sign, float err, float rate, float accel) {
  return sign * fminf(rate, sqrtf(2.0f * accel * fmaxf(0.0f, fabsf(err) - 1.0f)));
}
}  // namespace raton

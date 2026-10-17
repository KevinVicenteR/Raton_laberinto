#pragma once
#include <cmath>

namespace raton {
// Residuo respecto a la ultima celda alcanzada, incluso si el tramo se interrumpio.
struct MotionProgress { int cells; float offset; };
inline MotionProgress motionProgress(float distance, float initial, float cellMm, int requested, bool reverse) {
  const float net = reverse ? distance - initial : distance + initial;
  int cells = static_cast<int>(std::floor(net / cellMm + 0.5f));
  if (cells < 0) cells = 0;
  if (cells > requested) cells = requested;
  return {cells, reverse ? cells * cellMm - net : net - cells * cellMm};
}
}  // namespace raton

// Laterales a 90 grados: geometria de una pared plana, solo lejos de cruces.
#pragma once
#include <math.h>
#include "raton/params.h"
#include "raton/platform.h"

namespace raton {
struct SideObservation {
  bool usable = false;
  int wall = 0;  // +1 pared, -1 hueco confirmado por lectura lejana, 0 ambiguo
  float y = 0;  // desplazamiento + izquierda
};
inline SideObservation sideObservation(const Params& p, const Range& r, int sign,
                                      float half, float pos, float lateral, float heading,
                                      float speed, float longSigma) {
  SideObservation out;
  if (!p.sideEnabled || !r.fresh || r.mm < 10 || r.mm > p.sensorMaxMm ||
      !isfinite(pos) || !isfinite(lateral) || !isfinite(heading) || !isfinite(speed) ||
      !isfinite(longSigma) || fabsf(heading) > 8 || half <= p.sideLateralMm + 10) return out;
  const float a = heading * 0.01745329252f, c = cosf(a), s = sinf(a);
  const float originY = lateral + p.sideAheadMm * s + sign * p.sideLateralMm * c;
  const float expected = (half - sign * originY) / c;
  if (expected < 10) return out;
  const float originX = pos - speed * p.sensorLatencyS + p.sideAheadMm * c - sign * p.sideLateralMm * s;
  const float hitX = originX - sign * expected * s;
  // Excluir el cono que alcanza postes/bordes y la incertidumbre longitudinal.
  const float margin = 12 + expected * tanf(0.5f * p.sensorFovDeg * 0.01745329252f) + 2 * longSigma;
  if (fabsf(hitX) + margin >= half) return out;
  out.usable = true;
  out.y = sign * (half - (p.sideLateralMm + r.mm) * c) - p.sideAheadMm * s;
  const float delta = r.mm - expected;
  if (fabsf(delta) < 14) out.wall = 1;
  else if (delta > 35) out.wall = -1;
  return out;
}
// Requiere dos muestras nuevas consistentes; no acumula evidencia entre celdas/giros.
struct SideEvidence {
  float lastT = -1, lastMm = -1;
  int key = -1, kind = 0, count = 0;
  bool accept(const SideObservation& o, int mm, float t, int cellKey) {
    if (!o.usable || !o.wall) { count = 0; return false; }
    if (key != cellKey || kind != o.wall || t - lastT > 0.12f || t <= lastT || fabsf(mm - lastMm) > 15)
      count = 0;
    key = cellKey; kind = o.wall; lastT = t; lastMm = static_cast<float>(mm);
    if (count < 3) ++count;
    return count >= 2;
  }
};
}  // namespace raton

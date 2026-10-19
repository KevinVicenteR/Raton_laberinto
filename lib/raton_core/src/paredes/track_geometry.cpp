#include "paredes/track_geometry.h"

#include <math.h>

namespace raton {

namespace {
float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
}  // namespace

TrackGeometry::TrackGeometry(const GeometryParams& p, float corridorMm, float cellMm, float sideAngleDeg,
                             int deadzoneMm)
    : p_(p), corridorMm_(corridorMm), cellMm_(cellMm), sideSin_(sinf(sideAngleDeg / 57.2958f)),
      deadzone_(deadzoneMm) {}

void TrackGeometry::setCorridor(float corridor) {
  corridorMm_ = clampf(corridor, 100.0f, 400.0f);
  cellMm_ = corridorMm_ + p_.wallMm;
}

void TrackGeometry::blendCorridor(float measuredWidth) {
  corridorMm_ = clampf(0.98f * corridorMm_ + 0.02f * measuredWidth, 80.0f, 400.0f);
}

int TrackGeometry::sideFromRay(int rawMm) const {
  // El laser lateral mira en diagonal: lo que mide es la hipotenusa. Distancia del costado a la pared =
  // lectura * sin(angulo) + separacion del sensor - medio ancho del cuerpo.
  if (rawMm < 0) return -1;
  const float v = rawMm * sideSin_ + p_.sideSensorLateralMm - 0.5f * p_.bodyWidthMm;
  return v > 0 ? static_cast<int>(v + 0.5f) : 0;
}

float TrackGeometry::sideLookAhead() const {
  // El rayo avanza (separacion que le falta) / tan(angulo) hasta tocar la pared.
  const float cosA = sqrtf(fmaxf(0.05f, 1.0f - sideSin_ * sideSin_));
  return p_.sideSensorAheadMm + (0.5f * corridorMm_ - p_.sideSensorLateralMm) * cosA / sideSin_;
}

int TrackGeometry::cellSideWall() const {
  return static_cast<int>(fmaxf(45.0f, p_.sideWallFactor * corridorMm_));
}

int TrackGeometry::frontTurnMm() const {
  // Del morro a la pared hay medio pasillo menos lo que va el morro por delante del eje; el laser marca
  // eso mas su punto muerto (lo que lee tocando).
  const float v = 0.5f * corridorMm_ - p_.frontSensorAheadMm + deadzone_;
  const int minimum = deadzone_ + p_.frontMarginMm;
  return v > minimum ? static_cast<int>(v) : minimum;
}

}  // namespace raton

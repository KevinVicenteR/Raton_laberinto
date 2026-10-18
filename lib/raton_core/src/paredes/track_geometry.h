// Geometria de la pista y del raton: pasillo y celda medidos, angulo real de los laseres laterales y
// punto muerto del frontal. De aqui salen todas las distancias de decision (paradas, umbrales de pared).
#pragma once

namespace raton {

struct GeometryParams {
  float sideSensorAheadMm = 30;    // laser lateral por delante del eje
  float sideSensorLateralMm = 30;  // separacion del laser lateral respecto al eje longitudinal
  float frontSensorAheadMm = 40;   // laser frontal por delante del eje
  int frontMarginMm = 20;          // nunca parar con el morro a menos de esto sobre el punto muerto
  float bodyWidthMm = 80;
  float wallMm = 12;               // grosor de pared: celda = pasillo + esto
  float sideWallFactor = 0.42f;    // pared lateral en la celda si el costado queda a menos de esto x pasillo
};

class TrackGeometry {
 public:
  TrackGeometry(const GeometryParams& p, float corridorMm, float cellMm, float sideAngleDeg, int deadzoneMm);

  const GeometryParams& params() const { return p_; }

  float corridorMm() const { return corridorMm_; }
  float cellMm() const { return cellMm_; }
  // Fija el pasillo (y la celda = pasillo + pared).
  void setCorridor(float corridor);
  // Ajuste lento del pasillo con lo que miden los dos laterales en marcha (no toca la celda).
  void blendCorridor(float measuredWidth);
  // Fijada a mano (ordenes c y K): la medicion automatica ya no la cambia.
  bool manual() const { return manual_; }
  void setManual(bool on) { manual_ = on; }

  // Seno del angulo real de los laseres laterales respecto al eje longitudinal.
  float sideSin() const { return sideSin_; }
  void setSideSin(float s) { sideSin_ = s; }
  // Lo que marca el frontal con el morro tocando la pared.
  int deadzoneMm() const { return deadzone_; }
  void setDeadzone(int mm) { deadzone_ = mm; }

  // Distancia del costado a la pared (mm, -1 = nada) a partir de la lectura en diagonal del lateral.
  int sideFromRay(int rawMm) const;
  // Donde toca la pared el rayo lateral con el raton centrado (mm por delante del eje).
  float sideLookAhead() const;
  // Umbral de pared lateral al leer una celda en marcha (desde el costado).
  int cellSideWall() const;
  // Lectura frontal con el raton centrado en la celda (parada para girar sin rozar).
  int frontTurnMm() const;
  // Umbral de pared al leer una celda parado.
  int cellFrontWall() const { return frontTurnMm() + 60; }

 private:
  GeometryParams p_;
  float corridorMm_;
  float cellMm_;
  float sideSin_;
  int deadzone_;
  bool manual_ = false;
};

}  // namespace raton

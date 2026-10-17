// Puertos del explorador por celdas: lo que necesita del robot y de la telemetria. Los implementa el
// firmware (RobotHW, Reporter) y, en las pruebas del PC, un robot simulado.
#pragma once

#include "geometry/track_geometry.h"

namespace raton {

// Recibe las paredes laterales de cada celda de un tramo, en marcha. Devuelve false para parar en esa
// celda (habra que girar).
class CellObserver {
 public:
  virtual bool onCellSides(int index, bool leftWall, bool rightWall, int frontMm) = 0;

 protected:
  ~CellObserver() = default;
};

class CellDriver {
 public:
  virtual bool aborted() = 0;
  virtual void setState(const char* state) = 0;
  virtual TrackGeometry& geometry() = 0;
  // Giro en el sitio hasta el rumbo de la cuadricula (+90 izquierda), sin salir de la celda.
  virtual void turnInCell(float degrees) = 0;
  // Avanza celdas completas a vmax avisando a observer de cada celda. Devuelve las avanzadas de verdad.
  virtual int runCells(int cells, float vmax, bool* unexpected, CellObserver* observer) = 0;
  // Paredes laterales de la celda i del ultimo tramo; false si no llego a medirse.
  virtual bool cellSides(int i, bool& left, bool& right) const = 0;
  // Pared delante, leida parado.
  virtual bool frontWallStopped() = 0;
  // Frontal parado con varias lecturas (mm, -1 = nada).
  virtual int stableFrontMm() = 0;
  // Calibracion quieto con los laterales: true si ajusto el pasillo.
  virtual bool calibrateStill(int& left, int& right, int& front) = 0;
  // Si hay pared delante, se coloca a la distancia de parada.
  virtual void alignToFront() = 0;

 protected:
  ~CellDriver() = default;
};

class MissionLog {
 public:
  // Decision con las lecturas del momento (linea DECIDE).
  virtual void decide(const char* why) = 0;
  // Linea de telemetria importante (CELL, GEOM, MISION...).
  virtual void event(const char* line) = 0;

 protected:
  ~MissionLog() = default;
};

}  // namespace raton

// Mision completa del raton sobre un Platform: explorar (meta, ruta mas corta y vuelta) y carrera.
#pragma once

#include "raton/explorer.h"
#include "raton/maze.h"
#include "raton/navigator.h"
#include "raton/params.h"
#include "raton/platform.h"

namespace raton {

struct MissionResult {
  bool reachedGoal = false;
  float goalTime = -1;     // s desde la salida hasta entrar en la meta
  float totalTime = 0;     // s de toda la mision
  bool proven = false;     // ruta mas corta demostrada al terminar
  bool backHome = false;
  bool exploredAll = false;  // sin meta: conoce todo lo alcanzable
  int recoveries = 0;
  int escapes = 0;
  float maxAreaTime = 0;   // s maximos seguidos en la misma zona
};

class Mouse {
 public:
  Mouse(Platform& io, const Params& p);

  Maze& maze() { return maze_; }
  Navigator& navigator() { return nav_; }
  // Borra el mapa y pone salida, meta y tamano de los parametros.
  void resetMap();
  // Exploracion completa: meta, busqueda de la ruta mas corta (con limite de tiempo) y vuelta.
  MissionResult explore(bool searchAfterGoal = true, bool returnHome = true);
  // Carrera con el mapa que haya (si falta, explora lo necesario por el camino).
  MissionResult race(bool returnHome = true);
  // Solo ir a la meta y parar (la primera pasada de una prueba).
  MissionResult toGoal() { return explore(false, false); }
  // Modo 2: todas las celdas alcanzables, vuelta, ruta minima en celdas y mapa en flash.
  MissionResult mapAll();
  // Modo 3: solo aristas conocidas, sin exploracion optimista. Termina en la meta.
  MissionResult solveLearned();
  // Calibra los tres sensores parado en el centro de una celda con pared a la izquierda y a la derecha:
  // el frontal mide cada pared de frente y cada lateral se pone perpendicular a su pared. Corrige
  // out.*OffsetMm (y el frontal solo si se conoce la celda). Devuelve el ancho del pasillo, -1 si no puede,
  // -2 si con la celda conocida las paredes estan a otra distancia (es otra pista).
  float calibrateSensors(Params& out, bool cellKnown);
  // Mide el pasillo parado en la salida (paredes a los dos lados) y fija out.cellMm. -1 si no puede.
  float measureCell(Params& out);
  // Ajuste del paso de celda en marcha (lo activa measureCell; la competencia lo deja apagado).
  void setLearnCell(bool on) { nav_.setLearnCell(on); }

 private:
  MissionResult finish(MissionResult r, float t0);
  void goHome(MissionResult& r);

  Platform& io_;
  const Params& p_;
  Maze maze_;
  Navigator nav_;
  Explorer explorer_;
};

}  // namespace raton

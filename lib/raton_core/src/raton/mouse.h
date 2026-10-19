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

 private:
  MissionResult finish(MissionResult r, float t0);

  Platform& io_;
  const Params& p_;
  Maze maze_;
  Navigator nav_;
  Explorer explorer_;
};

}  // namespace raton

// Estrategia completa: explorar con flood fill hasta probar el camino mas corto y luego correrlo.
#pragma once

#include "floodfill.h"
#include "maze.h"
#include "robot.h"

namespace raton {

struct SolverStats {
  int searchRuns = 0;      // viajes ida y vuelta completados
  int searchCells = 0;     // celdas recorridas explorando
  int speedRunCells = 0;   // celdas de la carrera rapida
  int speedRunMoves = 0;   // tramos rectos de la carrera rapida
  int turns = 0;
  bool pathProven = false;
};

class Solver {
 public:
  Solver(Robot& robot, int width = MAX_MAZE_SIZE, int height = MAX_MAZE_SIZE);

  // Olvida el mapa y vuelve a la salida (0,0) mirando al norte.
  void reset(int width, int height);
  void reset() { reset(maze_.width(), maze_.height()); }

  // Viajes salida -> meta -> salida hasta probar el camino optimo o llegar a maxRuns.
  // Devuelve false si el robot no pudo llegar (laberinto sin salida o abortado).
  bool explore(int maxRuns);

  // Carrera rapida a la meta usando solo paredes confirmadas, juntando rectas.
  bool speedRun();

  // Va explorando hasta cualquiera de las celdas indicadas.
  bool searchTo(const Cell* targets, int count);
  bool searchToGoal();
  bool returnToStart();

  // Gira en el sitio hasta mirar hacia d.
  void face(Dir d);

  // El camino optimo esta probado si el optimista y el confirmado miden lo mismo.
  bool pathProven();
  uint16_t knownPathLength();  // INF_DIST si aun no hay camino confirmado

  const Maze& maze() const { return maze_; }
  const FloodFill& flood() const { return flood_; }
  Cell position() const { return pos_; }
  Dir heading() const { return heading_; }
  const SolverStats& stats() const { return stats_; }

 private:
  void senseWalls();

  Robot& robot_;
  Maze maze_;
  FloodFill flood_;
  FloodFill check_;
  Cell pos_;
  Dir heading_;
  SolverStats stats_;
};

}  // namespace raton

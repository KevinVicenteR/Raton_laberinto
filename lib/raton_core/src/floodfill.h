// Flood fill (BFS) desde las celdas objetivo hacia todo el laberinto.
#pragma once

#include <stdint.h>

#include "maze.h"

namespace raton {

constexpr uint16_t INF_DIST = 0xFFFF;

class FloodFill {
 public:
  // unknownIsWall = false: optimista, para explorar.
  // unknownIsWall = true: solo caminos ya confirmados, para la carrera rapida.
  void compute(const Maze& maze, const Cell* targets, int count, bool unknownIsWall);

  uint16_t distance(Cell c) const;

  // Direccion que baja un paso hacia el objetivo. Desempata prefiriendo seguir recto.
  // Devuelve false si no hay camino.
  bool bestDirection(const Maze& maze, Cell from, Dir heading, bool unknownIsWall, Dir& out) const;

 private:
  uint16_t dist_[MAX_MAZE_SIZE][MAX_MAZE_SIZE];
};

}  // namespace raton

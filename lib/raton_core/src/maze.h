// Mapa del laberinto: paredes conocidas y celdas visitadas.
// Codigo portable: se compila igual en el Nano 33 (PlatformIO) y en el simulador (CMake).
#pragma once

#include <stdint.h>

namespace raton {

constexpr int MAX_MAZE_SIZE = 32;  // la pista de 1.5 x 3 m con celdas de ~15.5 cm son ~10 x 19

// Coordenadas: (0,0) es la celda de abajo a la izquierda. NORTH es +y. La salida y la meta se
// eligen con Solver::setStart y Maze::setGoal (por defecto: salida (0,0), meta en el centro).
enum Dir : uint8_t { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };

inline Dir rightOf(Dir d) { return static_cast<Dir>((d + 1) & 3); }
inline Dir leftOf(Dir d) { return static_cast<Dir>((d + 3) & 3); }
inline Dir behind(Dir d) { return static_cast<Dir>((d + 2) & 3); }
inline int deltaX(Dir d) { return d == EAST ? 1 : (d == WEST ? -1 : 0); }
inline int deltaY(Dir d) { return d == NORTH ? 1 : (d == SOUTH ? -1 : 0); }
char dirChar(Dir d);  // 'n', 'e', 's', 'w'

struct Cell {
  int8_t x;
  int8_t y;
};

inline bool operator==(Cell a, Cell b) { return a.x == b.x && a.y == b.y; }
inline bool operator!=(Cell a, Cell b) { return !(a == b); }
inline Cell step(Cell c, Dir d) {
  return Cell{static_cast<int8_t>(c.x + deltaX(d)), static_cast<int8_t>(c.y + deltaY(d))};
}

class Maze {
 public:
  Maze(int width = MAX_MAZE_SIZE, int height = MAX_MAZE_SIZE);

  // Borra todo: paredes desconocidas salvo el borde exterior.
  void reset(int width, int height);

  int width() const { return width_; }
  int height() const { return height_; }
  bool inside(Cell c) const { return c.x >= 0 && c.y >= 0 && c.x < width_ && c.y < height_; }

  // Marca la pared como conocida en las dos celdas que separa.
  void setWall(Cell c, Dir d, bool present);
  bool hasWall(Cell c, Dir d) const;
  bool isKnown(Cell c, Dir d) const;

  // Si no se puede pasar. Una pared desconocida cuenta como abierta salvo unknownIsWall.
  bool blocked(Cell c, Dir d, bool unknownIsWall) const;

  void markVisited(Cell c);
  bool visited(Cell c) const;

  // Celdas meta: la elegida con setGoal o, si no hay, el centro (2x2 si el lado es par, 1 si es
  // impar). Devuelve cuantas.
  int goalCells(Cell out[4]) const;
  void setGoal(Cell c) {
    goal_ = c;
    hasGoal_ = true;
  }
  void clearGoal() { hasGoal_ = false; }
  bool isGoal(Cell c) const;

 private:
  int8_t width_;
  int8_t height_;
  uint8_t walls_[MAX_MAZE_SIZE][MAX_MAZE_SIZE];  // bits 0-3: pared, bits 4-7: conocida
  uint8_t visited_[MAX_MAZE_SIZE][MAX_MAZE_SIZE];
  Cell goal_ = {0, 0};
  bool hasGoal_ = false;
};

}  // namespace raton

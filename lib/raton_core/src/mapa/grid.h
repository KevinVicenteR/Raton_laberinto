// Celdas y rumbos de la rejilla del laberinto. Codigo portable (firmware y pruebas en el PC).
// Coordenadas: (0,0) es la celda de abajo a la izquierda; NORTH es +y.
#pragma once

#include <stdint.h>

namespace raton {

enum Dir : uint8_t { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };

inline Dir rightOf(Dir d) { return static_cast<Dir>((d + 1) & 3); }
inline Dir leftOf(Dir d) { return static_cast<Dir>((d + 3) & 3); }
inline int deltaX(Dir d) { return d == EAST ? 1 : (d == WEST ? -1 : 0); }
inline int deltaY(Dir d) { return d == NORTH ? 1 : (d == SOUTH ? -1 : 0); }
inline char dirChar(Dir d) { return "nesw"[d & 3]; }

struct Cell {
  int8_t x;
  int8_t y;
};

inline bool operator==(Cell a, Cell b) { return a.x == b.x && a.y == b.y; }
inline bool operator!=(Cell a, Cell b) { return !(a == b); }
inline Cell makeCell(int x, int y) { return Cell{static_cast<int8_t>(x), static_cast<int8_t>(y)}; }
inline Cell step(Cell c, Dir d) { return makeCell(c.x + deltaX(d), c.y + deltaY(d)); }

}  // namespace raton

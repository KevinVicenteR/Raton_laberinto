// Mapa propio del raton por celdas: paredes, celdas leidas, visitas, callejones e inundacion.
// Rejilla maxima de kMax x kMax: la de la competencia o, en una pista desconocida, una grande con la
// salida en el centro (no se sabe hacia donde se extiende la pista).
#pragma once

#include <stdint.h>

#include "maze/grid.h"

namespace raton {

class CellMap {
 public:
  static constexpr int kMax = 40;
  static constexpr uint8_t kUnreached = 255;
  // Mapa previo: true si hay pared en (x, y) hacia dir (N=0 E=1 S=2 W=3).
  using WallPrior = bool (*)(int x, int y, int dir);

  // Vacia el mapa; con prior, parte de ese dibujo (lo que se vea en cada celda manda sobre el).
  void clear(WallPrior prior, int width, int height);

  int width() const { return width_; }
  int height() const { return height_; }
  bool inside(Cell c) const { return c.x >= 0 && c.y >= 0 && c.x < width_ && c.y < height_; }

  // Fuera de la rejilla cuenta como pared.
  bool wall(Cell c, int dir) const;
  // Pone o quita la pared en las dos celdas que separa.
  void setWall(Cell c, int dir, bool on);
  uint8_t wallBits(Cell c) const { return walls_[c.x][c.y] & 0x0F; }

  bool seen(Cell c) const { return seen_[c.x][c.y]; }
  void markSeen(Cell c) { seen_[c.x][c.y] = true; }
  int seenCount() const;
  uint8_t visits(Cell c) const { return visits_[c.x][c.y]; }
  void addVisit(Cell c);
  bool dead(Cell c) const { return dead_[c.x][c.y]; }

  // Celdas que nunca cuentan como callejon (0 = salida, 1 = destino).
  void setKeep(int slot, Cell c) { keep_[slot] = c; }
  // Marca callejones: una celda ya vista con tres salidas tapadas (paredes o callejones) es callejon.
  // Se repite hasta que no cambia nada, asi un pasillo sin salida entero queda cerrado.
  void pruneDeadEnds();

  // Inundacion desde goal (lo desconocido cuenta como abierto). Con avoidDead no cruza callejones.
  void setAvoidDead(bool on) { avoidDead_ = on; }
  void flood(Cell goal);
  uint8_t dist(Cell c) const { return inside(c) ? dist_[c.x][c.y] : kUnreached; }

  // Celda sin leer mas cercana a la que se puede llegar por lo ya visto (frontera).
  bool nearestUnseen(Cell from, Cell& out);
  // Celdas sin ver en la mejor ruta optimista start->goal (recto si empata); devuelve la mas cercana
  // a from. false si la ruta ya no pasa por ninguna celda sin ver (ruta demostrada).
  bool unseenOnRoute(Cell start, Cell goal, Cell from, Cell& out);

 private:
  bool keepCell(Cell c) const;

  int width_ = kMax;
  int height_ = kMax;
  uint8_t walls_[kMax][kMax] = {};
  bool seen_[kMax][kMax] = {};
  uint8_t visits_[kMax][kMax] = {};
  uint8_t dist_[kMax][kMax] = {};
  bool dead_[kMax][kMax] = {};
  Cell keep_[2] = {{-1, -1}, {-1, -1}};
  bool avoidDead_ = true;
  Cell queue_[kMax * kMax];  // cola de trabajo de la inundacion y de la busqueda de frontera
  Cell route_[kMax * kMax];
  bool mark_[kMax][kMax];    // celdas ya encoladas en la busqueda de frontera
};

}  // namespace raton

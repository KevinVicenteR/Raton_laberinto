// Mapa del laberinto: paredes con votos (cada lectura suma o resta), celdas visitadas y meta.
// Una pared es conocida con kKnown votos netos (dos lecturas que coinciden): una lectura perdida o falsa
// suelta no decide nada. Para cambiar una pared confirmada hacen falta varias lecturas contrarias.
#pragma once

#include <stdint.h>

namespace raton {

enum Dir : uint8_t { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };

inline Dir turnLeft(Dir d) { return static_cast<Dir>((d + 3) & 3); }
inline Dir turnRight(Dir d) { return static_cast<Dir>((d + 1) & 3); }
inline Dir opposite(Dir d) { return static_cast<Dir>((d + 2) & 3); }
inline int dx(Dir d) { return d == EAST ? 1 : (d == WEST ? -1 : 0); }
inline int dy(Dir d) { return d == NORTH ? 1 : (d == SOUTH ? -1 : 0); }

enum class Wall : uint8_t { Unknown, Open, Closed };

struct Cell {
  int8_t x;
  int8_t y;
};

class Maze {
 public:
  static const int kMax = 32;
  static const int8_t kVoteMax = 4;
  static const int8_t kKnown = 2;

  void reset(int w, int h, bool knownBounds);
  int width() const { return w_; }
  int height() const { return h_; }
  bool inside(int x, int y) const { return x >= 0 && y >= 0 && x < w_ && y < h_; }
  int index(int x, int y) const { return y * w_ + x; }
  int cells() const { return w_ * h_; }

  Wall wall(int x, int y, Dir d) const;
  int8_t votes(int x, int y, Dir d) const;
  // Pasable: abierta, o desconocida si optimistic.
  bool passable(int x, int y, Dir d, bool optimistic) const {
    const Wall s = wall(x, y, d);
    return s == Wall::Open || (optimistic && s == Wall::Unknown);
  }
  // Suma una lectura (weight > 0). Devuelve true si cambia el estado de la pared.
  bool observe(int x, int y, Dir d, bool closed, int weight = 1);
  void set(int x, int y, Dir d, Wall s);
  // Olvida las paredes poco confirmadas (|votos| < minVotes), salvo el borde. Para salir de un mapa
  // contradictorio.
  // Devuelve cuantas paredes olvido.
  int forgetWeak(int minVotes);

  void markVisited(int x, int y);
  bool visited(int x, int y) const { return inside(x, y) && visits_[index(x, y)] > 0; }
  uint8_t visits(int x, int y) const { return inside(x, y) ? visits_[index(x, y)] : 0; }
  // Las cuatro paredes conocidas.
  bool fullyKnown(int x, int y) const;

  void setGoal(int x, int y, int w, int h);
  bool isGoal(int x, int y) const;
  int goalX() const { return gx_; }
  int goalY() const { return gy_; }
  int goalW() const { return gw_; }
  int goalH() const { return gh_; }

  // Mascara de paredes cerradas N=1 E=2 S=4 W=8 (formato de la web).
  uint8_t wallMask(int x, int y) const;

  // Para guardar en flash: votos crudos.
  int8_t* rawH() { return &h_votes_[0][0]; }
  int8_t* rawV() { return &v_votes_[0][0]; }
  uint8_t* rawVisits() { return visits_; }

 private:
  // Paredes horizontales: h_votes_[y][x] = pared entre (x, y-1) y (x, y); y = 0..h.
  // Verticales: v_votes_[y][x] = pared entre (x-1, y) y (x, y); x = 0..w.
  int8_t* slot(int x, int y, Dir d);
  const int8_t* slot(int x, int y, Dir d) const;
  bool isBorder(int x, int y, Dir d) const;

  int w_ = 0;
  int h_ = 0;
  bool knownBounds_ = true;
  int8_t h_votes_[kMax + 1][kMax] = {};
  int8_t v_votes_[kMax][kMax + 1] = {};
  uint8_t visits_[kMax * kMax] = {};
  int gx_ = 0, gy_ = 0, gw_ = 1, gh_ = 1;
};

}  // namespace raton

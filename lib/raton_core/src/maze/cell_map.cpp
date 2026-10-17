#include "maze/cell_map.h"

#include <string.h>

namespace raton {

void CellMap::clear(WallPrior prior, int width, int height) {
  width_ = width;
  height_ = height;
  memset(walls_, 0, sizeof(walls_));
  memset(seen_, 0, sizeof(seen_));
  memset(visits_, 0, sizeof(visits_));
  memset(dead_, 0, sizeof(dead_));
  if (!prior) return;
  for (int x = 0; x < width_; ++x) {
    for (int y = 0; y < height_; ++y) {
      for (int d = 0; d < 4; ++d) {
        if (prior(x, y, d)) walls_[x][y] |= 1 << d;
      }
    }
  }
}

bool CellMap::wall(Cell c, int dir) const {
  if (!inside(step(c, static_cast<Dir>(dir)))) return true;
  return (walls_[c.x][c.y] >> dir) & 1;
}

void CellMap::setWall(Cell c, int dir, bool on) {
  const Cell n = step(c, static_cast<Dir>(dir));
  const uint8_t bit = 1 << dir, back = 1 << ((dir + 2) & 3);
  walls_[c.x][c.y] = on ? (walls_[c.x][c.y] | bit) : (walls_[c.x][c.y] & ~bit);
  if (inside(n)) walls_[n.x][n.y] = on ? (walls_[n.x][n.y] | back) : (walls_[n.x][n.y] & ~back);
}

int CellMap::seenCount() const {
  int seen = 0;
  for (int x = 0; x < width_; ++x)
    for (int y = 0; y < height_; ++y) seen += seen_[x][y];
  return seen;
}

void CellMap::addVisit(Cell c) {
  if (visits_[c.x][c.y] < 255) visits_[c.x][c.y]++;
}

bool CellMap::keepCell(Cell c) const {
  for (const Cell& k : keep_) if (k == c) return true;
  return false;
}

void CellMap::pruneDeadEnds() {
  bool changed = true;
  while (changed) {
    changed = false;
    for (int x = 0; x < width_; ++x) {
      for (int y = 0; y < height_; ++y) {
        const Cell c = makeCell(x, y);
        if (!seen_[x][y] || dead_[x][y] || keepCell(c)) continue;
        int blocked = 0;
        for (int d = 0; d < 4; ++d) {
          const Cell n = step(c, static_cast<Dir>(d));
          if (wall(c, d) || (inside(n) && dead_[n.x][n.y])) blocked++;
        }
        if (blocked >= 3) {
          dead_[x][y] = true;
          changed = true;
        }
      }
    }
  }
}

void CellMap::flood(Cell goal) {
  memset(dist_, kUnreached, sizeof(dist_));
  int head = 0, tail = 0;
  dist_[goal.x][goal.y] = 0;
  queue_[tail++] = goal;
  while (head < tail) {
    const Cell c = queue_[head++];
    for (int d = 0; d < 4; ++d) {
      if (wall(c, d)) continue;
      const Cell n = step(c, static_cast<Dir>(d));
      if (dist_[n.x][n.y] != kUnreached) continue;
      if (avoidDead_ && dead_[n.x][n.y]) continue;  // callejon ya visto: no se cruza
      dist_[n.x][n.y] = dist_[c.x][c.y] + 1;
      queue_[tail++] = n;
    }
  }
}

bool CellMap::nearestUnseen(Cell from, Cell& out) {
  memset(mark_, 0, sizeof(mark_));
  int head = 0, tail = 0;
  queue_[tail++] = from;
  mark_[from.x][from.y] = true;
  while (head < tail) {
    const Cell c = queue_[head++];
    if (!seen_[c.x][c.y]) {
      out = c;
      return true;
    }
    for (int d = 0; d < 4; ++d) {
      if (wall(c, d)) continue;
      const Cell n = step(c, static_cast<Dir>(d));
      if (mark_[n.x][n.y] || (dead_[n.x][n.y] && !dead_[from.x][from.y])) continue;
      mark_[n.x][n.y] = true;
      queue_[tail++] = n;
    }
  }
  return false;
}

bool CellMap::unseenOnRoute(Cell start, Cell goal, Cell from, Cell& out) {
  // La poda depende del destino temporal de explorar; no demuestra una ruta.
  setAvoidDead(false);
  flood(goal);
  if (dist(start) == kUnreached) return false;
  int n = 0;
  Cell c = start;
  int dir = -1;
  while (dist_[c.x][c.y] != 0 && n < kMax * kMax) {
    int best = -1;
    for (int d = 0; d < 4; ++d) {
      if (!wall(c, d) && dist(step(c, static_cast<Dir>(d))) + 1 == dist_[c.x][c.y]) {
        if (best < 0 || d == dir) best = d;  // recto si es igual de corto
      }
    }
    if (best < 0) break;
    dir = best;
    c = step(c, static_cast<Dir>(best));
    if (!seen_[c.x][c.y]) route_[n++] = c;
  }
  if (n == 0) return false;
  // La mas cercana al raton (distancia por lo conocido, inundando desde donde esta).
  flood(from);
  int bestI = 0;
  for (int i = 1; i < n; ++i)
    if (dist(route_[i]) < dist(route_[bestI])) bestI = i;
  out = route_[bestI];
  return true;
}

}  // namespace raton

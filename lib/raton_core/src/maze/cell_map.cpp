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

namespace raton {
namespace {
uint32_t mapCrc(const uint8_t* data, size_t size) {
  uint32_t crc = 0xffffffffu;
  for (size_t i = 0; i < size; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) crc = (crc >> 1) ^ (0xedb88320u & (0u - (crc & 1u)));
  }
  return ~crc;
}
}
size_t CellMap::serialize(uint8_t* out, size_t capacity) const {
  const size_t size = 12 + 3 * width_ * height_;
  if (!out || capacity < size) return 0;
  out[0] = 'R'; out[1] = 'M'; out[2] = 'A'; out[3] = 'P';
  out[4] = 1; out[5] = width_; out[6] = height_; out[7] = 0;
  size_t n = 8;
  for (int x = 0; x < width_; ++x) for (int y = 0; y < height_; ++y) {
    out[n++] = walls_[x][y]; out[n++] = seen_[x][y] ? 1 : 0; out[n++] = visits_[x][y];
  }
  const uint32_t crc = mapCrc(out, n);
  for (int i = 0; i < 4; ++i) out[n++] = static_cast<uint8_t>(crc >> (8 * i));
  return size;
}
bool CellMap::deserialize(const uint8_t* data, size_t size) {
  if (!data || size < 12 || data[0] != 'R' || data[1] != 'M' || data[2] != 'A' || data[3] != 'P' ||
      data[4] != 1 || data[7] != 0 || data[5] < 1 || data[5] > kMax || data[6] < 1 || data[6] > kMax ||
      size != static_cast<size_t>(12 + 3 * data[5] * data[6])) return false;
  uint32_t crc = 0;
  for (int i = 0; i < 4; ++i) crc |= static_cast<uint32_t>(data[size - 4 + i]) << (8 * i);
  if (crc != mapCrc(data, size - 4)) return false;
  const int w = data[5], h = data[6];
  for (int x = 0; x < w; ++x) for (int y = 0; y < h; ++y) {
    const size_t at = 8 + 3 * (x * h + y);
    const uint8_t walls = data[at];
    if (walls > 15 || data[at + 1] > 1) return false;
    if (x + 1 < w && !!(walls & 2) != !!(data[at + 3 * h] & 8)) return false;
    if (y + 1 < h && !!(walls & 1) != !!(data[at + 3] & 4)) return false;
  }
  clear(nullptr, w, h);
  size_t n = 8;
  for (int x = 0; x < w; ++x) for (int y = 0; y < h; ++y) {
    walls_[x][y] = data[n++]; seen_[x][y] = data[n++] != 0; visits_[x][y] = data[n++];
  }
  return true;
}
}  // namespace raton

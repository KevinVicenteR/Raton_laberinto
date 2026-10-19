#include "raton/planner.h"

#include <string.h>

namespace raton {

void CellSet::clear() {
  memset(bits, 0, sizeof(bits));
  count = 0;
}

void CellSet::add(int idx) {
  if (has(idx)) return;
  bits[idx >> 3] |= static_cast<uint8_t>(1 << (idx & 7));
  ++count;
}

void goalSet(const Maze& m, CellSet& out) {
  out.clear();
  for (int y = 0; y < m.height(); ++y)
    for (int x = 0; x < m.width(); ++x)
      if (m.isGoal(x, y)) out.add(m.index(x, y));
}

void flood(const Maze& m, const CellSet& targets, bool optimistic, uint16_t* dist) {
  static uint16_t queue[Maze::kMax * Maze::kMax];
  const int n = m.cells();
  int head = 0, tail = 0;
  for (int i = 0; i < n; ++i) {
    if (targets.has(i)) {
      dist[i] = 0;
      queue[tail++] = static_cast<uint16_t>(i);
    } else {
      dist[i] = kInf;
    }
  }
  while (head < tail) {
    const int i = queue[head++];
    const int x = i % m.width(), y = i / m.width();
    for (int d = 0; d < 4; ++d) {
      const Dir dir = static_cast<Dir>(d);
      if (!m.passable(x, y, dir, optimistic)) continue;
      const int j = m.index(x + dx(dir), y + dy(dir));
      if (dist[j] != kInf) continue;
      dist[j] = static_cast<uint16_t>(dist[i] + 1);
      queue[tail++] = static_cast<uint16_t>(j);
    }
  }
}

namespace {

// Monticulo binario minimo de estados (coste, estado) con memoria fija.
struct Heap {
  static const int kCap = Maze::kMax * Maze::kMax * 4;
  uint32_t key[kCap];
  uint16_t val[kCap];
  int size = 0;
  bool push(uint32_t k, uint16_t v) {
    if (size >= kCap) return false;
    int i = size++;
    while (i > 0) {
      const int p = (i - 1) / 2;
      if (key[p] <= k) break;
      key[i] = key[p];
      val[i] = val[p];
      i = p;
    }
    key[i] = k;
    val[i] = v;
    return true;
  }
  void pop(uint32_t& k, uint16_t& v) {
    k = key[0];
    v = val[0];
    const uint32_t lk = key[--size];
    const uint16_t lv = val[size];
    int i = 0;
    while (true) {
      int c = 2 * i + 1;
      if (c >= size) break;
      if (c + 1 < size && key[c + 1] < key[c]) ++c;
      if (key[c] >= lk) break;
      key[i] = key[c];
      val[i] = val[c];
      i = c;
    }
    key[i] = lk;
    val[i] = lv;
  }
};

}  // namespace

void bestRoute(const Maze& m, int x0, int y0, Dir heading, const CellSet& targets, bool optimistic, int turnCost,
               Route& out) {
  // Estado = celda * 4 + rumbo con el que se esta en la celda.
  static uint32_t cost[Maze::kMax * Maze::kMax * 4];
  static uint16_t prev[Maze::kMax * Maze::kMax * 4];
  static Heap heap;
  const int n = m.cells() * 4;
  for (int i = 0; i < n; ++i) {
    cost[i] = 0xFFFFFFFFu;
    prev[i] = 0xFFFF;
  }
  out.ok = false;
  out.length = 0;
  heap.size = 0;
  const uint16_t s0 = static_cast<uint16_t>(m.index(x0, y0) * 4 + heading);
  cost[s0] = 0;
  heap.push(0, s0);
  int found = -1;
  while (heap.size > 0) {
    uint32_t c;
    uint16_t s;
    heap.pop(c, s);
    if (c != cost[s]) continue;
    const int cell = s / 4;
    const Dir h = static_cast<Dir>(s % 4);
    if (targets.has(cell)) {
      found = s;
      break;
    }
    const int x = cell % m.width(), y = cell / m.width();
    for (int d = 0; d < 4; ++d) {
      const Dir nd = static_cast<Dir>(d);
      if (!m.passable(x, y, nd, optimistic)) continue;
      uint32_t step = 10;
      if (nd != h) step += (nd == opposite(h)) ? (s == s0 ? 3u * turnCost : 0xFFFFu) : static_cast<uint32_t>(turnCost);
      const uint16_t t = static_cast<uint16_t>(m.index(x + dx(nd), y + dy(nd)) * 4 + nd);
      if (c + step < cost[t]) {
        cost[t] = c + step;
        prev[t] = s;
        heap.push(cost[t], t);
      }
    }
  }
  if (found < 0) return;
  int len = 0;
  for (int s = found; s != s0 && prev[s] != 0xFFFF; s = prev[s]) ++len;
  if (len > Route::kMaxSteps) return;
  out.length = len;
  out.cost = cost[found];
  int i = len;
  for (int s = found; s != s0 && prev[s] != 0xFFFF; s = prev[s]) out.steps[--i] = static_cast<Dir>(s % 4);
  out.ok = true;
}

int nextStep(const Maze& m, int x, int y, Dir heading, const uint16_t* dist, bool optimistic) {
  int best = -1;
  int bestScore = 0x7FFFFFFF;
  // Preferencia: recto, izquierda, derecha, atras (atras solo si no hay otra).
  const Dir order[4] = {heading, turnLeft(heading), turnRight(heading), opposite(heading)};
  for (int k = 0; k < 4; ++k) {
    const Dir d = order[k];
    if (!m.passable(x, y, d, optimistic)) continue;
    const int nx = x + dx(d), ny = y + dy(d);
    const uint16_t dd = dist[m.index(nx, ny)];
    if (dd == kInf) continue;
    // Distancia manda; luego recto, luego celdas nuevas o poco visitadas.
    int score = dd * 1000;
    if (k != 0) score += 100;
    if (k == 3) score += 400;
    const int v = m.visits(nx, ny);
    score += v > 9 ? 90 : v * 10;
    if (score < bestScore) {
      bestScore = score;
      best = d;
    }
  }
  return best;
}

}  // namespace raton

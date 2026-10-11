#include "floodfill.h"

namespace raton {

void FloodFill::compute(const Maze& maze, const Cell* targets, int count, bool unknownIsWall) {
  for (int x = 0; x < MAX_MAZE_SIZE; ++x) {
    for (int y = 0; y < MAX_MAZE_SIZE; ++y) {
      dist_[x][y] = INF_DIST;
    }
  }

  Cell queue[MAX_MAZE_SIZE * MAX_MAZE_SIZE];
  int head = 0;
  int tail = 0;
  for (int i = 0; i < count; ++i) {
    const Cell t = targets[i];
    if (!maze.inside(t) || dist_[t.x][t.y] == 0) continue;
    dist_[t.x][t.y] = 0;
    queue[tail++] = t;
  }

  while (head < tail) {
    const Cell c = queue[head++];
    const uint16_t next = static_cast<uint16_t>(dist_[c.x][c.y] + 1);
    for (int i = 0; i < 4; ++i) {
      const Dir d = static_cast<Dir>(i);
      if (maze.blocked(c, d, unknownIsWall)) continue;
      const Cell n = step(c, d);
      if (dist_[n.x][n.y] != INF_DIST) continue;
      dist_[n.x][n.y] = next;
      queue[tail++] = n;
    }
  }
}

uint16_t FloodFill::distance(Cell c) const {
  if (c.x < 0 || c.y < 0 || c.x >= MAX_MAZE_SIZE || c.y >= MAX_MAZE_SIZE) return INF_DIST;
  return dist_[c.x][c.y];
}

bool FloodFill::bestDirection(const Maze& maze, Cell from, Dir heading, bool unknownIsWall, Dir& out) const {
  const Dir order[4] = {heading, rightOf(heading), leftOf(heading), behind(heading)};
  uint16_t best = INF_DIST;
  bool found = false;
  for (int i = 0; i < 4; ++i) {
    const Dir d = order[i];
    if (maze.blocked(from, d, unknownIsWall)) continue;
    const uint16_t dist = distance(step(from, d));
    if (dist < best) {
      best = dist;
      out = d;
      found = true;
    }
  }
  return found;
}

}  // namespace raton

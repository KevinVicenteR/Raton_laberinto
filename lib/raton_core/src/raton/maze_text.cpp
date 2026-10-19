#include "raton/maze_text.h"

#include <string.h>

namespace raton {

bool parseMazeText(const char* const* rows, int nrows, Maze& out) {
  if (nrows < 3 || nrows % 2 == 0) return false;
  const int len = static_cast<int>(strlen(rows[0]));
  if ((len - 1) % 4 != 0) return false;
  const int w = (len - 1) / 4, h = (nrows - 1) / 2;
  if (w < 1 || h < 1 || w > Maze::kMax || h > Maze::kMax) return false;
  for (int i = 0; i < nrows; ++i)
    if (static_cast<int>(strlen(rows[i])) < len) return false;
  out.reset(w, h, true);
  for (int y = 0; y < h; ++y) {
    const int top = 2 * (h - 1 - y);  // fila del texto con la pared norte de la fila y
    for (int x = 0; x < w; ++x) {
      out.set(x, y, NORTH, rows[top][4 * x + 2] == '-' ? Wall::Closed : Wall::Open);
      out.set(x, y, SOUTH, rows[top + 2][4 * x + 2] == '-' ? Wall::Closed : Wall::Open);
      out.set(x, y, WEST, rows[top + 1][4 * x] == '|' ? Wall::Closed : Wall::Open);
      out.set(x, y, EAST, rows[top + 1][4 * x + 4] == '|' ? Wall::Closed : Wall::Open);
    }
  }
  return true;
}

void printMaze(const Maze& m, char* out, int size, int rx, int ry) {
  int n = 0;
  auto put = [&](char c) {
    if (n < size - 1) out[n++] = c;
  };
  const int w = m.width(), h = m.height();
  for (int row = 2 * h; row >= 0; --row) {
    const int y = row / 2;
    for (int x = 0; x <= w; ++x) {
      if (row % 2 == 0) {
        put('+');
        if (x == w) break;
        const Wall s = y < h ? m.wall(x, y, SOUTH) : m.wall(x, h - 1, NORTH);
        const char c = s == Wall::Closed ? '-' : (s == Wall::Unknown ? '.' : ' ');
        put(c);
        put(c);
        put(c);
      } else {
        const Wall s = x < w ? m.wall(x, y, WEST) : m.wall(w - 1, y, EAST);
        put(s == Wall::Closed ? '|' : (s == Wall::Unknown ? ':' : ' '));
        if (x == w) break;
        put(' ');
        put(x == rx && y == ry ? 'R' : (m.isGoal(x, y) ? 'M' : (m.visited(x, y) ? ' ' : '?')));
        put(' ');
      }
    }
    put('\n');
  }
  out[n] = 0;
}

}  // namespace raton

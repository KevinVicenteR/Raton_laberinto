#include "maze.h"

namespace raton {

char dirChar(Dir d) {
  static const char kChars[4] = {'n', 'e', 's', 'w'};
  return kChars[d & 3];
}

Maze::Maze(int width, int height) { reset(width, height); }

void Maze::reset(int width, int height) {
  if (width < 1) width = 1;
  if (height < 1) height = 1;
  if (width > MAX_MAZE_SIZE) width = MAX_MAZE_SIZE;
  if (height > MAX_MAZE_SIZE) height = MAX_MAZE_SIZE;
  width_ = static_cast<int8_t>(width);
  height_ = static_cast<int8_t>(height);

  for (int x = 0; x < MAX_MAZE_SIZE; ++x) {
    for (int y = 0; y < MAX_MAZE_SIZE; ++y) {
      walls_[x][y] = 0;
      visited_[x][y] = 0;
    }
  }
  for (int x = 0; x < width_; ++x) {
    setWall(Cell{static_cast<int8_t>(x), 0}, SOUTH, true);
    setWall(Cell{static_cast<int8_t>(x), static_cast<int8_t>(height_ - 1)}, NORTH, true);
  }
  for (int y = 0; y < height_; ++y) {
    setWall(Cell{0, static_cast<int8_t>(y)}, WEST, true);
    setWall(Cell{static_cast<int8_t>(width_ - 1), static_cast<int8_t>(y)}, EAST, true);
  }
  visited_[0][0] = 1;
}

void Maze::setWall(Cell c, Dir d, bool present) {
  if (!inside(c)) return;
  const uint8_t wallBit = static_cast<uint8_t>(1u << d);
  const uint8_t knownBit = static_cast<uint8_t>(0x10u << d);
  walls_[c.x][c.y] = static_cast<uint8_t>((walls_[c.x][c.y] & ~wallBit) | knownBit | (present ? wallBit : 0));

  const Cell n = step(c, d);
  if (!inside(n)) return;
  const Dir o = behind(d);
  const uint8_t oWallBit = static_cast<uint8_t>(1u << o);
  const uint8_t oKnownBit = static_cast<uint8_t>(0x10u << o);
  walls_[n.x][n.y] = static_cast<uint8_t>((walls_[n.x][n.y] & ~oWallBit) | oKnownBit | (present ? oWallBit : 0));
}

bool Maze::hasWall(Cell c, Dir d) const {
  if (!inside(c)) return true;
  return (walls_[c.x][c.y] >> d) & 1u;
}

bool Maze::isKnown(Cell c, Dir d) const {
  if (!inside(c)) return true;
  return (walls_[c.x][c.y] >> (4 + d)) & 1u;
}

bool Maze::blocked(Cell c, Dir d, bool unknownIsWall) const {
  if (!inside(c) || !inside(step(c, d))) return true;
  if (!isKnown(c, d)) return unknownIsWall;
  return hasWall(c, d);
}

void Maze::markVisited(Cell c) {
  if (inside(c)) visited_[c.x][c.y] = 1;
}

bool Maze::visited(Cell c) const { return inside(c) && visited_[c.x][c.y]; }

int Maze::goalCells(Cell out[4]) const {
  int xs[2];
  int ys[2];
  int nx = 0;
  int ny = 0;
  if (width_ % 2 == 0) {
    xs[nx++] = width_ / 2 - 1;
  }
  xs[nx++] = width_ / 2;
  if (height_ % 2 == 0) {
    ys[ny++] = height_ / 2 - 1;
  }
  ys[ny++] = height_ / 2;

  int n = 0;
  for (int i = 0; i < nx; ++i) {
    for (int j = 0; j < ny; ++j) {
      out[n++] = Cell{static_cast<int8_t>(xs[i]), static_cast<int8_t>(ys[j])};
    }
  }
  return n;
}

bool Maze::isGoal(Cell c) const {
  Cell goals[4];
  const int n = goalCells(goals);
  for (int i = 0; i < n; ++i) {
    if (goals[i] == c) return true;
  }
  return false;
}

}  // namespace raton

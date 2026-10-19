#include "raton/maze.h"

#include <string.h>

namespace raton {

void Maze::reset(int w, int h, bool knownBounds) {
  w_ = w < 1 ? 1 : (w > kMax ? kMax : w);
  h_ = h < 1 ? 1 : (h > kMax ? kMax : h);
  knownBounds_ = knownBounds;
  memset(h_votes_, 0, sizeof(h_votes_));
  memset(v_votes_, 0, sizeof(v_votes_));
  memset(visits_, 0, sizeof(visits_));
  // Fuera de la rejilla siempre hay pared; con knownBounds ademas cuenta como confirmada.
  for (int x = 0; x < w_; ++x) {
    h_votes_[0][x] = kVoteMax;
    h_votes_[h_][x] = kVoteMax;
  }
  for (int y = 0; y < h_; ++y) {
    v_votes_[y][0] = kVoteMax;
    v_votes_[y][w_] = kVoteMax;
  }
}

int8_t* Maze::slot(int x, int y, Dir d) {
  return const_cast<int8_t*>(static_cast<const Maze*>(this)->slot(x, y, d));
}

const int8_t* Maze::slot(int x, int y, Dir d) const {
  if (!inside(x, y)) return nullptr;
  switch (d) {
    case NORTH: return &h_votes_[y + 1][x];
    case SOUTH: return &h_votes_[y][x];
    case EAST: return &v_votes_[y][x + 1];
    default: return &v_votes_[y][x];
  }
}

bool Maze::isBorder(int x, int y, Dir d) const { return !inside(x + dx(d), y + dy(d)); }

Wall Maze::wall(int x, int y, Dir d) const {
  const int8_t* s = slot(x, y, d);
  if (!s || isBorder(x, y, d)) return Wall::Closed;
  return *s >= kKnown ? Wall::Closed : (*s <= -kKnown ? Wall::Open : Wall::Unknown);
}

int8_t Maze::votes(int x, int y, Dir d) const {
  const int8_t* s = slot(x, y, d);
  return s ? *s : kVoteMax;
}

bool Maze::observe(int x, int y, Dir d, bool closed, int weight) {
  int8_t* s = slot(x, y, d);
  if (!s || isBorder(x, y, d)) return false;
  const Wall before = wall(x, y, d);
  int v = *s + (closed ? weight : -weight);
  if (v > kVoteMax) v = kVoteMax;
  if (v < -kVoteMax) v = -kVoteMax;
  *s = static_cast<int8_t>(v);
  return wall(x, y, d) != before;
}

void Maze::set(int x, int y, Dir d, Wall s) {
  int8_t* p = slot(x, y, d);
  if (!p || isBorder(x, y, d)) return;
  *p = s == Wall::Closed ? kVoteMax : (s == Wall::Open ? -kVoteMax : 0);
}

int Maze::forgetWeak(int minVotes) {
  int n = 0;
  for (int y = 1; y < h_; ++y)
    for (int x = 0; x < w_; ++x)
      if (h_votes_[y][x] != 0 && h_votes_[y][x] < minVotes && h_votes_[y][x] > -minVotes) {
        h_votes_[y][x] = 0;
        ++n;
      }
  for (int y = 0; y < h_; ++y)
    for (int x = 1; x < w_; ++x)
      if (v_votes_[y][x] != 0 && v_votes_[y][x] < minVotes && v_votes_[y][x] > -minVotes) {
        v_votes_[y][x] = 0;
        ++n;
      }
  return n;
}

void Maze::markVisited(int x, int y) {
  if (inside(x, y) && visits_[index(x, y)] < 255) ++visits_[index(x, y)];
}

bool Maze::fullyKnown(int x, int y) const {
  for (int d = 0; d < 4; ++d)
    if (wall(x, y, static_cast<Dir>(d)) == Wall::Unknown) return false;
  return true;
}

void Maze::setGoal(int x, int y, int w, int h) {
  gx_ = x;
  gy_ = y;
  gw_ = w < 1 ? 1 : w;
  gh_ = h < 1 ? 1 : h;
}

bool Maze::isGoal(int x, int y) const { return x >= gx_ && y >= gy_ && x < gx_ + gw_ && y < gy_ + gh_; }

uint8_t Maze::wallMask(int x, int y) const {
  uint8_t m = 0;
  for (int d = 0; d < 4; ++d)
    if (wall(x, y, static_cast<Dir>(d)) == Wall::Closed) m |= static_cast<uint8_t>(1 << d);
  return m;
}

}  // namespace raton

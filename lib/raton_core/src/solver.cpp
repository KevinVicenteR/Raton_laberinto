#include "solver.h"

namespace raton {

namespace {
const int kMaxSteps = 4 * MAX_MAZE_SIZE * MAX_MAZE_SIZE;
}  // namespace

Solver::Solver(Robot& robot, int width, int height) : robot_(robot), maze_(width, height) {
  reset(width, height);
}

void Solver::reset(int width, int height) {
  maze_.reset(width, height);
  pos_ = start_;
  heading_ = NORTH;
  stats_ = SolverStats();
}

void Solver::senseWalls() {
  const Walls w = robot_.readWalls();
  maze_.setWall(pos_, leftOf(heading_), w.left);
  maze_.setWall(pos_, heading_, w.front);
  maze_.setWall(pos_, rightOf(heading_), w.right);
  maze_.markVisited(pos_);
}

void Solver::face(Dir d) {
  switch ((d - heading_ + 4) & 3) {
    case 0:
      return;
    case 1:
      robot_.turnRight();
      break;
    case 2:
      robot_.turnAround();
      break;
    case 3:
      robot_.turnLeft();
      break;
  }
  heading_ = d;
  stats_.turns++;
}

bool Solver::searchTo(const Cell* targets, int count) {
  robot_.setRunMode(RunMode::Search);
  for (int i = 0; i < kMaxSteps; ++i) {
    senseWalls();
    flood_.compute(maze_, targets, count, false);
    robot_.onStep(maze_, flood_, pos_, heading_);
    if (flood_.distance(pos_) == 0) return true;

    Dir d;
    if (!flood_.bestDirection(maze_, pos_, heading_, false, d)) return false;
    if (robot_.aborted()) return false;
    face(d);
    robot_.moveForward(1);
    pos_ = step(pos_, d);
    stats_.searchCells++;
  }
  return false;
}

bool Solver::searchToGoal() {
  Cell goals[4];
  const int n = maze_.goalCells(goals);
  return searchTo(goals, n);
}

bool Solver::returnToStart() { return searchTo(&start_, 1); }

uint16_t Solver::knownPathLength() {
  Cell goals[4];
  const int n = maze_.goalCells(goals);
  check_.compute(maze_, goals, n, true);
  return check_.distance(start_);
}

bool Solver::pathProven() {
  Cell goals[4];
  const int n = maze_.goalCells(goals);
  check_.compute(maze_, goals, n, false);
  const uint16_t optimistic = check_.distance(start_);
  const uint16_t known = knownPathLength();
  return known != INF_DIST && known == optimistic;
}

bool Solver::explore(int maxRuns) {
  for (int run = 0; run < maxRuns; ++run) {
    if (!searchToGoal()) return false;
    if (!returnToStart()) return false;
    stats_.searchRuns++;
    if (pathProven()) {
      stats_.pathProven = true;
      return true;
    }
  }
  return knownPathLength() != INF_DIST;
}

bool Solver::speedRun() {
  Cell goals[4];
  const int n = maze_.goalCells(goals);
  flood_.compute(maze_, goals, n, true);
  if (flood_.distance(pos_) == INF_DIST) return false;

  robot_.setRunMode(RunMode::Speed);
  while (flood_.distance(pos_) != 0) {
    Dir d;
    if (!flood_.bestDirection(maze_, pos_, heading_, true, d)) return false;
    if (robot_.aborted()) return false;
    face(d);

    // Junta todas las celdas seguidas en la misma direccion en un solo movimiento.
    int cells = 0;
    Cell c = pos_;
    while (flood_.distance(c) != 0 && !maze_.blocked(c, d, true) &&
           flood_.distance(step(c, d)) + 1 == flood_.distance(c)) {
      c = step(c, d);
      cells++;
    }
    robot_.moveForward(cells);
    pos_ = c;
    stats_.speedRunCells += cells;
    stats_.speedRunMoves++;
    robot_.onStep(maze_, flood_, pos_, heading_);
  }
  return true;
}

}  // namespace raton

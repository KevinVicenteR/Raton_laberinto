#include "raton/mouse.h"

#include <stdio.h>

namespace raton {

Mouse::Mouse(Platform& io, const Params& p) : io_(io), p_(p), nav_(io, p, maze_), explorer_(maze_, p, nav_, io) {
  resetMap();
}

void Mouse::resetMap() {
  maze_.reset(p_.mazeW, p_.mazeH, p_.knownBounds);
  maze_.setGoal(p_.goalX, p_.goalY, p_.goalW, p_.goalH);
}

MissionResult Mouse::finish(MissionResult r, float t0) {
  r.totalTime = nav_.now() - t0;
  r.recoveries = nav_.recoveries();
  r.escapes = explorer_.escapes();
  r.maxAreaTime = explorer_.maxAreaTime();
  r.proven = explorer_.routeProven();
  return r;
}

MissionResult Mouse::explore(bool searchAfterGoal, bool returnHome) {
  MissionResult r;
  maze_.setGoal(p_.goalX, p_.goalY, p_.goalW, p_.goalH);
  nav_.begin(p_.startX, p_.startY, static_cast<Dir>(p_.startDir & 3));
  const float t0 = nav_.now();
  explorer_.setHardDeadline(t0 + 595.0f);  // reglamento: 10 minutos por intento
  io_.report("INICIO_MAPA", true);

  explorer_.setPhase(Explorer::Phase::ToGoal);
  NavResult nr = nav_.run(explorer_, p_.searchSpeed, p_.smoothTurnsSearch);
  if (nr != NavResult::Arrived || !maze_.isGoal(nav_.x(), nav_.y())) {
    io_.report(nr == NavResult::Aborted ? "PARADO" : "SIN_META: no encuentra camino a la meta", true);
    return finish(r, t0);
  }
  r.reachedGoal = true;
  r.goalTime = nav_.now() - t0;
  io_.report("META", true);
  char line[48];
  snprintf(line, sizeof(line), "TIEMPO_META %d ms", static_cast<int>(r.goalTime * 1000));
  io_.report(line, true);
  io_.saveMap(maze_);

  if (searchAfterGoal && !explorer_.routeProven()) {
    explorer_.setPhase(Explorer::Phase::Search);
    explorer_.setTimeLimit(nav_.now() + p_.exploreBudgetS);
    nr = nav_.run(explorer_, p_.searchSpeed, p_.smoothTurnsSearch);
    if (nr == NavResult::Aborted) return finish(r, t0);
  }
  if (returnHome) {
    explorer_.setPhase(Explorer::Phase::ToStart);
    nr = nav_.run(explorer_, p_.searchSpeed, p_.smoothTurnsSearch);
    if (nr == NavResult::Arrived && nav_.x() == p_.startX && nav_.y() == p_.startY) {
      nav_.face(static_cast<Dir>(p_.startDir & 3));
      r.backHome = true;
      io_.report("EN_SALIDA", true);
    }
  }
  if (io_.saveMap(maze_)) io_.report("MAPA_GUARDADO", true);
  return finish(r, t0);
}

MissionResult Mouse::race(bool returnHome) {
  MissionResult r;
  maze_.setGoal(p_.goalX, p_.goalY, p_.goalW, p_.goalH);
  nav_.begin(p_.startX, p_.startY, static_cast<Dir>(p_.startDir & 3));
  const float t0 = nav_.now();
  explorer_.setHardDeadline(t0 + 595.0f);
  io_.report("INICIO_MAPA", true);
  explorer_.setPhase(Explorer::Phase::Run);
  NavResult nr = nav_.run(explorer_, p_.runSpeed, p_.smoothTurnsRun);
  if (!(nr == NavResult::Arrived && maze_.isGoal(nav_.x(), nav_.y())))
    io_.report(nr == NavResult::Aborted ? "PARADO" : "SIN_META: no encuentra camino a la meta", true);
  if (nr == NavResult::Arrived && maze_.isGoal(nav_.x(), nav_.y())) {
    r.reachedGoal = true;
    r.goalTime = nav_.now() - t0;
    io_.report("META", true);
    char line[48];
    snprintf(line, sizeof(line), "TIEMPO_META %d ms", static_cast<int>(r.goalTime * 1000));
    io_.report(line, true);
    io_.saveMap(maze_);
    if (returnHome) {
      explorer_.setPhase(Explorer::Phase::ToStart);
      nr = nav_.run(explorer_, p_.searchSpeed, p_.smoothTurnsSearch);
      if (nr == NavResult::Arrived && nav_.x() == p_.startX && nav_.y() == p_.startY) {
        nav_.face(static_cast<Dir>(p_.startDir & 3));
        r.backHome = true;
      }
    }
  }
  return finish(r, t0);
}

}  // namespace raton

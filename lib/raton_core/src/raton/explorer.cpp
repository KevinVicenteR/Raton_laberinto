#include "raton/explorer.h"

#include <stdio.h>
#include <stdlib.h>

namespace raton {

namespace {
const int kTurnCost = 8;  // un giro cuesta como 0.8 celdas rectas
}

void Explorer::setPhase(Phase phase) {
  phase_ = phase;
  escaping_ = false;
  ax_ = -1;
  routeX_ = -1;
  static const char* const kNames[] = {"FASE META", "FASE BUSCAR_RUTA", "FASE VOLVER", "FASE CARRERA"};
  io_.report(kNames[static_cast<int>(phase)], true);
}

bool Explorer::routeProven(uint32_t* knownCost) {
  CellSet goals;
  goalSet(maze_, goals);
  const Dir sd = static_cast<Dir>(p_.startDir & 3);
  static Route opt, known;
  bestRoute(maze_, p_.startX, p_.startY, sd, goals, true, kTurnCost, opt);
  bestRoute(maze_, p_.startX, p_.startY, sd, goals, false, kTurnCost, known);
  if (knownCost) *knownCost = known.ok ? known.cost : 0;
  return opt.ok && known.ok && known.cost <= opt.cost;
}

void Explorer::buildTargets(int x, int y) {
  targets_.clear();
  if (escaping_) {
    // Cualquier celda alcanzable fuera de la zona donde se ha quedado; mejor si no esta visitada.
    for (int pass = 0; pass < 2 && targets_.count == 0; ++pass) {
      for (int cy = 0; cy < maze_.height(); ++cy)
        for (int cx = 0; cx < maze_.width(); ++cx) {
          const int d = abs(cx - ax_) + abs(cy - ay_);
          if (d <= p_.areaRadius + 1) continue;
          if (pass == 0 && maze_.visited(cx, cy)) continue;
          targets_.add(maze_.index(cx, cy));
        }
    }
    return;
  }
  switch (phase_) {
    case Phase::ToGoal:
    case Phase::Run:
      goalSet(maze_, targets_);
      break;
    case Phase::ToStart:
      targets_.add(maze_.index(p_.startX, p_.startY));
      break;
    case Phase::Search: {
      CellSet goals;
      goalSet(maze_, goals);
      static Route opt;
      bestRoute(maze_, p_.startX, p_.startY, static_cast<Dir>(p_.startDir & 3), goals, true, kTurnCost, opt);
      if (!opt.ok) break;
      int cx = p_.startX, cy = p_.startY;
      for (int i = 0; i < opt.length; ++i) {
        cx += dx(opt.steps[i]);
        cy += dy(opt.steps[i]);
        if (!maze_.fullyKnown(cx, cy)) targets_.add(maze_.index(cx, cy));
      }
      // Paredes de la salida sin confirmar tambien cuentan (la ruta empieza alli).
      if (!maze_.fullyKnown(p_.startX, p_.startY) && targets_.count > 0)
        targets_.add(maze_.index(p_.startX, p_.startY));
      (void)x;
      (void)y;
      break;
    }
  }
}

void Explorer::watchArea(int x, int y) {
  const float t = nav_.now();
  if (ax_ < 0 || abs(x - ax_) + abs(y - ay_) > p_.areaRadius) {
    if (escaping_) io_.report("ZONA_LIBRE", true);
    ax_ = x;
    ay_ = y;
    anchorT_ = t;
    escaping_ = false;
    return;
  }
  const float inArea = t - anchorT_;
  if (inArea > maxAreaTime_) maxAreaTime_ = inArea;
  if (!escaping_ && inArea > p_.areaLimitS) {
    escaping_ = true;
    ++escapes_;
    char line[48];
    snprintf(line, sizeof(line), "ZONA_ATRAPADO x=%d y=%d", x, y);
    io_.report(line, true);
    // Atrapado varias veces: el mapa tiene paredes mal leidas. Olvidar las poco confirmadas.
    if (escapes_ % 2 == 0) maze_.forgetWeak(3);
  }
}

int Explorer::stepTowardTargets(int x, int y, Dir h) {
  for (int attempt = 0; attempt < 3; ++attempt) {
    buildTargets(x, y);
    if (targets_.count == 0) return -1;
    flood(maze_, targets_, true, dist_);
    if (dist_[maze_.index(x, y)] != kInf) return nextStep(maze_, x, y, h, dist_, true);
    // Sin camino con el mapa actual: alguna pared esta mal. Olvidar primero las dudosas y luego todas.
    io_.report("MAPA_SIN_CAMINO", true);
    maze_.forgetWeak(attempt == 0 ? 3 : Maze::kVoteMax + 1);
  }
  return -1;
}

int Explorer::decide(int x, int y, Dir h) {
  if (!maze_.inside(x, y)) return -1;
  watchArea(x, y);
  if (!escaping_) {
    switch (phase_) {
      case Phase::ToGoal:
        if (maze_.isGoal(x, y)) return -1;
        break;
      case Phase::ToStart:
        if (x == p_.startX && y == p_.startY) return -1;
        break;
      case Phase::Search:
        if (nav_.now() > deadline_) return -1;
        break;
      case Phase::Run: {
        if (maze_.isGoal(x, y)) return -1;
        CellSet goals;
        goalSet(maze_, goals);
        bestRoute(maze_, x, y, h, goals, false, kTurnCost, route_);
        if (!route_.ok) bestRoute(maze_, x, y, h, goals, true, kTurnCost, route_);
        if (route_.ok && route_.length > 0) {
          routeX_ = x;
          routeY_ = y;
          return route_.steps[0];
        }
        break;
      }
    }
  }
  const int d = stepTowardTargets(x, y, h);
  if (d < 0 && escaping_) {
    escaping_ = false;
    return stepTowardTargets(x, y, h);
  }
  return d;
}

int Explorer::straightAfter(int x, int y, Dir d) {
  if (phase_ != Phase::Run || escaping_ || x != routeX_ || y != routeY_ || !route_.ok) return -1;
  int n = 0;
  while (n < route_.length && route_.steps[n] == d) ++n;
  return n > 0 ? n - 1 : -1;
}

}  // namespace raton

#include "raton/explorer.h"

#include <stdio.h>
#include <stdlib.h>

namespace raton {

namespace {
const int kTurnCost = 8;  // un giro cuesta como 0.8 celdas rectas
}

void Explorer::setPhase(Phase phase) {
  phase_ = phase;
  stickyTarget_ = -1;
  escaping_ = false;
  ax_ = -1;
  routeX_ = -1;
  forgotOnce_ = false;
  forgotAll_ = false;
  if (phase == Phase::ExploreAll) exploredAll_ = false;
  static const char* const kNames[] = {"FASE META", "FASE BUSCAR_RUTA", "FASE VOLVER", "FASE CARRERA",
                                       "FASE EXPLORAR_TODO"};
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

bool Explorer::somethingLeft(int x, int y) {
  CellSet left;
  for (int cy = 0; cy < maze_.height(); ++cy)
    for (int cx = 0; cx < maze_.width(); ++cx)
      if (!maze_.fullyKnown(cx, cy)) left.add(maze_.index(cx, cy));
  if (left.count == 0) return false;
  flood(maze_, left, true, dist_);
  return dist_[maze_.index(x, y)] != kInf;
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
    case Phase::ExploreAll:
      // Celdas con alguna pared sin conocer; la inundacion lleva a la mas cercana.
      for (int cy = 0; cy < maze_.height(); ++cy)
        for (int cx = 0; cx < maze_.width(); ++cx)
          if (!maze_.fullyKnown(cx, cy)) targets_.add(maze_.index(cx, cy));
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
  }
}

int Explorer::stepTowardTargets(int x, int y, Dir h) {
  for (int attempt = 0; attempt < 2; ++attempt) {
    buildTargets(x, y);
    if (autonomous_ && phase_ == Phase::ExploreAll && !escaping_ && targets_.count > 1) {
      // Commit to one reachable frontier until observed, blocked or visited.
      // Both lateral openings remain in the map and become subsequent targets.
      CellSet origin; origin.add(maze_.index(x, y));
      flood(maze_, origin, true, dist_);
      const int here = maze_.index(x, y);
      if (stickyTarget_ < 0 || !targets_.has(stickyTarget_) ||
          stickyTarget_ == here || dist_[stickyTarget_] == kInf) {
        stickyTarget_ = -1;
        int best = kInf;
        for (int cy = 0; cy < maze_.height(); ++cy)
          for (int cx = 0; cx < maze_.width(); ++cx) {
            const int idx = maze_.index(cx, cy);
            if (idx != here && targets_.has(idx) && dist_[idx] < best) {
              best = dist_[idx]; stickyTarget_ = idx;
            }
          }
      }
      if (stickyTarget_ >= 0) { targets_.clear(); targets_.add(stickyTarget_); }
    }
    if (targets_.count > 0) {
      flood(maze_, targets_, true, dist_);
      if (dist_[maze_.index(x, y)] != kInf) return nextStep(maze_, x, y, h, dist_, true);
    }
    if (phase_ == Phase::ExploreAll && !escaping_ && !forgotOnce_) {
      // Antes de dar la pista por terminada, revisar una vez las paredes dudosas (una mal leida puede
      // cerrar una parte sin explorar).
      forgotOnce_ = true;
      if (maze_.forgetWeak(3) > 0) continue;
    }
    if (phase_ == Phase::ExploreAll && !escaping_) {
      exploredAll_ = true;  // nada alcanzable por conocer
      io_.report("EXPLORADO_TODO", true);
      return -1;
    }
    if (escaping_ || forgotOnce_) break;
    // Sin camino: puede haber una pared mal leida. Se olvidan una sola vez las poco confirmadas (el resto
    // del mapa y lo ya visitado se conserva).
    forgotOnce_ = true;
    io_.report("MAPA_SIN_CAMINO: reviso paredes dudosas", true);
    maze_.forgetWeak(3);
  }
  if (phase_ == Phase::ToGoal && !escaping_) {
    // La meta no se alcanza (o no existe en esta pista): explorar todo lo que falta sin olvidar nada.
    io_.report("META_NO_ALCANZABLE: exploro todo", true);
    setPhase(Phase::ExploreAll);
    return stepTowardTargets(x, y, h);
  }
  if (phase_ == Phase::ToStart && !escaping_ && !forgotAll_) {
    // Sin camino a la salida ni olvidando lo dudoso: alguna pared confirmada esta mal. Se olvidan todas (las
    // celdas visitadas se conservan) y se vuelve mirando de nuevo; mas vale volver que quedarse.
    forgotAll_ = true;
    ++wallResets_;
    io_.report("MAPA_REVISADO: olvido paredes para volver", true);
    maze_.forgetWeak(Maze::kVoteMax + 1);
    return stepTowardTargets(x, y, h);
  }
  return -1;
}

int Explorer::decide(int x, int y, Dir h) {
  if (!maze_.inside(x, y)) return -1;
  watchArea(x, y);
  if (!escaping_) {
    switch (phase_) {
      case Phase::ToGoal: {
        if (maze_.isGoal(x, y)) return -1;
        CellSet goals;
        goalSet(maze_, goals);
        if (goals.count == 0) {
          io_.report("SIN_META: exploro todo", true);
          setPhase(Phase::ExploreAll);
        }
        break;
      }
      case Phase::ToStart:
        if (x == p_.startX && y == p_.startY) return -1;
        break;
      case Phase::Search:
        if (nav_.now() > deadline_) return -1;
        break;
      case Phase::ExploreAll: {
        // Hay meta y vuelve a haber camino posible (una pared mal leida se corrigio): ir a por ella.
        CellSet goals;
        goalSet(maze_, goals);
        if (!fullMapping_ && goals.count > 0) {
          flood(maze_, goals, true, dist_);
          if (dist_[maze_.index(x, y)] != kInf) {
            io_.report("META_ALCANZABLE", true);
            setPhase(Phase::ToGoal);
            if (maze_.isGoal(x, y)) return -1;
          }
        }
        break;
      }
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


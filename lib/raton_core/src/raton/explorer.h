// Estrategia de exploracion y carrera (el "cerebro" del navegador).
//
// Fases:
//  ToGoal   inundacion optimista hacia la meta (lo desconocido cuenta como abierto).
//  Search   tras la meta: visita solo las celdas sin confirmar de la mejor ruta posible hasta que la
//           ruta con paredes conocidas es tan buena como la optimista (ruta demostrada) o se acaba el
//           tiempo.
//  ToStart  vuelve a la salida.
//  Run      carrera: ruta mas rapida con coste por giro; si encuentra una pared inesperada, recalcula.
//  ExploreAll  sin meta (o meta que no existe en esta pista): va siempre a la celda sin conocer mas cercana
//           hasta conocer todo lo alcanzable. Lo ya recorrido no se vuelve a visitar si no hace falta.
//
// Seguridad (regla de los 30 s): si el raton sigue en la misma zona mas de areaLimitS, se fuerza un
// objetivo fuera de ella. Si la meta queda sin camino se olvidan una vez las paredes poco confirmadas; si
// sigue sin camino, la meta no esta en esta pista: se explora todo (sin olvidar el mapa) y se vuelve.
#pragma once

#include "raton/maze.h"
#include "raton/navigator.h"
#include "raton/params.h"
#include "raton/planner.h"

namespace raton {

class Explorer : public Brain {
 public:
  enum class Phase { ToGoal, Search, ToStart, Run, ExploreAll };

  Explorer(Maze& maze, const Params& p, Navigator& nav, Platform& io) : maze_(maze), p_(p), nav_(nav), io_(io) {}

  void setPhase(Phase phase);
  Phase phase() const { return phase_; }
  void setFullMapping(bool on) { fullMapping_ = on; }
  void setAutonomous(bool on) { autonomous_ = on; stickyTarget_ = -1; }
  void setTimeLimit(float t) { deadline_ = t; }

  int decide(int x, int y, Dir h) override;
  int straightAfter(int x, int y, Dir d) override;
  bool shouldAbort() override { return nav_.now() > hardDeadline_; }
  void setHardDeadline(float t) { hardDeadline_ = t; }

  // Ruta demostrada desde la salida (solo paredes conocidas) tan buena como la optimista.
  bool routeProven(uint32_t* knownCost = nullptr);
  int escapes() const { return escapes_; }
  float maxAreaTime() const { return maxAreaTime_; }
  // La exploracion completa termino (no queda nada alcanzable por conocer).
  bool exploredAll() const { return exploredAll_; }
  // Veces que se olvidaron todas las paredes para poder volver (el mapa ya no es de fiar).
  int wallResets() const { return wallResets_; }
  // Con el mapa de ahora, queda alguna celda por conocer a la que se pueda llegar desde (x, y).
  bool somethingLeft(int x, int y);

 private:
  void buildTargets(int x, int y);
  int stepTowardTargets(int x, int y, Dir h);
  void watchArea(int x, int y);

  Maze& maze_;
  const Params& p_;
  Navigator& nav_;
  Platform& io_;
  Phase phase_ = Phase::ToGoal;
  CellSet targets_;
  uint16_t dist_[Maze::kMax * Maze::kMax];
  Route route_;
  int routeX_ = -1, routeY_ = -1;
  float deadline_ = 1e9f;
  float hardDeadline_ = 1e9f;

  // Vigilante de zona
  int ax_ = -1, ay_ = -1;
  float anchorT_ = 0;
  bool escaping_ = false;
  int escapes_ = 0;
  float maxAreaTime_ = 0;
  bool forgotOnce_ = false;
  bool forgotAll_ = false;
  int wallResets_ = 0;
  bool exploredAll_ = false;
  bool fullMapping_ = false;
  bool autonomous_ = false;
  int stickyTarget_ = -1;
};

}  // namespace raton


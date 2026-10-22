#include "raton/mouse.h"

#include "raton/planner.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

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
  char line[80];
  snprintf(line, sizeof(line), "SENSORES inclinacion I%+.1f F%+.1f D%+.1f grados, piso %.2f", nav_.sensorTilt(0),
           nav_.sensorTilt(1), nav_.sensorTilt(2), nav_.floorReflect());
  io_.report(line, true);
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
  const int resets = explorer_.wallResets();
  explorer_.setHardDeadline(t0 + 595.0f);  // reglamento: 10 minutos por intento
  io_.report("INICIO_MAPA", true);

  explorer_.setPhase(Explorer::Phase::ToGoal);
  NavResult nr = nav_.run(explorer_, p_.searchSpeed, p_.smoothTurnsSearch);
  CellSet goals;
  goalSet(maze_, goals);
  for (int retry = 0; retry < 2 && nr == NavResult::Arrived && !maze_.isGoal(nav_.x(), nav_.y()) && goals.count > 0 &&
                     explorer_.exploredAll();
       ++retry) {
    // Hay meta, no queda nada por explorar y sigue sin camino: alguna pared confirmada esta mal leida.
    // Ultimo recurso: olvidar las paredes (las celdas visitadas se conservan) y volver a intentarlo.
    io_.report("MAPA_REVISADO: olvido paredes y vuelvo a buscar la meta", true);
    maze_.forgetWeak(Maze::kVoteMax + 1);
    explorer_.setPhase(Explorer::Phase::ToGoal);
    nr = nav_.run(explorer_, p_.searchSpeed, p_.smoothTurnsSearch);
  }
  if (nr != NavResult::Arrived || !maze_.isGoal(nav_.x(), nav_.y())) {
    if (nr == NavResult::Aborted || nr == NavResult::Failed) {
      io_.report("PARADO", true);
      return finish(r, t0);
    }
    // Sin meta en esta pista: ya exploro todo lo alcanzable. Volver a la salida y guardar el mapa.
    r.exploredAll = explorer_.exploredAll();
    if (!r.exploredAll) io_.report("SIN_META: no encuentra camino a la meta", true);
    if (returnHome) goHome(r);
    // Sin meta (practica): si al volver se corrigio alguna pared y aparece algo sin conocer alcanzable,
    // otra vuelta de exploracion (como mucho dos).
    for (int extra = 0; extra < 2 && goals.count == 0 && r.exploredAll && explorer_.wallResets() == resets &&
                        explorer_.somethingLeft(nav_.x(), nav_.y());
         ++extra) {
      io_.report("QUEDA_POR_EXPLORAR", true);
      explorer_.setPhase(Explorer::Phase::ExploreAll);
      nr = nav_.run(explorer_, p_.searchSpeed, p_.smoothTurnsSearch);
      if (nr == NavResult::Aborted || nr == NavResult::Failed) {
        io_.report("PARADO", true);
        return finish(r, t0);
      }
      r.exploredAll = explorer_.exploredAll();
      if (returnHome) {
        r.backHome = false;
        goHome(r);
      }
    }
    if (nav_.sensorFault()) return finish(r, t0);
    if (io_.saveMap(maze_)) io_.report("MAPA_GUARDADO", true);
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
    if (nr == NavResult::Aborted || nr == NavResult::Failed) return finish(r, t0);
  }
  if (returnHome) goHome(r);
  if (nav_.sensorFault()) return finish(r, t0);
  if (io_.saveMap(maze_)) io_.report("MAPA_GUARDADO", true);
  return finish(r, t0);
}

void Mouse::goHome(MissionResult& r) {
  explorer_.setPhase(Explorer::Phase::ToStart);
  const NavResult nr = nav_.run(explorer_, p_.searchSpeed, p_.smoothTurnsSearch);
  if (nr == NavResult::Arrived && nav_.x() == p_.startX && nav_.y() == p_.startY) {
    nav_.face(static_cast<Dir>(p_.startDir & 3));
    r.backHome = true;
    io_.report("EN_SALIDA", true);
  }
}

namespace {
// Lectura de un sensor perpendicular a una pared plana durante el barrido, cerca de center (grados respecto a
// la cuadricula): ahi la lectura es la mas corta. Lecturas sueltas falsas fuera; luego se ajusta una parabola
// a las de +-20 grados (el minimo de las lecturas sueltas saldria corto por el ruido). Si la parabola no sale
// clara (cerca de la perpendicular la lectura es casi plana), la mediana de las de +-5 grados. angle = donde
// esta el minimo, o -1e9 si no se sabe. Devuelve -1 sin medida.
float sweepMin(const Navigator::SweepLog& log, int sensor, float center, float& angle) {
  angle = -1e9f;
  int16_t v[Navigator::SweepLog::kMax];
  float u[Navigator::SweepLog::kMax];
  int n = 0;
  for (int i = 0; i < log.n[sensor]; ++i) {
    const float d = log.ang[sensor][i] - center;
    if (d < -20.0f || d > 20.0f) continue;
    u[n] = d;
    v[n++] = log.mm[sensor][i];
  }
  if (n < 5) return -1.0f;
  auto median = [](int16_t* a, int m) {
    for (int x = 1; x < m; ++x)
      for (int y = x; y > 0 && a[y] < a[y - 1]; --y) {
        const int16_t t = a[y];
        a[y] = a[y - 1];
        a[y - 1] = t;
      }
    return static_cast<float>(a[m / 2]);
  };
  int16_t tmp[Navigator::SweepLog::kMax];
  for (int i = 0; i < n; ++i) tmp[i] = v[i];
  const float med = median(tmp, n);
  const float tol = med * 0.3f > 10.0f ? med * 0.3f : 10.0f;
  float s0 = 0, s1 = 0, s2 = 0, s3 = 0, s4 = 0, r0 = 0, r1 = 0, r2 = 0, lo = 1e9f, hi = -1e9f;
  int nc = 0, k = 0;
  for (int i = 0; i < n; ++i) {
    if (fabsf(v[i] - med) > tol) continue;
    const float x = u[i], r = v[i];
    s0 += 1;
    s1 += x;
    s2 += x * x;
    s3 += x * x * x;
    s4 += x * x * x * x;
    r0 += r;
    r1 += r * x;
    r2 += r * x * x;
    if (x < lo) lo = x;
    if (x > hi) hi = x;
    if (fabsf(x) <= 5.0f) tmp[nc++] = v[i];
    ++k;
  }
  const float near = nc >= 3 ? median(tmp, nc) : -1.0f;
  if (k >= 8 && lo < -8.0f && hi > 8.0f) {
    // Minimos cuadrados de r = c0 + c1 x + c2 x^2 (Cramer).
    const float det = s0 * (s2 * s4 - s3 * s3) - s1 * (s1 * s4 - s3 * s2) + s2 * (s1 * s3 - s2 * s2);
    if (fabsf(det) > 1e-6f) {
      const float c0 = (r0 * (s2 * s4 - s3 * s3) - s1 * (r1 * s4 - s3 * r2) + s2 * (r1 * s3 - s2 * r2)) / det;
      const float c1 = (s0 * (r1 * s4 - r2 * s3) - r0 * (s1 * s4 - s3 * s2) + s2 * (s1 * r2 - r1 * s2)) / det;
      const float c2 = (s0 * (s2 * r2 - s3 * r1) - s1 * (s1 * r2 - s3 * r0) + r0 * (s1 * s3 - s2 * s2)) / det;
      // Curvatura clara (al menos ~1 mm a 10 grados) y minimo dentro del barrido.
      if (c2 > 0.01f) {
        const float x = -c1 / (2.0f * c2);
        if (x > -12.0f && x < 12.0f) {
          angle = center + x;
          return c0 - c1 * c1 / (4.0f * c2);
        }
      }
    }
  }
  return near;
}
}  // namespace

float Mouse::calibrateSensors(Params& out, bool cellKnown) {
  // Quieto en el centro de una celda mirando a start, con pared a los dos lados:
  //  1) mira a la izquierda: el frontal, perpendicular a esa pared, da la distancia del eje a ella;
  //  2) gira 45 grados atras: ahora el lateral izquierdo es el perpendicular a la misma pared;
  //  3) lo mismo a la derecha y vuelve. Un cuarto de vuelta a cada mano: el error del giroscopio se anula.
  // Con el cono del VL53L0X una pared de frente lee la distancia x G(90) (~1.006).
  const Dir start = static_cast<Dir>(out.startDir & 3);
  nav_.begin(out.startX, out.startY, start);
  nav_.setOffsetsCell(nav_.cellPitch());  // lecturas sin corregir mientras se calibra
  const float g90 = nav_.sonda().parallelFactor(1.5708f);
  // Un barrido continuo a cada lado (sin paradas): el frontal pasa por la perpendicular a cada pared (+-90
  // grados) y cada lateral por la suya (+-(90 - montaje)); ahi la lectura es la mas corta. (Donde cae el
  // minimo diria tambien lo torcido que se coloco el raton y el angulo real de montaje, pero cerca de la
  // perpendicular la lectura es casi plana y en el simulador esas estimaciones salian peor que no hacerlas.)
  static Navigator::SweepLog log;
  log = Navigator::SweepLog();
  const float rate = 120.0f;
  nav_.sweep(110.0f, rate, log);
  if (nav_.motion().gyroDisagreement() || nav_.sensorFault() || io_.stopRequested()) return -1;
  nav_.sweep(0.0f, rate, log);
  if (nav_.motion().gyroDisagreement() || nav_.sensorFault() || io_.stopRequested()) return -1;
  nav_.sweep(-110.0f, rate, log);
  if (nav_.motion().gyroDisagreement() || nav_.sensorFault() || io_.stopRequested()) return -1;
  nav_.sweep(0.0f, rate, log);
  if (nav_.motion().gyroDisagreement() || nav_.sensorFault() || io_.stopRequested()) return -1;
  if (nav_.sensorFault() || io_.stopRequested()) return -1;
#ifdef RATON_DEBUG
  if (getenv("SWEEPDBG"))
    for (int k = 0; k < 3; ++k)
      for (int i = 0; i < log.n[k]; ++i) printf("BARRIDO s%d %.1f %d\n", k, log.ang[k][i], log.mm[k][i]);
#endif
  const float m0 = out.diagMountDeg;
  float front[2], diag[2], ang;
  front[0] = sweepMin(log, 1, 90.0f, ang);
  front[1] = sweepMin(log, 1, -90.0f, ang);
  diag[0] = sweepMin(log, 0, 90.0f - m0, ang);
  diag[1] = sweepMin(log, 2, m0 - 90.0f, ang);
  nav_.face(start);  // terminar alineado con la cuadricula
  // Con el lateral perpendicular a la pared, lo que el sensor esta mas cerca de ella que el eje.
  const float mr = m0 / 57.2957795f;
  const float diagPerp = out.diagAheadMm * cosf(mr) + out.diagLateralMm * sinf(mr);
  char line[128];
  // Que lados tienen pared de verdad: con la celda conocida, la lectura tiene que ser la de la pared de la
  // celda (+-3 cm); sin conocerla (practica) cualquier pared cerca.
  bool wall[2];
  const float expected = g90 * (0.5f * (out.cellMm - out.wallMm) - out.frontAheadMm);
  for (int i = 0; i < 2; ++i)
    wall[i] = front[i] >= 5 && (cellKnown ? fabsf(front[i] - expected) < 30.0f : front[i] < 250);
  // Lo medido de frente es lo mas fiable del mapa: las paredes de los lados de la salida.
  for (int i = 0; i < 2; ++i)
    if (front[i] >= 5) maze_.observe(out.startX, out.startY, i == 0 ? turnLeft(start) : turnRight(start), wall[i], 3);
  if (cellKnown && front[0] >= 5 && front[1] >= 5 && front[0] < 300 && front[1] < 300) {
    // Pared cerca a los dos lados pero a otra distancia que la de la competencia: es otra pista. Un VL53L0X
    // sin calibrar puede leer 1-2 cm de mas o de menos: lo confirman los laterales (otros sensores, otro
    // desfase) si pudieron medir.
    const float nominal = out.cellMm - out.wallMm;
    auto off = [nominal](float measured) { return measured > 1.25f * nominal || measured < 0.75f * nominal; };
    const bool frontOff = off((front[0] + front[1]) / g90 + 2.0f * out.frontAheadMm);
    const bool diagOk = diag[0] >= 5 && diag[1] >= 5 && diag[0] < 300 && diag[1] < 300;
    const bool diagOff = !diagOk || off((diag[0] + diag[1]) / g90 + 2.0f * diagPerp);
    const float measured = (front[0] + front[1]) / g90 + 2.0f * out.frontAheadMm;
    if (frontOff && diagOff) {
      snprintf(line, sizeof(line), "PISTA_DISTINTA pasillo %d mm y cellMm %d: usa el modo 3 (practica) o set cellMm",
               static_cast<int>(measured), static_cast<int>(out.cellMm));
      io_.report(line, true);
      return -2;
    }
  }
  if (!wall[0] && !wall[1]) {
    snprintf(line, sizeof(line), "CALIB_SONDA sin paredes a los lados (izq %d der %d)", static_cast<int>(front[0]),
             static_cast<int>(front[1]));
    io_.report(line, true);
    return -1;
  }
  // Desfase del frontal: con la celda conocida y pared a los dos lados, izquierda + derecha = pasillo.
  float eF = 0;
  if (cellKnown && wall[0] && wall[1]) {
    const float corridor = out.cellMm - out.wallMm;
    eF = 0.5f * (front[0] + front[1] - g90 * (corridor - 2.0f * out.frontAheadMm));
    if (eF > 40.0f || eF < -40.0f) eF = 0;  // no cuadra: mejor no tocar
  }
  float wallDist[2] = {0, 0};
  for (int i = 0; i < 2; ++i)
    if (wall[i]) wallDist[i] = (front[i] - eF) / g90 + out.frontAheadMm;  // eje -> cara de la pared
  float eD[2] = {0, 0};
  for (int i = 0; i < 2; ++i) {
    if (!wall[i] || diag[i] < 0) continue;
    eD[i] = diag[i] - g90 * (wallDist[i] - diagPerp);
    if (eD[i] > 40.0f || eD[i] < -40.0f) eD[i] = 0;
  }
  out.frontOffsetMm -= static_cast<int>(floorf(eF + 0.5f));
  out.leftOffsetMm -= static_cast<int>(floorf(eD[0] + 0.5f));
  out.rightOffsetMm -= static_cast<int>(floorf(eD[1] + 0.5f));
  const float corridor = wall[0] && wall[1] ? wallDist[0] + wallDist[1] : -1.0f;
  // Desfases calibrados con este pasillo: si luego en marcha el paso de celda resulta otro, se corrigen.
  nav_.setOffsetsCell(cellKnown || corridor <= 0 ? out.cellMm : corridor + out.wallMm);
  snprintf(line, sizeof(line), "CALIB_SONDA desfases F%d I%d D%d (corregido %d %d %d) pasillo %d mm",
           out.frontOffsetMm, out.leftOffsetMm, out.rightOffsetMm, static_cast<int>(-eF), static_cast<int>(-eD[0]),
           static_cast<int>(-eD[1]), static_cast<int>(corridor));
  io_.report(line, true);
  return corridor;
}

float Mouse::measureCell(Params& out) {
  // Pista de practica: los sensores se calibran con las dos paredes de la salida (el frontal ya calibrado en
  // la competencia) y el pasillo medido da la celda = pasillo + pared.
  // Pista de medida incierta: el paso de celda se afina luego en marcha.
  nav_.setLearnCell(true);
  const float corridor = calibrateSensors(out, false);
  char line[80];
  if (corridor < 100.0f || corridor > 450.0f) {
    snprintf(line, sizeof(line), "CELDA no medida: sigo con %d mm", static_cast<int>(out.cellMm));
    io_.report(line, true);
    return -1;
  }
  out.cellMm = corridor + out.wallMm;
  snprintf(line, sizeof(line), "CELDA medida=%d mm (pasillo %d)", static_cast<int>(out.cellMm), static_cast<int>(corridor));
  io_.report(line, true);
  return out.cellMm;
}

MissionResult Mouse::mapAll() {
  MissionResult r;
  maze_.setGoal(p_.goalX, p_.goalY, p_.goalW, p_.goalH);
  nav_.begin(p_.startX, p_.startY, static_cast<Dir>(p_.startDir & 3));
  const float t0 = nav_.now();
  explorer_.setHardDeadline(t0 + 595.0f);
  explorer_.setFullMapping(true);
  explorer_.setPhase(Explorer::Phase::ExploreAll);
  io_.report("INICIO_MAPA", true);
  const NavResult nr = nav_.run(explorer_, p_.searchSpeed, false);
  explorer_.setFullMapping(false);
  if (nr != NavResult::Arrived || nav_.sensorFault()) return finish(r, t0);
  r.exploredAll = explorer_.exploredAll();
  goHome(r);
  if (nav_.sensorFault() || io_.stopRequested()) return finish(r, t0);
  CellSet goals;
  goalSet(maze_, goals);
  static Route known, optimistic;
  bestRoute(maze_, p_.startX, p_.startY, static_cast<Dir>(p_.startDir & 3), goals, false, 0, known);
  bestRoute(maze_, p_.startX, p_.startY, static_cast<Dir>(p_.startDir & 3), goals, true, 0, optimistic);
  if (known.ok) {
    char line[64];
    snprintf(line, sizeof(line), "RUTA_MAS_CORTA celdas=%d", known.length);
    io_.report(line, true);
  } else io_.report("SIN_RUTA_CONOCIDA_A_META", true);
  if (r.exploredAll && r.backHome) {
    io_.report(io_.saveMap(maze_) ? "MAPA_GUARDADO" : "ERROR_GUARDAR_MAPA", true);
  }
  r = finish(r, t0);
  r.proven = known.ok && optimistic.ok && known.length == optimistic.length;
  return r;
}

MissionResult Mouse::solveLearned() {
  // Cada decision recalcula la ruta minima sin atravesar paredes desconocidas.
  class LearnedBrain : public Brain {
   public:
    LearnedBrain(Maze& m, Navigator& nav) : m_(m), nav_(nav), deadline_(nav.now() + 595.0f) {}
    bool shouldAbort() override { return !nav_.persistent() && nav_.now() >= deadline_; }
    int decide(int x, int y, Dir h) override {
      if (m_.isGoal(x, y)) return -1;
      CellSet goals;
      goalSet(m_, goals);
      bestRoute(m_, x, y, h, goals, false, 0, route_);
      return route_.ok && route_.length ? route_.steps[0] : -1;
    }
   private:
    Maze& m_;
    Navigator& nav_;
    float deadline_;
    Route route_;
  };
  static Route route;
  MissionResult r;
  maze_.setGoal(p_.goalX, p_.goalY, p_.goalW, p_.goalH);
  CellSet goals;
  goalSet(maze_, goals);
  bestRoute(maze_, p_.startX, p_.startY, static_cast<Dir>(p_.startDir & 3), goals, false, 0, route);
  if (!route.ok) {
    io_.setMotors(0, 0);
    io_.report("SIN_RUTA_APRENDIDA: ejecuta el modo 1 autonomo", true);
    return r;
  }
  nav_.begin(p_.startX, p_.startY, static_cast<Dir>(p_.startDir & 3));
  const float t0 = nav_.now();
  LearnedBrain brain(maze_, nav_);
  io_.report("INICIO_MAPA", true);
  const NavResult nr = nav_.run(brain, p_.runSpeed, p_.smoothTurnsRun);
  if (nr == NavResult::Arrived && maze_.isGoal(nav_.x(), nav_.y())) {
    r.reachedGoal = true;
    r.goalTime = nav_.now() - t0;
    io_.report("META", true);
  } else io_.report("PARADO: ruta aprendida interrumpida", true);
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
    io_.report((nr == NavResult::Aborted || nr == NavResult::Failed) ? "PARADO" : "SIN_META: no encuentra camino a la meta", true);
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


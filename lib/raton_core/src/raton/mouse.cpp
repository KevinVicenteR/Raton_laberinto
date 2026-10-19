#include "raton/mouse.h"

#include "raton/planner.h"

#include <math.h>
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
    if (nr == NavResult::Aborted) {
      io_.report("PARADO", true);
      return finish(r, t0);
    }
    // Sin meta en esta pista: ya exploro todo lo alcanzable. Volver a la salida y guardar el mapa.
    r.exploredAll = explorer_.exploredAll();
    if (!r.exploredAll) io_.report("SIN_META: no encuentra camino a la meta", true);
    if (returnHome) goHome(r);
    // Sin meta (practica): si al volver se corrigio alguna pared y aparece algo sin conocer alcanzable,
    // otra vuelta de exploracion (como mucho dos).
    for (int extra = 0; extra < 2 && goals.count == 0 && r.exploredAll && explorer_.somethingLeft(nav_.x(), nav_.y());
         ++extra) {
      io_.report("QUEDA_POR_EXPLORAR", true);
      explorer_.setPhase(Explorer::Phase::ExploreAll);
      nr = nav_.run(explorer_, p_.searchSpeed, p_.smoothTurnsSearch);
      if (nr == NavResult::Aborted) {
        io_.report("PARADO", true);
        return finish(r, t0);
      }
      r.exploredAll = explorer_.exploredAll();
      if (returnHome) {
        r.backHome = false;
        goHome(r);
      }
    }
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
    if (nr == NavResult::Aborted) return finish(r, t0);
  }
  if (returnHome) goHome(r);
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

float Mouse::measureCell(Params& out) {
  // Raton quieto en el centro de la salida: gira a la izquierda y a la derecha y mide cada pared lateral con
  // el frontal (perpendicular a la pared, el sensor mas preciso). Ancho del pasillo = las dos distancias +
  // 2 x lo que el frontal va por delante del eje. Celda = pasillo + pared. Si solo hay pared a un lado, se
  // supone el raton centrado (colocado a mano).
  const Dir start = static_cast<Dir>(out.startDir & 3);
  nav_.begin(out.startX, out.startY, start);
  // Pista de medida incierta: el paso de celda se afina luego en marcha.
  nav_.setLearnCell(true);
  // Desfase del frontal calibrado con calfront en la pista de la competencia: se descuenta al medir y se
  // traslada a la pista nueva (si no, pararia a la distancia de la otra pista).
  const float frontOffset =
      out.frontStopMm >= 0 ? out.frontStopMm - (0.5f * out.cellMm - 0.5f * out.wallMm - out.frontAheadMm) : 0.0f;
  int dist[2] = {-1, -1};
  const Dir sides[2] = {turnLeft(start), turnRight(start)};
  for (int i = 0; i < 2; ++i) {
    nav_.face(sides[i]);
    int l, f, r;
    nav_.sample(2, l, f, r);  // descartar: esas medidas empezaron aun girando
    int a, b, c;
    nav_.sample(3, l, a, r);
    nav_.sample(3, l, b, r);
    nav_.sample(3, l, c, r);
    // Mediana de tres tandas: una lectura mala suelta no cambia la medida.
    f = a > b ? (b > c ? b : (a > c ? c : a)) : (a > c ? a : (b > c ? c : b));
    f -= static_cast<int>(frontOffset);
    if (a >= 0 && b >= 0 && c >= 0 && f >= 5 && f < 450) dist[i] = f;
    // Volver de frente despues de cada lado: un cuarto de vuelta a cada mano y vuelta, asi el error de escala
    // del giroscopio se anula (una vuelta entera en el mismo sentido deja el raton torcido varios grados).
    nav_.face(start);
  }
  float corridor = -1;
  if (dist[0] >= 0 && dist[1] >= 0) corridor = dist[0] + dist[1] + 2.0f * out.frontAheadMm;
  else if (dist[0] >= 0 || dist[1] >= 0) corridor = 2.0f * ((dist[0] >= 0 ? dist[0] : dist[1]) + out.frontAheadMm);
  char line[80];
  if (corridor < 100.0f || corridor > 450.0f) {
    snprintf(line, sizeof(line), "CELDA no medida (izq %d der %d): sigo con %d mm", dist[0], dist[1],
             static_cast<int>(out.cellMm));
    io_.report(line, true);
    return -1;
  }
  out.cellMm = corridor + out.wallMm;
  if (out.frontStopMm >= 0)
    out.frontStopMm = static_cast<int>(0.5f * out.cellMm - 0.5f * out.wallMm - out.frontAheadMm + frontOffset + 0.5f);
  snprintf(line, sizeof(line), "CELDA medida=%d mm (pasillo %d%s) I%d D%d", static_cast<int>(out.cellMm),
           static_cast<int>(corridor), dist[0] >= 0 && dist[1] >= 0 ? "" : ", un solo lado", dist[0], dist[1]);
  io_.report(line, true);
  return out.cellMm;
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

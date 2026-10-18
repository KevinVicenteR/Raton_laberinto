#include "exploracion/orquestador.h"
#include "exploracion/cell_explorer.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

#include "seguridad/orquestador.h"

namespace raton {

CellExplorer::CellExplorer(CellDriver& robot, MissionLog& log, CellMap& map, const ExplorerParams& params)
    : robot_(robot), log_(log), map_(map), p_(params) {}

void CellExplorer::decidef(const char* fmt, ...) {
  char line[80];
  va_list args;
  va_start(args, fmt);
  vsnprintf(line, sizeof(line), fmt, args);
  va_end(args);
  log_.decide(line);
}

// Celda ya leida: se manda a la web para dibujar el mapa aprendido (bits N=1 E=2 S=4 W=8).
void CellExplorer::reportCell(Cell c) {
  char line[40];
  snprintf(line, sizeof(line), "CELL x=%d y=%d w=%d d=%d", c.x, c.y, map_.wallBits(c), map_.dead(c) ? 1 : 0);
  log_.event(line);
}

// Paredes de una celda en la que el raton esta parado y centrado sin haber llegado por un tramo: parado,
// el rayo lateral en diagonal apunta al poste de la esquina y no distingue pared de hueco, asi que cada
// lado se mira girando y leyendo con el laser frontal. Con measure, ademas mide el pasillo (izquierda +
// derecha) y ajusta el tamano de celda: asi vale para pistas de cualquier tamano.
void CellExplorer::senseTurning(Cell pos, Dir heading, bool measure) {
  TrackGeometry& geo = robot_.geometry();
  const int front = robot_.stableFrontMm();
  robot_.turnInCell(90.0f);
  if (robot_.aborted()) return;
  const int left = robot_.stableFrontMm();
  robot_.turnInCell(-180.0f);
  if (robot_.aborted()) return;
  const int right = robot_.stableFrontMm();
  robot_.turnInCell(90.0f);
  // Medir el pasillo solo si de verdad hay pared a los dos lados y el raton esta centrado (las dos
  // lecturas parecidas): si un lado es un hueco, su lectura seria la de una pared mas lejana y daria una
  // celda falsa. Con la celda fijada a mano (orden c) no se toca.
  if (measure && !geo.manual() && left >= 0 && right >= 0 && abs(left - right) <= 40 && left < p_.laserMaxMm &&
      right < p_.laserMaxMm) {
    // Del eje a cada pared = lectura + lo que va el laser por delante del eje.
    const float corridor = (left - geo.deadzoneMm()) + (right - geo.deadzoneMm()) + 2.0f * geo.params().frontSensorAheadMm;
    const float cell = corridor + geo.params().wallMm;
    if (cell >= 140.0f && cell <= 400.0f) geo.setCorridor(corridor);
  }
  char line[64];
  snprintf(line, sizeof(line), "GEOM pasillo=%d celda=%d (I%d F%d D%d)", static_cast<int>(geo.corridorMm()),
           static_cast<int>(geo.cellMm()), left, front, right);
  log_.event(line);
  // Pared de esta celda si el frontal lee menos que el punto medio entre "pared de esta celda" (con el
  // raton centrado, frontTurnMm) y "pared de la celda siguiente": aguanta que el raton no este bien
  // centrado en la salida.
  const int limit = geo.frontTurnMm() + static_cast<int>(p_.frontWallFactor * geo.cellMm());
  auto wall = [&](int mm) { return mm >= 0 && mm < limit; };
  bool wf = wall(front), wl = wall(left), wr = wall(right);
  if (wf && wl && wr && measure) {
    // La salida no puede estar cerrada por los tres lados que se ven: el que lee mas lejos es la salida.
    if (front >= left && front >= right) wf = false;
    else if (left >= right) wl = false;
    else wr = false;
    log_.decide("salida cerrada por tres lados: abre el lado mas lejano");
  }
  map_.setWall(pos, heading, wf);
  map_.setWall(pos, leftOf(heading), wl);
  map_.setWall(pos, rightOf(heading), wr);
  map_.markSeen(pos);
  reportCell(pos);
  log_.decide("celda leida girando");
}

// Salida leida QUIETO (sin girar): primero calibra el pasillo con los laseres laterales y luego decide
// las paredes con esas mismas lecturas. Detras se deja como desconocido (abierto).
void CellExplorer::senseStill(Cell pos, Dir heading) {
  TrackGeometry& geo = robot_.geometry();
  int left, right, front;
  bool measured = robot_.calibrateStill(left, right, front);
  // Salida descentrada a lo largo: se centra con la pared de delante (si la hay) y vuelve a leer.
  if (front >= 0 && front < geo.frontTurnMm() + static_cast<int>(0.75f * geo.cellMm()) &&
      abs(front - geo.frontTurnMm()) > 5) {
    robot_.alignToFront();
    if (robot_.aborted()) return;
    measured = robot_.calibrateStill(left, right, front) || measured;
  }
  char line[80];
  snprintf(line, sizeof(line), "GEOM pasillo=%d celda=%d (quieto I%d F%d D%d%s)", static_cast<int>(geo.corridorMm()),
           static_cast<int>(geo.cellMm()), left, front, right, measured ? "" : " sin medir");
  log_.event(line);
  // Costado a pared: centrado queda a (pasillo - cuerpo) / 2; pared si esta a menos de esa fraccion.
  const int sideLimit = static_cast<int>(geo.params().sideWallFactor * geo.corridorMm());
  const bool wl = left >= 0 && left < sideLimit;
  const bool wr = right >= 0 && right < sideLimit;
  const bool wf = front >= 0 && front < geo.frontTurnMm() + static_cast<int>(p_.frontWallFactor * geo.cellMm());
  map_.setWall(pos, heading, wf);
  map_.setWall(pos, leftOf(heading), wl);
  map_.setWall(pos, rightOf(heading), wr);
  map_.markSeen(pos);
  reportCell(pos);
  log_.decide("salida leida quieto (sin girar)");
}

// Tras un tramo: paredes laterales de cada celda recorrida (medidas al llegar a ella) y la de delante
// de la ultima (con el frontal, ya parado y centrado).
void CellExplorer::applyRun(Cell from, Dir dir, int moved) {
  Cell c = from;
  for (int i = 0; i < moved; ++i) {
    c = step(c, dir);
    bool left = false, right = false;
    if (robot_.cellSides(i, left, right)) {
      map_.setWall(c, leftOf(dir), left);
      map_.setWall(c, rightOf(dir), right);
      map_.markSeen(c);
    }
    if (i + 1 < moved) map_.setWall(c, dir, false);  // la atraveso
    if (i + 1 < moved && map_.seen(c)) reportCell(c);
  }
  if (moved > 0) {
    map_.setWall(c, dir, robot_.frontWallStopped());
    if (map_.seen(c)) reportCell(c);
  }
}

// Exploracion sin pararse: con las paredes recien leidas de la celda index del tramo, sigue recto solo
// si recto sigue siendo lo mejor desde esa celda.
bool CellExplorer::onCellSides(int index, bool leftWall, bool rightWall, int frontMm) {
  Cell c = runFrom_;
  for (int i = 0; i <= index; ++i) c = step(c, runDir_);
  if (!map_.inside(c)) return false;
  map_.setWall(c, leftOf(runDir_), leftWall);
  map_.setWall(c, rightOf(runDir_), rightWall);
  map_.markSeen(c);
  reportCell(c);
  if (c == runGoal_) return false;  // llego al destino
  const Cell next = step(c, runDir_);
  if (!map_.inside(next) || map_.wall(c, runDir_)) return false;  // pared conocida delante
  // Pared delante a menos de una celda y media (el frontal ya la ve): para en esta celda.
  if (frontMm >= 0 && frontMm < static_cast<int>(1.5f * robot_.geometry().cellMm())) return false;
  map_.setKeep(1, runGoal_);
  map_.pruneDeadEnds();
  map_.setAvoidDead(true);
  map_.flood(runGoal_);
  const uint16_t here = map_.dist(c);
  if (here == CellMap::kUnreached || map_.dead(next)) return false;
  return map_.dist(next) < here;
}

bool CellExplorer::exploreTo(Cell& pos, Dir& heading, Cell goal) {
  int noProgress = 0;
  while (!robot_.aborted() && pos != goal) {
    if (!map_.seen(pos)) senseTurning(pos, heading);
    if (robot_.aborted()) return false;
    map_.addVisit(pos);
    map_.setKeep(1, goal);
    map_.pruneDeadEnds();
    map_.setAvoidDead(true);
    map_.flood(goal);
    if (map_.dist(pos) == CellMap::kUnreached) {
      // Esta dentro de un callejon (o al lado): para salir si puede cruzarlo.
      map_.setAvoidDead(false);
      map_.flood(goal);
    }
    const uint16_t here = map_.dist(pos);
    if (here == CellMap::kUnreached) {
      log_.decide("no hay camino a la meta con las paredes vistas");
      return false;
    }
    // Siguiente direccion: baja la distancia; en empate recto, luego giro, y media vuelta lo ultimo;
    // entre iguales, la celda menos visitada.
    int best = -1, bestKey = 1 << 30;
    for (int d = 0; d < 4; ++d) {
      const Cell n = step(pos, static_cast<Dir>(d));
      if (map_.wall(pos, d) || map_.dist(n) >= here) continue;
      const int diff = (d - heading + 4) & 3;
      const int key = (diff == 0 ? 0 : (diff == 2 ? 2 : 1)) * 1000 + map_.visits(n);
      if (key < bestKey) {
        bestKey = key;
        best = d;
      }
    }
    if (best < 0) return false;
    const Dir dir = static_cast<Dir>(best);
    const int diff = (dir - heading + 4) & 3;
    // Tramo largo en la direccion elegida: sigue mientras lo vaya permitiendo el mapa (lo visto y lo
    // desconocido); onCellSides lo para en la celda donde haya que girar, leyendo paredes en marcha.
    int cells = 1;
    Cell c = step(pos, dir);
    while (cells < 12 && c != goal && !map_.wall(c, dir) && map_.inside(step(c, dir)) && !map_.dead(step(c, dir))) {
      c = step(c, dir);
      cells++;
    }
    // Giro en el sitio (el raton cabe en el centro de la celda); en un callejon, media vuelta.
    if (diff != 0) robot_.turnInCell(diff == 1 ? -90.0f : (diff == 3 ? 90.0f : 180.0f));
    if (robot_.aborted()) return false;
    heading = dir;
    bool unexpected = false;
    const Cell from = pos;
    runFrom_ = pos;
    runDir_ = dir;
    runGoal_ = goal;
    const int moved = robot_.runCells(cells, p_.exploreSpeed, &unexpected, this);
    for (int i = 0; i < moved; ++i) pos = step(pos, dir);
    applyRun(from, dir, moved);
    decidef("%d celdas al %c -> %d,%d", moved, dirChar(dir), pos.x, pos.y);
    if (moved == 0) {
      // No avanzo. Solo se apunta pared delante si el frontal (varias lecturas nuevas) la ve de verdad;
      // si se repite, se para en vez de girar sin fin.
      const int ahead = robot_.stableFrontMm();
      if (ahead >= 0 && ahead < robot_.geometry().frontTurnMm() + 50) map_.setWall(pos, dir, true);
      if (++noProgress >= p_.maxNoProgress) {
        robot_.setState("SIN_AVANCE");
        log_.decide("no consigue avanzar: parado");
        return false;
      }
    } else {
      noProgress = 0;
    }
  }
  return !robot_.aborted();
}

bool CellExplorer::runRoute(Cell& pos, Dir& heading, Cell goal, float vmax) {
  // Una ruta puede terminar en un callejon podado para otro destino.
  map_.setAvoidDead(false);
  map_.flood(goal);
  while (!robot_.aborted() && map_.dist(pos) != 0 && map_.dist(pos) != CellMap::kUnreached) {
    const uint16_t here = map_.dist(pos);
    int best = -1;
    for (int d = 0; d < 4 && best < 0; ++d) {
      if (!map_.wall(pos, d) && map_.dist(step(pos, static_cast<Dir>(d))) + 1 == here) best = d;
    }
    // Recto si es igual de corto (menos giros).
    if (!map_.wall(pos, heading) && map_.dist(step(pos, heading)) + 1 == here) best = heading;
    if (best < 0) return false;
    const Dir dir = static_cast<Dir>(best);
    int cells = 0;
    Cell c = pos;
    while (map_.dist(c) != 0 && !map_.wall(c, dir) && map_.dist(step(c, dir)) + 1 == map_.dist(c)) {
      c = step(c, dir);
      cells++;
    }
    const int diff = (dir - heading + 4) & 3;
    if (diff == 2) return false;  // la ruta mas corta nunca da media vuelta
    if (diff == 1 || diff == 3) robot_.turnInCell(diff == 1 ? -90.0f : 90.0f);
    if (robot_.aborted()) return false;
    heading = dir;
    bool unexpected = false;
    const Cell from = pos;
    const int moved = robot_.runCells(cells, vmax, &unexpected, nullptr);
    for (int i = 0; i < moved; ++i) pos = step(pos, dir);
    applyRun(from, dir, moved);
    decidef("tramo de %d celdas al %c -> celda %d,%d", moved, dirChar(dir), pos.x, pos.y);
    if (unexpected || moved < cells) return false;  // el mapa no coincide: sigue explorando desde aqui
  }
  return map_.dist(pos) == 0;
}

// Entrenar (mapear toda la pista) gobernado por el orquestador de mision: en cada paso decide que toca
// (leer la salida, explorar, recuperarse o terminar) y lleva la politica de fallos.
bool CellExplorer::mapTrack(Cell& pos, Dir& heading) {
  const Cell start = pos;
  map_.setKeep(0, start);
  const OrquestadorMision mission;
  Cell target{-1, -1};
  Cell lastTarget{-1, -1};
  int failures = 0;
  int steps = 0;
  bool startRead = false;
  AccionMision lastAction = AccionMision::Terminar;
  while (++steps <= CellMap::kMax * CellMap::kMax) {
    EstadoMision st{};
    st.salidaLeida = startRead;
    st.hayFrontera = map_.nearestUnseen(pos, target);
    st.hayMeta = false;
    st.fallosSeguidos = failures;
    st.abortado = robot_.aborted();
    const AccionMision act = mission.decidir(st);
    if (act != lastAction) {
      char line[40];
      snprintf(line, sizeof(line), "MISION %s", OrquestadorMision::nombre(act));
      log_.event(line);
      lastAction = act;
    }
    if (act == AccionMision::Terminar) break;
    if (act == AccionMision::LeerSalida) {
      senseStill(pos, heading);  // la salida se calibra y se lee quieto, sin girar
      startRead = true;
      continue;
    }
    if (act == AccionMision::Recuperar) {
      // Demasiados fallos con el mismo objetivo: se da por inalcanzable (de momento) y se sigue.
      log_.event("MAPA_INCOMPLETO: frontera inalcanzable");
      return false;
      decidef("celda %d,%d inalcanzable tras %d intentos", target.x, target.y, failures);
      failures = 0;
      continue;
    }
    if (target == pos) {
      senseTurning(pos, heading);
      continue;
    }
    decidef("mapeando: siguiente celda sin ver %d,%d", target.x, target.y);
    if (!exploreTo(pos, heading, target) && !robot_.aborted()) {
      // No se llego (pared nueva, roce o sin avance): solo se vuelve a mirar delante (sin girar) y cuenta
      // el fallo; el orquestador decide cuando dejar ese objetivo.
      failures = (target == lastTarget) ? failures + 1 : 1;
      lastTarget = target;
      const int ahead = robot_.stableFrontMm();
      map_.setWall(pos, heading, ahead >= 0 && ahead < robot_.geometry().frontTurnMm() + 50);
      reportCell(pos);
    } else {
      failures = 0;
    }
  }
  if (robot_.aborted() || steps > CellMap::kMax * CellMap::kMax) return false;
  for (int x = 0; x < map_.width(); ++x) for (int y = 0; y < map_.height(); ++y) {
    if ((x == 0 || y == 0 || x == map_.width() - 1 || y == map_.height() - 1) && map_.seen(makeCell(x, y))) {
      log_.event("MAPA_LIMITE: pista fuera de la rejilla");
      return false;
    }
  }
  char line[40];
  snprintf(line, sizeof(line), "MAPA COMPLETO celdas=%d", map_.seenCount());
  log_.event(line);
  log_.decide("mapa completo: vuelve a la salida");
  return exploreTo(pos, heading, start);
}

// Busqueda Adachi 1: tras llegar a la meta, mira la mejor ruta OPTIMISTA salida->meta (lo desconocido
// cuenta como abierto) y va a leer las celdas sin ver que haya EN ESA RUTA, la mas cercana primero.
// Cuando la ruta ya no pasa por celdas sin ver esta demostrada. Despues vuelve a la salida explorando.
void CellExplorer::adachiSearch(Cell& pos, Dir& heading, Cell start, Cell goal) {
  map_.setKeep(0, start);
  map_.setKeep(1, goal);
  Cell target{-1, -1}, lastTarget{-1, -1};
  int failures = 0;
  for (int i = 0; i < p_.searchMaxTargets && !robot_.aborted(); ++i) {
    if (!map_.unseenOnRoute(start, goal, pos, target)) {
      log_.decide("BUSQUEDA ruta demostrada: no quedan celdas sin ver en la mejor ruta");
      break;
    }
    decidef("BUSQUEDA celda sin ver en la mejor ruta %d,%d", target.x, target.y);
    if (exploreTo(pos, heading, target)) {
      failures = 0;
      continue;
    }
    if (robot_.aborted()) return;
    failures = (target == lastTarget) ? failures + 1 : 1;
    lastTarget = target;
    if (failures >= p_.maxNoProgress) {
      map_.markSeen(target);  // inalcanzable de momento: no se insiste
      failures = 0;
    }
  }
  if (robot_.aborted()) return;
  log_.decide("BUSQUEDA vuelve a la salida explorando");
  exploreTo(pos, heading, start);
}

}  // namespace raton

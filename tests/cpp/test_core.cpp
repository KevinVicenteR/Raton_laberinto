// Pruebas del nucleo portable (lib/raton_core) en el PC. Se corren con ctest.
// El explorador por celdas es el MISMO codigo que lleva el firmware; aqui conduce un robot simulado
// que lee las paredes de un laberinto de tests/mazes.
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <queue>
#include <string>
#include <vector>

#include "../../firmware/app/competition_maze.h"
#include "control/motion_progress.h"
#include "control/orquestador.h"
#include "exploration/cell_explorer.h"
#include "geometry/track_geometry.h"
#include "maze/cell_map.h"
#include "protocol/frame.h"
#include "protocol/line_queue.h"

namespace {

int failures = 0;

#define CHECK(cond)                                                          \
  do {                                                                       \
    if (!(cond)) {                                                           \
      std::cerr << __FILE__ << ":" << __LINE__ << ": fallo: " #cond << '\n'; \
      failures++;                                                            \
    }                                                                        \
  } while (0)

using raton::Cell;
using raton::Dir;
using raton::makeCell;

// ---------------------------------------------------------------- laberinto de referencia

// Dibujo ASCII con la fila de arriba primero ("---" pared horizontal, "|" vertical).
struct Truth {
  int width = 0;
  int height = 0;
  std::vector<std::string> rows;

  static Truth load(const std::string& path) {
    Truth t;
    std::ifstream in(path);
    for (std::string line; std::getline(in, line);) {
      if (!line.empty() && line.back() == '\r') line.pop_back();
      if (!line.empty()) t.rows.push_back(line);
    }
    t.height = (static_cast<int>(t.rows.size()) - 1) / 2;
    t.width = t.rows.empty() ? 0 : (static_cast<int>(t.rows[0].size()) - 1) / 4;
    return t;
  }
  bool inside(int x, int y) const { return x >= 0 && y >= 0 && x < width && y < height; }
  bool wall(int x, int y, int d) const {
    if (!inside(x, y)) return true;
    const int r = height - 1 - y;
    switch (d) {
      case raton::NORTH: return rows[2 * r][4 * x + 2] == '-';
      case raton::SOUTH: return rows[2 * r + 2][4 * x + 2] == '-';
      case raton::WEST: return rows[2 * r + 1][4 * x] == '|';
      default: return rows[2 * r + 1][4 * x + 4] == '|';
    }
  }
  // Pasos minimos entre dos celdas por el laberinto real.
  int shortest(Cell from, Cell to) const {
    std::vector<int> dist(width * height, -1);
    std::queue<Cell> q;
    dist[from.y * width + from.x] = 0;
    q.push(from);
    while (!q.empty()) {
      const Cell c = q.front();
      q.pop();
      for (int d = 0; d < 4; ++d) {
        const Cell n = raton::step(c, static_cast<Dir>(d));
        if (wall(c.x, c.y, d) || !inside(n.x, n.y) || dist[n.y * width + n.x] >= 0) continue;
        dist[n.y * width + n.x] = dist[c.y * width + c.x] + 1;
        q.push(n);
      }
    }
    return dist[to.y * width + to.x];
  }
};

// ---------------------------------------------------------------- robot simulado (puertos)

class SimRobot : public raton::CellDriver {
 public:
  // offset: celda del mapa del raton que corresponde a la (0,0) del laberinto real.
  SimRobot(const Truth& truth, Cell offset, Cell start, Dir heading)
      : truth_(truth), offset_(offset), pos_(start), heading_(heading),
        geom_(raton::GeometryParams(), 140.0f, 152.0f, 31.0f, 50) {}

  bool aborted() override { return steps_ > 5000; }
  void setState(const char*) override {}
  raton::TrackGeometry& geometry() override { return geom_; }
  void turnInCell(float degrees) override {
    const int quarters = static_cast<int>(std::lround(degrees / 90.0f));  // + = izquierda
    heading_ = static_cast<Dir>(((heading_ - quarters) % 4 + 4) % 4);
    ++turns_;
    ++steps_;
  }
  int runCells(int cells, float, bool* unexpected, raton::CellObserver* observer) override {
    if (unexpected) *unexpected = false;
    sides_.clear();
    int moved = 0;
    for (int i = 0; i < cells; ++i) {
      if (wallAt(heading_)) {
        if (unexpected) *unexpected = true;
        break;
      }
      pos_ = raton::step(pos_, heading_);
      ++moved;
      ++cellsDriven_;
      ++steps_;
      const bool left = wallAt(raton::leftOf(heading_)), right = wallAt(raton::rightOf(heading_));
      sides_.push_back({left, right});
      const int front = wallAt(heading_) ? 100 : -1;
      if (observer && !observer->onCellSides(i, left, right, front)) break;
    }
    return moved;
  }
  bool cellSides(int i, bool& left, bool& right) const override {
    if (i < 0 || i >= static_cast<int>(sides_.size())) return false;
    left = sides_[i].first;
    right = sides_[i].second;
    return true;
  }
  bool frontWallStopped() override { return wallAt(heading_); }
  int stableFrontMm() override { return wallAt(heading_) ? geom_.frontTurnMm() : -1; }
  bool calibrateStill(int& left, int& right, int& front) override {
    left = wallAt(raton::leftOf(heading_)) ? 30 : -1;
    right = wallAt(raton::rightOf(heading_)) ? 30 : -1;
    front = stableFrontMm();
    return false;
  }
  void alignToFront() override {}

  Cell pos() const { return pos_; }
  int cellsDriven() const { return cellsDriven_; }
  void resetCount() { cellsDriven_ = 0; }

 private:
  bool wallAt(Dir d) const { return truth_.wall(pos_.x - offset_.x, pos_.y - offset_.y, d); }

  const Truth& truth_;
  Cell offset_;
  Cell pos_;
  Dir heading_;
  raton::TrackGeometry geom_;
  std::vector<std::pair<bool, bool>> sides_;
  int turns_ = 0;
  int steps_ = 0;
  int cellsDriven_ = 0;
};

class SilentLog : public raton::MissionLog {
 public:
  void decide(const char*) override { ++decisions; }
  void event(const char* line) override {
    if (strncmp(line, "CELL ", 5) == 0) ++cells;
  }
  int decisions = 0;
  int cells = 0;
};

std::string mazePath(const char* name) { return std::string(RATON_MAZES_DIR) + "/" + name; }

// ---------------------------------------------------------------- pruebas

void testFrame() {
  CHECK(raton::crc8("123456789", 9) == 0xF4);  // valor de referencia de CRC-8 (poly 0x07)
  char line[64], payload[32];
  const size_t n = raton::frameLine(7, "m3", line, sizeof(line));
  CHECK(n == strlen(line));
  uint16_t seq = 0;
  CHECK(raton::parseFrame(line, seq, payload, sizeof(payload)) == raton::FrameResult::Ok);
  CHECK(seq == 7 && std::string(payload) == "m3");
  line[3] = '4';  // "$7,m4*.." con el CRC de m3
  CHECK(raton::parseFrame(line, seq, payload, sizeof(payload)) == raton::FrameResult::BadCrc);
  CHECK(raton::parseFrame("m3", seq, payload, sizeof(payload)) == raton::FrameResult::Plain);
  CHECK(raton::parseFrame("$7,m3", seq, payload, sizeof(payload)) == raton::FrameResult::Malformed);
  CHECK(raton::parseFrame("$x,m3*00", seq, payload, sizeof(payload)) != raton::FrameResult::Ok);
  char small[6];
  CHECK(raton::frameLine(1, "una linea larga", small, sizeof(small)) == 0);
  // Misma trama que la web (frame(65535,'x')).
  raton::frameLine(65535, "x", line, sizeof(line));
  CHECK(raton::parseFrame(line, seq, payload, sizeof(payload)) == raton::FrameResult::Ok && seq == 65535);
}

void testLineQueue() {
  char buf[16];
  raton::LineQueue q(buf, sizeof(buf));
  char out[16];
  CHECK(q.pop(out, sizeof(out)) == -1);
  CHECK(q.push("abc", 3) == 0);
  CHECK(q.push("defg", 4) == 0);
  CHECK(q.pop(out, sizeof(out)) == 3 && std::string(out) == "abc");
  // Sin sitio: tira la linea mas vieja.
  CHECK(q.push("123456789A", 10) == 1);  // "defg" + esta no caben en 16
  CHECK(q.pop(out, sizeof(out)) == 10 && std::string(out) == "123456789A");
  CHECK(q.empty());
  CHECK(q.push("0123456789abcdef", 16) == -1);  // no cabe nunca
}

void testOrquestadores() {
  using namespace raton;
  OrquestadorMision m;
  EstadoMision e{};
  CHECK(m.decidir(e) == AccionMision::LeerSalida);
  e.salidaLeida = true;
  e.hayFrontera = true;
  CHECK(m.decidir(e) == AccionMision::Explorar);
  e.fallosSeguidos = 3;
  CHECK(m.decidir(e) == AccionMision::Recuperar);
  e.fallosSeguidos = 0;
  e.hayFrontera = false;
  CHECK(m.decidir(e) == AccionMision::Terminar);
  e.hayMeta = true;
  e.mapaAprendido = true;
  CHECK(m.decidir(e) == AccionMision::RutaCorta);
  e.abortado = true;
  CHECK(m.decidir(e) == AccionMision::Terminar);

  OrquestadorMovimiento mv;
  mv.empezarTramo(0);
  EntradaMovimiento in{};
  in.speed = 100;
  in.remaining = 300;
  in.front = -1;
  in.stopMm = 80;
  in.wallNearMm = 200;
  in.cellMm = 155;
  in.sideL = in.sideR = 30;
  for (uint32_t t = 0; t < 50; t += 10) {
    in.ms = t;
    CHECK(mv.decidir(in).seguridad == Seguridad::Ok);
  }
  // Pared delante: frena por el laser y avisa si es inesperada.
  in.front = 120;
  in.remaining = 600;
  DecisionMovimiento d = mv.decidir(in);
  CHECK(d.paredInesperada && d.remaining == 40);
  // Empujando: avanza 50 mm y el frontal no se acerca -> pegado.
  in.pos = 0;
  mv.decidir(in);
  in.pos = 50;
  CHECK(mv.decidir(in).seguridad == Seguridad::Pegado);
  // Roce sostenido > rubMs -> roce; al tercero, pegado.
  OrquestadorMovimiento r;
  r.empezarTramo(0);
  EntradaMovimiento ri = in;
  ri.front = -1;
  ri.sideL = 3;
  int roces = 0, pegado = 0;
  for (uint32_t t = 1; t < 2000; t += 20) {
    ri.ms = t;
    const Seguridad sg = r.decidir(ri).seguridad;
    roces += sg == Seguridad::Roce;
    if (sg == Seguridad::Pegado) {
      pegado = 1;
      break;
    }
  }
  CHECK(roces == 2 && pegado == 1);
}

void testMotionProgress() {
  // Tramo de 8 celdas interrumpido tras 2: el residuo no puede ser -6 celdas.
  const auto partial = raton::motionProgress(320, 0, 155, 8, false);
  CHECK(partial.cells == 2 && partial.offset == 10);
  const auto reverse = raton::motionProgress(300, 10, 155, 8, true);
  CHECK(reverse.cells == 2 && reverse.offset == 20);
  const auto stopped = raton::motionProgress(0, 30, 155, 1, true);
  CHECK(stopped.cells == 0 && stopped.offset == 30);
  const auto overshoot = raton::motionProgress(330, 0, 155, 1, false);
  CHECK(overshoot.cells == 1 && overshoot.offset == 175);
}

void testTrackGeometry() {
  raton::TrackGeometry g(raton::GeometryParams(), 140.0f, 155.0f, 31.0f, 50);
  CHECK(g.frontTurnMm() == 80);    // 70 - 40 + 50
  CHECK(g.cellFrontWall() == 140);
  CHECK(g.cellSideWall() == 58);   // 0.42 x 140
  CHECK(g.sideFromRay(-1) == -1);
  CHECK(g.sideFromRay(60) == 21);  // 60 x sen(31) + 30 - 40
  CHECK(g.sideFromRay(5) == 0);
  g.setCorridor(192);
  CHECK(g.corridorMm() == 192 && g.cellMm() == 204);
  g.setCorridor(1000);
  CHECK(g.corridorMm() == 400);
  g.blendCorridor(400);
  CHECK(g.cellMm() == 412);  // el ajuste en marcha no toca la celda
  g.setCorridor(100);
  g.setDeadzone(0);
  CHECK(g.frontTurnMm() == 20);  // 50 - 40 + 0 queda por debajo de punto muerto + margen  // nunca menos que punto muerto + margen
}

void testCellMap() {
  raton::CellMap m;
  m.clear(nullptr, 4, 3);
  CHECK(m.wall(makeCell(0, 0), raton::SOUTH) && m.wall(makeCell(3, 2), raton::EAST));  // borde
  CHECK(!m.wall(makeCell(1, 1), raton::NORTH));
  m.setWall(makeCell(1, 1), raton::NORTH, true);
  CHECK(m.wall(makeCell(1, 2), raton::SOUTH));  // la otra cara de la pared
  m.flood(makeCell(0, 0));
  CHECK(m.dist(makeCell(0, 0)) == 0 && m.dist(makeCell(3, 2)) == 5);
  // Pasillo sin salida (1,2)->(2,2)->(3,2) cerrado por debajo: se poda entero.
  for (int x = 1; x < 4; ++x) m.setWall(makeCell(x, 2), raton::SOUTH, true);
  for (int x = 0; x < 4; ++x)
    for (int y = 0; y < 3; ++y) m.markSeen(makeCell(x, y));
  m.setWall(makeCell(0, 2), raton::EAST, false);
  m.pruneDeadEnds();
  CHECK(m.dead(makeCell(3, 2)) && m.dead(makeCell(2, 2)) && m.dead(makeCell(1, 2)));
  CHECK(!m.dead(makeCell(0, 0)));
  Cell out{};
  CHECK(!m.nearestUnseen(makeCell(0, 0), out));  // todo visto

  // La salida podada no implica que la ruta optimista este demostrada.
  m.clear(nullptr, 3, 1);
  m.markSeen(makeCell(0, 0));
  m.setKeep(1, makeCell(2, 0));
  m.pruneDeadEnds();
  CHECK(m.dead(makeCell(0, 0)));
  CHECK(m.unseenOnRoute(makeCell(0, 0), makeCell(2, 0), makeCell(2, 0), out));
  CHECK(out == makeCell(2, 0) || out == makeCell(1, 0));
}

// Explora hasta la meta, busca la ruta demostrada (Adachi) y vuelve; la carrera rapida tiene que ir por
// el camino mas corto real.
void checkSolve(const Truth& truth, Cell goal, Cell start = makeCell(0, 0)) {
  CHECK(truth.width > 0);
  SimRobot robot(truth, makeCell(0, 0), start, raton::NORTH);
  SilentLog log;
  raton::CellMap map;
  map.clear(nullptr, truth.width, truth.height);
  raton::CellExplorer explorer(robot, log, map, raton::ExplorerParams());
  Cell pos = start;
  Dir heading = raton::NORTH;
  CHECK(explorer.exploreTo(pos, heading, goal));
  CHECK(pos == goal && robot.pos() == goal);
  std::cout << "solve " << truth.width << "x" << truth.height << ": " << robot.cellsDriven() << " celdas, " << log.decisions << " decisiones\n";
  // La prueba exige demostrar la ruta; el limite operativo de 12 destinos puede
  // agotarse antes en una pista grande sin que eso sea un error de navegacion.
  raton::ExplorerParams searchParams;
  searchParams.searchMaxTargets = truth.width * truth.height;
  raton::CellExplorer search(robot, log, map, searchParams);
  search.adachiSearch(pos, heading, start, goal);
  CHECK(pos == start && robot.pos() == start);
  Cell unseen{};
  CHECK(!map.unseenOnRoute(start, goal, pos, unseen));

  // Carrera rapida desde la salida mirando al norte.
  SimRobot racer(truth, makeCell(0, 0), start, raton::NORTH);
  raton::CellExplorer race(racer, log, map, raton::ExplorerParams());
  pos = start;
  heading = raton::NORTH;
  map.setAvoidDead(true);  // la carrera debe admitir extremos podados por una busqueda previa
  CHECK(race.runRoute(pos, heading, goal, 250));
  CHECK(pos == goal);
  CHECK(racer.cellsDriven() == truth.shortest(start, goal));
  std::cout << "race " << truth.width << "x" << truth.height << ": " << racer.cellsDriven() << " celdas\n";
}

// Pista desconocida: mapea todo desde el centro de la rejilla grande y vuelve a la salida.
void checkMapTrack(const Truth& truth, Cell start = makeCell(0, 0)) {
  const int c = raton::CellMap::kMax / 2;
  const Cell offset = makeCell(c, c);  // la (0,0) real queda en el centro de la rejilla
  const Cell origin = makeCell(c + start.x, c + start.y);
  SimRobot robot(truth, offset, origin, raton::NORTH);
  SilentLog log;
  raton::CellMap map;
  map.clear(nullptr, raton::CellMap::kMax, raton::CellMap::kMax);
  raton::CellExplorer explorer(robot, log, map, raton::ExplorerParams());
  Cell pos = origin;
  Dir heading = raton::NORTH;
  CHECK(explorer.mapTrack(pos, heading));
  CHECK(pos == origin);
  std::cout << "mapTrack " << truth.width << "x" << truth.height << ": " << robot.cellsDriven() << " celdas, " << map.seenCount() << " vistas\n";
  CHECK(map.seenCount() == truth.width * truth.height);
  int wrong = 0;
  for (int x = 0; x < truth.width; ++x)
    for (int y = 0; y < truth.height; ++y)
      for (int d = 0; d < 4; ++d)
        if (map.wall(makeCell(x + c, y + c), d) != truth.wall(x, y, d)) ++wrong;
  CHECK(wrong == 0);
  CHECK(robot.cellsDriven() <= 4 * truth.width * truth.height);
}

}  // namespace

int main() {
  testFrame();
  testLineQueue();
  testOrquestadores();
  testMotionProgress();
  testTrackGeometry();
  testCellMap();
  checkSolve(Truth::load(mazePath("impar_05.txt")), makeCell(2, 2));
  checkSolve(Truth::load(mazePath("clasico_01.txt")), makeCell(7, 7));
  checkMapTrack(Truth::load(mazePath("impar_05.txt")));
  checkMapTrack(Truth::load(mazePath("clasico_01.txt")));
  Truth competitionTruth;
  competitionTruth.width = competition::WIDTH;
  competitionTruth.height = competition::HEIGHT;
  for (const char* row : competition::MAP) competitionTruth.rows.emplace_back(row);
  const Cell competitionStart = makeCell(competition::START_X, competition::START_Y);
  checkSolve(competitionTruth, makeCell(competition::GOAL_X, competition::GOAL_Y), competitionStart);
  checkMapTrack(competitionTruth, competitionStart);
  if (failures) {
    std::cerr << failures << " fallos\n";
    return 1;
  }
  std::cout << "Nucleo: todas las pruebas pasan\n";
  return 0;
}

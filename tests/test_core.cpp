// Pruebas de la logica comun (sin dependencias: se corren con ctest).
#include <filesystem>
#include <iostream>
#include <sstream>
#include <string>

#include "maze_io.h"
#include "motion_progress.h"
#include "motion_safety.h"
#include "risk_predictor.h"
#include <limits>
#include "sim_robot.h"
#include "solver.h"

namespace {

int failures = 0;

#define CHECK(cond)                                                              \
  do {                                                                           \
    if (!(cond)) {                                                               \
      std::cerr << __FILE__ << ":" << __LINE__ << ": fallo: " #cond << '\n';     \
      failures++;                                                                \
    }                                                                            \
  } while (0)

raton::Cell C(int x, int y) { return raton::Cell{static_cast<int8_t>(x), static_cast<int8_t>(y)}; }

uint16_t optimal(const raton::Maze& truth) {
  raton::Cell goals[4];
  const int n = truth.goalCells(goals);
  raton::FloodFill f;
  f.compute(truth, goals, n, true);
  return f.distance(C(0, 0));
}

void testMotionSafety() {
  raton::BodyGeometry body;
  raton::SensorObservation missing[3];
  const auto unknown=raton::assessTurn(missing,body,90);
  CHECK(unknown.feasible && !unknown.observed); // absence of evidence is explicit
  const raton::ObservedSegment inside[]={{{0,-5},{0,5}}};
  CHECK(!raton::sweptTurnClear(inside,1,body,90,70,75,0,3));
  const raton::ObservedSegment tail[]={{{-18,-60},{-18,60}}};
  CHECK(!raton::sweptTurnClear(tail,1,body,90,70,75,0,3));
  const raton::ObservedSegment far[]={{{500,-300},{500,300}}};
  CHECK(raton::sweptTurnClear(far,1,body,-90,70,75,0,3));
  CHECK(!raton::sweptTurnClear(far,1,body,90,0,75,0,3));
  CHECK(!raton::sweptTurnClear(far,1,body,90,70,std::numeric_limits<float>::quiet_NaN(),0,3));
  raton::RiskModel zero={};float x[43]={};
  CHECK(raton::predictRisk(zero,x)==0.5f);
  x[0]=std::numeric_limits<float>::quiet_NaN();
  CHECK(std::isnan(raton::predictRisk(zero,x)));
}

void testMotionProgress() {
  // Tramo de 8 celdas interrumpido tras 2: el residuo no puede ser -6 celdas.
  const auto partial = raton::motionProgress(320, 0, 155, 8, false);
  CHECK(partial.cells == 2 && partial.offset == 10);
  const auto reverse = raton::motionProgress(300, 10, 155, 8, true);
  CHECK(reverse.cells == 2 && reverse.offset == 20);
  const auto stopped = raton::motionProgress(0, 30, 155, 1, true);
  CHECK(stopped.cells == 0 && stopped.offset == 30);
  const auto turned = raton::motionProgress(52, 103, 155, 1, false);
  CHECK(turned.cells == 1 && turned.offset == 0);
  const auto overshoot = raton::motionProgress(330, 0, 155, 1, false);
  CHECK(overshoot.cells == 1 && overshoot.offset == 175);
}

void testMazeBasics() {
  raton::Maze m(16, 16);
  CHECK(m.isKnown(C(0, 0), raton::SOUTH) && m.hasWall(C(0, 0), raton::SOUTH));
  CHECK(m.isKnown(C(15, 3), raton::EAST) && m.hasWall(C(15, 3), raton::EAST));
  CHECK(!m.isKnown(C(3, 3), raton::NORTH));

  m.setWall(C(3, 3), raton::NORTH, true);
  CHECK(m.isKnown(C(3, 4), raton::SOUTH) && m.hasWall(C(3, 4), raton::SOUTH));

  raton::Cell goals[4];
  CHECK(m.goalCells(goals) == 4);
  CHECK(m.isGoal(C(7, 7)) && m.isGoal(C(8, 8)) && !m.isGoal(C(6, 7)));

  raton::Maze odd(5, 5);
  CHECK(odd.goalCells(goals) == 1 && goals[0] == C(2, 2));
}

void testFloodFillOpenMaze() {
  raton::Maze m(16, 16);
  raton::Cell goals[4];
  const int n = m.goalCells(goals);
  raton::FloodFill f;
  f.compute(m, goals, n, false);
  CHECK(f.distance(C(0, 0)) == 14);
  CHECK(f.distance(C(7, 7)) == 0);
  f.compute(m, goals, n, true);  // todo desconocido = todo cerrado
  CHECK(f.distance(C(0, 0)) == raton::INF_DIST);
}

void testParseAndWrite() {
  const char* text =
      "o---o---o---o\n"
      "|           |\n"
      "o   o---o   o\n"
      "|   |       |\n"
      "o   o   o---o\n"
      "|   |       |\n"
      "o---o---o---o\n";
  std::istringstream in(text);
  raton::Maze m;
  std::string err;
  CHECK(sim::parseMaze(in, m, err));
  CHECK(m.width() == 3 && m.height() == 3);
  CHECK(m.hasWall(C(0, 0), raton::EAST));
  CHECK(!m.hasWall(C(0, 0), raton::NORTH));
  CHECK(m.hasWall(C(1, 1), raton::NORTH));
  CHECK(m.hasWall(C(2, 0), raton::NORTH));
  CHECK(!m.hasWall(C(0, 2), raton::EAST));

  std::ostringstream out;
  sim::writeMaze(out, m);
  CHECK(out.str() == text);
}

void testGeneratorRoundTrip() {
  const raton::Maze a = sim::generateMaze(16, 16, 123, 10);
  std::ostringstream out;
  sim::writeMaze(out, a);
  std::istringstream in(out.str());
  raton::Maze b;
  std::string err;
  CHECK(sim::parseMaze(in, b, err));
  std::ostringstream again;
  sim::writeMaze(again, b);
  CHECK(again.str() == out.str());
  CHECK(optimal(a) != raton::INF_DIST);
  // Reglas clasicas: la salida solo abre al norte
  CHECK(a.hasWall(C(0, 0), raton::EAST) && !a.hasWall(C(0, 0), raton::NORTH));
}

bool solve(const raton::Maze& truth, const std::string& name) {
  sim::SimRobot robot(truth);
  raton::Solver solver(robot, truth.width(), truth.height());
  try {
    if (!solver.explore(30) || !solver.speedRun()) {
      std::cerr << name << ": no resuelto\n";
      return false;
    }
  } catch (const sim::Crash& e) {
    std::cerr << name << ": " << e.what() << '\n';
    return false;
  }
  const raton::SolverStats& s = solver.stats();
  if (s.pathProven && s.speedRunCells != optimal(truth)) {
    std::cerr << name << ": camino probado de " << s.speedRunCells << " celdas pero el optimo es " << optimal(truth)
              << '\n';
    return false;
  }
  if (s.speedRunCells < optimal(truth)) {
    std::cerr << name << ": carrera mas corta que el optimo (imposible)\n";
    return false;
  }
  return true;
}

void testRandomMazes() {
  for (uint32_t seed = 1; seed <= 300; ++seed) {
    const int size = 4 + static_cast<int>(seed % 13);
    CHECK(solve(sim::generateMaze(size, size, seed, static_cast<int>(seed % 20)), "semilla " + std::to_string(seed)));
  }
}

void testMazeFiles() {
  const std::filesystem::path dir(RATON_MAZES_DIR);
  CHECK(std::filesystem::is_directory(dir));
  if (!std::filesystem::is_directory(dir)) return;
  CHECK(std::filesystem::is_regular_file(dir / "clasico_01.txt"));
  CHECK(std::filesystem::is_regular_file(dir / "impar_05.txt"));
  int count = 0;
  for (const auto& entry : std::filesystem::directory_iterator(dir)) {
    if (entry.path().extension() != ".txt") continue;
    raton::Maze m;
    std::string err;
    const bool loaded = sim::loadMaze(entry.path().string(), m, err);
    if (!loaded) std::cerr << entry.path() << ": " << err << '\n';
    CHECK(loaded);
    if (loaded) CHECK(solve(m, entry.path().filename().string()));
    count++;
  }
  CHECK(count > 0);
}

}  // namespace

int main() {
  testMotionSafety();
  testMotionProgress();
  testMazeBasics();
  testFloodFillOpenMaze();
  testParseAndWrite();
  testGeneratorRoundTrip();
  testRandomMazes();
  testMazeFiles();
  if (failures == 0) {
    std::cout << "OK: todas las pruebas pasaron\n";
    return 0;
  }
  std::cout << failures << " pruebas fallaron\n";
  return 1;
}

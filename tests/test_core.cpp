// Pruebas de la logica comun (sin dependencias: se corren con ctest).
#include <filesystem>
#include <iostream>
#include <sstream>
#include <string>

#include "maze_io.h"
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
  int count = 0;
  for (const auto& entry : std::filesystem::directory_iterator(RATON_MAZES_DIR)) {
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

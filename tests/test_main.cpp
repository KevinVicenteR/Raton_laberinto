// Pruebas del nucleo portable y del raton completo en el simulador. Una sola entrada para CTest.
#include <math.h>
#include <stdio.h>
#include <string.h>

#include <string>

#include "raton/frame.h"
#include "raton/lateral.h"
#include "raton/line_queue.h"
#include "raton/maze.h"
#include "raton/maze_text.h"
#include "raton/param_table.h"
#include "raton/params.h"
#include "raton/planner.h"
#include "run_case.h"
#include "sim.h"

namespace {

int failures = 0;
int checks = 0;

#define CHECK(cond)                                                     \
  do {                                                                  \
    ++checks;                                                           \
    if (!(cond)) {                                                      \
      ++failures;                                                       \
      printf("FALLO %s:%d: %s\n", __FILE__, __LINE__, #cond);           \
    }                                                                   \
  } while (0)

using namespace raton;

void testMazeVotes() {
  Maze m;
  m.reset(5, 4, true);
  CHECK(m.wall(0, 0, SOUTH) == Wall::Closed);  // borde
  CHECK(m.wall(0, 0, WEST) == Wall::Closed);
  CHECK(m.wall(4, 3, NORTH) == Wall::Closed);
  CHECK(m.wall(2, 2, NORTH) == Wall::Unknown);
  // Una lectura suelta no decide; dos si.
  m.observe(2, 2, NORTH, true);
  CHECK(m.wall(2, 2, NORTH) == Wall::Unknown);
  m.observe(2, 2, NORTH, true);
  CHECK(m.wall(2, 2, NORTH) == Wall::Closed);
  CHECK(m.wall(2, 3, SOUTH) == Wall::Closed);  // la misma pared vista desde el otro lado
  // Una lectura contraria suelta no cambia una pared confirmada.
  m.observe(2, 2, NORTH, true);
  m.observe(2, 2, NORTH, true);
  m.observe(2, 3, SOUTH, false);
  CHECK(m.wall(2, 2, NORTH) == Wall::Closed);
  // El borde nunca cambia.
  m.observe(0, 0, WEST, false, 4);
  CHECK(m.wall(0, 0, WEST) == Wall::Closed);
  // Olvidar lo poco confirmado.
  m.observe(1, 1, EAST, false, 2);
  m.forgetWeak(3);
  CHECK(m.wall(1, 1, EAST) == Wall::Unknown);
  CHECK(m.wall(2, 2, NORTH) == Wall::Closed);
  CHECK(m.wallMask(0, 0) == (4 | 8));
}

void testFloodAndRoute() {
  const char* rows[] = {
      "+---+---+---+",
      "|           |",
      "+   +---+   +",
      "|   |       |",
      "+   +   +---+",
      "|       |   |",
      "+---+---+---+",
  };
  Maze m;
  CHECK(parseMazeText(rows, 7, m));
  CHECK(m.width() == 3 && m.height() == 3);
  m.setGoal(2, 0, 1, 1);  // abajo a la derecha, encerrada salvo por arriba? (2,0) tiene pared N
  CHECK(m.wall(2, 0, NORTH) == Wall::Closed);
  CHECK(m.wall(0, 0, NORTH) == Wall::Open);
  m.setGoal(1, 1, 1, 1);
  CellSet goals;
  goalSet(m, goals);
  CHECK(goals.count == 1);
  uint16_t dist[Maze::kMax * Maze::kMax];
  flood(m, goals, false, dist);
  CHECK(dist[m.index(1, 1)] == 0);
  CHECK(dist[m.index(1, 0)] == 1);
  CHECK(dist[m.index(0, 0)] == 2);
  CHECK(dist[m.index(2, 0)] == kInf);  // encerrada
  // Desde (0,0) mirando al norte, el paso es al este (hacia (1,0)).
  CHECK(nextStep(m, 0, 0, NORTH, dist, false) == EAST);
  static Route r;
  bestRoute(m, 0, 2, EAST, goals, false, 8, r);
  CHECK(r.ok);
  // (0,2) -> (1,2) -> (2,2) -> (2,1) -> (1,1): 4 pasos.
  CHECK(r.length == 4);
  CHECK(r.steps[0] == EAST && r.steps[3] == WEST);
}

void testOptimisticFlood() {
  Maze m;
  m.reset(4, 4, true);
  m.setGoal(3, 3, 1, 1);
  CellSet goals;
  goalSet(m, goals);
  uint16_t dist[Maze::kMax * Maze::kMax];
  flood(m, goals, true, dist);
  CHECK(dist[m.index(0, 0)] == 6);  // todo desconocido = abierto
  flood(m, goals, false, dist);
  CHECK(dist[m.index(0, 0)] == kInf);  // con solo lo conocido no hay camino
}

void testTextRoundTrip() {
  Maze m;
  std::vector<std::string> rows;
  CHECK(sim::loadMazeFile(std::string(RATON_MAZES_DIR) + "/reglamento_2026.txt", m, &rows));
  CHECK(m.width() == 19 && m.height() == 10);
  static char buf[8192];
  printMaze(m, buf, sizeof(buf));
  // El texto impreso vuelve a dar el mismo laberinto.
  std::vector<std::string> lines;
  std::string cur;
  for (const char* p = buf; *p; ++p) {
    if (*p == '\n') {
      for (char& c : cur)
        if (c != '+' && c != '-' && c != '|') c = ' ';
      lines.push_back(cur);
      cur.clear();
    } else {
      cur += *p;
    }
  }
  std::vector<const char*> ptrs;
  for (auto& l : lines) ptrs.push_back(l.c_str());
  Maze back;
  CHECK(parseMazeText(ptrs.data(), static_cast<int>(ptrs.size()), back));
  int diff = 0;
  for (int y = 0; y < m.height(); ++y)
    for (int x = 0; x < m.width(); ++x)
      for (int d = 0; d < 4; ++d)
        if (m.wall(x, y, static_cast<Dir>(d)) != back.wall(x, y, static_cast<Dir>(d))) ++diff;
  CHECK(diff == 0);
}

void testParamTable() {
  Params p;
  CHECK(setParam(p, "kpRot", 0.03f));
  CHECK(fabsf(p.kpRot - 0.03f) < 1e-6f);
  CHECK(!setParam(p, "kpRot", 5.0f));  // fuera de rango
  CHECK(!setParam(p, "noExiste", 1.0f));
  CHECK(setParam(p, "goalX", 7.0f) && p.goalX == 7);
  const int i = findParam("cellMm");
  CHECK(i >= 0 && fabsf(getParam(p, i) - 155.0f) < 1e-3f);
  // Todos los valores por defecto estan dentro de su rango.
  for (int k = 0; k < paramCount(); ++k) {
    const float v = getParam(p, k);
    CHECK(v >= paramInfo(k).min && v <= paramInfo(k).max);
  }
}

void testLateralFilter() {
  // Raton 10 mm a la izquierda y torcido 2 grados que el giroscopio no ve: el filtro encuentra los dos.
  LateralFilter f;
  f.reset(0, 100.0f, 1e-3f);
  float yTrue = 10.0f;
  const float psiTrue = 2.0f / 57.3f;
  for (int i = 0; i < 400; ++i) {
    f.predict(1.0f, 0.0f);  // 1 mm por paso, el giroscopio dice recto
    yTrue += psiTrue;
    if (i % 6 == 0) f.update(yTrue, 16.0f);
  }
  CHECK(fabsf(f.y() - yTrue) < 3.0f);
  CHECK(fabsf(f.bias() - psiTrue) < 0.6f / 57.3f);
}

void testFrames() {
  char out[64];
  const size_t n = frameLine(12, "s1", out, sizeof(out));
  CHECK(n > 0);
  uint16_t seq = 0;
  char payload[32];
  CHECK(parseFrame(out, seq, payload, sizeof(payload)) == FrameResult::Ok);
  CHECK(seq == 12 && strcmp(payload, "s1") == 0);
  out[2] = '9';  // corromper
  CHECK(parseFrame(out, seq, payload, sizeof(payload)) == FrameResult::BadCrc);
  char buf[32];
  LineQueue q(buf, sizeof(buf));
  CHECK(q.push("hola", 4) == 0);
  char line[16];
  CHECK(q.pop(line, sizeof(line)) == 4 && strcmp(line, "hola") == 0);
}

// El raton completo en el simulador: laberinto del reglamento y aleatorios, con fisica ideal y con
// errores (pista de otra medida, ruedas distintas, sensores y giroscopio descalibrados).
void testMissions() {
  Params p;
  Maze truth;
  CHECK(sim::loadMazeFile(std::string(RATON_MAZES_DIR) + "/reglamento_2026.txt", truth));
  sim::CaseResult r = sim::runCase(truth, p, sim::perturbed(p, 0, 1), 1, false);
  printf("  reglamento ideal:  %s\n", sim::describe(r).c_str());
  CHECK(r.clean());
  CHECK(r.proven && r.backHome && r.wrongWalls == 0);
  CHECK(r.goalTime2 < r.goalTime1 * 0.5f);  // la carrera es mucho mas rapida que la exploracion
  r = sim::runCase(truth, p, sim::perturbed(p, 2, 76), 4, false);
  printf("  reglamento errores: %s\n", sim::describe(r).c_str());
  CHECK(r.ok());
  for (int k = 0; k < 4; ++k) {
    Params pc = p;
    sim::randomMaze(pc.mazeW, pc.mazeH, 12, 2000u + k, truth);
    pc.goalX = 2 + k;
    pc.goalY = 5 + (k % 3);
    r = sim::runCase(truth, pc, sim::perturbed(pc, (k % 2) * 2, 300u + k), 9u + k, false);
    printf("  aleatorio %d:       %s\n", k, sim::describe(r).c_str());
    CHECK(r.ok());
    CHECK(r.maxArea < 20.0f);
  }
}

}  // namespace

int main() {
  testMazeVotes();
  testFloodAndRoute();
  testOptimisticFlood();
  testTextRoundTrip();
  testParamTable();
  testLateralFilter();
  testFrames();
  testMissions();
  printf("%d comprobaciones, %d fallos\n", checks, failures);
  return failures ? 1 : 0;
}

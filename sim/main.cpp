// Simulador de consola: corre el mismo Solver que el robot sobre laberintos de archivo o aleatorios.
//
//   raton_sim --maze mazes/clasico_01.txt --animate 80
//   raton_sim --random 42 --size 16
//   raton_sim --batch 500                # estadisticas sobre 500 laberintos aleatorios
//   raton_sim --generate 7 > mazes/x.txt # guarda un laberinto aleatorio

#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>

#include "maze_io.h"
#include "sim_robot.h"
#include "solver.h"

namespace {

struct Options {
  std::string mazeFile;
  uint32_t seed = 1;
  bool random = false;
  int size = 16;
  int loops = 10;
  int animateMs = -1;
  int maxRuns = 8;
  int batch = 0;
  bool generate = false;
};

void usage() {
  std::cerr << "Uso: raton_sim [--maze ARCHIVO | --random SEMILLA] [--size N] [--loops K]\n"
               "                [--animate MS] [--runs N] [--batch N] [--generate SEMILLA]\n"
               "  --maze      laberinto en formato ASCII (o---o / |)\n"
               "  --random    laberinto aleatorio con esa semilla\n"
               "  --size      lado del laberinto aleatorio (por defecto 16)\n"
               "  --loops     paredes extra quitadas para crear varios caminos (por defecto 10)\n"
               "  --animate   dibuja cada paso esperando MS milisegundos\n"
               "  --runs      maximo de viajes de exploracion (por defecto 8)\n"
               "  --batch     corre N laberintos aleatorios y muestra estadisticas\n"
               "  --generate  imprime un laberinto aleatorio y sale\n";
}

bool parseArgs(int argc, char** argv, Options& o) {
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&]() -> const char* { return i + 1 < argc ? argv[++i] : nullptr; };
    const char* v = nullptr;
    if (a == "--maze" && (v = next())) {
      o.mazeFile = v;
    } else if (a == "--random" && (v = next())) {
      o.random = true;
      o.seed = static_cast<uint32_t>(std::strtoul(v, nullptr, 10));
    } else if (a == "--size" && (v = next())) {
      o.size = std::atoi(v);
    } else if (a == "--loops" && (v = next())) {
      o.loops = std::atoi(v);
    } else if (a == "--animate" && (v = next())) {
      o.animateMs = std::atoi(v);
    } else if (a == "--runs" && (v = next())) {
      o.maxRuns = std::atoi(v);
    } else if (a == "--batch" && (v = next())) {
      o.batch = std::atoi(v);
    } else if (a == "--generate" && (v = next())) {
      o.generate = true;
      o.seed = static_cast<uint32_t>(std::strtoul(v, nullptr, 10));
    } else {
      return false;
    }
  }
  return o.size >= 2 && o.size <= raton::MAX_MAZE_SIZE;
}

uint16_t optimalLength(const raton::Maze& truth) {
  raton::Cell goals[4];
  const int n = truth.goalCells(goals);
  raton::FloodFill f;
  f.compute(truth, goals, n, true);
  return f.distance(raton::Cell{0, 0});
}

struct Result {
  bool ok = false;
  std::string error;
  raton::SolverStats stats;
  double searchSeconds = 0;
  double speedSeconds = 0;
  uint16_t optimal = 0;
};

Result runMaze(const raton::Maze& truth, const Options& o, bool verbose) {
  Result r;
  r.optimal = optimalLength(truth);
  sim::SimRobot robot(truth);
  if (o.animateMs >= 0) robot.animate(&std::cout, o.animateMs);
  raton::Solver solver(robot, truth.width(), truth.height());
  try {
    if (!solver.explore(o.maxRuns)) {
      r.error = "no encontro camino a la meta";
      return r;
    }
    r.searchSeconds = robot.seconds();
    robot.resetClock();
    if (!solver.speedRun()) {
      r.error = "la carrera rapida fallo";
      return r;
    }
    r.speedSeconds = robot.seconds();
    r.ok = true;
  } catch (const sim::Crash& e) {
    r.error = std::string("CHOQUE: ") + e.what();
  }
  r.stats = solver.stats();
  if (verbose) {
    std::cout << "\nMapa final (lo que el raton conoce):\n";
    sim::render(std::cout, solver.maze(), &truth, nullptr, solver.position(), solver.heading());
  }
  return r;
}

int runBatch(const Options& o) {
  int ok = 0;
  int proven = 0;
  int optimalRuns = 0;
  double searchCells = 0;
  double searchSec = 0;
  double speedSec = 0;
  for (int i = 0; i < o.batch; ++i) {
    const uint32_t seed = o.seed + static_cast<uint32_t>(i);
    const raton::Maze truth = sim::generateMaze(o.size, o.size, seed, o.loops);
    Options quiet = o;
    quiet.animateMs = -1;
    const Result r = runMaze(truth, quiet, false);
    if (!r.ok) {
      std::cout << "semilla " << seed << ": " << r.error << '\n';
      continue;
    }
    ok++;
    proven += r.stats.pathProven ? 1 : 0;
    optimalRuns += r.stats.speedRunCells == r.optimal ? 1 : 0;
    searchCells += r.stats.searchCells;
    searchSec += r.searchSeconds;
    speedSec += r.speedSeconds;
  }
  std::cout << "Laberintos:            " << o.batch << " (" << o.size << "x" << o.size << ", loops " << o.loops
            << ")\n"
            << "Resueltos:             " << ok << '\n'
            << "Camino optimo probado: " << proven << '\n'
            << "Carrera optima:        " << optimalRuns << '\n';
  if (ok > 0) {
    std::cout << "Promedio exploracion:  " << searchCells / ok << " celdas, " << searchSec / ok << " s\n"
              << "Promedio carrera:      " << speedSec / ok << " s\n";
  }
  return ok == o.batch ? 0 : 1;
}

}  // namespace

int main(int argc, char** argv) {
  Options o;
  if (!parseArgs(argc, argv, o)) {
    usage();
    return 2;
  }

  if (o.generate) {
    sim::writeMaze(std::cout, sim::generateMaze(o.size, o.size, o.seed, o.loops));
    return 0;
  }
  if (o.batch > 0) return runBatch(o);

  raton::Maze truth;
  if (!o.mazeFile.empty()) {
    std::string error;
    if (!sim::loadMaze(o.mazeFile, truth, error)) {
      std::cerr << "Error: " << error << '\n';
      return 2;
    }
  } else {
    truth = sim::generateMaze(o.size, o.size, o.seed, o.loops);
  }

  const Result r = runMaze(truth, o, true);
  if (!r.ok) {
    std::cout << "\nFALLO: " << r.error << '\n';
    return 1;
  }
  std::cout << "\nViajes de exploracion: " << r.stats.searchRuns
            << (r.stats.pathProven ? " (camino optimo probado)" : " (camino NO probado)") << '\n'
            << "Celdas exploradas:     " << r.stats.searchCells << "  (" << r.searchSeconds << " s)\n"
            << "Carrera rapida:        " << r.stats.speedRunCells << " celdas en " << r.stats.speedRunMoves
            << " tramos  (" << r.speedSeconds << " s)\n"
            << "Camino optimo real:    " << r.optimal << " celdas\n";
  return 0;
}

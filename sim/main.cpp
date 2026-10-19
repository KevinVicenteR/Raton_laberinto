// Simulador del raton de laberinto.
//
//   raton_sim --maze mazes/reglamento_2026.txt          un caso, con detalle
//   raton_sim --random 5                                laberinto aleatorio 19 x 10 (semilla 5)
//   raton_sim --bench [n]                               reglamento + n aleatorios con fisica nominal y
//                                                       con errores; codigo de salida 1 si alguno falla
// Opciones: --goal x,y  --start x,y,dir  --size WxH  --loops n  --perturb 0..3  --seed n  --trace
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <random>
#include <string>
#include <vector>

#include "raton/param_table.h"
#include "raton/params.h"
#include "run_case.h"
#include "sim.h"

#ifndef RATON_MAZES_DIR
#define RATON_MAZES_DIR "mazes"
#endif

namespace {

void randomGoal(const raton::Maze& m, const raton::Params& p, uint32_t seed, raton::Params& out) {
  std::mt19937 rng(seed * 31u + 7u);
  // Meta lejos de la salida, como en el reglamento.
  for (int i = 0; i < 100; ++i) {
    const int x = static_cast<int>(rng() % m.width()), y = static_cast<int>(rng() % m.height());
    if (abs(x - p.startX) + abs(y - p.startY) >= (m.width() + m.height()) / 2) {
      out.goalX = x;
      out.goalY = y;
      return;
    }
  }
}

}  // namespace

int main(int argc, char** argv) {
  raton::Params p;
  std::string mazePath;
  int randomSeed = -1;
  int bench = -1;
  int perturb = 0;
  int loops = 12;
  uint32_t seed = 1;
  int simSeed = -1;
  bool trace = false;
  float timeLimit = 900.0f;
  bool goalGiven = false;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&]() { return i + 1 < argc ? argv[++i] : ""; };
    if (a == "--maze") mazePath = next();
    else if (a == "--random") randomSeed = atoi(next());
    else if (a == "--bench") bench = (i + 1 < argc && argv[i + 1][0] != '-') ? atoi(next()) : 20;
    else if (a == "--perturb") perturb = atoi(next());
    else if (a == "--seed") seed = static_cast<uint32_t>(atoi(next()));
    else if (a == "--simseed") simSeed = atoi(next());
    else if (a == "--loops") loops = atoi(next());
    else if (a == "--trace") trace = true;
    else if (a == "--time") timeLimit = static_cast<float>(atof(next()));
    else if (a == "--smooth") p.smoothTurnsSearch = true;
    else if (a == "--set") {
      char name[40];
      float v;
      if (sscanf(next(), "%39[^=]=%f", name, &v) != 2 || !raton::setParam(p, name, v)) {
        fprintf(stderr, "parametro no valido\n");
        return 2;
      }
    }
    else if (a == "--goal") {
      sscanf(next(), "%d,%d", &p.goalX, &p.goalY);
      goalGiven = true;
    } else if (a == "--start") sscanf(next(), "%d,%d,%d", &p.startX, &p.startY, &p.startDir);
    else if (a == "--size") sscanf(next(), "%dx%d", &p.mazeW, &p.mazeH);
    else {
      fprintf(stderr, "opcion desconocida: %s\n", a.c_str());
      return 2;
    }
  }

  if (bench >= 0) {
    int fails = 0, total = 0, dirty = 0;
    float sum1 = 0, sum2 = 0, sumE = 0;
    for (int level = 0; level <= 2; level += 2) {
      for (int k = -1; k < bench; ++k) {
        raton::Params pc = p;
        raton::Maze truth;
        std::string name;
        if (k < 0) {
          sim::loadMazeFile(std::string(RATON_MAZES_DIR) + "/reglamento_2026.txt", truth);
          name = "reglamento";
        } else {
          sim::randomMaze(pc.mazeW, pc.mazeH, loops, 1000u + k, truth);
          randomGoal(truth, pc, 1000u + k, pc);
          name = "aleatorio " + std::to_string(1000 + k);
        }
        const sim::Physical ph = sim::perturbed(pc, level, 77u + k);
        const sim::CaseResult r = sim::runCase(truth, pc, ph, 5u + k, false);
        ++total;
        if (!r.ok()) ++fails;
        if (!r.clean()) ++dirty;
        sum1 += r.goalTime1;
        sum2 += r.goalTime2;
        sumE += r.exploreTime;
        printf("%-5s err%d %-15s %s\n", r.clean() ? "OK" : (r.ok() ? "ROCE" : "FALLO"), level, name.c_str(),
               sim::describe(r).c_str());
        fflush(stdout);
      }
    }
    printf("\n%d de %d cumplen el reglamento (%d sin ningun roce). Media: meta explorando %.1f s, exploracion "
           "completa %.1f s, carrera %.1f s\n",
           total - fails, total, total - dirty, sum1 / total, sumE / total, sum2 / total);
    return fails ? 1 : 0;
  }

  raton::Maze truth;
  if (randomSeed >= 0) {
    sim::randomMaze(p.mazeW, p.mazeH, loops, static_cast<uint32_t>(randomSeed), truth);
    if (!goalGiven) randomGoal(truth, p, static_cast<uint32_t>(randomSeed), p);
  } else {
    if (mazePath.empty()) mazePath = std::string(RATON_MAZES_DIR) + "/reglamento_2026.txt";
    if (!sim::loadMazeFile(mazePath, truth)) {
      fprintf(stderr, "no se pudo leer %s\n", mazePath.c_str());
      return 2;
    }
    p.mazeW = truth.width();
    p.mazeH = truth.height();
  }
  const sim::Physical ph = sim::perturbed(p, perturb, seed);
  printf("fisica: celda=%.1f pared=%.1f ruedas=%.2f/%.2f eje=%.1f angulo=%.1f tau=%.3f vmax=%.0f asim=%.3f "
         "giro_escala=%.4f offsets F%d I%d D%d salida %.1f,%.1f %.1fgr\n",
         ph.cellMm, ph.wallMm, ph.wheelDiamL, ph.wheelDiamR, ph.trackMm, ph.diagAngleDeg, ph.motorTau, ph.vmaxFull,
         ph.motorAsym, ph.gyroScale, ph.offsetF, ph.offsetL, ph.offsetR, ph.startX, ph.startY, ph.startDeg);
  const sim::CaseResult r = sim::runCase(truth, p, ph, simSeed >= 0 ? static_cast<uint32_t>(simSeed) : seed, true, true, trace, timeLimit);
  printf("%s %s\n", r.clean() ? "OK" : (r.ok() ? "ROCE" : "FALLO"), sim::describe(r).c_str());
  return r.ok() ? 0 : 1;
}

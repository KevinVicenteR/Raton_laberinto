#include "run_case.h"

#include <stdio.h>

#include <random>

#include "raton/maze_text.h"
#include "raton/mouse.h"

namespace sim {

using raton::Maze;

Physical perturbed(const raton::Params& p, int level, uint32_t seed) {
  Physical ph;
  ph.cellMm = p.cellMm;
  ph.wallMm = p.wallMm;
  ph.wheelDiamL = ph.wheelDiamR = p.wheelDiamMm;
  ph.trackMm = p.trackMm;
  ph.bodyDiamMm = p.bodyDiamMm;
  ph.diagAngleDeg = p.diagAngleDeg;
  ph.vmaxFull = 1.0f / p.kV;
  ph.stictionDuty = p.kS;
  ph.motorTau = p.motorTauS;
  if (level <= 0) return ph;
  std::mt19937 rng(seed * 7919u + 13u);
  std::uniform_real_distribution<float> u(-1, 1);
  const float k = static_cast<float>(level);
  ph.cellMm = p.cellMm + 3.0f * k * u(rng);           // la pista real no mide lo nominal
  ph.wallMm = p.wallMm + 2.0f * k * u(rng);
  ph.wheelDiamL = p.wheelDiamMm * (1.0f + 0.01f * k * u(rng));
  ph.wheelDiamR = p.wheelDiamMm * (1.0f + 0.01f * k * u(rng));
  ph.trackMm = p.trackMm * (1.0f + 0.02f * k * u(rng));
  ph.diagAngleDeg = p.diagAngleDeg + 2.0f * k * u(rng);
  ph.motorTau = p.motorTauS * (1.0f + 0.3f * k * u(rng));
  ph.vmaxFull = (1.0f / p.kV) * (1.0f + 0.08f * k * u(rng));
  ph.stictionDuty = p.kS * (1.0f + 0.3f * k * u(rng));
  ph.motorAsym = 0.04f * k * u(rng);
  ph.gyroBiasDps = 0.05f + 0.1f * k;
  ph.gyroScale = 1.0f + 0.01f * k * u(rng);
  ph.rangeNoise = 0.02f + 0.01f * k;
  ph.dropout = 0.02f * k;
  ph.outlier = 0.003f * k;
  ph.offsetF = static_cast<int>(3.0f * k * u(rng));
  ph.offsetL = static_cast<int>(3.0f * k * u(rng));
  ph.offsetR = static_cast<int>(3.0f * k * u(rng));
  ph.startX = 4.0f * k * u(rng);
  ph.startY = 4.0f * k * u(rng);
  ph.startDeg = 1.5f * k * u(rng);
  return ph;
}

CaseResult runCase(const Maze& truth, const raton::Params& p, const Physical& ph, uint32_t seed, bool verbose,
                   bool race, bool trace, float timeLimit) {
  CaseResult res;
  World world(truth, ph);
  SimPlatform io(world, p, ph, seed);
  io.verbose = verbose;
  io.trace = trace;
  io.timeLimit = timeLimit;
  io.place(p.startX, p.startY, p.startDir);
  raton::Mouse mouse(io, p);
  io.watch = &mouse.maze();
  const raton::MissionResult e = mouse.explore(true, true);
  res.goal1 = e.reachedGoal;
  res.goalTime1 = e.goalTime;
  res.exploreTime = e.totalTime;
  res.backHome = e.backHome;
  res.proven = e.proven;
  res.recoveries = e.recoveries;
  res.escapes = e.escapes;
  if (race && e.reachedGoal) {
    // Como en la competencia: se recoge el raton y se pone en la salida con el mapa guardado.
    io.place(p.startX, p.startY, p.startDir);
    io.timeLimit = io.time() + 300.0f;
    const raton::MissionResult r = mouse.race(false);
    res.goal2 = r.reachedGoal;
    res.goalTime2 = r.goalTime;
    res.recoveries += r.recoveries;
  }
  res.contacts = io.contacts();
  res.contactTime = io.contactTime();
  res.maxArea = io.maxAreaTime();
  // Paredes conocidas que no coinciden con la realidad.
  const Maze& m = mouse.maze();
  for (int y = 0; y < m.height(); ++y)
    for (int x = 0; x < m.width(); ++x)
      for (int d = 0; d < 2; ++d) {
        const raton::Dir dir = d == 0 ? raton::NORTH : raton::EAST;
        const raton::Wall known = m.wall(x, y, dir);
        if (known != raton::Wall::Unknown && known != truth.wall(x, y, dir)) ++res.wrongWalls;
      }
  if (verbose) {
    static char buf[8192];
    raton::printMaze(m, buf, sizeof(buf), io.cellX(), io.cellY());
    printf("%s", buf);
  }
  return res;
}

std::string describe(const CaseResult& r) {
  char line[256];
  snprintf(line, sizeof(line),
           "meta1=%s %6.1fs explora=%6.1fs vuelta=%s demostrada=%s carrera=%s %5.1fs choques=%d atascos=%d "
           "escapes=%d zona_max=%4.1fs paredes_mal=%d",
           r.goal1 ? "si" : "NO", r.goalTime1, r.exploreTime, r.backHome ? "si" : "no", r.proven ? "si" : "no",
           r.goal2 ? "si" : "NO", r.goalTime2, r.contacts, r.recoveries, r.escapes, r.maxArea, r.wrongWalls);
  return line;
}

}  // namespace sim

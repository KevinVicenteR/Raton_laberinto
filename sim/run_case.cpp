#include "run_case.h"

#include <stdio.h>
#include <stdlib.h>

#include <string>

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
  ph.diagAngleDeg = p.diagMountDeg;
  ph.vmaxFull = 1.0f / p.kV;
  ph.stictionDuty = p.kS;
  ph.motorTau = p.motorTauS;
  ph.fovDeg = gFovDeg;
  ph.sensorHeightMm = p.sensorHeightMm;
  ph.sensorTiltDeg = p.sensorTiltDeg;
  ph.wallHeightMm = p.wallHeightMm;
  ph.floorReflect = p.floorReflect;
  ph.outsideMm = gOutsideMm;
  ph.bodyFrontMm = p.bodyFrontMm;
  ph.bodyRearMm = p.bodyRearMm;
  ph.bodyWidthMm = p.bodyWidthMm;
  ph.rectBody = !(p.bodyFrontMm == 45.0f && p.bodyRearMm == 45.0f && p.bodyWidthMm == 90.0f);
  const char* faults[3] = {"SIM_FAIL_LEFT", "SIM_FAIL_FRONT", "SIM_FAIL_RIGHT"};
  for (int i = 0; i < 3; ++i)
    if (const char* v = getenv(faults[i])) ph.laserFailAt[i] = static_cast<float>(atof(v));
  if (const char* v = getenv("SIM_FAIL_GYRO")) ph.gyroFailAt = static_cast<float>(atof(v));
  ph.gyroFrozen = getenv("SIM_FREEZE_GYRO") != nullptr;
  if (level <= 0) return ph;
  std::mt19937 rng(seed * 7919u + 13u);
  std::uniform_real_distribution<float> u(-1, 1);
  const float k = static_cast<float>(level);
  ph.cellMm = p.cellMm + 3.0f * k * u(rng);           // la pista real no mide lo nominal
  ph.wallMm = p.wallMm + 2.0f * k * u(rng);
  ph.wheelDiamL = p.wheelDiamMm * (1.0f + 0.01f * k * u(rng));
  ph.wheelDiamR = p.wheelDiamMm * (1.0f + 0.01f * k * u(rng));
  ph.trackMm = p.trackMm * (1.0f + 0.02f * k * u(rng));
  ph.diagAngleDeg = p.diagMountDeg + 2.0f * k * u(rng);
  ph.motorTau = p.motorTauS * (1.0f + 0.3f * k * u(rng));
  ph.vmaxFull = (1.0f / p.kV) * (1.0f + 0.08f * k * u(rng));
  ph.stictionDuty = p.kS * (1.0f + 0.3f * k * u(rng));
  ph.motorAsym = 0.04f * k * u(rng);
  ph.gyroBiasDps = 0.05f + 0.1f * k;
  ph.gyroScale = 1.0f + 0.01f * k * u(rng);
  ph.rangeNoise = 0.02f + 0.01f * k;
  ph.dropout = 0.02f * k;
  ph.outlier = 0.003f * k;
  // Desfase de cada VL53L0X sin calibrar (de fabrica pueden ser 1-2 cm).
  ph.offsetF = static_cast<int>(8.0f * k * u(rng));
  ph.offsetL = static_cast<int>(8.0f * k * u(rng));
  ph.offsetR = static_cast<int>(8.0f * k * u(rng));
  ph.startX = 4.0f * k * u(rng);
  ph.startY = 4.0f * k * u(rng);
  ph.startDeg = 1.5f * k * u(rng);
  ph.mountDeg = 45.0f + 1.5f * k * u(rng);  // montaje real de los laterales con error
  ph.sensorHeightMm = p.sensorHeightMm + 2.0f * k * u(rng);
  ph.sensorTiltDeg = p.sensorTiltDeg + 1.0f * k * u(rng);
  ph.wallHeightMm = p.wallHeightMm + 2.0f * k * u(rng);
  ph.floorReflect = p.floorReflect * (1.0f + 0.25f * k * u(rng));
  // Motores muy distintos de los parametros (sin calibrar, bateria baja): SIM_MOTOR=escala de velocidad.
  if (getenv("SIM_NOVERT")) {  // diagnostico: sin errores en lo vertical (altura, inclinacion, piso)
    ph.sensorHeightMm = p.sensorHeightMm;
    ph.sensorTiltDeg = p.sensorTiltDeg;
    ph.wallHeightMm = p.wallHeightMm;
    ph.floorReflect = p.floorReflect;
  }
  if (const char* fr = getenv("SIM_FLOOR")) ph.floorReflect = static_cast<float>(atof(fr));
  if (const char* hh = getenv("SIM_H")) ph.sensorHeightMm = static_cast<float>(atof(hh));  // altura real distinta
  if (const char* m = getenv("SIM_MOTOR")) {
    ph.vmaxFull *= static_cast<float>(atof(m));
    ph.stictionDuty *= 2.0f - static_cast<float>(atof(m));
  }
  // Diagnostico: SIM_SOLO=grupo deja solo ese grupo de errores (los demas como el nominal).
  if (const char* only = getenv("SIM_SOLO")) {
    const Physical nom = perturbed(p, 0, seed);
    const std::string o = only;
    Physical r = nom;
    r.rangeNoise = 0.02f;
    if (o == "pista") { r.cellMm = ph.cellMm; r.wallMm = ph.wallMm; }
    if (o == "ruedas") { r.wheelDiamL = ph.wheelDiamL; r.wheelDiamR = ph.wheelDiamR; r.trackMm = ph.trackMm; }
    if (o == "motor") { r.motorTau = ph.motorTau; r.vmaxFull = ph.vmaxFull; r.stictionDuty = ph.stictionDuty; r.motorAsym = ph.motorAsym; }
    if (o == "giro") { r.gyroBiasDps = ph.gyroBiasDps; r.gyroScale = ph.gyroScale; }
    if (o == "ruido") { r.rangeNoise = ph.rangeNoise; r.dropout = ph.dropout; r.outlier = ph.outlier; }
    if (o == "offsets") { r.offsetF = ph.offsetF; r.offsetL = ph.offsetL; r.offsetR = ph.offsetR; }
    if (o == "salida") { r.startX = ph.startX; r.startY = ph.startY; r.startDeg = ph.startDeg; }
    if (o == "montaje") r.mountDeg = ph.mountDeg;
    return r;
  }
  return ph;
}

CaseResult runCase(const Maze& truth, const raton::Params& p, const Physical& ph, uint32_t seed, bool verbose,
                   bool race, bool trace, float timeLimit) {
  CaseResult res;
  World world(truth, ph);
  raton::Params pc = p;  // la calibracion de los sensores cambia los desfases
  SimPlatform io(world, pc, ph, seed);
  io.verbose = verbose;
  io.trace = trace;
  io.timeLimit = timeLimit;
  io.place(p.startX, p.startY, p.startDir);
  raton::Mouse mouse(io, pc);
  io.watch = &mouse.maze();
  if (gAutoCal) mouse.calibrateSensors(pc, true);  // como el modo 1 al arrancar
  const raton::MissionResult e = mouse.explore(true, true);
  res.goal1 = e.reachedGoal;
  res.goalTime1 = e.goalTime;
  res.exploreTime = e.totalTime;
  res.backHome = e.backHome;
  res.proven = e.proven;
  res.recoveries = e.recoveries;
  res.escapes = e.escapes;
  res.contactsExplore = io.contacts();
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
  for (int i = 0; i < 3; ++i) {
    res.loc[i] = io.locRms(i);
    res.locMax[i] = io.locMax(i);
  }
  res.wobble = io.wobbleRms();
  res.offRms = io.offsetRms();
  res.offMax = io.offsetMax();
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
           "meta1=%s %6.1fs explora=%6.1fs vuelta=%s demostrada=%s carrera=%s %5.1fs choques=%d(%d) atascos=%d "
           "escapes=%d zona_max=%4.1fs paredes_mal=%d error_pose=%.1f/%.1f/%.1f max=%.0f/%.0f/%.0f "
           "zigzag=%.1f descentrado=%.1f/%.0f",
           r.goal1 ? "si" : "NO", r.goalTime1, r.exploreTime, r.backHome ? "si" : "no", r.proven ? "si" : "no",
           r.goal2 ? "si" : "NO", r.goalTime2, r.contacts, r.contacts - r.contactsExplore, r.recoveries, r.escapes, r.maxArea,
           r.wrongWalls, r.loc[0], r.loc[1], r.loc[2], r.locMax[0], r.locMax[1], r.locMax[2], r.wobble, r.offRms,
           r.offMax);
  return line;
}

}  // namespace sim

namespace sim {

PracticeResult runPractice(int w, int h, float cellMm, int level, uint32_t seed, bool verbose) {
  PracticeResult res;
  // Pista real: laberinto aleatorio de w x h colocado en el mapa de 32 x 32; fuera, todo cerrado.
  Maze small, truth;
  randomMaze(w, h, (w * h) / 6, seed, small);
  truth.reset(32, 32, true);
  for (int y = 0; y < 32; ++y)
    for (int x = 0; x < 32; ++x)
      for (int d = 0; d < 4; ++d) truth.set(x, y, static_cast<raton::Dir>(d), raton::Wall::Closed);
  std::mt19937 rng(seed * 13u + 5u);
  // Salida: una celda con pared a izquierda y derecha mirando a una salida libre.
  int sx = 0, sy = 0, sd = 0;
  for (int i = 0; i < 1000; ++i) {
    sx = static_cast<int>(rng() % w);
    sy = static_cast<int>(rng() % h);
    sd = static_cast<int>(rng() % 4);
    const raton::Dir d = static_cast<raton::Dir>(sd);
    if (small.wall(sx, sy, d) == raton::Wall::Open && small.wall(sx, sy, raton::turnLeft(d)) == raton::Wall::Closed &&
        small.wall(sx, sy, raton::turnRight(d)) == raton::Wall::Closed)
      break;
  }
  // El mapa del raton pone la salida en (16,16) mirando al "norte": se gira la pista para que coincida.
  const int ox = 16, oy = 16;
  auto toTruth = [&](int x, int y, int& tx, int& ty) {
    const int rx = x - sx, ry = y - sy;  // relativo a la salida en la pista
    // rotar para que sd quede como norte
    int ax = rx, ay = ry;
    for (int k = 0; k < sd; ++k) {  // girar a la izquierda sd veces: (x, y) -> (-y, x)
      const int t = ax;
      ax = -ay;
      ay = t;
    }
    tx = ox + ax;
    ty = oy + ay;
  };
  for (int y = 0; y < h; ++y)
    for (int x = 0; x < w; ++x) {
      int tx, ty;
      toTruth(x, y, tx, ty);
      for (int d = 0; d < 4; ++d) {
        const raton::Dir dir = static_cast<raton::Dir>(d);
        const raton::Dir td = static_cast<raton::Dir>((d - sd + 4) & 3);
        if (small.wall(x, y, dir) == raton::Wall::Open && truth.inside(tx + raton::dx(td), ty + raton::dy(td)))
          truth.set(tx, ty, td, raton::Wall::Open);
      }
    }
  raton::Params p;
  p.mazeW = p.mazeH = 32;
  p.startX = p.startY = 16;
  p.startDir = 0;
  p.goalX = p.goalY = -1;
  Physical ph = perturbed(p, level, seed + 101u);
  ph.cellMm = cellMm;  // la medida real de la pista; el raton cree que mide p.cellMm hasta que la mide
  // El frontal ya se calibro en la pista de la competencia (calsonda / calfront), con 2 mm de error.
  p.frontOffsetMm = -ph.offsetF + (level > 0 ? 2 : 0);
  World world(truth, ph);
  SimPlatform io(world, p, ph, seed);
  io.verbose = verbose;
  io.trace = verbose && getenv("SIMTRACE");
  io.timeLimit = 600.0f;
  io.place(16, 16, 0);
  raton::Mouse mouse(io, p);
  io.watch = &mouse.maze();
  res.measured = mouse.measureCell(p) > 0;
  res.cellMeasured = p.cellMm;
  const raton::MissionResult r = mouse.explore(false, true);
  res.exploredAll = r.exploredAll;
  res.backHome = r.backHome;
  res.time = r.totalTime;
  res.contacts = io.contacts();
  res.maxArea = io.maxAreaTime();
  const Maze& m = mouse.maze();
  int visits = 0, cells = 0;
  for (int y = 0; y < h; ++y)
    for (int x = 0; x < w; ++x) {
      int tx, ty;
      toTruth(x, y, tx, ty);
      if (!m.fullyKnown(tx, ty)) {
        ++res.unknownCells;
        for (int d = 0; d < 4 && verbose; ++d) {
          const raton::Dir dir = static_cast<raton::Dir>(d);
          if (m.wall(tx, ty, dir) == raton::Wall::Unknown)
            printf("SIN_CONOCER %d,%d dir %d votos %d verdad %d visitas %d\n", tx, ty, d, m.votes(tx, ty, dir),
                   static_cast<int>(truth.wall(tx, ty, dir)), m.visits(tx, ty));
        }
      }
      visits += m.visits(tx, ty);
      ++cells;
      for (int d = 0; d < 4; ++d) {
        const raton::Wall k = m.wall(tx, ty, static_cast<raton::Dir>(d));
        if (k != raton::Wall::Unknown && k != truth.wall(tx, ty, static_cast<raton::Dir>(d))) ++res.wrongWalls;
      }
    }
  res.revisits = visits - cells;
  if (verbose) {
    static char buf[16384];
    raton::printMaze(m, buf, sizeof(buf), io.cellX(), io.cellY());
    printf("%s", buf);
  }
  return res;
}

std::string describe(const PracticeResult& r) {
  char line[200];
  snprintf(line, sizeof(line),
           "celda_medida=%4.0f explorado=%s vuelta=%s sin_conocer=%d paredes_mal=%d choques=%d revisitas=%d "
           "zona_max=%4.1fs tiempo=%5.1fs",
           r.cellMeasured, r.exploredAll ? "si" : "NO", r.backHome ? "si" : "NO", r.unknownCells, r.wrongWalls,
           r.contacts, r.revisits, r.maxArea, r.time);
  return line;
}

}  // namespace sim

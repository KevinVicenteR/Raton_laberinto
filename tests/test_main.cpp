// Pruebas del nucleo portable y del raton completo en el simulador. Una sola entrada para CTest.
#include <math.h>
#include <stdio.h>
#include <string.h>

#include <random>
#include <string>

#include "raton/frame.h"
#include "raton/autocal.h"
#include "raton/lateral.h"
#include "raton/line_queue.h"
#include "raton/maze.h"
#include "raton/navigator.h"
#include "raton/mouse.h"
#include "raton/maze_text.h"
#include "raton/param_table.h"
#include "raton/params.h"
#include "raton/planner.h"
#include "raton/sonda.h"
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

// La sonda predice lo mismo que lee el VL53L0X simulado (cono, postes, huecos).
void testSonda() {
  Maze truth;
  CHECK(sim::loadMazeFile(std::string(RATON_MAZES_DIR) + "/reglamento_2026.txt", truth));
  sim::Physical ph;
  ph.sensorHeightMm = 40.0f;  // sensor alto: parte del cono pasa por encima de las paredes y toca el piso
  sim::World world(truth, ph);
  Sonda sonda;
  sonda.setup(ph.cellMm, ph.wallMm, 25.0f, 600);
  sonda.setVertical(ph.sensorHeightMm, ph.sensorTiltDeg, ph.wallHeightMm, ph.floorReflect);
  std::mt19937 rng(5);
  std::uniform_real_distribution<float> u(0, 1);
  int n = 0, bad = 0, missing = 0;
  float sumErr = 0;
  for (int i = 0; i < 4000; ++i) {
    // Pose dentro de una celda, lejos de las paredes, con un rumbo cualquiera y un sensor a 0 o +-45.
    const int cx = static_cast<int>(u(rng) * truth.width()), cy = static_cast<int>(u(rng) * truth.height());
    const float x = (cx + 0.25f + 0.5f * u(rng)) * ph.cellMm, y = (cy + 0.25f + 0.5f * u(rng)) * ph.cellMm;
    const float ang = u(rng) * 6.2832f;
    const float real = world.coneReading(x, y, ang, 25.0f);
    Sonda::Scan scan;
    sonda.trace(truth, x, y, ang, scan);
    const float pred = sonda.predict(truth, scan);
    const bool realOk = real < 600.0f;
    if (realOk != (pred >= 0)) {
      if (fabsf((realOk ? real : pred) - 600.0f) > 30.0f) ++missing;
      continue;
    }
    if (!realOk) continue;
    ++n;
    const float e = fabsf(pred - real);
    sumErr += e;
    if (e > 4.0f + 0.04f * real) ++bad;
  }
  printf("  sonda: %d lecturas, error medio %.1f mm, %d con error grande, %d de alcance distinto\n", n,
         sumErr / (n ? n : 1), bad, missing);
  CHECK(n > 2500);
  // El simulador usa 13 rayos y la sonda 9: la diferencia es como la del modelo con el sensor real.
  CHECK(sumErr / n < 3.5f);
  CHECK(bad < n / 15);
  CHECK(missing < 20);
}

// El raton completo en el simulador: laberinto del reglamento y aleatorios, con fisica ideal y con
// errores (pista de otra medida, ruedas distintas, sensores y giroscopio descalibrados).
// Transporte sin respuesta: parada enclavada. Desacuerdo de giro con muestras validas: recuperacion.
void testSensorFaults() {
  Params p;
  Sensors s;
  Motion motion(p);
  motion.reset(s);
  motion.cruise(200);
  float l = 0, r = 0;
  for (int k = 1; k <= 60; ++k) {
    s.t = k * p.dt;
    motion.update(s, l, r);
  }
  CHECK(motion.sensorFault());
  CHECK(l == 0 && r == 0);
  s.gyroFresh = true;
  s.t += p.dt;
  motion.update(s, l, r);
  CHECK(motion.sensorFault() && l == 0 && r == 0);
  motion.reset(s);
  motion.turnTo(90, p.rotSpeed, p.rotAccel);
  for (int k = 0; k < 130; ++k) {
    s.t += p.dt;
    s.encRightMm += 0.3f;
    s.encLeftMm -= 0.3f;
    motion.update(s, l, r);  // gyroFresh pero angulo congelado
  }
  CHECK(!motion.sensorFault() && motion.gyroDisagreement());
  motion.reset(s);
  s.gyroDeg = NAN;
  motion.update(s, l, r);
  CHECK(motion.sensorFault() && l == 0 && r == 0);

  Maze truth;
  CHECK(sim::loadMazeFile(std::string(RATON_MAZES_DIR) + "/reglamento_2026.txt", truth));
  // Fuera de alcance no es una averia: llegan muestras nuevas sin pared visible.
  {
    class NoWalls : public sim::SimPlatform {
     public:
      using sim::SimPlatform::SimPlatform;
      const Sensors& tick() override {
        sample_ = sim::SimPlatform::tick();
        sample_.left.mm = sample_.front.mm = sample_.right.mm = -1;
        return sample_;
      }
     private:
      Sensors sample_;
    };
    const auto ph = sim::perturbed(p, 0, 1);
    sim::World world(truth, ph);
    NoWalls io(world, p, ph, 1);
    io.place(p.startX, p.startY, p.startDir);
    Maze map;
    map.reset(p.mazeW, p.mazeH, true);
    Navigator nav(io, p, map);
    nav.begin(p.startX, p.startY, static_cast<Dir>(p.startDir));
    for (int k = 0; k < 250; ++k) nav.step();
    CHECK(!nav.sensorFault());
  }
  // Cada laser se corta por separado; los otros siguen publicando.
  for (int failed = 0; failed < 3; ++failed) {
    sim::Physical ph = sim::perturbed(p, 0, 1);
    ph.laserFailAt[failed] = 0.1f;
    sim::World world(truth, ph);
    sim::SimPlatform io(world, p, ph, 1);
    io.place(p.startX, p.startY, p.startDir);
    Maze map;
    Navigator nav(io, p, map);
    nav.begin(p.startX, p.startY, static_cast<Dir>(p.startDir));
    bool others[3] = {};
    for (int k = 0; k < 200; ++k) {
      nav.step();
      const Sensors& v = nav.sensors();
      const Range rr[3] = {v.left, v.front, v.right};
      if (v.t > 0.3f) {
        CHECK(!rr[failed].fresh && rr[failed].mm == -1);
        for (int i = 0; i < 3; ++i) others[i] = others[i] || rr[i].fresh;
      }
    }
    CHECK(!nav.motion().sensorFault());
    CHECK(nav.sensorFault() == (failed == 1));
    CHECK(others[(failed + 1) % 3] && others[(failed + 2) % 3]);
  }
  // Una mision interrumpida por IMU/frontal no guarda un mapa parcial.
  for (int which = 0; which < 2; ++which) {
    sim::Physical ph = sim::perturbed(p, 0, 1);
    if (which == 0) ph.gyroFailAt = 0.1f;
    else ph.laserFailAt[1] = 0.1f;
    sim::World world(truth, ph);
    sim::SimPlatform io(world, p, ph, 1);
    io.place(p.startX, p.startY, p.startDir);
    Mouse mouse(io, p);
    const MissionResult result = mouse.explore(true, true);
    CHECK(!result.reachedGoal && !result.backHome);
    CHECK(!io.saved);
    CHECK(io.time() < 1.0f);
    CHECK(mouse.navigator().sensorFault());
  }
  // IMU rota durante calibracion: el barrido retorna sin consumir el limite del simulador.
  for (int frozen = 0; frozen < 2; ++frozen) {
    sim::Physical ph = sim::perturbed(p, 0, 1);
    ph.gyroFailAt = 0.1f;
    ph.gyroFrozen = frozen;
    sim::World world(truth, ph);
    sim::SimPlatform io(world, p, ph, 1);
    io.place(p.startX, p.startY, p.startDir);
    Maze map;
    Navigator nav(io, p, map);
    nav.begin(p.startX, p.startY, static_cast<Dir>(p.startDir));
    Navigator::SweepLog log;
    nav.sweep(110, 120, log);
    CHECK(frozen ? nav.motion().gyroDisagreement() : nav.motion().sensorFault());
    CHECK(nav.now() < 1.0f);
  }
}

void testRecoveryAndTiming() {
  Params p;
  Sensors s;
  s.gyroFresh = true;
  Motion motion(p);
  motion.reset(s);
  motion.cruise(200);
  float l = 0, r = 0;
  for (int k = 0; k < 150; ++k) {
    const float intervals[3] = {0.002f, 0.006f, 0.03f};
    const float dt = intervals[k % 3];
    s.t += dt;
    s.encLeftMm += 200 * dt; s.encRightMm += 200 * dt;
    motion.update(s, l, r);
  }
  CHECK(fabsf(motion.measuredSpeed() - 200) < 1);
  const float before = motion.measuredSpeed();
  motion.hold(s);
  CHECK(motion.measuredSpeed() == before);

  class Jammed : public sim::SimPlatform {
   public:
    using sim::SimPlatform::SimPlatform;
    bool reversed = false;
    void setMotors(float left, float right) override {
      if (time() < 0.65f) sim::SimPlatform::setMotors(0, 0);
      else { reversed |= left < -0.05f && right < -0.05f; sim::SimPlatform::setMotors(left, right); }
    }
  };
  p.mazeW = p.mazeH = 5;
  p.startX = 2; p.startY = 0; p.startDir = NORTH;
  p.goalX = 2; p.goalY = 3; p.goalW = p.goalH = 1;
  Maze truth;
  truth.reset(5, 5, true);
  const auto ph = sim::perturbed(p, 0, 1);
  sim::World world(truth, ph);
  Maze corridor;
  corridor.reset(5, 5, true);
  for (int y = 0; y < 5; ++y) {
    corridor.set(2, y, WEST, Wall::Closed);
    corridor.set(2, y, EAST, Wall::Closed);
  }
  auto shifted = ph;
  shifted.startX = 18; // el centrado lateral no debe oponerse al giro de recuperacion
  sim::World jamWorld(corridor, shifted);
  Jammed io(jamWorld, p, shifted, 1);
  io.place(2, 0, NORTH);
  Mouse mouse(io, p);
  const auto result = mouse.toGoal();
  if (!result.reachedGoal) for (const auto& event : io.events) printf("JAM %.2f %s\n", event.t, event.text.c_str());
  CHECK(result.reachedGoal);
  CHECK(result.recoveries > 0 && io.reversed);
  CHECK(!mouse.navigator().sensorFault());
  CHECK(result.totalTime < 30);
  CHECK(io.contacts() == 0); // rescate en pasillo estrecho sin golpear las paredes

  // Bloqueo durante un pivote: no dar por completados los 90 grados ni salir hacia otra celda.
  class SpinJammed : public sim::SimPlatform {
   public:
    using sim::SimPlatform::SimPlatform;
    void setMotors(float left, float right) override {
      const bool jam = time() > 0.09f && time() < 0.70f && left * right < 0;
      sim::SimPlatform::setMotors(jam ? 0 : left, jam ? 0 : right);
    }
  };
  p.goalX = 1; p.goalY = 0;
  SpinJammed spin(world, p, ph, 1);
  spin.place(2, 0, NORTH);
  Mouse turning(spin, p);
  const auto turned = turning.toGoal();
  bool incomplete = false;
  for (const auto& event : spin.events) incomplete |= event.text.find("GIRO_INCOMPLETO") == 0;
  if (!turned.reachedGoal) for (const auto& event : spin.events) printf("SPIN %.2f %s\n", event.t, event.text.c_str());
  CHECK(incomplete && turned.recoveries > 0);
  CHECK(turned.reachedGoal && spin.cellX() == 1 && spin.cellY() == 0);
  CHECK(!turning.navigator().sensorFault());

  class Locked : public sim::SimPlatform {
   public:
    using sim::SimPlatform::SimPlatform;
    void setMotors(float, float) override { sim::SimPlatform::setMotors(0, 0); }
  };
  p.goalX = 2; p.goalY = 3;
  Locked locked(world, p, ph, 1);
  locked.place(2, 0, NORTH);
  Mouse blocked(locked, p);
  const auto failure = blocked.toGoal();
  CHECK(!failure.reachedGoal && failure.recoveries == 3);
  CHECK(!locked.saved && failure.totalTime < 10);
  CHECK(!blocked.navigator().sensorFault()); // bloqueo mecanico no se etiqueta como averia de IMU
}


void testAutoCalibration() {
  GyroBiasWindow stable, moving, empty, invalid;
  for (int k = 0; k < 100; ++k) {
    stable.add(0.4f + (k % 2 ? 0.02f : -0.02f));
    moving.add(k % 2 ? 4.0f : -4.0f);
    invalid.add(k == 20 ? NAN : 0.4f);
  }
  CHECK(stable.ready() && fabsf(stable.bias() - 0.4f) < 0.01f);
  CHECK(!moving.ready() && !empty.ready() && !invalid.ready());
  Breakaway startup;
  float extra = 0, help = 0;
  for (int k = 0; k < 200; ++k) help = startup.update(80, 0, 0.002f, true, extra);
  CHECK(help > 0.15f && help <= 0.221f);
  CHECK(extra == 0); // atascado: no se guarda un supuesto rozamiento
  for (int k = 0; k < 30; ++k) startup.update(80, 30, 0.002f, true, extra);
  CHECK(extra > 0 && extra <= 0.22f); // el encoder confirmo que volvio a girar
  CHECK(startup.update(0, 0, 0.002f, true, extra) == 0);
  startup.reset();
  float disabled = 0;
  for (int k = 0; k < 200; ++k) help = startup.update(80, 0, 0.002f, false, disabled);
  CHECK(help == 0 && disabled == 0);

  Params p;
  Motion motion(p);
  Sensors v;
  v.gyroFresh = true;
  motion.reset(v);
  motion.cruise(200);
  float l = 0, r = 0;
  for (int k = 0; k < 200; ++k) {
    v.t += p.dt;
    v.encLeftMm -= 0.2f; v.encRightMm -= 0.2f;
    motion.update(v, l, r);
  }
  CHECK(motion.encoderFault() && l == 0 && r == 0);

  p.motorBalance = 0.12f;
  p.startBoostLF = 0.08f;
  motion.loadLearned();
  CHECK(motion.motorBalance() == p.motorBalance && motion.startBoost(0) == p.startBoostLF);
  p = Params();
  motion.loadLearned();
  CHECK(motion.motorBalance() == 0 && motion.startBoost(0) == 0);

  // El paso aprendido solo se carga para la misma geometria y unidades de encoder.
  Maze truth;
  CHECK(sim::loadMazeFile(std::string(RATON_MAZES_DIR) + "/reglamento_2026.txt", truth));
  p.learnedCellMm = 159;
  p.learnedCellRefMm = p.cellMm;
  p.learnedWheelDiamMm = p.wheelDiamMm;
  p.learnedCountsPerRev = p.countsPerRev;
  auto ph = sim::perturbed(p, 0, 1);
  sim::World world(truth, ph);
  sim::SimPlatform io(world, p, ph, 1);
  io.place(p.startX, p.startY, p.startDir);
  Mouse mouse(io, p);
  mouse.navigator().begin(p.startX, p.startY, static_cast<Dir>(p.startDir));
  CHECK(fabsf(mouse.navigator().cellPitch() - 159) < 0.01f);
  p.wheelDiamMm += 1;
  mouse.navigator().begin(p.startX, p.startY, static_cast<Dir>(p.startDir));
  CHECK(fabsf(mouse.navigator().cellPitch() - p.cellMm) < 0.01f);

  // Un motor derecho mas fuerte necesita menos anticipacion que el izquierdo.
  {
    Params drive;
    Maze open;
    open.reset(4, 12, true);
    auto physical = sim::perturbed(drive, 0, 1);
    physical.motorAsym = 0.2f;
    sim::World flat(open, physical);
    sim::SimPlatform wheels(flat, drive, physical, 1);
    wheels.place(1, 0, NORTH);
    Sensors sample = wheels.tick();
    Motion controller(drive);
    controller.reset(sample);
    controller.cruise(200);
    for (int k = 0; k < 1600; ++k) {
      sample = wheels.tick();
      controller.update(sample, l, r);
      wheels.setMotors(l, r);
    }
    wheels.setMotors(0, 0);
    CHECK(!controller.sensorFault());
    CHECK(controller.motorBalance() < -0.0001f);
    CHECK(fabsf(controller.angle()) < 2);
  }

  // Giro lento con rozamiento alto: ambos encoders y el giroscopio deben confirmar el giro.
  p = Params();
  ph = sim::perturbed(p, 0, 1);
  ph.stictionDuty = 0.24f;
  sim::World hardWorld(truth, ph);
  sim::SimPlatform hardIo(hardWorld, p, ph, 1);
  hardIo.place(p.startX, p.startY, p.startDir);
  Motion turn(p);
  Sensors t = hardIo.tick();
  turn.reset(t);
  turn.turnTo(90, 90, 360);
  float settledAt = -1;
  for (int k = 0; k < 1400; ++k) {
    t = hardIo.tick();
    turn.update(t, l, r);
    if (settledAt < 0 && fabsf(turn.angle() - 90) < 3 && fabsf(turn.measuredRate()) < 40) settledAt = t.t;
    hardIo.setMotors(l, r);
  }
  hardIo.setMotors(0, 0);
  CHECK(!turn.sensorFault());
  CHECK(fabsf(turn.angle() - 90) < 3);
  CHECK(settledAt > 0 && settledAt < 2.0f);
}

void testThreeModes() {
  Params p;
  p.mazeW = 4; p.mazeH = 4;
  p.startX = 0; p.startY = 0; p.startDir = NORTH;
  p.goalX = 2; p.goalY = 2; p.goalW = 1; p.goalH = 1;
  Maze truth;
  sim::randomMaze(4, 4, 2, 918, truth);
  sim::makeStartCell(truth, 0, 0, NORTH, 918);
  const auto ph = sim::perturbed(p, 0, 1);
  sim::World world(truth, ph);
  sim::SimPlatform io(world, p, ph, 1);
  io.place(0, 0, NORTH);
  Mouse mouse(io, p);
  const auto mapped = mouse.mapAll();
  CHECK(mapped.exploredAll && mapped.backHome);
  CHECK(io.saved);
  CHECK(mapped.proven);
  io.place(0, 0, NORTH);
  const auto solved = mouse.solveLearned();
  CHECK(solved.reachedGoal);
  mouse.resetMap();
  io.place(0, 0, NORTH);
  const float before = io.time();
  const auto missing = mouse.solveLearned();
  CHECK(!missing.reachedGoal);
  CHECK(io.time() == before); // sin ruta conocida no comienza a moverse
  const auto unknown = mouse.toGoal(); // arranque sin calibracion contra paredes
  CHECK(unknown.reachedGoal);
  CHECK(!mouse.navigator().sensorFault());
}

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
    sim::makeStartCell(truth, pc.startX, pc.startY, static_cast<Dir>(pc.startDir & 3), 2000u + k);
    pc.goalX = 2 + k;
    pc.goalY = 5 + (k % 3);
    r = sim::runCase(truth, pc, sim::perturbed(pc, (k % 2) * 2, 300u + k), 9u + k, false);
    printf("  aleatorio %d:       %s\n", k, sim::describe(r).c_str());
    CHECK(r.ok());
    CHECK(r.maxArea < 20.0f);
  }
}

// Modo practica: pistas de cualquier medida, sin meta. Mide el pasillo y explora todo sin olvidar lo
// recorrido.
void testPractice() {
  const float cells[] = {200.0f, 260.0f, 170.0f};
  for (int k = 0; k < 3; ++k) {
    const sim::PracticeResult r = sim::runPractice(5 + k, 5, cells[k], 0, 600u + k, false);
    printf("  practica %3.0f mm:  %s\n", cells[k], sim::describe(r).c_str());
    CHECK(r.measured && fabsf(r.cellMeasured - cells[k]) < 12.0f);
    CHECK(r.exploredAll && r.backHome);
    CHECK(r.unknownCells <= 1 && r.wrongWalls == 0);
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
  testSonda();
  testSensorFaults();
  testRecoveryAndTiming();
  testAutoCalibration();
  testThreeModes();
  testMissions();
  testPractice();
  printf("%d comprobaciones, %d fallos\n", checks, failures);
  return failures ? 1 : 0;
}

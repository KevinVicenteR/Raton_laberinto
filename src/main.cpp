// Firmware del raton de laberinto.
//
// Uso (sin arrancador): pon el raton en un pasillo de la pista y enciendelo. En los primeros
// MODE_SELECT_MS, cada pasada de la mano delante del laser frontal cambia de modo:
//   1 = seguir pared derecha (por defecto), 2 = seguir pared izquierda, 3 = resolver laberinto,
//   4 = ruta mas corta del laberinto de la competencia (competition_maze.h),
//   5 = espacio libre (va hacia donde los laseres no ven pared).
// Seguir la pared usa solo los laseres: no depende del tamano de las celdas ni de la pista.
// Telemetria, mapa y ordenes por Bluetooth (tools/telemetria.html) o Monitor Serie:
//   x = parar, s = arrancar, m1..m5 = modo. Parado, una pasada de la mano tambien lo arranca.

#include <Arduino.h>
#include <Wire.h>

#include "config.h"
#include "hw/display.h"
#include "hw/encoders.h"
#include "hw/gyro.h"
#include "hw/lasers.h"
#include "hw/motors.h"
#include "robot_hw.h"
#include "competition_maze.h"
#include "solver.h"
#include "telemetry.h"

namespace {

Motors motors;
Encoders encoders;
Lasers lasers;
Gyro gyro;
Display display;
RobotHW robot(motors, encoders, lasers, gyro, display);
raton::Solver solver(robot, cfg::MAZE_WIDTH, cfg::MAZE_HEIGHT);

const int kModes = 5;
const char* const kModeNames[] = {"", "Pared derecha", "Pared izquierda", "Laberinto", "Ruta competencia",
                                  "Espacio libre"};

void fail(const char* msg) {
  motors.coast();
  Serial.println(msg);
  display.show("ERROR", msg);
  while (true) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(150);
  }
}

// Cuenta atras con el raton quieto y calibra el giroscopio antes de moverse.
void waitAutoStart(const char* title, uint32_t ms) {
  Serial.print(title);
  Serial.println(": arranque automatico");
  const uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    char l2[22];
    snprintf(l2, sizeof(l2), "Arranca en %lu s", (unsigned long)((ms - (millis() - t0) + 999) / 1000));
    display.show(title, "", l2, "No tocar");
    digitalWrite(LED_BUILTIN, (millis() / 250) % 2);
    delay(100);
  }
  digitalWrite(LED_BUILTIN, HIGH);
  display.show(title, "", "Calibrando...");
  gyro.calibrate();  // el raton tiene que estar quieto
  digitalWrite(LED_BUILTIN, LOW);
}

// Cuenta pasadas de la mano delante del laser frontal durante MODE_SELECT_MS (cada pasada alarga
// el plazo). Una pared fija delante no cuenta: hace falta acercar y retirar la mano.
int selectMode() {
  int mode = 1;
  if (cfg::MODE_SELECT_MS == 0) return mode;
  uint32_t deadline = millis() + cfg::MODE_SELECT_MS;
  bool far = false;
  bool near = false;
  while (static_cast<int32_t>(millis() - deadline) < 0) {
    robot.idle(20);
    const int f = lasers.rawMm(Lasers::FRONT);
    if (f < 0 || f > cfg::HAND_FAR_MM) {
      if (near) {
        mode = mode % kModes + 1;
        deadline = millis() + cfg::MODE_SELECT_MS;
      }
      far = true;
      near = false;
    } else if (far && f < cfg::HAND_NEAR_MM) {
      near = true;
    }
    char l2[22];
    snprintf(l2, sizeof(l2), "Modo %d", mode);
    display.show("ELEGIR MODO", l2, kModeNames[mode], "Pasa la mano");
    digitalWrite(LED_BUILTIN, (millis() / 250) % 2);
  }
  return mode;
}

// Parado: espera la orden de arrancar ('s' por Bluetooth o Serie) o, si WAIT_FOR_WEB_START es false,
// una pasada de la mano. 'm1'..'m3' cambia el modo mientras espera.
int waitForStart(int mode) {
  bool far = false;
  bool near = false;
  robot.setState("PARADO");
  while (true) {
    robot.idle(20);
    const int m = robot.takeMode();
    if (m) mode = m;
    if (robot.takeStart()) return mode;
    if (cfg::WAIT_FOR_WEB_START) {
      char l1[22];
      snprintf(l1, sizeof(l1), "Modo %d", mode);
      display.show("ESPERANDO", l1, kModeNames[mode], "Pulsa ARRANCAR en web");
      digitalWrite(LED_BUILTIN, (millis() / 1000) % 2);
      continue;
    }
    const int f = lasers.rawMm(Lasers::FRONT);
    if (f < 0 || f > cfg::HAND_FAR_MM) {
      if (near) return mode;
      far = true;
    } else if (far && f < cfg::HAND_NEAR_MM) {
      near = true;
    }
    char l1[22];
    snprintf(l1, sizeof(l1), "Modo %d", mode);
    display.show("PARADO", l1, kModeNames[mode], "Mano o web = arrancar");
  }
}

// Cuenta en la telemetria que decide y con que lecturas, para entender los movimientos raros.
void decide(const char* why) {
  char line[96];
  snprintf(line, sizeof(line), "DECIDE: %s | I%d F%d D%d", why, lasers.mm(Lasers::LEFT), robot.frontMm(),
           lasers.mm(Lasers::RIGHT));
  telemetry::println(line);
  Serial.println(line);
}

// Hay pared delante de verdad (no un atasco con el frente libre).
bool frontBlocked() {
  const int f = robot.frontMm();
  return f >= 0 && f < cfg::FRONT_WALL_MM;
}

// Atascado con el frente libre (el morro roza una esquina): atras y enderezarse hacia el lado libre.
void unstick() {
  auto space = [](int mm) { return mm < 0 ? 10000 : mm; };
  const int left = space(lasers.mm(Lasers::LEFT));
  const int right = space(lasers.mm(Lasers::RIGHT));
  robot.setState("DESATASCO");
  decide(left > right ? "atascado con el frente libre -> atras y endereza a la izquierda"
                      : "atascado con el frente libre -> atras y endereza a la derecha");
  robot.backUp(cfg::UNSTICK_BACK_MM);
  robot.turnDegrees(left > right ? cfg::UNSTICK_DEG : -cfg::UNSTICK_DEG);
}

// Callejon sin salida: en 14 cm no cabe media vuelta. Sale marcha atras hasta el primer hueco y entra
// en el con el giro en dos tiempos; si no encuentra hueco, intenta la media vuelta.
void deadEnd() {
  decide("callejon -> sale marcha atras buscando un hueco");
  const int side = robot.backOut(3.0f * robot.corridorMm());
  if (side != 0) {
    decide(side > 0 ? "hueco a la izquierda al salir -> entra" : "hueco a la derecha al salir -> entra");
    robot.turnCorner(90.0f * side);
  } else {
    decide("sin hueco al salir -> media vuelta");
    robot.turnDegrees(180.0f);
  }
}

// side: +1 sigue la pared izquierda, -1 la derecha. Con maxMm > 0 para tras recorrer esa distancia.
void followWall(int side, float maxMm) {
  const bool right = side < 0;
  const float start = robot.travelledMm();
  while (!robot.aborted() && (maxMm <= 0 || robot.travelledMm() - start < maxMm)) {
    const RobotHW::DriveEvent ev = robot.drive(1e6f, side);
    if (ev == RobotHW::DriveEvent::SideOpen) {
      // Hueco en la pared que se sigue: llevar el eje de las ruedas al centro del hueco, girar y entrar.
      // El laser lateral va SIDE_SENSOR_AHEAD_MM por delante del eje y el hueco mide como el pasillo.
      const float half = 0.5f * robot.corridorMm();
      robot.drive(fmaxf(cfg::SIDE_SENSOR_AHEAD_MM + half - cfg::OPEN_CONFIRM_MM, 0.0f), 0);
      decide(side < 0 ? "hueco en la pared derecha -> entra" : "hueco en la pared izquierda -> entra");
      robot.turnCorner(90.0f * side);
      // Entrar hasta que el laser lateral quede dentro del pasillo nuevo.
      robot.drive(fmaxf(half - cfg::SIDE_SENSOR_AHEAD_MM + 15.0f, 15.0f), 0);
      robot.ignoreSidesFor(half + cfg::SIDE_SENSOR_AHEAD_MM);
    } else if (ev == RobotHW::DriveEvent::FrontWall && !frontBlocked()) {
      unstick();
    } else if (ev == RobotHW::DriveEvent::CrashLeft || ev == RobotHW::DriveEvent::CrashRight) {
      // Pegado a un lateral: un poco atras y enderezarse hacia el otro lado.
      decide(ev == RobotHW::DriveEvent::CrashLeft ? "roza la pared izquierda -> endereza a la derecha"
                                                  : "roza la pared derecha -> endereza a la izquierda");
      robot.backUp(cfg::NUDGE_BACK_MM);
      robot.turnDegrees(ev == RobotHW::DriveEvent::CrashLeft ? -cfg::NUDGE_DEG : cfg::NUDGE_DEG);
    } else if (ev == RobotHW::DriveEvent::FrontWall) {
      const raton::Walls w = robot.readWalls();
      const bool followOpen = right ? !w.right : !w.left;
      const bool otherOpen = right ? !w.left : !w.right;
      // Retroceder lo justo para que las esquinas del raton no toquen la pared al girar.
      const int front = robot.frontMm();
      const float clearance = front >= 0 ? static_cast<float>(robot.frontTurnMm() - front) : 0.0f;
      robot.backUp(fmaxf(cfg::BACKUP_MM, clearance));
      if (followOpen) {
        decide(right ? "pared delante, derecha libre -> gira derecha" : "pared delante, izquierda libre -> gira izquierda");
        robot.turnCorner(90.0f * side);
      } else if (otherOpen) {
        decide(right ? "pared delante y a la derecha -> gira izquierda" : "pared delante y a la izquierda -> gira derecha");
        robot.turnCorner(-90.0f * side);
      } else {
        deadEnd();
      }
      robot.ignoreSidesFor(0.5f * robot.corridorMm() + cfg::SIDE_SENSOR_AHEAD_MM);
    }
  }
}

// Espacio libre: avanza recto vigilando los dos lados. Al ver terminar una pared lateral (hueco) o al
// llegar a una pared delante, lleva el eje de ruedas al centro del cruce, se para, mira los tres
// laseres quieto y va hacia donde haya mas espacio libre (sin lectura = libre). Tecnica de micromouse:
// detectar el borde de la pared, avanzar una distancia medida y decidir parado.
void followWall(int side, float maxMm);

void freeSpace() {
  auto space = [](int mm) { return mm < 0 ? 10000 : mm; };  // sin lectura = muy lejos
  float lastTurnAt = robot.travelledMm();
  int turnsInPlace = 0;  // giros seguidos sin avanzar: esta dando vueltas en el mismo sitio
  int stuck = 0;         // atascos seguidos con el frente libre
  while (!robot.aborted()) {
    if (turnsInPlace >= cfg::LOOP_TURNS) {
      // Atrapado girando en un cruce: seguir la pared derecha un tramo para salir de ahi.
      telemetry::println("BUCLE: sigue la pared un tramo");
      followWall(-1, cfg::LOOP_ESCAPE_MM);
      turnsInPlace = 0;
      lastTurnAt = robot.travelledMm();
      continue;
    }
    const RobotHW::DriveEvent ev = robot.drive(1e6f, RobotHW::kWatchBoth);
    if (ev == RobotHW::DriveEvent::CrashLeft || ev == RobotHW::DriveEvent::CrashRight) {
      decide(ev == RobotHW::DriveEvent::CrashLeft ? "roza la pared izquierda -> endereza a la derecha"
                                                  : "roza la pared derecha -> endereza a la izquierda");
      robot.backUp(cfg::NUDGE_BACK_MM);
      robot.turnDegrees(ev == RobotHW::DriveEvent::CrashLeft ? -cfg::NUDGE_DEG : cfg::NUDGE_DEG);
      continue;
    }
    if (ev != RobotHW::DriveEvent::SideOpen && ev != RobotHW::DriveEvent::FrontWall) continue;
    if (ev == RobotHW::DriveEvent::FrontWall && !frontBlocked()) {
      // Atasco con el frente libre: desatascar; a la tercera seguida, salir marcha atras.
      if (++stuck < 3) {
        unstick();
      } else {
        stuck = 0;
        deadEnd();
      }
      continue;
    }
    stuck = 0;

    const float half = 0.5f * robot.corridorMm();
    if (ev == RobotHW::DriveEvent::SideOpen) {
      // El laser lateral va por delante del eje: avanzar hasta que el eje quede en el centro del hueco.
      robot.drive(fmaxf(cfg::SIDE_SENSOR_AHEAD_MM + half - cfg::OPEN_CONFIRM_MM, 0.0f), 0);
    }

    // Parado en el cruce: leer los tres laseres con medidas nuevas.
    robot.setState("CRUCE");
    lasers.waitFresh(3);
    // Sin lectura justo despues de una lectura muy corta = demasiado cerca para el laser, no libre.
    auto side = [&](Lasers::Id id) {
      const int mm = lasers.mm(id);
      if (mm < 0 && lasers.lastValidMm(id) < cfg::SIDE_CLOSE_MM && lasers.lastValidAgeMs(id) < 300) {
        return lasers.lastValidMm(id);
      }
      return space(mm);
    };
    const int left = side(Lasers::LEFT);
    const int right = side(Lasers::RIGHT);
    const int frontRaw = robot.frontMm();
    const int front = space(frontRaw);
    const bool leftOpen = left > robot.sideWall();
    const bool rightOpen = right > robot.sideWall();
    const bool frontOpen = front > cfg::FRONT_WALL_MM;
    char line[48];
    snprintf(line, sizeof(line), "CRUCE izq%d del%d der%d", left, front, right);
    telemetry::println(line);

    // Decision ponderada (una neurona por direccion): espacio libre, novedad, coste de girar y sesgo.
    const float* w = robot.weights();
    auto score = [&](int mm, float turnDeg) {
      const float freeN = (mm > 300 ? 300 : mm) / 300.0f;
      const float novel = 1.0f / (1.0f + robot.visitsToward(turnDeg));
      return w[0] * freeN + w[1] * novel - w[2] * fabsf(turnDeg) / 90.0f + (turnDeg == 0.0f ? w[3] : 0.0f);
    };
    const float sF = frontOpen ? score(front, 0) : -99, sL = leftOpen ? score(left, 90) : -99,
                sR = rightOpen ? score(right, -90) : -99;
    int turn = 180;  // grados: +90 izquierda, -90 derecha, 0 recto, 180 media vuelta (callejon)
    float best = -50;
    if (sF > best) { best = sF; turn = 0; }
    if (sL > best) { best = sL; turn = 90; }
    if (sR > best) { best = sR; turn = -90; }
    char why[80];
    snprintf(why, sizeof(why), "cruce izq %d recto %d der %d (x100) -> %s", static_cast<int>(sL * 100),
             static_cast<int>(sF * 100), static_cast<int>(sR * 100),
             turn == 0 ? "recto" : turn == 90 ? "izquierda" : turn == -90 ? "derecha" : "callejon");

    if (turn == 0) {
      decide(why);
      // Recto: pasar el hueco sin volver a detectarlo.
      robot.drive(half + cfg::SIDE_SENSOR_AHEAD_MM, 0);
      robot.ignoreSidesFor(half);
      continue;
    }
    if (frontRaw >= 0) {
      const float clearance = static_cast<float>(robot.frontTurnMm() - frontRaw);
      if (clearance > 0) robot.backUp(clearance);
    }
    turnsInPlace = robot.travelledMm() - lastTurnAt < cfg::LOOP_MIN_TRAVEL_MM ? turnsInPlace + 1 : 1;
    lastTurnAt = robot.travelledMm();
    if (turn == 180) {
      deadEnd();
    } else {
      decide(why);
      robot.turnCorner(static_cast<float>(turn));
    }
    if (turn != 180) {
      // Entrar en el pasillo nuevo hasta que los laterales vean sus paredes.
      robot.drive(fmaxf(half - cfg::SIDE_SENSOR_AHEAD_MM + 15.0f, 15.0f), 0);
    }
    robot.ignoreSidesFor(half + cfg::SIDE_SENSOR_AHEAD_MM);
  }
}

// Flood fill: explora hasta probar el camino optimo y luego hace carreras rapidas. Necesita
// MAZE_WIDTH, MAZE_HEIGHT y CELL_MM de la pista; sale de (START_X, START_Y) mirando al norte.
void solveMaze() {
  robot.setExpectedMap(nullptr);  // la puerta del mapa solo tiene sentido en el modo 4
  solver.setStart(raton::Cell{static_cast<int8_t>(cfg::START_X), static_cast<int8_t>(cfg::START_Y)});
  if (robot.goalX() >= 0) {
    solver.setGoal(raton::Cell{static_cast<int8_t>(robot.goalX()), static_cast<int8_t>(robot.goalY())});
  }
  solver.reset();
  if (!solver.explore(cfg::MAX_SEARCH_RUNS)) {
    robot.setState("FALLO");
    display.show("LABERINTO", "Exploracion fallo");
    return;
  }
  solver.face(raton::NORTH);
  while (!robot.aborted()) {
    robot.setState("ESPERA");
    display.show("LABERINTO", "Mapa listo", "Carrera rapida...");
    robot.idle(cfg::RUN_PAUSE_MS);
    if (!solver.speedRun() || !solver.returnToStart()) {
      robot.setState("FALLO");
      display.show("LABERINTO", "Carrera fallo");
      return;
    }
    solver.face(raton::NORTH);
  }
}

// Ruta de la competencia: carga el laberinto del reglamento (competition_maze.h) y va por el camino
// mas corto. En cada celda sigue mirando las paredes: si alguna no coincide con el dibujo, corrige
// el mapa y recalcula, asi llega aunque el laberinto real cambie un poco.
bool competitionWall(int x, int y, int dir) {
  using namespace competition;
  const int r = HEIGHT - 1 - y;
  switch (dir) {
    case raton::NORTH: return MAP[2 * r][4 * x + 2] == '-';
    case raton::SOUTH: return MAP[2 * r + 2][4 * x + 2] == '-';
    case raton::WEST: return MAP[2 * r + 1][4 * x] == '|';
    default: return MAP[2 * r + 1][4 * x + 4] == '|';
  }
}

void competitionRun() {
  using namespace competition;
  robot.setExpectedMap(competitionWall);
  solver.setStart(raton::Cell{START_X, START_Y});
  solver.setGoal(raton::Cell{GOAL_X, GOAL_Y});
  solver.reset(WIDTH, HEIGHT);
  for (int r = 0; r < HEIGHT; ++r) {
    const int y = HEIGHT - 1 - r;  // el dibujo empieza por la fila de arriba
    const char* north = MAP[2 * r];
    const char* cells = MAP[2 * r + 1];
    const char* south = MAP[2 * r + 2];
    for (int x = 0; x < WIDTH; ++x) {
      const raton::Cell c{static_cast<int8_t>(x), static_cast<int8_t>(y)};
      solver.setKnownWall(c, raton::NORTH, north[4 * x + 2] == '-');
      solver.setKnownWall(c, raton::SOUTH, south[4 * x + 2] == '-');
      solver.setKnownWall(c, raton::WEST, cells[4 * x] == '|');
      solver.setKnownWall(c, raton::EAST, cells[4 * x + 4] == '|');
    }
  }
  display.show("RUTA COMPETENCIA", "Camino conocido", "a la meta 4,2");
  if (solver.searchToGoal()) {
    robot.setState("META");
    display.show("META!", kModeNames[4]);
    telemetry::println("META");
  } else {
    robot.setState("FALLO");
    display.show("RUTA COMPETENCIA", "No pudo llegar");
  }
}

}  // namespace

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);  // sin esperar al USB: arranca al encender

  motors.begin();
  motors.coast();
  encoders.begin();
  delay(cfg::POWER_SETTLE_MS);  // al encender con bateria los sensores tardan en estar listos
  Wire.begin();
  Wire.setClock(400000);

  if (!display.begin()) Serial.println("Aviso: OLED no encontrada");
  bool ok = false;
  for (int i = 0; i < cfg::SENSOR_INIT_TRIES && !ok; ++i) {
    ok = lasers.begin();
    if (!ok) delay(100);
  }
  if (!ok) fail("Falla un laser");
  ok = false;
  for (int i = 0; i < cfg::SENSOR_INIT_TRIES && !ok; ++i) {
    ok = gyro.begin();
    if (!ok) delay(100);
  }
  if (!ok) fail("Falla el IMU");
  if (!telemetry::begin()) Serial.println("Aviso: Bluetooth no disponible");
}

void loop() {
  static bool firstRun = true;
  static int mode = 1;
  if (firstRun && !cfg::WAIT_FOR_WEB_START) {
    mode = selectMode();
  } else {
    mode = waitForStart(mode);
  }
  robot.clearAbort();
  robot.takeStart();  // descarta un 's' repetido
  waitAutoStart(kModeNames[mode], cfg::START_DELAY_MS);
  if (firstRun) {
    // La revision (motores/laseres cruzados, distancia entre ruedas) solo al encender.
    char check[22];
    robot.selfCheck(check);
    Serial.println(check);
    telemetry::println(check);
    char eje[22];
    snprintf(eje, sizeof(eje), "Eje %d mm", static_cast<int>(robot.trackWidthMm()));
    display.show(kModeNames[mode], check, eje);
    firstRun = false;
  }
  robot.resetOdometry();  // el mapa empieza donde arranca
  telemetry::println("INICIO_MAPA");

  if (mode == 5) {
    freeSpace();
  } else if (mode == 4) {
    competitionRun();
  } else if (mode == 3) {
    solveMaze();
  } else {
    followWall(mode == 1 ? -1 : 1, 0);
  }
  motors.coast();
}

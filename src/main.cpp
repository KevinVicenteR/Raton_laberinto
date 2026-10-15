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
#include "telemetry.h"

namespace {

Motors motors;
Encoders encoders;
Lasers lasers;
Gyro gyro;
Display display;
RobotHW robot(motors, encoders, lasers, gyro, display);

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

// Devuelve true si llega a la meta (cuando hay meta: robot.setGoal).
bool freeSpace() {
  auto space = [](int mm) { return mm < 0 ? 10000 : mm; };  // sin lectura = muy lejos
  float lastTurnAt = robot.travelledMm();
  int turnsInPlace = 0;  // giros seguidos sin avanzar: esta dando vueltas en el mismo sitio
  int stuck = 0;         // atascos seguidos con el frente libre
  while (!robot.aborted()) {
    if (robot.goalReached()) return true;
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

    // Puntuacion lineal por direccion (predictor de riesgo separado): espacio libre, novedad, coste de girar y sesgo.
    const float* w = robot.weights();
    auto score = [&](int mm, float turnDeg) {
      const float freeN = (mm > 300 ? 300 : mm) / 300.0f;
      const float novel = 1.0f / (1.0f + robot.visitsToward(turnDeg));
      // + direccion hacia la meta (si la hay) - salidas ya usadas de este cruce (memoria Tremaux)
      return w[0] * freeN + w[1] * novel - w[2] * fabsf(turnDeg) / 90.0f + (turnDeg == 0.0f ? w[3] : 0.0f) +
             cfg::W_GOAL * robot.goalAlignment(turnDeg) - cfg::W_EXIT * robot.exitsToward(turnDeg);
    };
    const float sF = frontOpen && robot.actionAllowed(0) ? score(front, 0) : -99,
                sL = leftOpen && robot.actionAllowed(90) ? score(left, 90) : -99,
                sR = rightOpen && robot.actionAllowed(-90) ? score(right, -90) : -99;
    if ((frontOpen || leftOpen || rightOpen) && sF == -99 && sL == -99 && sR == -99) {
      robot.abort();
      robot.setState("RIESGO");
      decide("salidas rechazadas por riesgo; no hacer media vuelta a ciegas");
      return false;
    }
    int turn = 180;  // grados: +90 izquierda, -90 derecha, 0 recto, 180 media vuelta (callejon)
    float best = -50;
    if (sF > best) { best = sF; turn = 0; }
    if (sL > best) { best = sL; turn = 90; }
    if (sR > best) { best = sR; turn = -90; }
    char why[80];
    snprintf(why, sizeof(why), "cruce izq %d recto %d der %d (x100) -> %s", static_cast<int>(sL * 100),
             static_cast<int>(sF * 100), static_cast<int>(sR * 100),
             turn == 0 ? "recto" : turn == 90 ? "izquierda" : turn == -90 ? "derecha" : "callejon");

    if (turn != 180) robot.markExit(static_cast<float>(turn));
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
  return robot.goalReached();
}

// Meta de la pista en coordenadas de odometria: el arranque es el eje del raton centrado en la salida
// (START_X, START_Y) mirando al norte; la meta es la celda (gx, gy) con CELL_MM entre celdas.
void setGoalCell(int gx, int gy) {
  robot.setGoal((gx - cfg::START_X) * cfg::CELL_MM, (gy - cfg::START_Y) * cfg::CELL_MM + cfg::AXLE_BEHIND_CENTER_MM);
}

// ---------------------------------------------------------------- laberinto por celdas (modos 3 y 4)
// Mapa propio: bit d de gWalls = pared hacia d (N=0 E=1 S=2 W=3). gSeen = celda leida con los laseres.
// En 14 cm el raton no cabe girando 180: de los callejones sale marcha atras hasta poder girar.
const int MW = cfg::MAZE_WIDTH;
const int MH = cfg::MAZE_HEIGHT;
static_assert(competition::WIDTH <= cfg::MAZE_WIDTH && competition::HEIGHT <= cfg::MAZE_HEIGHT,
              "el mapa de la competencia no cabe en MAZE_WIDTH x MAZE_HEIGHT");
uint8_t gWalls[MW][MH];
bool gSeen[MW][MH];
uint8_t gVisits[MW][MH];
uint8_t gDist[MW][MH];
bool gMapReady = false;  // modo 3: ya llego una vez a la meta; el siguiente arranque es carrera rapida

bool inside(raton::Cell c) { return c.x >= 0 && c.y >= 0 && c.x < MW && c.y < MH; }

bool mapWall(raton::Cell c, int d) {
  if (!inside(raton::step(c, static_cast<raton::Dir>(d)))) return true;
  return (gWalls[c.x][c.y] >> d) & 1;
}

void setMapWall(raton::Cell c, int d, bool on) {
  const raton::Cell n = raton::step(c, static_cast<raton::Dir>(d));
  const uint8_t bit = 1 << d, back = 1 << ((d + 2) & 3);
  gWalls[c.x][c.y] = on ? (gWalls[c.x][c.y] | bit) : (gWalls[c.x][c.y] & ~bit);
  if (inside(n)) gWalls[n.x][n.y] = on ? (gWalls[n.x][n.y] | back) : (gWalls[n.x][n.y] & ~back);
}

// Vacia el mapa; con prior, parte de ese dibujo (lo que se vea en cada celda manda sobre el).
void clearMap(bool (*prior)(int, int, int)) {
  memset(gWalls, 0, sizeof(gWalls));
  memset(gSeen, 0, sizeof(gSeen));
  memset(gVisits, 0, sizeof(gVisits));
  if (!prior) return;
  for (int x = 0; x < MW; ++x) {
    for (int y = 0; y < MH; ++y) {
      for (int d = 0; d < 4; ++d) {
        if (prior(x, y, d)) gWalls[x][y] |= 1 << d;
      }
    }
  }
}

// Inundacion desde la meta (lo desconocido cuenta como abierto).
void flood(raton::Cell goal) {
  memset(gDist, 255, sizeof(gDist));
  static raton::Cell queue[MW * MH];
  int head = 0, tail = 0;
  gDist[goal.x][goal.y] = 0;
  queue[tail++] = goal;
  while (head < tail) {
    const raton::Cell c = queue[head++];
    for (int d = 0; d < 4; ++d) {
      if (mapWall(c, d)) continue;
      const raton::Cell n = raton::step(c, static_cast<raton::Dir>(d));
      if (gDist[n.x][n.y] != 255) continue;
      gDist[n.x][n.y] = gDist[c.x][c.y] + 1;
      queue[tail++] = n;
    }
  }
}

uint8_t distAt(raton::Cell c) { return inside(c) ? gDist[c.x][c.y] : 255; }

void senseHere(raton::Cell pos, raton::Dir heading) {
  const raton::Walls w = robot.senseCell();
  setMapWall(pos, heading, w.front);
  setMapWall(pos, raton::leftOf(heading), w.left);
  setMapWall(pos, raton::rightOf(heading), w.right);
  gSeen[pos.x][pos.y] = true;
}

// Va a la meta por celdas: lee las paredes de cada celda nueva, recalcula la inundacion y avanza por
// tramos rectos (los ya vistos de una vez). Devuelve true al llegar.
bool exploreTo(raton::Cell& pos, raton::Dir& heading, raton::Cell goal) {
  while (!robot.aborted() && !(pos.x == goal.x && pos.y == goal.y)) {
    if (!gSeen[pos.x][pos.y]) senseHere(pos, heading);
    if (gVisits[pos.x][pos.y] < 255) gVisits[pos.x][pos.y]++;
    flood(goal);
    const uint8_t here = gDist[pos.x][pos.y];
    if (here == 255) {
      decide("no hay camino a la meta con las paredes vistas");
      return false;
    }
    // Siguiente direccion: baja la distancia; en empate recto, luego giro, y marcha atras lo ultimo;
    // entre iguales, la celda menos visitada.
    int best = -1, bestKey = 1 << 30;
    for (int d = 0; d < 4; ++d) {
      const raton::Cell n = raton::step(pos, static_cast<raton::Dir>(d));
      if (mapWall(pos, d) || distAt(n) >= here) continue;
      const int diff = (d - heading + 4) & 3;
      if(diff!=2 && !robot.actionAllowed(diff==0?0:(diff==1?-90:90)))continue;
      const int key = (diff == 0 ? 0 : (diff == 2 ? 2 : 1)) * 1000 + gVisits[n.x][n.y];
      if (key < bestKey) {
        bestKey = key;
        best = d;
      }
    }
    if (best < 0) return false;
    const raton::Dir dir = static_cast<raton::Dir>(best);
    const int diff = (dir - heading + 4) & 3;
    if (diff == 2) {
      // Callejon: marcha atras una celda (parando antes si la de llegada tiene pared detras).
      const raton::Cell next = raton::step(pos, dir);
      bool stuck = false;
      const float shortMm = mapWall(next, dir) ? cfg::REVERSE_SHORT_MM : 0.0f;
      if (robot.reverseCells(1, shortMm, &stuck) > 0) {
        pos = next;
      } else if (stuck) {
        setMapWall(pos, dir, true);
      }
      decide("callejon: marcha atras una celda");
      continue;
    }
    // Celdas seguidas por terreno ya visto (de una vez, mas rapido).
    int cells = 1;
    raton::Cell c = raton::step(pos, dir);
    while (gSeen[c.x][c.y] && !(c.x == goal.x && c.y == goal.y) && !mapWall(c, dir) &&
           distAt(raton::step(c, dir)) < gDist[c.x][c.y]) {
      c = raton::step(c, dir);
      cells++;
    }
    if (diff == 1 || diff == 3) robot.turnCorner(diff == 1 ? -90.0f : 90.0f, true);
    if (robot.aborted()) return false;
    heading = dir;
    bool unexpected = false;
    const int moved = robot.runCells(cells, cfg::EXPLORE_SPEED, &unexpected);
    for (int i = 0; i < moved; ++i) pos = raton::step(pos, dir);
    char line[64];
    snprintf(line, sizeof(line), "%d celdas al %c -> %d,%d (dist %d)", moved, raton::dirChar(dir), pos.x, pos.y,
             gDist[pos.x][pos.y]);
    decide(line);
    if (unexpected) gSeen[pos.x][pos.y] = false;  // algo no cuadra: volver a leer esta celda
  }
  return !robot.aborted();
}

// Carrera por el mapa: tramos rectos de una vez a vmax. Devuelve false si algo no coincide (pos queda
// donde se paro) para seguir explorando desde ahi.
bool runRoute(raton::Cell& pos, raton::Dir& heading, raton::Cell goal, float vmax) {
  flood(goal);
  while (!robot.aborted() && gDist[pos.x][pos.y] != 0 && gDist[pos.x][pos.y] != 255) {
    int best = -1;
    for (int d = 0; d < 4 && best < 0; ++d) {
      if (!mapWall(pos, d) && distAt(raton::step(pos, static_cast<raton::Dir>(d))) + 1 == gDist[pos.x][pos.y]) best = d;
    }
    // Recto si es igual de corto (menos giros).
    if (!mapWall(pos, heading) && distAt(raton::step(pos, heading)) + 1 == gDist[pos.x][pos.y]) best = heading;
    if (best < 0) return false;
    const raton::Dir dir = static_cast<raton::Dir>(best);
    int cells = 0;
    raton::Cell c = pos;
    while (gDist[c.x][c.y] != 0 && !mapWall(c, dir) && distAt(raton::step(c, dir)) + 1 == gDist[c.x][c.y]) {
      c = raton::step(c, dir);
      cells++;
    }
    const int diff = (dir - heading + 4) & 3;
    if (diff == 2) return false;  // la ruta mas corta nunca da media vuelta
    if (diff == 1 || diff == 3) {
      // Antes de girar, comprobar que ese lado esta abierto de verdad.
      const raton::Walls w = robot.senseCell();
      if (diff == 1 ? w.right : w.left) {
        setMapWall(pos, dir, true);
        return false;
      }
      robot.turnCorner(diff == 1 ? -90.0f : 90.0f, true);
    }
    if (robot.aborted()) return false;
    heading = dir;
    bool unexpected = false;
    const int moved = robot.runCells(cells, vmax, &unexpected);
    for (int i = 0; i < moved; ++i) pos = raton::step(pos, dir);
    char line[64];
    snprintf(line, sizeof(line), "tramo de %d celdas al %c -> celda %d,%d", moved, raton::dirChar(dir), pos.x, pos.y);
    decide(line);
    if (unexpected || moved < cells) {
      gSeen[pos.x][pos.y] = false;
      return false;
    }
  }
  return gDist[pos.x][pos.y] == 0;
}

raton::Cell goalCell() {
  if (robot.goalX() >= 0) return raton::Cell{static_cast<int8_t>(robot.goalX()), static_cast<int8_t>(robot.goalY())};
  return raton::Cell{static_cast<int8_t>(competition::GOAL_X), static_cast<int8_t>(competition::GOAL_Y)};
}

void reachedGoal(const char* how) {
  robot.setState("META");
  display.show("META!", how);
  telemetry::println("META");
}

// Modo 3, pista desconocida: el primer arranque explora por celdas hasta la meta (y guarda el mapa);
// los siguientes hacen la carrera rapida por el mapa aprendido (si algo cambio, vuelve a explorar).
// Sale de (START_X, START_Y) mirando al norte.
void solveMaze() {
  robot.setExpectedMap(nullptr);
  robot.setGrid(true);
  raton::Cell pos{static_cast<int8_t>(cfg::START_X), static_cast<int8_t>(cfg::START_Y)};
  raton::Dir heading = raton::NORTH;
  const raton::Cell goal = goalCell();
  if (gMapReady) {
    display.show("LABERINTO", "Carrera rapida", "mapa aprendido");
    decide("carrera rapida por el mapa aprendido");
    if (runRoute(pos, heading, goal, cfg::COMP_SPEED)) {
      reachedGoal("carrera rapida");
      robot.setGrid(false);
      return;
    }
    decide("el mapa aprendido no coincide: sigue explorando");
  } else {
    clearMap(nullptr);
    display.show("LABERINTO", "Explorando", "por celdas");
  }
  if (exploreTo(pos, heading, goal)) {
    gMapReady = true;
    reachedGoal("explorando");
  }
  robot.setGrid(false);
}

// Ruta de la competencia: carga el laberinto del reglamento (competition_maze.h) y va por el camino
// mas corto por tramos. Si una pared no coincide con el dibujo, sigue explorando por celdas desde ahi
// (con el dibujo como punto de partida), asi llega aunque el laberinto real cambie.
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
  robot.setExpectedMap(nullptr);
  robot.setGrid(true);
  clearMap(competitionWall);
  raton::Cell pos{START_X, START_Y};
  raton::Dir heading = raton::NORTH;
  const raton::Cell goal{GOAL_X, GOAL_Y};
  display.show("RUTA COMPETENCIA", "Ruta mas corta", "por tramos");
  if (runRoute(pos, heading, goal, cfg::COMP_SPEED)) {
    reachedGoal(kModeNames[4]);
    robot.setGrid(false);
    return;
  }
  if (robot.aborted()) {
    robot.setGrid(false);
    return;
  }
  char line[72];
  snprintf(line, sizeof(line), "el mapa no coincide cerca de %d,%d -> explora por celdas", pos.x, pos.y);
  decide(line);
  if (exploreTo(pos, heading, goal)) reachedGoal("explorando");
  robot.setGrid(false);
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
  robot.clearGoal();      // cada modo pone su meta si la necesita
  telemetry::println("INICIO_MAPA");

  if (mode == 5) {
    // Pista desconocida: con meta (G x,y desde la web) va hacia ella; sin meta, explora.
    if (robot.goalX() >= 0) {
      setGoalCell(robot.goalX(), robot.goalY());
    } else {
      robot.clearGoal();
    }
    if (freeSpace()) {
      robot.setState("META");
      telemetry::println("META");
    }
  } else if (mode == 4) {
    competitionRun();
  } else if (mode == 3) {
    solveMaze();
  } else {
    followWall(mode == 1 ? -1 : 1, 0);
  }
  motors.coast();
}

#include "app.h"

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"
#include "hw/telemetry.h"
#include "raton/frame.h"
#include "raton/maze_text.h"
#include "raton/param_table.h"
#include "storage.h"

namespace {

App* gApp = nullptr;
void serviceTrampoline() {
  if (gApp) gApp->service();
}

const char* const kModeNames[] = {"",          "1 EXPLORAR",  "2 CARRERA", "3 SOLO META",
                                  "4 SENSORES", "5 CAL MOTOR", "6 RUEDAS",  "7 BORRAR MAPA"};
const char* const kModeHelp[] = {"",
                                 "meta+ruta+volver",
                                 "ruta guardada",
                                 "ir a la meta",
                                 "ver lecturas",
                                 "pasillo recto 2 celdas",
                                 "raton levantado",
                                 "olvida el mapa"};

// printf de la placa sin coma flotante: valor con 4 decimales significativos a mano.
const char* fmt(float v, char* buf, int size) {
  const bool neg = v < 0;
  if (neg) v = -v;
  long ip = static_cast<long>(v);
  long fp = static_cast<long>((v - ip) * 100000.0f + 0.5f);
  if (fp >= 100000) {
    ++ip;
    fp -= 100000;
  }
  snprintf(buf, size, "%s%ld.%05ld", neg ? "-" : "", ip, fp);
  return buf;
}

const int kHandNear = 60;   // mm: mano delante del frontal
const int kHandFar = 150;   // mm: mano retirada

}  // namespace

App::App()
    : board_(motors_, encoders_, lasers_, imu_, params_), mouse_(board_, params_) {
  gApp = this;
}

void App::fail(const char* msg) {
  // Sin parar el Bluetooth: la web sigue conectada y ve el error (y puede reiniciar la placa con R).
  motors_.coast();
  busy_ = true;
  display_.show("ERROR", msg, "", "Revisa cables y", "reinicia (R)");
  char line[64];
  snprintf(line, sizeof(line), "ERROR_ARRANQUE %s", msg);
  uint32_t last = 0;
  while (true) {
    service();
    if (millis() - last > 1000) {
      last = millis();
      reply(line);
    }
    digitalWrite(LED_BUILTIN, (millis() / 150) % 2);
    delay(5);
  }
}

void App::setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);  // sin esperar al USB: arranca con bateria
  storage::loadParams(params_);
  motors_.begin();
  motors_.coast();
  encoders_.begin();
  // Bluetooth lo primero: si luego falla un sensor, la web se puede conectar y ver el error.
  telemetry::begin();
  board_.service = serviceTrampoline;
  const uint32_t t0 = millis();
  while (millis() - t0 < 300) service();  // margen para que laseres, IMU y OLED se enciendan
  Wire.begin();
  Wire.setClock(400000);
  display_.begin();
  bool ok = false;
  for (int i = 0; i < 5 && !ok; ++i) {
    ok = lasers_.begin();
    service();
    if (!ok) delay(100);
  }
  char line[64];
  snprintf(line, sizeof(line), "SENSORES izq=%s frente=%s der=%s", lasers_.ok(0) ? "ok" : "FALLA",
           lasers_.ok(1) ? "ok" : "FALLA", lasers_.ok(2) ? "ok" : "FALLA");
  reply(line);
  if (!ok) fail(!lasers_.ok(0) ? "Laser izquierdo" : (!lasers_.ok(1) ? "Laser frontal" : "Laser derecho"));
  ok = false;
  for (int i = 0; i < 5 && !ok; ++i) {
    ok = imu_.begin();
    service();
    if (!ok) delay(100);
  }
  if (!ok) fail("IMU (giroscopio)");
  mouse_.resetMap();
  if (storage::loadMap(mouse_.maze()) && mouse_.maze().width() == params_.mazeW &&
      mouse_.maze().height() == params_.mazeH) {
    mouse_.maze().setGoal(params_.goalX, params_.goalY, params_.goalW, params_.goalH);
    reply("MAPA_CARGADO");
  } else {
    mouse_.resetMap();
  }
  snprintf(line, sizeof(line), "LISTO giroscopio %d Hz", static_cast<int>(imu_.odrHz()));
  reply(line);
}

// ---------------------------------------------------------------------------------------------
// Comunicaciones
// ---------------------------------------------------------------------------------------------

void App::reply(const char* line) {
  telemetry::println(line, true);
  if (Serial) Serial.println(line);
}

void App::service() {
  telemetry::poll();
  while (Serial.available()) feed(Serial.read());
  for (int c = telemetry::read(); c >= 0; c = telemetry::read()) feed(c);
}

void App::feed(int c) {
  // 'x' suelta (fuera de una orden) para al momento.
  if ((c == 'x' || c == 'X') && lineLen_ == 0) {
    board_.requestStop();
    motors_.set(0, 0);
    return;
  }
  if (c == '\n' || c == '\r') {
    if (lineLen_ > 0) {
      line_[lineLen_] = 0;
      uint16_t seq = 0;
      char payload[sizeof(line_)];
      char ack[16];
      switch (raton::parseFrame(line_, seq, payload, sizeof(payload))) {
        case raton::FrameResult::Plain:
          command(line_);
          break;
        case raton::FrameResult::Ok:
          snprintf(ack, sizeof(ack), "ACK %u", static_cast<unsigned>(seq));
          telemetry::println(ack, true);
          if (!seqValid_ || seq != lastSeq_) {  // un reintento no se ejecuta dos veces
            lastSeq_ = seq;
            seqValid_ = true;
            command(payload);
          }
          break;
        default:
          snprintf(ack, sizeof(ack), "NAK %u", static_cast<unsigned>(seq));
          telemetry::println(ack, true);
          break;
      }
    }
    lineLen_ = 0;
  } else if (lineLen_ < static_cast<int>(sizeof(line_)) - 1) {
    line_[lineLen_++] = static_cast<char>(c);
  }
}

void App::command(char* cmd) {
  char out[120];
  if (cmd[0] == 'x' && cmd[1] == 0) {
    board_.requestStop();
    return;
  }
  if (!strcmp(cmd, "R")) {
    reply("REINICIO");
    telemetry::flush(300);
    NVIC_SystemReset();
  }
  if (busy_) {
    reply("OCUPADO (x = parar)");
    return;
  }
  if (cmd[0] == 's' && (cmd[1] == 0 || (cmd[1] >= '1' && cmd[1] <= '7' && cmd[2] == 0))) {
    if (cmd[1]) mode_ = cmd[1] - '0';
    startRequested_ = true;
    snprintf(out, sizeof(out), "ARRANCANDO %s", kModeNames[mode_]);
    reply(out);
    return;
  }
  if (cmd[0] == 'm' && cmd[1] >= '1' && cmd[1] <= '7' && cmd[2] == 0) {
    mode_ = cmd[1] - '0';
    snprintf(out, sizeof(out), "MODE %d", mode_);  // la web marca el modo cuando la placa lo confirma
    reply(out);
    snprintf(out, sizeof(out), "MODO %s: %s", kModeNames[mode_], kModeHelp[mode_]);
    reply(out);
    return;
  }
  if (cmd[0] == 'G') {
    // G<x>,<y>: celda de la meta en el laberinto. GW<x>,<y>: en el mapa aprendido de la web, donde la
    // salida aparece en (20,20).
    const bool web = cmd[1] == 'W';
    const char* comma = strchr(cmd, ',');
    int gx = atoi(cmd + (web ? 2 : 1));
    int gy = comma ? atoi(comma + 1) : -1;
    if (web) {
      gx += params_.startX - 20;
      gy += params_.startY - 20;
    }
    if (comma && raton::setParam(params_, "goalX", static_cast<float>(gx)) &&
        raton::setParam(params_, "goalY", static_cast<float>(gy)) && gx < params_.mazeW && gy < params_.mazeH) {
      mouse_.maze().setGoal(params_.goalX, params_.goalY, params_.goalW, params_.goalH);
      snprintf(out, sizeof(out), "META %d,%d (save para guardarla)", gx, gy);
    } else {
      snprintf(out, sizeof(out), "META NO VALIDA %d,%d: el laberinto es de %d x %d", gx, gy, params_.mazeW,
               params_.mazeH);
    }
    reply(out);
    return;
  }
  // Ordenes de la web anterior.
  if (!strcmp(cmd, "ERASE_MAP")) cmd = const_cast<char*>("erase");
  else if (!strcmp(cmd, "LOAD_MAP")) cmd = const_cast<char*>("map");
  else if (!strcmp(cmd, "?")) cmd = const_cast<char*>("params");
  else if (!strcmp(cmd, "T")) {
    wheelTest();
    return;
  } else if (cmd[0] == 'K') {
    if (atof(cmd + 1) >= 100) calibrateSides(static_cast<float>(atof(cmd + 1)));
    else calibrateFront();
    return;
  }
  char name[32];
  char num[24];
  float value;
  if (!strncmp(cmd, "set ", 4)) {
    const char* p = cmd + 4;
    int n = 0;
    while (*p && *p != ' ' && n < 31) name[n++] = *p++;
    name[n] = 0;
    value = static_cast<float>(atof(p));
    if (*p == ' ' && raton::setParam(params_, name, value)) {
      snprintf(out, sizeof(out), "OK %s=%s", name, fmt(value, num, sizeof(num)));
      // El tamano o la meta cambian el mapa.
      if (!strcmp(name, "mazeW") || !strcmp(name, "mazeH")) mouse_.resetMap();
      mouse_.maze().setGoal(params_.goalX, params_.goalY, params_.goalW, params_.goalH);
    } else {
      snprintf(out, sizeof(out), "NO: %s fuera de rango o desconocido", name);
    }
    reply(out);
    return;
  }
  if (!strncmp(cmd, "get ", 4)) {
    const int i = raton::findParam(cmd + 4);
    if (i >= 0) snprintf(out, sizeof(out), "%s=%s", cmd + 4, fmt(raton::getParam(params_, i), num, sizeof(num)));
    else snprintf(out, sizeof(out), "NO existe %s", cmd + 4);
    reply(out);
    return;
  }
  if (!strcmp(cmd, "params")) {
    for (int i = 0; i < raton::paramCount(); ++i) {
      snprintf(out, sizeof(out), "%s=%s", raton::paramInfo(i).name, fmt(raton::getParam(params_, i), num, sizeof(num)));
      reply(out);
      service();
    }
    return;
  }
  if (!strcmp(cmd, "save")) {
    reply(storage::saveParams(params_) ? "PARAMETROS_GUARDADOS" : "ERROR_FLASH");
    return;
  }
  if (!strcmp(cmd, "defaults")) {
    params_ = raton::Params();
    storage::eraseParams();
    reply("PARAMETROS_DE_FABRICA");
    return;
  }
  if (!strcmp(cmd, "map")) {
    printMap();
    return;
  }
  if (!strcmp(cmd, "erase")) {
    storage::eraseMap();
    mouse_.resetMap();
    reply("MAPA_BORRADO");
    return;
  }
  if (!strcmp(cmd, "calfront")) {
    calibrateFront();
    return;
  }
  if (!strncmp(cmd, "calside ", 8)) {
    calibrateSides(static_cast<float>(atof(cmd + 8)));
    return;
  }
  if (!strcmp(cmd, "R")) {
    reply("REINICIO");
    telemetry::flush(300);
    NVIC_SystemReset();
  }
  if (!strcmp(cmd, "estado")) {
    snprintf(out, sizeof(out), "ciclo max %d us, tarde %lu, giroscopio %d Hz, escala %d/1000", static_cast<int>(board_.maxLoadUs()),
             static_cast<unsigned long>(board_.lateTicks()), static_cast<int>(imu_.odrHz()),
             static_cast<int>(mouse_.navigator().gyroScale() * 1000));
    reply(out);
    return;
  }
  snprintf(out, sizeof(out), "? orden desconocida: %s", cmd);
  reply(out);
  reply("ordenes: s s<n> x m<n> G<x>,<y> set <p> <v> get <p> params save defaults map erase calfront calside <mm> estado R");
}

// ---------------------------------------------------------------------------------------------
// Menu
// ---------------------------------------------------------------------------------------------

int App::menu() {
  // Pasar la mano delante del frontal (cerca y retirar) = siguiente modo.
  // Dejarla mas de 1 s = arrancar al retirarla.
  enum { FAR, NEAR } hand = FAR;
  uint32_t nearSince = 0;
  uint32_t lastShow = 0, lastMode = 0;
  bool armed = false;
  startRequested_ = false;
  board_.clearStop();
  while (true) {
    const raton::Sensors& s = board_.tick();
    if (startRequested_) return mode_;
    const int f = s.front.mm;
    const uint32_t now = millis();
    if (s.front.fresh) {
      if (hand == FAR && f >= 0 && f < kHandNear) {
        hand = NEAR;
        nearSince = now;
      } else if (hand == NEAR && (f < 0 || f > kHandFar)) {
        hand = FAR;
        if (armed) return mode_;
        mode_ = mode_ % kModes + 1;
        lastShow = 0;
      }
      armed = hand == NEAR && now - nearSince > 1000;
      if (hand == FAR && lastShow == 0) lastMode = 0;
    }
    digitalWrite(LED_BUILTIN, armed ? 1 : static_cast<int>((now / 500) % 2));
    if (now - lastShow > 400) {
      lastShow = now;
      char l3[22];
      snprintf(l3, sizeof(l3), "I%d F%d D%d", s.left.mm, s.front.mm, s.right.mm);
      char l4[22];
      snprintf(l4, sizeof(l4), "Meta %d,%d", params_.goalX, params_.goalY);
      display_.show(kModeNames[mode_], kModeHelp[mode_], armed ? "SUELTA = ARRANCA" : "mano: cambiar/1s",
                    l3, l4);
    }
    if (now - lastMode > 2000) {  // la web se puede conectar en cualquier momento
      lastMode = now;
      char st[16];
      snprintf(st, sizeof(st), "MODE %d", mode_);
      telemetry::println(st, true);
    }
  }
}

bool App::armStart() {
  // Manos fuera, raton quieto: calibrar el giroscopio y arrancar.
  display_.show(kModeNames[mode_], "", "No tocar...");
  const uint32_t t0 = millis();
  while (millis() - t0 < 700) board_.tick();
  display_.show(kModeNames[mode_], "", "Calibrando giro");
  imu_.calibrate(500);
  board_.clearStop();
  board_.resetLoad();
  display_.show(kModeNames[mode_], "", "EN MARCHA", "x = parar");
  return true;
}

void App::loop() {
  const int mode = menu();
  runMode(mode);
  motors_.coast();
}

void App::showResult(const raton::MissionResult& r, const char* title) {
  char l1[22], l2[22], l3[22], l4[22];
  if (r.reachedGoal) snprintf(l1, sizeof(l1), "Meta %d.%02d s", static_cast<int>(r.goalTime),
                              static_cast<int>(r.goalTime * 100) % 100);
  else snprintf(l1, sizeof(l1), "SIN META");
  snprintf(l2, sizeof(l2), "Total %d s", static_cast<int>(r.totalTime));
  snprintf(l3, sizeof(l3), "%s%s", r.proven ? "Ruta demostrada" : "Ruta parcial", r.backHome ? " +S" : "");
  snprintf(l4, sizeof(l4), "Atascos %d max %dus", r.recoveries, static_cast<int>(board_.maxLoadUs()));
  display_.show(title, l1, l2, l3, l4);
  char line[96];
  snprintf(line, sizeof(line), "RESULTADO meta=%d tiempo=%d ms total=%d s demostrada=%d atascos=%d escapes=%d ciclo_max=%d us",
           r.reachedGoal, static_cast<int>(r.goalTime * 1000), static_cast<int>(r.totalTime), r.proven, r.recoveries,
           r.escapes, static_cast<int>(board_.maxLoadUs()));
  reply(line);
}

void App::runMode(int mode) {
  busy_ = true;
  char line[32];
  snprintf(line, sizeof(line), "START %d", mode);
  reply(line);
  switch (mode) {
    case EXPLORE:
      armStart();
      showResult(mouse_.explore(true, true), "EXPLORAR");
      break;
    case RACE:
      armStart();
      showResult(mouse_.race(true), "CARRERA");
      break;
    case GOAL_ONLY:
      armStart();
      showResult(mouse_.explore(false, false), "SOLO META");
      break;
    case SENSORS:
      sensorsMode();
      break;
    case MOTOR_CAL:
      armStart();
      motorCalibration();
      break;
    case WHEELS:
      wheelTest();
      break;
    case ERASE:
      storage::eraseMap();
      mouse_.resetMap();
      reply("MAPA_BORRADO");
      display_.show("MAPA BORRADO");
      delay(800);
      break;
  }
  motors_.set(0, 0);
  busy_ = false;
}

// ---------------------------------------------------------------------------------------------
// Diagnostico y calibracion
// ---------------------------------------------------------------------------------------------

void App::sensorsMode() {
  // Lecturas en pantalla y por Bluetooth hasta 'x' o tapar el frontal 2 s.
  board_.clearStop();
  uint32_t last = 0, covered = 0;
  while (!board_.stopRequested()) {
    const raton::Sensors& s = board_.tick();
    const uint32_t now = millis();
    if (s.front.fresh) covered = (s.front.mm >= 0 && s.front.mm < kHandNear) ? (covered ? covered : now) : 0;
    if (covered && now - covered > 2000) break;
    if (now - last > 250) {
      last = now;
      char l1[22], l2[22], l3[22], line[96];
      snprintf(l1, sizeof(l1), "Izq %d  Der %d", s.left.mm, s.right.mm);
      snprintf(l2, sizeof(l2), "Frente %d", s.front.mm);
      snprintf(l3, sizeof(l3), "Giro %d", static_cast<int>(s.gyroDeg));
      display_.show("SENSORES", l1, l2, l3, "tapar 2 s = salir");
      snprintf(line, sizeof(line), "SENS I%d F%d D%d giro=%d encI=%d encD=%d", s.left.mm, s.front.mm, s.right.mm,
               static_cast<int>(s.gyroDeg), static_cast<int>(s.encLeftMm), static_cast<int>(s.encRightMm));
      telemetry::println(line, false);
    }
  }
}

void App::calibrateFront() {
  // Raton CENTRADO en una celda mirando a una pared: esa lectura es la de "parar en el centro".
  long sum = 0;
  int n = 0;
  for (int i = 0; i < 40 && n < 15; ++i) {
    const int mm = lasers_.read(Lasers::FRONT);
    if (mm >= 0) {
      sum += mm + params_.frontOffsetMm;
      ++n;
    }
  }
  char out[64];
  if (n < 10) {
    reply("CALIB frontal: sin lecturas");
    return;
  }
  params_.frontStopMm = static_cast<int>(sum / n);
  snprintf(out, sizeof(out), "CALIB frontStopMm=%d (save para guardar)", params_.frontStopMm);
  reply(out);
}

void App::calibrateSides(float corridor) {
  // Raton CENTRADO y recto en un pasillo de ancho conocido con pared a los dos lados.
  long sum[2] = {0, 0};
  int n[2] = {0, 0};
  for (int k = 0; k < 15; ++k) {
    for (int side = 0; side < 2; ++side) {
      const int mm = lasers_.read(side == 0 ? Lasers::LEFT : Lasers::RIGHT);
      if (mm > 0) {
        sum[side] += mm + (side == 0 ? params_.leftOffsetMm : params_.rightOffsetMm);
        ++n[side];
      }
    }
  }
  char out[96];
  if (n[0] < 8 || n[1] < 8) {
    reply("CALIB laterales: hace falta pared a los dos lados");
    return;
  }
  const float rl = static_cast<float>(sum[0]) / n[0], rr = static_cast<float>(sum[1]) / n[1];
  const float gap = 0.5f * corridor - params_.diagLateralMm;
  const float sl = gap / rl, sr = gap / rr;
  if (sl <= 0.2f || sl >= 1.0f || sr <= 0.2f || sr >= 1.0f) {
    reply("CALIB laterales: lecturas imposibles para ese pasillo");
    return;
  }
  const float ang = 0.5f * (asinf(sl) + asinf(sr)) * 57.2958f;
  params_.diagAngleDeg = ang;
  params_.cellMm = corridor + params_.wallMm;
  snprintf(out, sizeof(out), "CALIB I%d D%d -> diagAngleDeg=%d cellMm=%d (save para guardar)", static_cast<int>(rl),
           static_cast<int>(rr), static_cast<int>(ang + 0.5f), static_cast<int>(params_.cellMm));
  reply(out);
}

void App::motorCalibration() {
  // En el suelo, en un pasillo recto de al menos 2 celdas: dos escalones de duty con el rumbo sujeto por
  // el giroscopio. kV y kS salen de las dos velocidades estables; tau del tiempo hasta el 63 % del salto.
  const float d1 = 0.25f, d2 = 0.45f;
  static float speed[300];
  float v1 = 0, v2 = 0;
  int n1 = 0, n2 = 0;
  const raton::Sensors* s = &board_.tick();
  const float yaw0 = s->gyroDeg;
  float hist[5] = {0, 0, 0, 0, 0};
  float lastPos = 0.5f * (s->encLeftMm + s->encRightMm);
  int k = 0;
  for (; k < 600 && !board_.stopRequested(); ++k) {  // 1.2 s
    s = &board_.tick();
    const float d = k < 300 ? d1 : d2;
    const float corr = 0.01f * (yaw0 - s->gyroDeg);
    motors_.set(d - corr, d + corr);
    const float pos = 0.5f * (s->encLeftMm + s->encRightMm);
    hist[k % 5] = (pos - lastPos) / params_.dt;
    lastPos = pos;
    const float v = 0.2f * (hist[0] + hist[1] + hist[2] + hist[3] + hist[4]);  // media de 10 ms
    if (k >= 200 && k < 300) {
      v1 += v;
      ++n1;
    }
    if (k >= 300) speed[k - 300] = v;
    if (k >= 500) {
      v2 += v;
      ++n2;
    }
  }
  motors_.brake();
  delay(200);
  motors_.coast();
  char out[120];
  if (n1 == 0 || n2 == 0) {
    reply("CAL motores: interrumpida");
    return;
  }
  v1 /= n1;
  v2 /= n2;
  if (v2 - v1 < 20) {
    reply("CAL motores: no avanza (atascado o rueda levantada)");
    return;
  }
  float tau = 0;
  const float v63 = v1 + 0.63f * (v2 - v1);
  for (int i = 0; i < 300; ++i) {
    if (speed[i] >= v63) {
      tau = i * params_.dt - 0.004f;  // menos el retraso de la media de 10 ms
      break;
    }
  }
  const float kV = (d2 - d1) / (v2 - v1);
  const float kS = d1 - kV * v1;
  snprintf(out, sizeof(out), "CAL motores v1=%d v2=%d mm/s -> kV=1/%d kS=%d/1000 tau=%d ms", static_cast<int>(v1),
           static_cast<int>(v2), static_cast<int>(1.0f / kV), static_cast<int>(kS * 1000), static_cast<int>(tau * 1000));
  reply(out);
  if (kV > 1.0f / 3000 && kV < 1.0f / 150 && kS >= 0 && kS < 0.3f) {
    params_.kV = kV;
    params_.kS = kS;
    if (tau > 0.005f && tau < 0.3f) params_.motorTauS = tau;
    reply("CAL aplicada (save para guardar)");
  } else {
    reply("CAL fuera de rango: no se aplica");
  }
}

void App::wheelTest() {
  // Raton LEVANTADO: cada rueda sola adelante y atras. Cada linea dice cuanto cuenta cada encoder y lo que
  // gira el giroscopio: "izq adelante" debe dar izq > 0 y der ~ 0.
  struct Step {
    float l, r;
    const char* name;
  } steps[] = {{0.35f, 0, "izq adelante"}, {-0.35f, 0, "izq atras"}, {0, 0.35f, "der adelante"}, {0, -0.35f, "der atras"}};
  for (const Step& st : steps) {
    const long l0 = encoders_.left(), r0 = encoders_.right();
    motors_.set(st.l, st.r);
    delay(500);
    motors_.brake();
    delay(150);
    motors_.coast();
    char line[80];
    snprintf(line, sizeof(line), "RUEDAS %s: izq=%ld der=%ld", st.name, encoders_.left() - l0, encoders_.right() - r0);
    reply(line);
    service();
  }
}

void App::printMap() {
  static char buf[(4 * 32 + 2) * (2 * 32 + 1) + 1];
  raton::printMaze(mouse_.maze(), buf, sizeof(buf), params_.startX, params_.startY);
  char* p = buf;
  while (*p) {
    char* e = strchr(p, '\n');
    if (!e) break;
    *e = 0;
    reply(p);
    service();
    p = e + 1;
  }
  reply("MAPA_LISTO");
}

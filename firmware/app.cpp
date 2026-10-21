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

const char* const kModeNames[] = {"", "1 DESCONOCIDO", "2 MAPEAR Y GUARDAR", "3 APRENDIDO"};
const char* const kModeHelp[] = {"", "buscar meta desde cero", "mapa y ruta mas corta", "resolver mapa guardado"};

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
  startupFailed_ = true;
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
  if (!telemetry::begin() && Serial) Serial.println("ERROR Bluetooth: BLE.begin() fallo; no se anunciara Raton.");
  board_.service = serviceTrampoline;
  const uint32_t t0 = millis();
  while (millis() - t0 < 300) service();  // margen para que laseres, IMU y OLED se enciendan
  if (pins::sidePinsConflict())
    reply("CONFIG_LATERALES_CONFLICTO: XSHUT en pin ocupado");
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
  snprintf(line, sizeof(line), "LATERALES izq=%s der=%s control=%d",
           lasers_.ok(Lasers::SIDE_LEFT) ? "ok" : "no disponible",
           lasers_.ok(Lasers::SIDE_RIGHT) ? "ok" : "no disponible", params_.sideEnabled);
  reply(line);
  if (!lasers_.ok(1)) fail("Laser frontal");
  if (!lasers_.ok(0) && !lasers_.ok(2)) fail("Ambos laterales");
  ok = false;
  for (int i = 0; i < 5 && !ok; ++i) {
    ok = imu_.begin();
    service();
    if (!ok) delay(100);
  }
  if (!ok) fail("IMU (giroscopio)");
  mouse_.navigator().motion().setGyroScale(params_.gyroScale);
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
  if (lineLen_ == 0 && c >= '1' && c <= '3') {
    char digit[2] = {static_cast<char>(c), 0};
    command(digit);
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
  // Prueba sin movimiento disponible tambien si fallo el arranque del hardware.
  if (startupFailed_ && !strcmp(cmd, "pruebasensores")) {
    sensorTest();
    return;
  }
  if (busy_) {
    reply("OCUPADO (x = parar)");
    return;
  }
  if (cmd[0] >= '1' && cmd[0] <= '3' && cmd[1] == 0) {
    mode_ = cmd[0] - '0';
    startRequested_ = true;
    snprintf(out, sizeof(out), "ARRANCANDO %s", kModeNames[mode_]);
    reply(out);
    return;
  }
  if (cmd[0] == 's' && (cmd[1] == 0 || (cmd[1] >= '1' && cmd[1] <= '3' && cmd[2] == 0))) {
    if (cmd[1]) mode_ = cmd[1] - '0';
    startRequested_ = true;
    snprintf(out, sizeof(out), "ARRANCANDO %s", kModeNames[mode_]);
    reply(out);
    return;
  }
  if (cmd[0] == 'm' && cmd[1] >= '1' && cmd[1] <= '3' && cmd[2] == 0) {
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
      mouse_.navigator().motion().loadLearned();
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
    mouse_.navigator().motion().loadLearned();
    mouse_.navigator().invalidatePitch();
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
  if (!strcmp(cmd, "pruebasensores")) {
    requestedMode_ = kSensorTest;
    startRequested_ = true;
    reply("PRUEBA_SENSORES: 8 s sin movimiento; acercar una pared a cada lente");
    return;
  }
  if (!strcmp(cmd, "sensores")) {
    requestedMode_ = kSensors;
    startRequested_ = true;
    reply("DIAGNOSTICO_SENSORES: sin movimiento; x para salir");
    return;
  }
  if (!strcmp(cmd, "calsonda")) {
    // Gira en el sitio: se hace desde el bucle principal, como un modo.
    requestedMode_ = kCalibrate;
    startRequested_ = true;
    reply("CALIBRANDO sensores (raton en el centro de una celda con pared a los dos lados)");
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
  reply("ordenes: 1 2 3 s s<1-3> x m<n> sensores pruebasensores G<x>,<y> set <p> <v> get <p> params save defaults map erase calsonda calfront calside <mm> estado R");
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
    if (startRequested_) {
      const int m = requestedMode_ ? requestedMode_ : mode_;
      requestedMode_ = 0;
      return m;
    }
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
  reply("PREPARANDO: deja el raton quieto");
  const uint32_t t0 = millis();
  while (millis() - t0 < 700) {
    board_.tick();
    if (board_.stopRequested()) {
      reply("ARRANQUE_CANCELADO");
      return false;
    }
  }
  display_.show(kModeNames[mode_], "", "Calibrando giro");
  reply("CALIBRANDO_GIRO");
  bool ready = false;
  for (int attempt = 0; attempt < 3 && !ready; ++attempt) {
    const long l = encoders_.left(), r = encoders_.right();
    const bool gyroOk = imu_.calibrate(600, serviceTrampoline);
    service();
    if (board_.stopRequested()) break;
    ready = gyroOk && labs(encoders_.left() - l) <= 2 && labs(encoders_.right() - r) <= 2;
    if (!ready) reply("CAL_REINTENTO: no mover; comprobar IMU y encoders");
  }
  if (!ready) {
    board_.setMotors(0, 0);
    reply(board_.stopRequested() ? "ARRANQUE_CANCELADO" :
          "CAL_NO_ESTABLE: arranque cancelado; deja quieto y vuelve a iniciar");
    return false;
  }
  service();
  if (board_.stopRequested()) {
    reply("ARRANQUE_CANCELADO");
    return false;
  }
  board_.resetLoad();
  display_.show(kModeNames[mode_], "", "EN MARCHA", "x = parar");
  reply("GIRO_LISTO: iniciando movimiento");
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
  else snprintf(l1, sizeof(l1), r.exploredAll ? "Explorado todo" : "SIN META");
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
    case MAP: {
      if (!armStart()) break;
      // No exigir paredes en la salida: usar los parametros guardados y aprender en marcha.
      reply("INICIANDO_EXPLORACION");
      mouse_.resetMap();
      board_.saveEnabled = mode == MAP;
      const raton::MissionResult result = mode == MAP ? mouse_.mapAll() : mouse_.toGoal();
      showResult(result, mode == MAP ? "MAPEAR" : "DESCONOCIDO");
      const bool completed = mode == MAP ? result.exploredAll && result.backHome : result.reachedGoal;
      if (completed && !mouse_.navigator().sensorFault()) saveLearned();
      else mouse_.navigator().motion().loadLearned();
      board_.saveEnabled = true;
      break;
    }
    case kSensors:
      sensorsMode();
      break;
    case kSensorTest:
      sensorTest();
      break;
    case kCalibrate: {
      if (!armStart()) break;
      const float corridor = mouse_.calibrateSensors(params_, true);
      const bool saved = corridor > 0 && storage::saveParams(params_);
      char out[80];
      snprintf(out, sizeof(out), "CALIB desfases F%d I%d D%d pasillo %d %s", params_.frontOffsetMm,
               params_.leftOffsetMm, params_.rightOffsetMm, static_cast<int>(corridor), saved ? "(guardado)" : "");
      reply(out);
      display_.show(saved ? "CALIBRADO" : "SIN CALIBRAR", saved ? "guardado" : "falta pared");
      delay(800);
      break;
    }
    case RACE: {
      // No usar el mapa temporal del modo 1 ni inventar una ruta si falta el mapa aprendido.
      static raton::Maze learned;
      if (!storage::loadMap(learned) || learned.width() != params_.mazeW || learned.height() != params_.mazeH) {
        reply("SIN_MAPA_GUARDADO: ejecuta el modo 2 primero");
        display_.show("SIN MAPA", "Usa el modo 2");
        break;
      }
      mouse_.maze() = learned;
      if (!armStart()) break;
      const raton::MissionResult result = mouse_.solveLearned();
      showResult(result, "APRENDIDO");
      if (result.reachedGoal && !mouse_.navigator().sensorFault()) saveLearned();
      else mouse_.navigator().motion().loadLearned();
      break;
    }
  }
  motors_.set(0, 0);
  busy_ = false;
}

// ---------------------------------------------------------------------------------------------
// Diagnostico y calibracion
// ---------------------------------------------------------------------------------------------

void App::practiceRun() {
  // Pista de practica (cualquier medida y sin meta): no toca la geometria ni el mapa de la competencia.
  // Mapa de 32 x 32 con la salida en el centro (cabe una pista de hasta 15 celdas hacia cada lado), pasillo
  // medido al arrancar, explora todo lo alcanzable y vuelve a la salida.
  static raton::Maze competitionMap;
  competitionMap = mouse_.maze();
  const raton::Params competition = params_;
  params_.mazeW = params_.mazeH = 32;
  params_.startX = params_.startY = 16;
  params_.startDir = 0;
  params_.goalX = params_.goalY = -1;
  mouse_.resetMap();
  armStart();
  if (mouse_.measureCell(params_) < 0)
    reply("CELDA no medida (hace falta pared a los dos lados en la salida): uso cellMm");
  board_.saveEnabled = false;
  showResult(mouse_.explore(false, true), "PRACTICA");
  mouse_.setLearnCell(false);
  board_.saveEnabled = true;
  // Los desfases son de los sensores, no de la pista: se quedan los recien calibrados.
  const int off[2] = {params_.leftOffsetMm, params_.rightOffsetMm};
  params_ = competition;
  params_.leftOffsetMm = off[0];
  params_.rightOffsetMm = off[1];
  mouse_.maze() = competitionMap;
}

void App::saveLearned() {
  // Lo aprendido en la mision (modelo del motor con la bateria de ahora, escala del giroscopio) pasa a los
  // parametros y se guarda solo: la siguiente vez arranca ya ajustado, sin calibrar a mano.
  raton::Motion& mo = mouse_.navigator().motion();
  const float k = mo.motorScale();
  if (k > 0.6f && k < 1.6f) params_.kV *= k;
  mo.setMotorScale(1.0f);
  params_.motorBalance = mo.motorBalance();
  params_.startBoostLF = mo.startBoost(0);
  params_.startBoostLR = mo.startBoost(1);
  params_.startBoostRF = mo.startBoost(2);
  params_.startBoostRR = mo.startBoost(3);
  const float g = mo.gyroScale();
  if (g > 0.93f && g < 1.07f) params_.gyroScale = g;
  // Inclinacion de cada sensor y reflejo del piso, aprendidos con las paredes ya seguras.
  raton::Navigator& nav = mouse_.navigator();
  float* tilts[3] = {&params_.leftTiltDeg, &params_.frontTiltDeg, &params_.rightTiltDeg};
  for (int i = 0; i < 3; ++i) {
    const float t = nav.sensorTilt(i) - params_.sensorTiltDeg;
    if (t > -10.0f && t < 10.0f) *tilts[i] = t;
  }
  const float fl = nav.floorReflect();
  if (fl > 0.0f && fl <= 1.0f) params_.floorReflect = fl;
  if (nav.pitchConfident()) {
    params_.learnedCellMm = nav.cellPitch();
    params_.learnedCellRefMm = params_.cellMm;
    params_.learnedWheelDiamMm = params_.wheelDiamMm;
    params_.learnedCountsPerRev = params_.countsPerRev;
  }
  const bool ok = storage::saveParams(params_);
  mo.loadLearned();
  char out[120];
  snprintf(out, sizeof(out), "APRENDIDO motor x%d/1000 giro x%d/1000 inclinacion %d/%d/%d decimas piso %d%% %s",
           static_cast<int>(k * 1000), static_cast<int>(g * 1000), static_cast<int>(params_.leftTiltDeg * 10),
           static_cast<int>(params_.frontTiltDeg * 10), static_cast<int>(params_.rightTiltDeg * 10),
           static_cast<int>(params_.floorReflect * 100), ok ? "(guardado)" : "(ERROR_FLASH)");
  reply(out);
  snprintf(out, sizeof(out), "AUTOCAL balance=%d/1000 arranque=%d/%d/%d/%d paso=%d/10mm",
           static_cast<int>(params_.motorBalance * 1000), static_cast<int>(params_.startBoostLF * 1000),
           static_cast<int>(params_.startBoostLR * 1000), static_cast<int>(params_.startBoostRF * 1000),
           static_cast<int>(params_.startBoostRR * 1000), static_cast<int>(params_.learnedCellMm * 10));
  reply(out);
}

void App::sensorTest() {
  if (sensorTestRunning_) { reply("PRUEBA_EN_CURSO"); return; }
  sensorTestRunning_ = true;
  motors_.coast();
  board_.clearStop();
  // Usar Lasers directamente: no depender de que IMU/encoders terminaran el arranque.
  struct Stats { int n = 0, valid = 0, min = 32767, max = -1, last = -1; } stats[5];
  const char* names[5] = {"diagonal_izq", "frontal", "diagonal_der", "lateral_izq", "lateral_der"};
  const int pinsUsed[5] = {pins::XSHUT_LEFT, pins::XSHUT_FRONT, pins::XSHUT_RIGHT,
                           pins::XSHUT_SIDE_LEFT, pins::XSHUT_SIDE_RIGHT};
  reply("PRUEBA_INICIO: 8 s; sin movimiento; x interrumpe");
  const uint32_t start = millis();
  uint32_t reportAt = start;
  while (millis() - start < 8000 && !board_.stopRequested()) {
    service();
    for (int i = 0; i < 5; ++i) {
      int mm;
      if (!lasers_.poll(i, mm)) continue;
      Stats& st = stats[i];
      ++st.n; st.last = mm;
      if (mm >= 0) {
        ++st.valid;
        if (mm < st.min) st.min = mm;
        if (mm > st.max) st.max = mm;
      }
    }
    if (millis() - reportAt >= 500) {
      reportAt = millis();
      char line[96];
      snprintf(line, sizeof(line), "PRUEBA_MM DI=%d F=%d DD=%d LI=%d LD=%d",
               stats[0].last, stats[1].last, stats[2].last, stats[3].last, stats[4].last);
      telemetry::println(line, false);
      if (Serial) Serial.println(line);
    }
    delay(5);
  }
  for (int i = 0; i < 5; ++i) {
    const Stats& st = stats[i];
    const bool conflict = i >= 3 && pins::sidePinConflict(pinsUsed[i], pinsUsed[i == 3 ? 4 : 3]);
    const char* state = conflict ? "CONFLICTO_PIN" : (!lasers_.ok(i) ? "NO_INICIALIZADO" :
                         (st.n == 0 ? "SIN_RESPUESTA" : (st.valid == 0 ? "RESPONDE_SIN_DISTANCIA" : "LECTURAS_VALIDAS")));
    char line[180], pinLabel[8];
    if (pinsUsed[i] >= A0 && pinsUsed[i] <= A7) snprintf(pinLabel, sizeof(pinLabel), "A%d", pinsUsed[i] - A0);
    else snprintf(pinLabel, sizeof(pinLabel), "D%d", pinsUsed[i]);
    snprintf(line, sizeof(line), "PRUEBA_SENSOR %s pin=%s %s nuevas=%d validas=%d invalidas=%d min=%d max=%d",
             names[i], pinLabel, state, st.n, st.valid, st.n - st.valid,
             st.valid ? st.min : -1, st.max);
    reply(line);
  }
  reply(board_.stopRequested() ? "PRUEBA_FIN interrumpida" : "PRUEBA_FIN");
  motors_.coast();
  sensorTestRunning_ = false;
}

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
      snprintf(line, sizeof(line), "SENS I%d F%d D%d fresh=%d,%d,%d giro=%d encI=%d encD=%d", s.left.mm, s.front.mm,
               s.right.mm, s.left.fresh, s.front.fresh, s.right.fresh, static_cast<int>(s.gyroDeg),
               static_cast<int>(s.encLeftMm), static_cast<int>(s.encRightMm));
      telemetry::println(line, false);
      snprintf(line, sizeof(line), "LATERALES I%d D%d fresh=%d,%d control=%d",
               s.sideLeft.mm, s.sideRight.mm, s.sideLeft.fresh, s.sideRight.fresh, params_.sideEnabled);
      telemetry::println(line, false);
    }
  }
  reply("FIN_DIAGNOSTICO_SENSORES");
}

void App::calibrateFront() {
  // Raton CENTRADO en una celda de la competencia mirando a una pared: la lectura tiene que ser la distancia
  // geometrica (con el cono del sensor, x 1.006). La diferencia es el desfase del frontal.
  long sum = 0;
  int n = 0;
  for (int i = 0; i < 40 && n < 15; ++i) {
    const int mm = lasers_.read(Lasers::FRONT);
    if (mm >= 0) {
      sum += mm + params_.frontOffsetMm;
      ++n;
    }
  }
  char out[80];
  if (n < 10) {
    reply("CALIB frontal: sin lecturas");
    return;
  }
  const float g90 = mouse_.navigator().sonda().parallelFactor(1.5708f);
  const float expected = g90 * (0.5f * (params_.cellMm - params_.wallMm) - params_.frontAheadMm);
  params_.frontOffsetMm += static_cast<int>(expected - static_cast<float>(sum) / n + (expected > 0 ? 0.5f : -0.5f));
  params_.frontStopMm = -1;
  storage::saveParams(params_);
  snprintf(out, sizeof(out), "CALIB frontOffsetMm=%d (guardado)", params_.frontOffsetMm);
  reply(out);
}

void App::calibrateSides(float corridor) {
  // Ancho del pasillo conocido: fija la celda de la competencia y calibra los sensores girando en el sitio.
  if (corridor < 100.0f || corridor > 400.0f) {
    reply("CALIB laterales: pasillo entre 100 y 400 mm");
    return;
  }
  params_.cellMm = corridor + params_.wallMm;
  requestedMode_ = kCalibrate;
  startRequested_ = true;
  reply("CALIBRANDO sensores con ese pasillo");
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

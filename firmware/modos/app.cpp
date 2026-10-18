#include "modos/app.h"

#include <Wire.h>

#include "comunicacion/telemetry.h"
#include "config.h"

namespace {

const int kModes = 6;
const char* const kModeNames[] = {"", "Pared derecha", "Pared izquierda", "Laberinto", "Ruta competencia",
                                  "Espacio libre", "Aprender pista"};

}  // namespace

App::App()
    : robot_(motors_, encoders_, lasers_, gyro_, display_),
      log_(robot_),
      maneuvers_(robot_, log_),
      follower_(robot_, log_, maneuvers_),
      freeSpace_(robot_, log_, maneuvers_, follower_),
      maze_(robot_, log_, display_) {}

void App::setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);  // sin esperar al USB: arranca al encender

  motors_.begin();
  motors_.coast();
  encoders_.begin();
  delay(cfg::POWER_SETTLE_MS);  // al encender con bateria los sensores tardan en estar listos
  Wire.begin();
  Wire.setClock(400000);

  if (!display_.begin()) Serial.println("Aviso: OLED no encontrada");
  bool ok = false;
  for (int i = 0; i < cfg::SENSOR_INIT_TRIES && !ok; ++i) {
    ok = lasers_.begin();
    if (!ok) delay(100);
  }
  if (!ok) fail("Falla un laser");
  ok = false;
  for (int i = 0; i < cfg::SENSOR_INIT_TRIES && !ok; ++i) {
    ok = gyro_.begin();
    if (!ok) delay(100);
  }
  if (!ok) fail("Falla el IMU");
  if (!telemetry::begin()) Serial.println("Aviso: Bluetooth no disponible");
}

void App::loop() {
  if (firstRun_ && !cfg::WAIT_FOR_WEB_START) {
    mode_ = selectMode();
  } else {
    mode_ = waitForStart(mode_);
  }
  robot_.clearAbort();
  robot_.takeStart();  // descarta un 's' repetido
  waitAutoStart(kModeNames[mode_], cfg::START_DELAY_MS);
  if (firstRun_ && cfg::SELF_CHECK_ON_START) {
    // La revision (motores cruzados, distancia entre ruedas) solo al encender.
    char check[22];
    robot_.selfCheck(check);
    Serial.println(check);
    telemetry::println(check);
    char axle[22];
    snprintf(axle, sizeof(axle), "Eje %d mm", static_cast<int>(robot_.trackWidthMm()));
    display_.show(kModeNames[mode_], check, axle);
  }
  firstRun_ = false;
  robot_.clearAbort();     // un fallo de la revision no debe dejar el modo parado antes de empezar
  robot_.resetOdometry();  // el mapa empieza donde arranca
  robot_.clearGoal();      // cada modo pone su meta si la necesita
  telemetry::println("INICIO_MAPA");
  runMode(mode_);
  motors_.coast();
}

void App::runMode(int mode) {
  switch (mode) {
    case 3: modo3(); break;
    case 4: modo4(); break;
    case 5: modo5(); break;
    case 6: modo6(); break;
    default: if (mode == 1) modo1(); else modo2(); break;
  }
}

void App::fail(const char* msg) {
  motors_.coast();
  Serial.println(msg);
  display_.show("ERROR", msg);
  while (true) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(150);
  }
}

void App::waitAutoStart(const char* title, uint32_t ms) {
  Serial.print(title);
  Serial.println(": arranque automatico");
  const uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    char l2[22];
    snprintf(l2, sizeof(l2), "Arranca en %lu s", static_cast<unsigned long>((ms - (millis() - t0) + 999) / 1000));
    display_.show(title, "", l2, "No tocar");
    digitalWrite(LED_BUILTIN, (millis() / 250) % 2);
    delay(100);
  }
  digitalWrite(LED_BUILTIN, HIGH);
  display_.show(title, "", "Calibrando...");
  gyro_.calibrate();  // el raton tiene que estar quieto
  digitalWrite(LED_BUILTIN, LOW);
}

int App::selectMode() {
  // Una pared fija delante no cuenta: hace falta acercar y retirar la mano. Cada pasada alarga el plazo.
  int mode = 1;
  if (cfg::MODE_SELECT_MS == 0) return mode;
  uint32_t deadline = millis() + cfg::MODE_SELECT_MS;
  bool far = false;
  bool near = false;
  while (static_cast<int32_t>(millis() - deadline) < 0) {
    robot_.idle(20);
    const int f = lasers_.rawMm(Lasers::FRONT);
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
    display_.show("ELEGIR MODO", l2, kModeNames[mode], "Pasa la mano");
    digitalWrite(LED_BUILTIN, (millis() / 250) % 2);
  }
  return mode;
}

int App::waitForStart(int mode) {
  bool far = false;
  bool near = false;
  robot_.setState("PARADO");
  char conf[16];
  snprintf(conf, sizeof(conf), "MODE %d", mode);
  telemetry::println(conf);  // la web marca el modo cuando la placa lo confirma
  uint32_t lastConf = millis();
  while (true) {
    robot_.idle(20);
    if (robot_.requests().readMap) {
      robot_.requests().readMap = false;
      maze_.showSavedMap();
    }
    if (robot_.requests().eraseMap) {
      robot_.requests().eraseMap = false;
      maze_.eraseMap();
    }
    const int m = robot_.takeMode();
    if (m) {
      mode = m;
      lastConf = 0;  // confirmar ya
    }
    if (millis() - lastConf > 2000) {  // y cada 2 s, por si la web se conecta despues
      snprintf(conf, sizeof(conf), "MODE %d", mode);
      telemetry::println(conf);
      lastConf = millis();
    }
    if (robot_.takeStart()) {
      snprintf(conf, sizeof(conf), "START %d", mode);
      telemetry::println(conf);
      return mode;
    }
    char l1[22];
    snprintf(l1, sizeof(l1), "Modo %d", mode);
    if (cfg::WAIT_FOR_WEB_START) {
      display_.show("ESPERANDO", l1, kModeNames[mode], "Pulsa ARRANCAR en web");
      digitalWrite(LED_BUILTIN, (millis() / 1000) % 2);
      continue;
    }
    const int f = lasers_.rawMm(Lasers::FRONT);
    if (f < 0 || f > cfg::HAND_FAR_MM) {
      if (near) return mode;
      far = true;
    } else if (far && f < cfg::HAND_NEAR_MM) {
      near = true;
    }
    display_.show("PARADO", l1, kModeNames[mode], "Mano o web = arrancar");
  }
}

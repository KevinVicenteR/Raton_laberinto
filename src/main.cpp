// Firmware del raton de laberinto.
//
// Uso (sin arrancador):
//   1. Pon el raton centrado en la celda de salida (esquina inferior izquierda) mirando al norte.
//   2. Enciendelo y no lo toques: explora al instante hasta probar el camino optimo
//      y vuelve a la salida.
//   3. Despues encadena carreras rapidas a la meta y regreso, con RUN_PAUSE_MS entre cada una.
// 'x' por el Monitor Serie (115200) detiene el raton.

#include <Arduino.h>
#include <Wire.h>

#include "config.h"
#include "hw/display.h"
#include "hw/encoders.h"
#include "hw/gyro.h"
#include "hw/lasers.h"
#include "hw/motors.h"
#include "robot_hw.h"
#include "solver.h"

namespace {

Motors motors;
Encoders encoders;
Lasers lasers;
Gyro gyro;
Display display;
RobotHW robot(motors, encoders, lasers, gyro, display);
raton::Solver solver(robot, cfg::MAZE_WIDTH, cfg::MAZE_HEIGHT);

void fail(const char* msg) {
  motors.coast();
  Serial.println(msg);
  display.show("ERROR", msg);
  while (true) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(150);
  }
}

void showStats(const char* title);

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

// Sin nadie que lo recoloque, tras un fallo el raton se queda parado.
void halt(const char* title) {
  motors.coast();
  showStats(title);
  while (true) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(500);
  }
}

void showStats(const char* title) {
  const raton::SolverStats& s = solver.stats();
  char l1[22];
  char l2[22];
  char l3[22];
  snprintf(l1, sizeof(l1), "Viajes: %d", s.searchRuns);
  snprintf(l2, sizeof(l2), "Celdas: %d", s.searchCells);
  snprintf(l3, sizeof(l3), "Optimo: %s", s.pathProven ? "probado" : "no");
  display.show(title, l1, l2, l3);
  Serial.print(title);
  Serial.print(" | viajes ");
  Serial.print(s.searchRuns);
  Serial.print(" | celdas ");
  Serial.print(s.searchCells);
  Serial.print(" | optimo ");
  Serial.println(s.pathProven ? "probado" : "no probado");
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
}

void loop() {
  robot.clearAbort();
  solver.reset();
  waitAutoStart("EXPLORAR", cfg::START_DELAY_MS);
  if (!solver.explore(cfg::MAX_SEARCH_RUNS)) halt("EXPLORACION FALLO");
  solver.face(raton::NORTH);
  showStats("MAPA LISTO");

  // Con el mapa listo, carreras rapidas sin parar mientras no falle ninguna.
  while (true) {
    waitAutoStart("CARRERA", cfg::RUN_PAUSE_MS);
    if (!solver.speedRun() || !solver.returnToStart()) halt("CARRERA FALLO");
    solver.face(raton::NORTH);
    showStats("CARRERA OK");
  }
}

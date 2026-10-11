// Firmware del raton de laberinto.
//
// Uso:
//   1. Pon el raton centrado en la celda de salida (esquina inferior izquierda) mirando al norte.
//   2. Pasa la mano delante del laser frontal y retirala: explora hasta probar el camino optimo
//      y vuelve a la salida.
//   3. Vuelve a pasar la mano: carrera rapida a la meta y regreso explorando.
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

// Espera a que pasen la mano delante del laser frontal y la retiren.
void waitHandStart(const char* title) {
  display.show(title, "", "Mano delante del", "laser frontal");
  Serial.print(title);
  Serial.println(": pasa la mano delante del laser frontal");
  bool handSeen = false;
  while (true) {
    lasers.update();
    const int f = lasers.mm(Lasers::FRONT);
    if (f >= 0 && f < cfg::HAND_START_MM) handSeen = true;
    if (handSeen && (f < 0 || f > 2 * cfg::HAND_START_MM)) break;
    digitalWrite(LED_BUILTIN, (millis() / 500) % 2);
    delay(5);
  }
  digitalWrite(LED_BUILTIN, HIGH);
  display.show(title, "", "Arrancando...");
  delay(700);       // tiempo para retirar la mano
  gyro.calibrate();  // el raton tiene que estar quieto
  digitalWrite(LED_BUILTIN, LOW);
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
  Serial.begin(115200);
  const uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 2000) {
  }

  motors.begin();
  encoders.begin();
  Wire.begin();
  Wire.setClock(400000);

  if (!display.begin()) Serial.println("Aviso: OLED no encontrada");
  if (!lasers.begin()) fail("Falla un laser");
  if (!gyro.begin()) fail("Falla el IMU");
}

void loop() {
  robot.clearAbort();
  solver.reset();
  waitHandStart("EXPLORAR");
  if (!solver.explore(cfg::MAX_SEARCH_RUNS)) {
    motors.coast();
    showStats("EXPLORACION FALLO");
    delay(3000);
    return;  // vuelve a empezar con el mapa vacio
  }
  // Mirar al norte: si mira a la pared de atras, el laser frontal confundiria la pared con la mano.
  solver.face(raton::NORTH);
  showStats("MAPA LISTO");
  delay(1500);

  // Mientras haya mapa, cada pase de mano hace otra carrera rapida.
  while (true) {
    waitHandStart("CARRERA");
    if (!solver.speedRun() || !solver.returnToStart()) {
      motors.coast();
      showStats("CARRERA FALLO");
      delay(3000);
      return;
    }
    solver.face(raton::NORTH);
    showStats("CARRERA OK");
    delay(1500);
  }
}

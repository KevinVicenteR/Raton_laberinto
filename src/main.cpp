// Firmware del raton de laberinto.
//
// Uso (sin arrancador): pon el raton en un pasillo de la pista y enciendelo. Sigue la pared con la
// regla de la mano derecha (o izquierda, FOLLOW_RIGHT en config.h) usando solo los laseres, asi que
// no depende del tamano de las celdas ni de la pista.
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

namespace {

Motors motors;
Encoders encoders;
Lasers lasers;
Gyro gyro;
Display display;
RobotHW robot(motors, encoders, lasers, gyro, display);

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
  waitAutoStart("SEGUIR PARED", cfg::START_DELAY_MS);
  // Signo de giro hacia la pared que se sigue: +1 izquierda, -1 derecha.
  const int side = cfg::FOLLOW_RIGHT ? -1 : 1;
  char check[22];
  robot.selfCheck(check);
  Serial.println(check);
  display.show("SEGUIR PARED", cfg::FOLLOW_RIGHT ? "Mano derecha" : "Mano izquierda", check);

  while (!robot.aborted()) {
    const RobotHW::DriveEvent ev = robot.drive(1e6f, side);
    if (ev == RobotHW::DriveEvent::SideOpen) {
      // Hueco en la pared que se sigue: llevar el eje de las ruedas al centro del hueco, girar y entrar.
      // El laser lateral va SIDE_SENSOR_AHEAD_MM por delante del eje y el hueco mide como el pasillo.
      const float half = 0.5f * robot.corridorMm();
      robot.drive(fmaxf(cfg::SIDE_SENSOR_AHEAD_MM + half - cfg::OPEN_CONFIRM_MM, 0.0f), 0);
      robot.turnDegrees(90.0f * side);
      // Entrar hasta que el laser lateral quede dentro del pasillo nuevo.
      robot.drive(fmaxf(half - cfg::SIDE_SENSOR_AHEAD_MM + 15.0f, 15.0f), 0);
    } else if (ev == RobotHW::DriveEvent::CrashLeft || ev == RobotHW::DriveEvent::CrashRight) {
      // Pegado a un lateral: un poco atras y enderezarse hacia el otro lado.
      robot.backUp(cfg::NUDGE_BACK_MM);
      robot.turnDegrees(ev == RobotHW::DriveEvent::CrashLeft ? -cfg::NUDGE_DEG : cfg::NUDGE_DEG);
    } else if (ev == RobotHW::DriveEvent::FrontWall) {
      const raton::Walls w = robot.readWalls();
      const bool followOpen = cfg::FOLLOW_RIGHT ? !w.right : !w.left;
      const bool otherOpen = cfg::FOLLOW_RIGHT ? !w.left : !w.right;
      robot.backUp(cfg::BACKUP_MM);
      if (followOpen) {
        robot.turnDegrees(90.0f * side);
      } else if (otherOpen) {
        robot.turnDegrees(-90.0f * side);
      } else {
        robot.turnDegrees(180.0f);  // callejon sin salida
      }
    }
  }
  motors.coast();
  display.show("PARADO");
  while (true) {
    delay(100);
  }
}

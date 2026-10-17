// Prueba y calibracion del hardware: pio run -e prueba_hardware -t upload
//
// Monitor Serie (115200):
//   w = adelante   s = atras   a = girar izq   d = girar der   espacio = parar
//   + / - = cambiar PWM   r = encoders y yaw a cero   c = calibrar giroscopio (quieto)
//
// Que mirar:
//   - Con 'w' las dos ruedas avanzan y los dos encoders suben. Si no, cambia INVERT_* en config.h.
//   - Empuja el raton exactamente 1 m a mano: mm_izq / mm_der deben marcar ~1000. Si no, ajusta
//     WHEEL_DIAMETER_MM o ENCODER_COUNTS_PER_REV.
//   - Gira el raton a mano 90 grados a la izquierda: yaw debe marcar ~ +90. Si sale -90, GYRO_SIGN = -1.
//   - Pega el morro a una pared: la lectura del frontal es FRONT_DEADZONE_MM. Los laterales se calibran
//     con la orden K<mm> del firmware normal en un pasillo de ancho conocido.

#include <Arduino.h>
#include <Wire.h>

#include "../config.h"
#include "../hw/display.h"
#include "../hw/encoders.h"
#include "../hw/gyro.h"
#include "../hw/lasers.h"
#include "../hw/motors.h"

namespace {
class HardwareTest {
  Motors motors;
  Encoders encoders;
  Lasers lasers;
  Gyro gyro;
  Display display;
  float pwm = 0.30f;
  uint32_t last = 0;

 public:
  void setup() {
    Serial.begin(115200);
    const uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 3000) {
    }

    motors.begin();
    encoders.begin();
    Wire.begin();
    Wire.setClock(400000);

    Serial.println(display.begin() ? "OLED ok" : "OLED NO responde");
    lasers.begin();
    Serial.print("Laser izquierdo ");
    Serial.println(lasers.ok(Lasers::LEFT) ? "ok" : "NO responde");
    Serial.print("Laser frontal   ");
    Serial.println(lasers.ok(Lasers::FRONT) ? "ok" : "NO responde");
    Serial.print("Laser derecho   ");
    Serial.println(lasers.ok(Lasers::RIGHT) ? "ok" : "NO responde");
    if (gyro.begin()) {
      Serial.println("IMU ok, calibrando (no mover)...");
      gyro.calibrate();
    } else {
      Serial.println("IMU NO responde");
    }
    Serial.println("w/s/a/d mover, espacio parar, +/- PWM, r reset, c calibrar giroscopio");

    // Prueba automatica para hacerla solo con bateria (sin USB): avanza 1 s y gira a la izquierda
    // 0.5 s. En pantalla: los dos encoders deben ser positivos y el yaw positivo.
    delay(2000);
    encoders.reset();
    motors.set(pwm, pwm);
    delay(1000);
    motors.coast();
    const long fwdL = encoders.left();
    const long fwdR = encoders.right();
    delay(500);
    gyro.resetYaw();
    const uint32_t t0turn = millis();
    motors.set(-pwm, pwm);
    while (millis() - t0turn < 500) gyro.update();
    motors.coast();
    const uint32_t t0stop = millis();
    while (millis() - t0stop < 300) gyro.update();
    const int turnYaw = static_cast<int>(gyro.yaw());

    char l[4][22];
    snprintf(l[0], sizeof(l[0]), "Adel. I %ld", fwdL);
    snprintf(l[1], sizeof(l[1]), "Adel. D %ld", fwdR);
    snprintf(l[2], sizeof(l[2]), "Giro izq yaw %d", turnYaw);
    snprintf(l[3], sizeof(l[3]), "Todo debe ser +");
    display.show("AUTO TEST", l[0], l[1], l[2], l[3]);
    Serial.println(l[0]);
    Serial.println(l[1]);
    Serial.println(l[2]);
    delay(15000);  // tiempo para leer la pantalla
  }

  void loop() {
    lasers.update();
    gyro.update();

    if (Serial.available()) {
      switch (Serial.read()) {
        case 'w': motors.set(pwm, pwm); break;
        case 's': motors.set(-pwm, -pwm); break;
        case 'a': motors.set(-pwm, pwm); break;
        case 'd': motors.set(pwm, -pwm); break;
        case ' ': motors.coast(); break;
        case '+': pwm = fminf(pwm + 0.05f, cfg::PWM_MAX); break;
        case '-': pwm = fmaxf(pwm - 0.05f, 0.0f); break;
        case 'r':
          encoders.reset();
          gyro.resetYaw();
          break;
        case 'c':
          motors.coast();
          gyro.calibrate();
          break;
      }
    }

    if (millis() - last < 100) return;
    last = millis();

    const long el = encoders.left();
    const long er = encoders.right();
    // printf de float no esta garantizado en este core: se imprimen enteros
    const long mmL = static_cast<long>(el * cfg::MM_PER_COUNT);
    const long mmR = static_cast<long>(er * cfg::MM_PER_COUNT);
    const int yawDeg = static_cast<int>(gyro.yaw());
    const int pwmPct = static_cast<int>(pwm * 100.0f + 0.5f);
    char line[160];
    snprintf(line, sizeof(line), "laser I/F/D %4d %4d %4d mm | enc %6ld %6ld | mm_izq %6ld mm_der %6ld | yaw %4d | pwm %d%%",
             lasers.mm(Lasers::LEFT), lasers.mm(Lasers::FRONT), lasers.mm(Lasers::RIGHT), el, er, mmL, mmR, yawDeg,
             pwmPct);
    Serial.println(line);

    char l[4][22];
    snprintf(l[0], sizeof(l[0]), "I%4d F%4d D%4d", lasers.mm(Lasers::LEFT), lasers.mm(Lasers::FRONT),
             lasers.mm(Lasers::RIGHT));
    snprintf(l[1], sizeof(l[1]), "Enc I %ld %ldmm", el, mmL);
    snprintf(l[2], sizeof(l[2]), "Enc D %ld %ldmm", er, mmR);
    snprintf(l[3], sizeof(l[3]), "Yaw %d  PWM %d%%", yawDeg, pwmPct);
    display.show("PRUEBA HARDWARE", l[0], l[1], l[2], l[3]);
  }

};

HardwareTest& hardwareTest() {
  static HardwareTest app;
  return app;
}
}  // namespace

void setup() { hardwareTest().setup(); }
void loop() { hardwareTest().loop(); }

// Giroscopio BMI270 del Nano 33 BLE Sense Rev2 (bus Wire1 interno, no ocupa A4/A5).
// La libreria de Arduino fija 100 Hz; aqui se sube a 200 Hz escribiendo GYR_CONF tras iniciarla.
#pragma once

#include <Arduino.h>

class Imu {
 public:
  bool begin();
  // Sesgo con el raton quieto (ms de muestras); atiende comunicaciones mientras espera.
  void calibrate(uint32_t ms, void (*service)() = nullptr);
  // Si hay muestra nueva la integra (grados acumulados, + = izquierda) y devuelve true.
  bool poll(float nowS);
  float yawDeg() const { return yaw_; }
  float rateDps() const { return rate_; }
  // Con el raton parado corrige poco a poco el sesgo (temperatura).
  void trackBias(bool still);
  float odrHz() const { return odr_; }

 private:
  float bias_ = 0, yaw_ = 0, rate_ = 0, lastT_ = -1, lastZ_ = 0;
  float odr_ = 100.0f;
};

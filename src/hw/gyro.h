// Giroscopio BMI270 integrado en el Nano 33 BLE Sense Rev2 (va por Wire1, no ocupa A4/A5).
#pragma once

#include <Arduino.h>

class Gyro {
 public:
  bool begin();
  // Mide el sesgo con el robot quieto.
  void calibrate(uint32_t ms = 800);
  // Integra las muestras pendientes. Llamar seguido.
  void update();

  float yaw() const { return yaw_; }        // grados, positivo = giro a la izquierda
  float rate() const { return rate_; }      // grados/s
  void resetYaw() { yaw_ = 0; }

 private:
  float bias_ = 0;
  float yaw_ = 0;
  float rate_ = 0;
  float dt_ = 0.01f;
};

// Giroscopio BMI270 integrado en el Nano 33 BLE Sense Rev2 (va por Wire1, no ocupa A4/A5).
#pragma once

#include <Arduino.h>

class Gyro {
 public:
  bool begin();
  // Mide el sesgo con el robot quieto.
  void calibrate(uint32_t ms = 800);
  // Integra las muestras pendientes. Llamar seguido. Con stationary = true (raton parado) ademas
  // corrige poco a poco el sesgo, asi el rumbo no deriva con el tiempo ni con la temperatura.
  void update(bool stationary = false);

  float yaw() const { return yaw_; }        // grados, positivo = giro a la izquierda
  float rate() const { return rate_; }      // grados/s
  void resetYaw() { yaw_ = 0; }
  // Aceleracion horizontal (g) sobre la de reposo: un golpe contra una pared da un pico.
  float bumpG() const { return bump_; }

  // Rumbo acumulado desde el arranque (no le afecta resetYaw): para la odometria.
  float totalYaw() const { return totalYaw_; }

 private:
  float bias_ = 0;
  float yaw_ = 0;
  float totalYaw_ = 0;
  float ax0_ = 0;  // aceleracion en reposo (placa no del todo plana)
  float ay0_ = 0;
  float bump_ = 0;
  float rate_ = 0;
  float dt_ = 0.01f;
};

// DRV8833: PWM a 500 Hz, rueda libre entre pulsos, como el firmware anterior.
#pragma once

#include <Arduino.h>
#include <mbed.h>

class Motors {
 public:
  void begin();
  // -1..1 solicitado por rueda, limitado a wiring::PWM_MAX; positivo = adelante.
  void set(float left, float right);
  void brake();  // las dos entradas en alto: frena en corto
  void coast();  // las dos en bajo: rueda libre

 private:
  static void drive(mbed::PwmOut* in1, mbed::PwmOut* in2, float duty);
  mbed::PwmOut* ain1_ = nullptr;
  mbed::PwmOut* ain2_ = nullptr;
  mbed::PwmOut* bin1_ = nullptr;
  mbed::PwmOut* bin2_ = nullptr;
};

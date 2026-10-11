// DRV8833 con PWM de 20 kHz en las cuatro entradas.
#pragma once

#include <Arduino.h>
#include <mbed.h>

class Motors {
 public:
  void begin();
  // -1..1 por rueda, ya limitado a cfg::PWM_MAX. Positivo = adelante.
  void set(float left, float right);
  void brake();  // ambas entradas en alto: frena en corto
  void coast();  // ambas en bajo: rueda libre

 private:
  static void drive(mbed::PwmOut* in1, mbed::PwmOut* in2, float duty);

  mbed::PwmOut* ain1_ = nullptr;
  mbed::PwmOut* ain2_ = nullptr;
  mbed::PwmOut* bin1_ = nullptr;
  mbed::PwmOut* bin2_ = nullptr;
};

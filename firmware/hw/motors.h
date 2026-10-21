// DRV8833 con PWM en "slow decay" (frenado entre pulsos): velocidad proporcional al duty.
#pragma once

#include <Arduino.h>
#include <mbed.h>

class Motors {
 public:
  void begin();
  // -1..1 por rueda, positivo = adelante.
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

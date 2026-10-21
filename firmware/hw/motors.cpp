#include "hw/motors.h"

#include "config.h"

namespace {

mbed::PwmOut* makePwm(uint8_t pin) {
  mbed::PwmOut* pwm = new mbed::PwmOut(digitalPinToPinName(pin));
  pwm->period_us(wiring::PWM_PERIOD_US);
  pwm->write(0.0f);
  return pwm;
}

}  // namespace

void Motors::begin() {
  ain1_ = makePwm(pins::AIN1);
  ain2_ = makePwm(pins::AIN2);
  bin1_ = makePwm(pins::BIN1);
  bin2_ = makePwm(pins::BIN2);
}

void Motors::drive(mbed::PwmOut* in1, mbed::PwmOut* in2, float duty) {
  if (duty > 1.0f) duty = 1.0f;
  if (duty < -1.0f) duty = -1.0f;
  // Slow decay (hoja de datos del DRV8833): adelante = IN1 alto e IN2 con PWM invertido; entre pulsos
  // las dos entradas quedan en alto (freno), asi la corriente no cae y el par a baja velocidad se mantiene.
  if (duty >= 0) {
    in1->write(1.0f);
    in2->write(1.0f - duty);
  } else {
    in1->write(1.0f + duty);
    in2->write(1.0f);
  }
}

void Motors::set(float left, float right) {
  if (!ain1_) return;
  drive(ain1_, ain2_, wiring::INVERT_MOTOR_LEFT ? -left : left);
  drive(bin1_, bin2_, wiring::INVERT_MOTOR_RIGHT ? -right : right);
}

void Motors::brake() {
  if (!ain1_) return;
  ain1_->write(1.0f);
  ain2_->write(1.0f);
  bin1_->write(1.0f);
  bin2_->write(1.0f);
}

void Motors::coast() {
  if (!ain1_) return;
  ain1_->write(0.0f);
  ain2_->write(0.0f);
  bin1_->write(0.0f);
  bin2_->write(0.0f);
}

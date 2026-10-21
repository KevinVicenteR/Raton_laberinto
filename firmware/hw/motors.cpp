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
  // Recupera la senal usada antes de la reescritura: PWM activo y la otra
  // entrada baja (rueda libre entre pulsos). Mantener el limite para motores
  // de 6 V alimentados desde la bateria 2S del raton.
  if (duty > wiring::PWM_MAX) duty = wiring::PWM_MAX;
  if (duty < -wiring::PWM_MAX) duty = -wiring::PWM_MAX;
  if (duty >= 0) {
    in1->write(duty);
    in2->write(0.0f);
  } else {
    in1->write(0.0f);
    in2->write(-duty);
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

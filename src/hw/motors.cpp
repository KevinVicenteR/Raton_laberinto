#include "motors.h"

#include "../config.h"

namespace {

mbed::PwmOut* makePwm(uint8_t pin) {
  mbed::PwmOut* pwm = new mbed::PwmOut(digitalPinToPinName(pin));
  pwm->period_us(cfg::PWM_PERIOD_US);
  pwm->write(0.0f);
  return pwm;
}

}  // namespace

void Motors::begin() {
  ain1_ = makePwm(cfg::PIN_AIN1);
  ain2_ = makePwm(cfg::PIN_AIN2);
  bin1_ = makePwm(cfg::PIN_BIN1);
  bin2_ = makePwm(cfg::PIN_BIN2);
}

void Motors::drive(mbed::PwmOut* in1, mbed::PwmOut* in2, float duty) {
  if (duty > cfg::PWM_MAX) duty = cfg::PWM_MAX;
  if (duty < -cfg::PWM_MAX) duty = -cfg::PWM_MAX;
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
  drive(ain1_, ain2_, cfg::INVERT_MOTOR_IZQ ? -left : left);
  drive(bin1_, bin2_, cfg::INVERT_MOTOR_DER ? -right : right);
}

void Motors::brake() {
  if (!ain1_) return;
  ain1_->write(1.0f);
  ain2_->write(1.0f);
  bin1_->write(1.0f);
  bin2_->write(1.0f);
}

void Motors::coast() { set(0, 0); }

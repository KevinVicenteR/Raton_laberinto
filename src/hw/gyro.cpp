#include "gyro.h"

#include <Arduino_BMI270_BMM150.h>

#include "../config.h"

bool Gyro::begin() {
  if (!IMU.begin()) return false;
  const float rate = IMU.gyroscopeSampleRate();
  if (rate > 0) dt_ = 1.0f / rate;
  return true;
}

void Gyro::calibrate(uint32_t ms) {
  float sum = 0;
  int n = 0;
  const uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    float x, y, z;
    if (IMU.gyroscopeAvailable() && IMU.readGyroscope(x, y, z)) {
      sum += z;
      n++;
    }
  }
  if (n > 0) bias_ = sum / n;
  yaw_ = 0;
  rate_ = 0;
}

void Gyro::update() {
  float x, y, z;
  while (IMU.gyroscopeAvailable() && IMU.readGyroscope(x, y, z)) {
    rate_ = cfg::GYRO_SIGN * (z - bias_);
    yaw_ += rate_ * dt_;
  }
}

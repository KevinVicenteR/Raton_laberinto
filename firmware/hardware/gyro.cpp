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
  float sumX = 0;
  float sumY = 0;
  int n = 0;
  int na = 0;
  const uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    float x, y, z;
    if (IMU.gyroscopeAvailable() && IMU.readGyroscope(x, y, z)) {
      sum += z;
      n++;
    }
    if (IMU.accelerationAvailable() && IMU.readAcceleration(x, y, z)) {
      sumX += x;
      sumY += y;
      na++;
    }
  }
  if (n > 0) bias_ = sum / n;
  if (na > 0) {
    ax0_ = sumX / na;
    ay0_ = sumY / na;
  }
  yaw_ = 0;
  rate_ = 0;
}

void Gyro::update(bool stationary) {
  float x, y, z;
  while (IMU.gyroscopeAvailable() && IMU.readGyroscope(x, y, z)) {
    if (stationary && fabsf(z - bias_) < cfg::GYRO_BIAS_TRACK_DPS) bias_ += cfg::GYRO_BIAS_TRACK_GAIN * (z - bias_);
    rate_ = cfg::GYRO_SIGN * (z - bias_);
    yaw_ += rate_ * dt_;
    totalYaw_ += rate_ * dt_;
  }
  while (IMU.accelerationAvailable() && IMU.readAcceleration(x, y, z)) {
    bump_ = sqrtf((x - ax0_) * (x - ax0_) + (y - ay0_) * (y - ay0_));
  }
}

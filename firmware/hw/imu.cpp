#include "hw/imu.h"

#include <Arduino_BMI270_BMM150.h>
#include <Wire.h>

#include "config.h"

namespace {
const uint8_t kAddr = 0x68;     // BMI270 en Wire1
const uint8_t kGyrConf = 0x42;  // gyr_odr[3:0], gyr_bwp[5:4], gyr_noise_perf[6], gyr_filter_perf[7]

bool writeReg(uint8_t reg, uint8_t v) {
  Wire1.beginTransmission(kAddr);
  Wire1.write(reg);
  Wire1.write(v);
  return Wire1.endTransmission() == 0;
}

int readReg(uint8_t reg) {
  Wire1.beginTransmission(kAddr);
  Wire1.write(reg);
  if (Wire1.endTransmission(false) != 0) return -1;
  if (Wire1.requestFrom(kAddr, static_cast<uint8_t>(1)) != 1) return -1;
  return Wire1.read();
}
}  // namespace

bool Imu::begin() {
  if (!IMU.begin()) return false;
  Wire1.setClock(400000);  // el BMI270 admite hasta 1 MHz; la libreria lo deja a 100 kHz
  // 200 Hz, filtro normal, modo de alto rendimiento: 0x80 | 0x20 | 0x09.
  if (writeReg(kGyrConf, 0xA9) && readReg(kGyrConf) == 0xA9) odr_ = 200.0f;
  return true;
}

void Imu::calibrate(uint32_t ms) {
  float sum = 0;
  int n = 0;
  const uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    float x, y, z;
    if (IMU.gyroscopeAvailable() && IMU.readGyroscope(x, y, z)) {
      sum += z;
      ++n;
    }
  }
  if (n > 0) bias_ = sum / n;
  rate_ = 0;
  lastT_ = -1;
}

bool Imu::poll(float now) {
  float x, y, z;
  if (!IMU.gyroscopeAvailable() || !IMU.readGyroscope(x, y, z)) return false;
  lastZ_ = z;
  rate_ = wiring::GYRO_SIGN * (z - bias_);
  // Se integra con el tiempo real entre lecturas (vale a 100 o a 200 Hz).
  if (lastT_ >= 0) {
    float dt = now - lastT_;
    if (dt > 0.05f) dt = 0.05f;
    yaw_ += rate_ * dt;
  }
  lastT_ = now;
  return true;
}

void Imu::trackBias(bool still) {
  if (still && fabsf(lastZ_ - bias_) < 2.0f) bias_ += 0.002f * (lastZ_ - bias_);
}

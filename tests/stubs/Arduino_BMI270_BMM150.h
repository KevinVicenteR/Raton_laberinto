#pragma once
struct BoschSensorClass {
  int begin() { return 1; }
  int gyroscopeAvailable() { return 0; }
  int readGyroscope(float& x, float& y, float& z) { x = y = z = 0; return 1; }
};
extern BoschSensorClass IMU;

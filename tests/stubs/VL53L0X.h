#pragma once
#include <stdint.h>
struct VL53L0X {
  enum { RESULT_INTERRUPT_STATUS = 0x13, RESULT_RANGE_STATUS = 0x14 };
  void setTimeout(uint16_t) {}
  bool init() { return true; }
  void setAddress(uint8_t) {}
  bool setSignalRateLimit(float) { return true; }
  bool setMeasurementTimingBudget(uint32_t) { return true; }
  void startContinuous(uint32_t = 0) {}
  uint8_t readReg(uint8_t) { return 0; }
  uint16_t readRangeContinuousMillimeters() { return 0; }
  bool timeoutOccurred() { return false; }
};

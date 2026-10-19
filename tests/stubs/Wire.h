#pragma once
#include <stdint.h>
struct TwoWire {
  void begin() {}
  void setClock(uint32_t) {}
  void beginTransmission(uint8_t) {}
  int write(uint8_t) { return 1; }
  int endTransmission(bool = true) { return 0; }
  int requestFrom(uint8_t, uint8_t) { return 1; }
  int read() { return 0; }
};
extern TwoWire Wire, Wire1;

#pragma once
#include <stdint.h>
#include <stddef.h>
#include "Arduino.h"
struct NrfPort { volatile uint32_t IN; };
extern NrfPort* NRF_P0;
extern NrfPort* NRF_P1;
namespace mbed {
struct PwmOut {
  explicit PwmOut(PinName) {}
  void period_us(int) {}
  void write(float) {}
};
struct KVStore { struct info_t { size_t size; uint32_t flags; }; };
}
#define MBED_ERROR_ITEM_NOT_FOUND (-1)

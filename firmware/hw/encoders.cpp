#include "hw/encoders.h"

#include <mbed.h>

#include "config.h"

namespace {

// Indice: (estado anterior << 2) | estado actual, con estado = (A << 1) | B.
const int8_t kQuad[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

struct Channel {
  uint32_t a, b;  // pin del nRF (P0.xx = xx, P1.xx = 32 + xx)
  volatile long count;
  volatile uint8_t prev;
};

Channel chL, chR;

inline uint32_t readPin(uint32_t pin) { return ((pin < 32 ? NRF_P0->IN : NRF_P1->IN) >> (pin & 31)) & 1u; }

inline uint8_t readState(const Channel& c) { return static_cast<uint8_t>((readPin(c.a) << 1) | readPin(c.b)); }

void isrL() {
  const uint8_t now = readState(chL);
  chL.count += kQuad[(chL.prev << 2) | now];
  chL.prev = now;
}

void isrR() {
  const uint8_t now = readState(chR);
  chR.count += kQuad[(chR.prev << 2) | now];
  chR.prev = now;
}

long atomicRead(volatile long& v) {
  noInterrupts();
  const long r = v;
  interrupts();
  return r;
}

}  // namespace

void Encoders::begin() {
  const uint8_t p[4] = {pins::ENC_LEFT_A, pins::ENC_LEFT_B, pins::ENC_RIGHT_A, pins::ENC_RIGHT_B};
  for (uint8_t pin : p) pinMode(pin, INPUT_PULLUP);
  chL.a = static_cast<uint32_t>(digitalPinToPinName(pins::ENC_LEFT_A));
  chL.b = static_cast<uint32_t>(digitalPinToPinName(pins::ENC_LEFT_B));
  chR.a = static_cast<uint32_t>(digitalPinToPinName(pins::ENC_RIGHT_A));
  chR.b = static_cast<uint32_t>(digitalPinToPinName(pins::ENC_RIGHT_B));
  chL.prev = readState(chL);
  chR.prev = readState(chR);
  attachInterrupt(digitalPinToInterrupt(pins::ENC_LEFT_A), isrL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pins::ENC_LEFT_B), isrL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pins::ENC_RIGHT_A), isrR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pins::ENC_RIGHT_B), isrR, CHANGE);
}

long Encoders::left() {
  const long raw = wiring::ENCODERS_SWAPPED ? atomicRead(chR.count) : atomicRead(chL.count);
  const bool inv = wiring::ENCODERS_SWAPPED ? wiring::INVERT_ENC_RIGHT : wiring::INVERT_ENC_LEFT;
  return inv ? -raw : raw;
}

long Encoders::right() {
  const long raw = wiring::ENCODERS_SWAPPED ? atomicRead(chL.count) : atomicRead(chR.count);
  const bool inv = wiring::ENCODERS_SWAPPED ? wiring::INVERT_ENC_LEFT : wiring::INVERT_ENC_RIGHT;
  return inv ? -raw : raw;
}

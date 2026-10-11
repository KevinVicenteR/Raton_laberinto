#include "encoders.h"

#include "../config.h"

namespace {

volatile long countLeft = 0;
volatile long countRight = 0;
volatile uint8_t prevLeft = 0;
volatile uint8_t prevRight = 0;

// Indice: (estado anterior << 2) | estado actual, con estado = (A << 1) | B
const int8_t kQuadrature[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

void isrLeft() {
  const uint8_t now = static_cast<uint8_t>((digitalRead(cfg::PIN_ENC_IZQ_A) << 1) | digitalRead(cfg::PIN_ENC_IZQ_B));
  countLeft += kQuadrature[(prevLeft << 2) | now];
  prevLeft = now;
}

void isrRight() {
  const uint8_t now = static_cast<uint8_t>((digitalRead(cfg::PIN_ENC_DER_A) << 1) | digitalRead(cfg::PIN_ENC_DER_B));
  countRight += kQuadrature[(prevRight << 2) | now];
  prevRight = now;
}

long readAtomic(volatile long& v) {
  noInterrupts();
  const long value = v;
  interrupts();
  return value;
}

}  // namespace

void Encoders::begin() {
  pinMode(cfg::PIN_ENC_IZQ_A, INPUT_PULLUP);
  pinMode(cfg::PIN_ENC_IZQ_B, INPUT_PULLUP);
  pinMode(cfg::PIN_ENC_DER_A, INPUT_PULLUP);
  pinMode(cfg::PIN_ENC_DER_B, INPUT_PULLUP);
  prevLeft = static_cast<uint8_t>((digitalRead(cfg::PIN_ENC_IZQ_A) << 1) | digitalRead(cfg::PIN_ENC_IZQ_B));
  prevRight = static_cast<uint8_t>((digitalRead(cfg::PIN_ENC_DER_A) << 1) | digitalRead(cfg::PIN_ENC_DER_B));
  attachInterrupt(digitalPinToInterrupt(cfg::PIN_ENC_IZQ_A), isrLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(cfg::PIN_ENC_IZQ_B), isrLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(cfg::PIN_ENC_DER_A), isrRight, CHANGE);
  attachInterrupt(digitalPinToInterrupt(cfg::PIN_ENC_DER_B), isrRight, CHANGE);
}

long Encoders::left() {
  const long v = readAtomic(countLeft);
  return cfg::INVERT_ENC_IZQ ? -v : v;
}

long Encoders::right() {
  const long v = readAtomic(countRight);
  return cfg::INVERT_ENC_DER ? -v : v;
}

void Encoders::reset() {
  noInterrupts();
  countLeft = 0;
  countRight = 0;
  interrupts();
}

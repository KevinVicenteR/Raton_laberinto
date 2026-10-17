#include "encoders.h"

#include "../config.h"

namespace {

// Indice: (estado anterior << 2) | estado actual, con estado = (A << 1) | B
const int8_t kQuadrature[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

long readAtomic(volatile long& v) {
  noInterrupts();
  const long value = v;
  interrupts();
  return value;
}

}  // namespace

Encoders::Counters& Encoders::counters() {
  static Counters value;
  return value;
}

void Encoders::isrLeft() {
  const uint8_t now = static_cast<uint8_t>((digitalRead(cfg::PIN_ENC_IZQ_A) << 1) | digitalRead(cfg::PIN_ENC_IZQ_B));
  counters().countLeft += kQuadrature[(counters().prevLeft << 2) | now];
  counters().prevLeft = now;
}

void Encoders::isrRight() {
  const uint8_t now = static_cast<uint8_t>((digitalRead(cfg::PIN_ENC_DER_A) << 1) | digitalRead(cfg::PIN_ENC_DER_B));
  counters().countRight += kQuadrature[(counters().prevRight << 2) | now];
  counters().prevRight = now;
}

void Encoders::begin() {
  pinMode(cfg::PIN_ENC_IZQ_A, INPUT_PULLUP);
  pinMode(cfg::PIN_ENC_IZQ_B, INPUT_PULLUP);
  pinMode(cfg::PIN_ENC_DER_A, INPUT_PULLUP);
  pinMode(cfg::PIN_ENC_DER_B, INPUT_PULLUP);
  counters().prevLeft = static_cast<uint8_t>((digitalRead(cfg::PIN_ENC_IZQ_A) << 1) | digitalRead(cfg::PIN_ENC_IZQ_B));
  counters().prevRight = static_cast<uint8_t>((digitalRead(cfg::PIN_ENC_DER_A) << 1) | digitalRead(cfg::PIN_ENC_DER_B));
  attachInterrupt(digitalPinToInterrupt(cfg::PIN_ENC_IZQ_A), isrLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(cfg::PIN_ENC_IZQ_B), isrLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(cfg::PIN_ENC_DER_A), isrRight, CHANGE);
  attachInterrupt(digitalPinToInterrupt(cfg::PIN_ENC_DER_B), isrRight, CHANGE);
}

long Encoders::rawLeftTotal() {
  const long v = readAtomic(counters().countLeft);
  return cfg::INVERT_ENC_IZQ ? -v : v;
}
long Encoders::rawRightTotal() {
  const long v = readAtomic(counters().countRight);
  return cfg::INVERT_ENC_DER ? -v : v;
}

// Con ENCODERS_SWAPPED los cables de los encoders van cruzados respecto a los motores (la prueba de
// ruedas T lo mostro: mover la rueda izquierda contaba en el encoder derecho y al reves).
long Encoders::leftTotal() { return cfg::ENCODERS_SWAPPED ? rawRightTotal() : rawLeftTotal(); }

long Encoders::rightTotal() { return cfg::ENCODERS_SWAPPED ? rawLeftTotal() : rawRightTotal(); }

long Encoders::left() { return leftTotal() - baseLeft; }

long Encoders::right() { return rightTotal() - baseRight; }

void Encoders::reset() {
  baseLeft = leftTotal();
  baseRight = rightTotal();
}

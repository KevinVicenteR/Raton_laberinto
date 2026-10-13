// Encoders de cuadratura (x4) por interrupciones.
#pragma once

#include <Arduino.h>

class Encoders {
 public:
  void begin();
  long left();   // cuentas desde el ultimo reset(), positivo = adelante
  long right();
  void reset();
  // Cuentas totales desde el arranque (no les afecta reset): para la odometria.
  long leftTotal();
  long rightTotal();
};

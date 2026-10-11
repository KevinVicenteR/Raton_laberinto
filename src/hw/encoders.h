// Encoders de cuadratura (x4) por interrupciones.
#pragma once

#include <Arduino.h>

class Encoders {
 public:
  void begin();
  long left();   // cuentas, positivo = adelante
  long right();
  void reset();
};

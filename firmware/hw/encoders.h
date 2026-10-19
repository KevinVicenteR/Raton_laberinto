// Encoders de cuadratura x4 por interrupciones. Las ISR leen el puerto GPIO del nRF52840 directamente
// (digitalRead de mbed es lento: a 500 mm/s llegan ~26 000 flancos por segundo).
#pragma once

#include <Arduino.h>

class Encoders {
 public:
  void begin();
  // Cuentas acumuladas desde el arranque, positivo = adelante.
  long left();
  long right();
};

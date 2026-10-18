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
 private:
  // Las ISR sin contexto requieren un único banco de contadores para los pines físicos.
  struct Counters {
    volatile long countLeft = 0;
    volatile long countRight = 0;
    volatile uint8_t prevLeft = 0;
    volatile uint8_t prevRight = 0;
  };
  static Counters& counters();
  static void isrLeft();
  static void isrRight();
  static long rawLeftTotal();
  static long rawRightTotal();
  long baseLeft = 0;
  long baseRight = 0;
};

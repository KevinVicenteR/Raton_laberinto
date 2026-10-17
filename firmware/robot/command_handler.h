// Ordenes por Bluetooth o Monitor Serie. Una orden es una linea; con protocolo ("$seq,orden*CRC") se
// comprueba y se contesta ACK/NAK con su numero (un reintento repetido no se ejecuta dos veces).
//   s / s<1-5>  arrancar (en el modo n)      x  parar           R  reiniciar la placa
//   m<1-5>      elegir modo                  G<x>,<y>  celda meta
//   v k j d p   velocidad y control          w z r g   umbrales y laseres
//   n<a>,<b>,<c>,<d> pesos del modo 5        c<mm> celda   K<mm> calibrar laterales   T prueba de ruedas
//   ?           ver valores
#pragma once

#include <stdint.h>

class RobotHW;

class CommandHandler {
 public:
  explicit CommandHandler(RobotHW& robot) : robot_(robot) {}
  // Lee lo que haya llegado por Serie y Bluetooth y ejecuta las ordenes completas.
  void poll();

 private:
  void feed(int c);
  void onLine();
  void run(const char* cmd);
  void reportSettings();

  RobotHW& robot_;
  char cmd_[48] = {};
  int cmdLen_ = 0;
  uint16_t lastSeq_ = 0;
  bool seqValid_ = false;
};

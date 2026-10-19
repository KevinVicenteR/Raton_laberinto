// Tres VL53L0X en el mismo bus I2C (todos arrancan en 0x29: se encienden uno a uno con XSHUT y se les da
// direccion propia). Medida continua "back-to-back" con presupuesto de 20 ms (~50 Hz por sensor).
// La lectura no bloquea: solo se lee un sensor si ya tiene medida lista, y uno por ciclo de control.
#pragma once

#include <Arduino.h>
#include <VL53L0X.h>

class Lasers {
 public:
  enum Id { LEFT = 0, FRONT = 1, RIGHT = 2 };
  // Requiere Wire.begin(). Devuelve false si algun sensor no responde.
  bool begin();
  bool ok(int i) const { return ok_[i]; }
  // Si el sensor i tiene medida nueva la lee en mm (-1 = no valida) y devuelve true.
  bool poll(int i, int& mm);
  // Medida bloqueante (calibracion): espera hasta 100 ms.
  int read(int i);

 private:
  VL53L0X s_[3];
  bool ok_[3] = {false, false, false};
};

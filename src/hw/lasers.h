// Tres VL53L0X en el mismo bus I2C. Todos arrancan en 0x29, asi que se encienden uno a uno
// con XSHUT y se les asigna una direccion propia.
#pragma once

#include <Arduino.h>
#include <VL53L0X.h>

class Lasers {
 public:
  enum Id { LEFT = 0, FRONT = 1, RIGHT = 2 };

  // Requiere Wire.begin() antes. Devuelve false si algun sensor no responde.
  bool begin();

  // Lee los sensores que tengan medida nueva, sin bloquear. Llamar seguido.
  void update();

  // Distancia en mm, o -1 si no hay medida valida (fuera de rango).
  int mm(Id id) const { return mm_[id]; }

  // Espera hasta tener n medidas nuevas de los tres sensores (para leer paredes parado).
  void waitFresh(int n);

  bool ok(Id id) const { return ok_[id]; }

 private:
  VL53L0X sensors_[3];
  int mm_[3] = {-1, -1, -1};
  bool ok_[3] = {false, false, false};
  uint32_t samples_[3] = {0, 0, 0};
};

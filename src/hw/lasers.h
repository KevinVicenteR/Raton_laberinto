// Tres VL53L0X en el mismo bus I2C. Todos arrancan en 0x29, asi que se encienden uno a uno
// con XSHUT y se les asigna una direccion propia.
#pragma once

#include <Arduino.h>
#include <VL53L0X.h>

#include "../config.h"

class Lasers {
 public:
  enum Id { LEFT = 0, FRONT = 1, RIGHT = 2 };

  // Requiere Wire.begin() antes. Devuelve false si algun sensor no responde.
  bool begin();

  // Lee los sensores que tengan medida nueva, sin bloquear. Llamar seguido.
  void update();

  // Distancia filtrada (Kalman) en mm, o -1 si no hay medida valida (fuera de rango).
  int mm(Id id) const { return mm_[map(id)]; }
  // Ultima lectura sin filtrar, o -1.
  int rawMm(Id id) const { return raw_[map(id)]; }

  // Intercambia izquierdo y derecho si estan montados al reves.
  void setSwapped(bool swapped) { swapped_ = swapped; }

  // Espera hasta tener n medidas nuevas de los tres sensores (para leer paredes parado).
  void waitFresh(int n);

  bool ok(Id id) const { return ok_[id]; }

  // Ajustes en marcha: alcance maximo (mm; mas lejos = -1) y eco minimo aceptado (MCPS).
  void setMaxMm(int mm) { maxMm_ = mm; }
  int maxMm() const { return maxMm_; }
  void setSignalLimit(float mcps);
  float signalLimit() const { return signalLimit_; }

 private:
  VL53L0X sensors_[3];
  // Filtro de Kalman por sensor con estado [distancia, velocidad]: sigue la tendencia al acercarse
  // o alejarse de una pared, asi suaviza el ruido casi sin retraso.
  void filter(int i, int z);

  int mm_[3] = {-1, -1, -1};
  int raw_[3] = {-1, -1, -1};
  float kd_[3] = {0, 0, 0};    // distancia estimada (mm)
  float kv_[3] = {0, 0, 0};    // velocidad estimada (mm/s)
  float kp_[3][4] = {};        // covarianza 2x2 [dd, dv, vd, vv]
  uint32_t kt_[3] = {0, 0, 0}; // instante de la ultima medida (us)
  bool kValid_[3] = {false, false, false};
  uint8_t misses_[3] = {0, 0, 0};  // lecturas invalidas seguidas
  uint8_t jumps_[3] = {0, 0, 0};   // saltos grandes seguidos
  bool ok_[3] = {false, false, false};
  uint32_t samples_[3] = {0, 0, 0};
  bool swapped_ = false;
  int maxMm_ = cfg::LASER_MAX_MM;
  float signalLimit_ = cfg::LASER_SIGNAL_LIMIT_MCPS;

  int map(Id id) const { return (swapped_ && id != FRONT) ? 2 - id : id; }
};

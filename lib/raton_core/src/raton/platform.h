// Frontera entre la logica y el hardware. La placa (firmware/) y el simulador (sim/) implementan
// Platform; todo lo demas es el mismo codigo en los dos.
#pragma once

#include <stdint.h>

namespace raton {

class Maze;

struct Range {
  int mm = -1;         // distancia ya corregida (offset), -1 = sin medida valida
  bool fresh = false;  // medida nueva en este ciclo
};

struct Sensors {
  float t = 0;            // s desde el arranque
  float encLeftMm = 0;    // recorrido acumulado de cada rueda (positivo = adelante)
  float encRightMm = 0;
  float gyroDeg = 0;      // rumbo acumulado del giroscopio (positivo = izquierda, antihorario)
  bool gyroFresh = false; // muestra nueva del giroscopio en este ciclo
  Range front, left, right;  // frontal y diagonales originales
  Range sideLeft, sideRight;  // opcionales a 90 grados; sin dato = -1 / fresh=false
};

class Platform {
 public:
  virtual ~Platform() {}
  // Duty de cada motor (-1..1, positivo = adelante).
  virtual void setMotors(float left, float right) = 0;
  // Espera al siguiente periodo de control (Params::dt) y devuelve los sensores de ese instante.
  virtual const Sensors& tick() = 0;
  // Linea de telemetria. important = no se puede perder (eventos, celdas).
  virtual void report(const char* line, bool important) = 0;
  // Orden externa de parar (Bluetooth 'x').
  virtual bool stopRequested() { return false; }
  // Called only with motors at zero. Reset one VL53L0X, never the whole bus/IMU.
  virtual bool recoverRange(int sensor) { (void)sensor; return false; }
  // Guarda el mapa (se llama al llegar a la meta y al volver). Devuelve true si se guardo.
  virtual bool saveMap(const Maze& maze) {
    (void)maze;
    return false;
  }
};

}  // namespace raton


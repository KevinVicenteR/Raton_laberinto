// Interfaz que separa el algoritmo del hardware.
// La implementan el robot real (src/robot_hw), el simulador (sim/sim_robot) y el adaptador de mms.
#pragma once

#include "floodfill.h"
#include "maze.h"

namespace raton {

struct Walls {
  bool left;
  bool front;
  bool right;
};

enum class RunMode { Search, Speed };

class Robot {
 public:
  virtual ~Robot() {}

  // Se llama con el robot parado en el centro de una celda.
  virtual Walls readWalls() = 0;

  // Avanza celdas completas y termina parado en el centro de la ultima.
  virtual void moveForward(int cells) = 0;
  virtual void turnLeft() = 0;
  virtual void turnRight() = 0;
  virtual void turnAround() = 0;

  // Opcionales: velocidad segun la fase, y aviso despues de cada paso (pantalla, log, mms).
  virtual void setRunMode(RunMode) {}
  virtual void onStep(const Maze&, const FloodFill&, Cell, Dir) {}

  // Devuelve true para cancelar la carrera (boton, robot levantado, etc.).
  virtual bool aborted() { return false; }
};

}  // namespace raton

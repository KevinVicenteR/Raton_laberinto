// Modos 1 y 2: seguir la pared derecha o izquierda. Solo usa los laseres: no depende del tamano de las
// celdas ni de la pista.
#pragma once

#include "seguridad/maneuvers.h"
#include "comunicacion/reporter.h"
#include "seguridad/robot_hw.h"

class WallFollower {
 public:
  WallFollower(RobotHW& robot, Reporter& log, Maneuvers& maneuvers) : robot_(robot), log_(log), maneuvers_(maneuvers) {}
  // side: +1 sigue la pared izquierda, -1 la derecha. Con maxMm > 0 para tras recorrer esa distancia.
  void run(int side, float maxMm);

 private:
  RobotHW& robot_;
  Reporter& log_;
  Maneuvers& maneuvers_;
};

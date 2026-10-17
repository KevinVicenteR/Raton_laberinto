// Maniobras de recuperacion comunes al seguidor de paredes y al modo 5.
#pragma once

#include "app/reporter.h"
#include "robot/robot_hw.h"

class Maneuvers {
 public:
  Maneuvers(RobotHW& robot, Reporter& log) : robot_(robot), log_(log) {}
  // Hay pared delante de verdad (no un atasco con el frente libre).
  bool frontBlocked();
  // Atascado con el frente libre (el morro roza una esquina): atras y enderezarse hacia el lado libre.
  void unstick();
  // Callejon sin salida: sale marcha atras hasta el primer hueco y entra en el; si no lo encuentra,
  // media vuelta.
  void deadEnd();
  // Rozando un lateral: un poco atras y enderezarse hacia el otro lado.
  void nudge(bool rubbingLeft);

 private:
  RobotHW& robot_;
  Reporter& log_;
};

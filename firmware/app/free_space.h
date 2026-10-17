// Modo 5, espacio libre: avanza recto vigilando los dos lados. Al ver terminar una pared lateral (hueco)
// o al llegar a una pared delante, lleva el eje al centro del cruce, se para, mira los tres laseres
// quieto y va hacia donde puntua mejor (espacio libre, novedad, giro, meta y salidas ya usadas).
#pragma once

#include "app/maneuvers.h"
#include "app/reporter.h"
#include "app/wall_follower.h"
#include "robot/robot_hw.h"

class FreeSpaceNavigator {
 public:
  FreeSpaceNavigator(RobotHW& robot, Reporter& log, Maneuvers& maneuvers, WallFollower& follower)
      : robot_(robot), log_(log), maneuvers_(maneuvers), follower_(follower) {}
  // Devuelve true si llega a la meta (cuando hay meta: robot.setGoal).
  bool run();

 private:
  // Distancia del costado a la pared en el cruce; 0 si esta pegado (sin lectura tras una muy corta).
  int sideSpace(Lasers::Id id);

  RobotHW& robot_;
  Reporter& log_;
  Maneuvers& maneuvers_;
  WallFollower& follower_;
};

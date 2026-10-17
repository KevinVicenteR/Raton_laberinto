// Aplicacion del raton: arranque del hardware, eleccion de modo y ejecucion de cada modo.
//   1 = seguir pared derecha, 2 = seguir pared izquierda, 3 = laberinto por celdas (pista desconocida),
//   4 = ruta mas corta del laberinto de la competencia, 5 = espacio libre.
#pragma once

#include "app/free_space.h"
#include "app/maneuvers.h"
#include "app/maze_mission.h"
#include "app/reporter.h"
#include "app/wall_follower.h"
#include "hw/display.h"
#include "hw/encoders.h"
#include "hw/gyro.h"
#include "hw/lasers.h"
#include "hw/motors.h"
#include "robot/robot_hw.h"

class App {
 public:
  App();
  void setup();
  void loop();

 private:
  void fail(const char* msg);
  // Cuenta atras con el raton quieto y calibra el giroscopio antes de moverse.
  void waitAutoStart(const char* title, uint32_t ms);
  // Pasadas de la mano delante del laser frontal durante MODE_SELECT_MS: cada una cambia de modo.
  int selectMode();
  // Parado: espera la orden de arrancar ('s' por Bluetooth o Serie) o una pasada de la mano.
  int waitForStart(int mode);
  void runMode(int mode);

  Motors motors_;
  Encoders encoders_;
  Lasers lasers_;
  Gyro gyro_;
  Display display_;
  RobotHW robot_;
  Reporter log_;
  Maneuvers maneuvers_;
  WallFollower follower_;
  FreeSpaceNavigator freeSpace_;
  MazeMission maze_;
  bool firstRun_ = true;
  int mode_ = 1;
};

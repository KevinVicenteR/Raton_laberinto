// Aplicacion del raton: arranque del hardware, eleccion de modo y ejecucion de cada modo.
//   1 = seguir pared derecha, 2 = seguir pared izquierda, 3 = laberinto por celdas (pista desconocida),
//   4 = ruta mas corta del laberinto de la competencia, 5 = espacio libre.
#pragma once

#include "modos/free_space.h"
#include "seguridad/maneuvers.h"
#include "exploracion/maze_mission.h"
#include "comunicacion/reporter.h"
#include "modos/wall_follower.h"
#include "hardware/display.h"
#include "hardware/encoders.h"
#include "hardware/gyro.h"
#include "hardware/lasers.h"
#include "hardware/motors.h"
#include "seguridad/robot_hw.h"

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
  void modo1();
  void modo2();
  void modo3();
  void modo4();
  void modo5();
  void modo6();


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

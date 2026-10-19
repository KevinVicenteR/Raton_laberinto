// Modos por celdas:
//  3  pista desconocida. Sin meta: MAPEA toda la pista (rejilla grande con la salida en el centro) y
//     vuelve a la salida; luego, con una meta elegida en la web, va por la ruta mas corta del mapa
//     aprendido. Con meta en el marco de la competencia: explora hasta ella, busca la ruta demostrada
//     (Adachi 1) y vuelve; el siguiente arranque es la carrera rapida.
//  4  ruta mas corta del laberinto del reglamento (competition_maze.h); si una pared no coincide,
//     sigue explorando por celdas desde ahi.
#pragma once

#include "comunicacion/reporter.h"
#include "exploracion/cell_explorer.h"
#include "hardware/display.h"
#include "mapa/cell_map.h"
#include "seguridad/robot_hw.h"

class MazeMission {
 public:
  MazeMission(RobotHW& robot, Reporter& log, Display& display);
  void learnTrack();
  void eraseMap();
  void showSavedMap();
  void solveUnknown();    // modo 3
  void runCompetition();  // modo 4

 private:
  void reachedGoal(const char* how);
  raton::Cell goalCell() const;

  RobotHW& robot_;
  Reporter& log_;
  Display& display_;
  raton::CellMap map_;
  raton::CellExplorer explorer_;
  bool learnedFrame_ = false;  // el mapa actual es el de una pista desconocida (rejilla grande)
  bool mapReady_ = false;      // ya llego una vez a la meta: el siguiente arranque es carrera rapida
};

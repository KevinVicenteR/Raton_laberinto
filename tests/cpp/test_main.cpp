#include "exploracion/orquestador.h"
// Pruebas del nucleo portable (lib/raton_core) en el PC. Se corren con ctest.
// El explorador por celdas es el MISMO codigo que lleva el firmware; aqui conduce un robot simulado
// que lee las paredes de un laberinto de tests/mazes.
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <queue>
#include <string>
#include <vector>

#include "../../firmware/mapa/competition_maze.h"
#include "odometria/motion_progress.h"
#include "velocidad/adaptive_speed.h"
#include "seguridad/orquestador.h"
#include "exploracion/cell_explorer.h"
#include "paredes/track_geometry.h"
#include "mapa/cell_map.h"
#include "comunicacion/frame.h"
#include "comunicacion/line_queue.h"

#include "centrado/control.h"
#include "velocidad/control.h"
#include "giros/perfil.h"
#include "paredes/cell_window.h"
#include "../../firmware/centrado/params.h"
#include "../../firmware/velocidad/params.h"
#include "../../firmware/giros/params.h"

namespace {

int failures = 0;

#define CHECK(cond)                                                          \
  do {                                                                       \
    if (!(cond)) {                                                           \
      std::cerr << __FILE__ << ":" << __LINE__ << ": fallo: " #cond << '\n'; \
      failures++;                                                            \
    }                                                                        \
  } while (0)

using raton::Cell;
using raton::Dir;
using raton::makeCell;

#include "exploracion/fixtures.h"
#include "comunicacion/pruebas.h"
#include "seguridad/pruebas.h"
#include "odometria/pruebas.h"
#include "paredes/pruebas.h"
#include "mapa/pruebas.h"
#include "velocidad/pruebas.h"
#include "centrado/pruebas.h"
#include "giros/pruebas.h"

}  // namespace

int main() {
  testAdaptiveSpeed();
  testSpeedPI();
  testSteering();
  testAngularProfile();
  testWallMedian();
  testMapPersistence();
  testFrame();
  testLineQueue();
  testMission();
  testOrquestadores();
  testMotionProgress();
  testTrackGeometry();
  testCellMap();
  testExploration();
  if (failures) {
    std::cerr << failures << " fallos\n";
    return 1;
  }
  std::cout << "Nucleo: todas las pruebas pasan\n";
  return 0;
}

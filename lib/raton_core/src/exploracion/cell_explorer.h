// Explorador por celdas (modos 3 y 4): lee paredes, inunda y avanza por tramos rectos leyendo en
// marcha; mapea pistas desconocidas, busca la ruta demostrada (Adachi 1) y hace la carrera rapida.
#pragma once

#include "exploracion/ports.h"
#include "mapa/cell_map.h"

namespace raton {

struct ExplorerParams {
  float exploreSpeed = 132;    // mm/s explorando
  int maxNoProgress = 3;       // tramos seguidos sin avanzar antes de pararse (SIN_AVANCE)
  int searchMaxTargets = 12;   // busqueda Adachi: celdas sin ver de la mejor ruta como maximo
  float frontWallFactor = 0.28f;
  int laserMaxMm = 300;
};

class CellExplorer : public CellObserver {
 public:
  CellExplorer(CellDriver& robot, MissionLog& log, CellMap& map, const ExplorerParams& params);

  // Paredes de la celda leidas girando con el frontal. Con measure, ademas mide el pasillo.
  void senseTurning(Cell pos, Dir heading, bool measure = false);
  // Salida leida quieto (sin girar): calibra el pasillo con los laterales y decide las paredes.
  void senseStill(Cell pos, Dir heading);
  // Va a goal por celdas; true al llegar.
  bool exploreTo(Cell& pos, Dir& heading, Cell goal);
  // Carrera por el mapa: false si algo no coincide (pos queda donde se paro).
  bool runRoute(Cell& pos, Dir& heading, Cell goal, float vmax);
  // Recorre toda la pista que se puede alcanzar (orquestador de mision) y vuelve a la salida.
  bool mapTrack(Cell& pos, Dir& heading);
  // Busqueda Adachi 1: lee las celdas sin ver de la mejor ruta y vuelve a la salida explorando.
  void adachiSearch(Cell& pos, Dir& heading, Cell start, Cell goal);

  bool onCellSides(int index, bool leftWall, bool rightWall, int frontMm) override;
  bool knownCell(int index) const override {
    Cell c = runFrom_;
    for (int i = 0; i <= index; ++i) c = step(c, runDir_);
    return map_.seen(c);
  }

 private:
  void applyRun(Cell from, Dir dir, int moved);
  void reportCell(Cell c);
  void decidef(const char* fmt, ...);

  CellDriver& robot_;
  MissionLog& log_;
  CellMap& map_;
  ExplorerParams p_;
  // Tramo en marcha (para onCellSides).
  Cell runFrom_{0, 0};
  Dir runDir_ = NORTH;
  Cell runGoal_{0, 0};
};

}  // namespace raton

// Planificacion sobre el mapa:
//  - flood(): distancia en celdas desde cada celda a un conjunto de objetivos (inundacion BFS).
//  - bestRoute(): ruta mas rapida con coste por giro (Dijkstra sobre celda + rumbo), para la carrera.
//  - nextStep(): siguiente direccion explorando (la vecina mas cerca del objetivo; empates: recto,
//    sin visitar, menos visitas).
// Todo con memoria fija: 32 x 32 celdas como mucho, menos de 1 ms en la placa para 19 x 10.
#pragma once

#include <stdint.h>

#include "raton/maze.h"

namespace raton {

const uint16_t kInf = 0xFFFF;

struct CellSet {
  uint8_t bits[Maze::kMax * Maze::kMax / 8] = {};
  int count = 0;
  void clear();
  void add(int idx);
  bool has(int idx) const { return (bits[idx >> 3] >> (idx & 7)) & 1; }
};

// dist[i] = celdas hasta el objetivo mas cercano (kInf si no se llega).
void flood(const Maze& m, const CellSet& targets, bool optimistic, uint16_t* dist);
void goalSet(const Maze& m, CellSet& out);

struct Route {
  static const int kMaxSteps = Maze::kMax * Maze::kMax;
  Dir steps[kMaxSteps];  // direccion de cada paso de celda a celda
  int length = 0;
  uint32_t cost = 0;     // en unidades de coste (ver turnCost)
  bool ok = false;
};

// Ruta de (x, y, heading) a cualquier celda de targets. Coste: 10 por celda recta, + turnCost por
// cada giro de 90 grados, + 3 * turnCost por media vuelta (solo al salir).
void bestRoute(const Maze& m, int x, int y, Dir heading, const CellSet& targets, bool optimistic, int turnCost,
               Route& out);

// Siguiente direccion explorando hacia targets desde (x, y) mirando a heading. -1 si no hay camino.
int nextStep(const Maze& m, int x, int y, Dir heading, const uint16_t* dist, bool optimistic);

}  // namespace raton

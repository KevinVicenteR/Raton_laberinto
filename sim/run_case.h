// Un caso de simulacion completo: exploracion (meta, ruta mas corta y vuelta) y carrera rapida.
#pragma once

#include <stdint.h>

#include <string>

#include "raton/maze.h"
#include "raton/params.h"
#include "sim.h"

namespace sim {

struct CaseResult {
  bool goal1 = false;        // llego a la meta explorando
  float goalTime1 = -1;      // s
  float exploreTime = 0;     // s de toda la exploracion
  bool backHome = false;
  bool proven = false;
  bool goal2 = false;        // carrera
  float goalTime2 = -1;
  int contacts = 0;          // choques con paredes (cuerpo)
  float contactTime = 0;
  float maxArea = 0;         // s maximos en la misma zona (posicion real)
  int recoveries = 0;
  int escapes = 0;
  int wrongWalls = 0;        // paredes del mapa final que no coinciden con la realidad
  // Cumple el reglamento: llega a la meta en las dos pasadas y nunca pasa 30 s en la misma zona.
  bool ok() const { return goal1 && goal2 && maxArea < 30.0f; }
  // Ademas sin tocar ninguna pared.
  bool clean() const { return ok() && contacts == 0; }
};

CaseResult runCase(const raton::Maze& truth, const raton::Params& p, const Physical& ph, uint32_t seed, bool verbose,
                   bool race = true, bool trace = false, float timeLimit = 900.0f);

// Fisica con errores realistas de nivel 0 (ideal) a 3 (malo).
Physical perturbed(const raton::Params& p, int level, uint32_t seed);

std::string describe(const CaseResult& r);

}  // namespace sim

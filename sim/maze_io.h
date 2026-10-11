// Lectura/escritura de laberintos en formato ASCII (el de github.com/micromouseonline/mazefiles):
//
//   o---o---o
//   |       |
//   o   o---o
//   |   |   |
//   o---o---o
#pragma once

#include <cstdint>
#include <iosfwd>
#include <string>

#include "maze.h"

namespace sim {

bool parseMaze(std::istream& in, raton::Maze& out, std::string& error);
bool loadMaze(const std::string& path, raton::Maze& out, std::string& error);
void writeMaze(std::ostream& out, const raton::Maze& maze);

// Laberinto aleatorio resoluble (backtracker recursivo). La salida (0,0) solo abre al norte,
// como en las reglas clasicas. extraOpenings quita paredes al azar para crear varios caminos.
raton::Maze generateMaze(int width, int height, uint32_t seed, int extraOpenings);

}  // namespace sim

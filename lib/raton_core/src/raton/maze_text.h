// Laberintos en texto (fila de arriba primero): "+---+" paredes horizontales, "|" verticales.
#pragma once

#include "raton/maze.h"

namespace raton {

// rows: 2 * alto + 1 lineas de 4 * ancho + 1 caracteres. Pone todas las paredes como conocidas.
bool parseMazeText(const char* const* rows, int nrows, Maze& out);
// Escribe el mapa (paredes conocidas cerradas; "?" desconocidas) en out (tamano >= (4w+2)(2h+1)+1).
void printMaze(const Maze& m, char* out, int size, int robotX = -1, int robotY = -1);

}  // namespace raton

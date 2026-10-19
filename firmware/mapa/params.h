#pragma once
#include <stdint.h>

namespace cfg {


// ---------- Laberinto ----------
// Reglamento: pista de ~1.5 x 3 m, pasillos de ~14 cm (celda ~15.5 cm con la pared). CALIBRAR con la pista real.
const int MAZE_WIDTH = 19;           // celdas a lo ancho (3 m)
const int MAZE_HEIGHT = 10;          // celdas a lo alto (1.5 m)
const float CELL_MM = 155.0f;        // de centro a centro de celda
// Salida: en el ejemplo del reglamento, "Inicio" esta abajo a la derecha, saliendo hacia arriba (norte).
const int START_X = MAZE_WIDTH - 1;
const int START_Y = 0;
// Meta: celda (x, y) contando desde abajo a la izquierda; -1 = el centro. Se puede cambiar desde la web.
const int GOAL_X = -1;
const int GOAL_Y = -1;

}  // namespace cfg

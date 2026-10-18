// Laberinto del ejemplo del reglamento (19 x 10 celdas), sacado de la imagen de las bases.
// Dibujo con la fila de arriba primero: "---" = pared horizontal, "|" = pared vertical.
// Si el de la competicion cambia, se corrige aqui a mano. La salida (INICIO) es la celda de abajo
// a la derecha saliendo hacia arriba; la meta, la entrada del hueco con META (celda 4,2 desde abajo
// a la izquierda). El raton igualmente mira las paredes en cada celda y corrige el mapa si no coincide.
#pragma once

namespace competition {

const int WIDTH = 19;
const int HEIGHT = 10;
const int START_X = 18;
const int START_Y = 0;
const int GOAL_X = 4;
const int GOAL_Y = 2;

const char* const MAP[2 * HEIGHT + 1] = {
    "+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+",
    "|                       |           |                   |           |       |",
    "+   +   +---+   +---+   +   +   +---+   +---+   +---+   +   +---+   +---+   +",
    "|   |   |                   |               |               |               |",
    "+---+   +   +   +---+   +---+   +---+---+---+   +---+   +   +   +---+---+   +",
    "|   |   |   |   |   |   |           |               |   |               |   |",
    "+   +   +   +---+   +---+---+---+   +---+   +---+   +---+---+   +---+   +   +",
    "|           |   |           |   |               |   |           |       |   |",
    "+---+   +   +   +   +---+---+   +---+---+   +   +   +   +   +---+   +---+   +",
    "|   |   |           |   |       |           |   |       |                   |",
    "+   +   +   +   +---+   +---+   +---+   +---+   +---+---+---+---+   +---+   +",
    "|           |               |       |       |   |   |                   |   |",
    "+   +---+   +---+   +---+   +   +   +---+   +   +   +   +---+---+   +---+   +",
    "|               |       |   |   |       |                   |               |",
    "+   +---+   +---+   +---+   +   +---+   +---+   +   +---+   +   +   +---+   +",
    "|           |       |   |   |   |   |       |   |               |       |   |",
    "+---+   +---+   +   +   +   +   +   +   +   +   +---+---+---+   +   +   +   +",
    "|       |   |       |   |       |       |       |                   |   |   |",
    "+   +---+   +---+---+   +   +   +---+   +---+   +   +---+   +---+   +   +   +",
    "|                           |   |               |           |           |   |",
    "+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+",
};

}  // namespace competition

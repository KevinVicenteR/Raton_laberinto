// Flash interna (TDBStore en los ultimos 32 KiB, reservados por nano33ble_map.ld): parametros calibrados y
// mapa del laberinto. Cada registro lleva marca, version, tamano y CRC32: si algo no cuadra se ignora.
#pragma once

#include "raton/maze.h"
#include "raton/params.h"

namespace storage {

bool saveParams(const raton::Params& p);
bool loadParams(raton::Params& p);
bool saveMap(const raton::Maze& m);
bool loadMap(raton::Maze& m);
bool eraseMap();
bool eraseParams();

}  // namespace storage

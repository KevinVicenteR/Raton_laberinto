#pragma once

#include "paredes/track_geometry.h"
#include "mapa/cell_map.h"

// TDBStore transaccional en 32 KiB reservados por nano33ble_map.ld.
namespace mapStorage {
bool save(const raton::CellMap& map, const raton::TrackGeometry& geometry);
bool load(raton::CellMap& map, raton::TrackGeometry& geometry);
bool erase();
}

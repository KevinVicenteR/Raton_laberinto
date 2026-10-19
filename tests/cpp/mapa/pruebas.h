void testCellMap() {
  raton::CellMap m;
  m.clear(nullptr, 4, 3);
  CHECK(m.wall(makeCell(0, 0), raton::SOUTH) && m.wall(makeCell(3, 2), raton::EAST));  // borde
  CHECK(!m.wall(makeCell(1, 1), raton::NORTH));
  m.setWall(makeCell(1, 1), raton::NORTH, true);
  CHECK(m.wall(makeCell(1, 2), raton::SOUTH));  // la otra cara de la pared
  m.flood(makeCell(0, 0));
  CHECK(m.dist(makeCell(0, 0)) == 0 && m.dist(makeCell(3, 2)) == 5);
  // Pasillo sin salida (1,2)->(2,2)->(3,2) cerrado por debajo: se poda entero.
  for (int x = 1; x < 4; ++x) m.setWall(makeCell(x, 2), raton::SOUTH, true);
  for (int x = 0; x < 4; ++x)
    for (int y = 0; y < 3; ++y) m.markSeen(makeCell(x, y));
  m.setWall(makeCell(0, 2), raton::EAST, false);
  m.pruneDeadEnds();
  CHECK(m.dead(makeCell(3, 2)) && m.dead(makeCell(2, 2)) && m.dead(makeCell(1, 2)));
  CHECK(!m.dead(makeCell(0, 0)));
  Cell out{};
  CHECK(!m.nearestUnseen(makeCell(0, 0), out));  // todo visto

  // La salida podada no implica que la ruta optimista este demostrada.
  m.clear(nullptr, 3, 1);
  m.markSeen(makeCell(0, 0));
  m.setKeep(1, makeCell(2, 0));
  m.pruneDeadEnds();
  CHECK(m.dead(makeCell(0, 0)));
  CHECK(m.unseenOnRoute(makeCell(0, 0), makeCell(2, 0), makeCell(2, 0), out));
  CHECK(out == makeCell(2, 0) || out == makeCell(1, 0));
}

// Explora hasta la meta, busca la ruta demostrada (Adachi) y vuelve; la carrera rapida tiene que ir por
// el camino mas corto real.
void testMapPersistence() {
  raton::CellMap map, restored;
  map.clear(nullptr, 40, 40);
  map.setWall(makeCell(20, 20), raton::NORTH, true);
  map.markSeen(makeCell(20, 20));
  map.addVisit(makeCell(20, 20));
  std::vector<uint8_t> bytes(raton::CellMap::kSerializedMax);
  CHECK(map.serialize(bytes.data(), 1) == 0);
  const size_t size = map.serialize(bytes.data(), bytes.size());
  CHECK(restored.deserialize(bytes.data(), size));
  CHECK(restored.wall(makeCell(20, 20), raton::NORTH));
  CHECK(restored.seen(makeCell(20, 20)));
  CHECK(restored.visits(makeCell(20, 20)) == 1);
  std::vector<uint8_t> again(bytes.size());
  CHECK(restored.serialize(again.data(), again.size()) == size);
  CHECK(bytes == again);
  CHECK(!restored.deserialize(bytes.data(), size - 1));
  for (size_t i = 0; i < size; ++i) {
    bytes[i] ^= 1;
    CHECK(!restored.deserialize(bytes.data(), size));
    bytes[i] ^= 1;
  }
  CHECK(restored.seen(makeCell(20, 20))); // un rechazo no modifica el mapa
  // Serpiente de 1600 celdas: una distancia de 8 bits truncaba la ruta a 254.
  map.clear([](int, int, int) { return true; }, 40, 40);
  for (int y = 0; y < 40; ++y) {
    for (int x = 0; x < 39; ++x) map.setWall(makeCell(x, y), raton::EAST, false);
    if (y < 39) map.setWall(makeCell(y % 2 == 0 ? 39 : 0, y), raton::NORTH, false);
  }
  map.flood(makeCell(0, 39));
  CHECK(map.dist(makeCell(0, 0)) == 1599);
}


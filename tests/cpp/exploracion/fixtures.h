// ---------------------------------------------------------------- laberinto de referencia

// Dibujo ASCII con la fila de arriba primero ("---" pared horizontal, "|" vertical).
struct Truth {
  int width = 0;
  int height = 0;
  std::vector<std::string> rows;

  static Truth load(const std::string& path) {
    Truth t;
    std::ifstream in(path);
    for (std::string line; std::getline(in, line);) {
      if (!line.empty() && line.back() == '\r') line.pop_back();
      if (!line.empty()) t.rows.push_back(line);
    }
    t.height = (static_cast<int>(t.rows.size()) - 1) / 2;
    t.width = t.rows.empty() ? 0 : (static_cast<int>(t.rows[0].size()) - 1) / 4;
    return t;
  }
  bool inside(int x, int y) const { return x >= 0 && y >= 0 && x < width && y < height; }
  bool wall(int x, int y, int d) const {
    if (!inside(x, y)) return true;
    const int r = height - 1 - y;
    switch (d) {
      case raton::NORTH: return rows[2 * r][4 * x + 2] == '-';
      case raton::SOUTH: return rows[2 * r + 2][4 * x + 2] == '-';
      case raton::WEST: return rows[2 * r + 1][4 * x] == '|';
      default: return rows[2 * r + 1][4 * x + 4] == '|';
    }
  }
  // Pasos minimos entre dos celdas por el laberinto real.
  int shortest(Cell from, Cell to) const {
    std::vector<int> dist(width * height, -1);
    std::queue<Cell> q;
    dist[from.y * width + from.x] = 0;
    q.push(from);
    while (!q.empty()) {
      const Cell c = q.front();
      q.pop();
      for (int d = 0; d < 4; ++d) {
        const Cell n = raton::step(c, static_cast<Dir>(d));
        if (wall(c.x, c.y, d) || !inside(n.x, n.y) || dist[n.y * width + n.x] >= 0) continue;
        dist[n.y * width + n.x] = dist[c.y * width + c.x] + 1;
        q.push(n);
      }
    }
    return dist[to.y * width + to.x];
  }
};

// ---------------------------------------------------------------- robot simulado (puertos)

class SimRobot : public raton::CellDriver {
 public:
  // offset: celda del mapa del raton que corresponde a la (0,0) del laberinto real.
  SimRobot(const Truth& truth, Cell offset, Cell start, Dir heading)
      : truth_(truth), offset_(offset), pos_(start), heading_(heading),
        geom_(raton::GeometryParams(), 140.0f, 152.0f, 31.0f, 50) {}

  bool aborted() override { return steps_ > 5000; }
  void setState(const char*) override {}
  raton::TrackGeometry& geometry() override { return geom_; }
  void turnInCell(float degrees) override {
    const int quarters = static_cast<int>(std::lround(degrees / 90.0f));  // + = izquierda
    heading_ = static_cast<Dir>(((heading_ - quarters) % 4 + 4) % 4);
    ++turns_;
    ++steps_;
  }
  int runCells(int cells, float, bool* unexpected, raton::CellObserver* observer) override {
    if (unexpected) *unexpected = false;
    sides_.clear();
    int moved = 0;
    for (int i = 0; i < cells; ++i) {
      if (wallAt(heading_)) {
        if (unexpected) *unexpected = true;
        break;
      }
      pos_ = raton::step(pos_, heading_);
      ++moved;
      ++cellsDriven_;
      ++steps_;
      const bool left = wallAt(raton::leftOf(heading_)), right = wallAt(raton::rightOf(heading_));
      sides_.push_back({left, right});
      const int front = wallAt(heading_) ? 100 : -1;
      if (observer && !observer->onCellSides(i, left, right, front)) break;
    }
    return moved;
  }
  bool cellSides(int i, bool& left, bool& right) const override {
    if (i < 0 || i >= static_cast<int>(sides_.size())) return false;
    left = sides_[i].first;
    right = sides_[i].second;
    return true;
  }
  bool frontWallStopped() override { return wallAt(heading_); }
  int stableFrontMm() override { return wallAt(heading_) ? geom_.frontTurnMm() : -1; }
  bool calibrateStill(int& left, int& right, int& front) override {
    left = wallAt(raton::leftOf(heading_)) ? 30 : -1;
    right = wallAt(raton::rightOf(heading_)) ? 30 : -1;
    front = stableFrontMm();
    return false;
  }
  void alignToFront() override {}

  Cell pos() const { return pos_; }
  int cellsDriven() const { return cellsDriven_; }
  void resetCount() { cellsDriven_ = 0; }

 private:
  bool wallAt(Dir d) const { return truth_.wall(pos_.x - offset_.x, pos_.y - offset_.y, d); }

  const Truth& truth_;
  Cell offset_;
  Cell pos_;
  Dir heading_;
  raton::TrackGeometry geom_;
  std::vector<std::pair<bool, bool>> sides_;
  int turns_ = 0;
  int steps_ = 0;
  int cellsDriven_ = 0;
};

class SilentLog : public raton::MissionLog {
 public:
  void decide(const char*) override { ++decisions; }
  void event(const char* line) override {
    if (strncmp(line, "CELL ", 5) == 0) ++cells;
  }
  int decisions = 0;
  int cells = 0;
};

std::string mazePath(const char* name) { return std::string(RATON_MAZES_DIR) + "/" + name; }


void checkSolve(const Truth& truth, Cell goal, Cell start = makeCell(0, 0)) {
  CHECK(truth.width > 0);
  SimRobot robot(truth, makeCell(0, 0), start, raton::NORTH);
  SilentLog log;
  raton::CellMap map;
  map.clear(nullptr, truth.width, truth.height);
  raton::CellExplorer explorer(robot, log, map, raton::ExplorerParams());
  Cell pos = start;
  Dir heading = raton::NORTH;
  CHECK(explorer.exploreTo(pos, heading, goal));
  CHECK(pos == goal && robot.pos() == goal);
  std::cout << "solve " << truth.width << "x" << truth.height << ": " << robot.cellsDriven() << " celdas, " << log.decisions << " decisiones\n";
  // La prueba exige demostrar la ruta; el limite operativo de 12 destinos puede
  // agotarse antes en una pista grande sin que eso sea un error de navegacion.
  raton::ExplorerParams searchParams;
  searchParams.searchMaxTargets = truth.width * truth.height;
  raton::CellExplorer search(robot, log, map, searchParams);
  search.adachiSearch(pos, heading, start, goal);
  CHECK(pos == start && robot.pos() == start);
  Cell unseen{};
  CHECK(!map.unseenOnRoute(start, goal, pos, unseen));

  // Carrera rapida desde la salida mirando al norte.
  SimRobot racer(truth, makeCell(0, 0), start, raton::NORTH);
  raton::CellExplorer race(racer, log, map, raton::ExplorerParams());
  pos = start;
  heading = raton::NORTH;
  map.setAvoidDead(true);  // la carrera debe admitir extremos podados por una busqueda previa
  CHECK(race.runRoute(pos, heading, goal, 250));
  CHECK(pos == goal);
  CHECK(racer.cellsDriven() == truth.shortest(start, goal));
  std::cout << "race " << truth.width << "x" << truth.height << ": " << racer.cellsDriven() << " celdas\n";
}

// Pista desconocida: mapea todo desde el centro de la rejilla grande y vuelve a la salida.
void checkMapTrack(const Truth& truth, Cell start = makeCell(0, 0)) {
  const int c = raton::CellMap::kMax / 2;
  const Cell offset = makeCell(c, c);  // la (0,0) real queda en el centro de la rejilla
  const Cell origin = makeCell(c + start.x, c + start.y);
  SimRobot robot(truth, offset, origin, raton::NORTH);
  SilentLog log;
  raton::CellMap map;
  map.clear(nullptr, raton::CellMap::kMax, raton::CellMap::kMax);
  raton::CellExplorer explorer(robot, log, map, raton::ExplorerParams());
  Cell pos = origin;
  Dir heading = raton::NORTH;
  CHECK(explorer.mapTrack(pos, heading));
  CHECK(pos == origin);
  std::cout << "mapTrack " << truth.width << "x" << truth.height << ": " << robot.cellsDriven() << " celdas, " << map.seenCount() << " vistas\n";
  CHECK(map.seenCount() == truth.width * truth.height);
  int wrong = 0;
  for (int x = 0; x < truth.width; ++x)
    for (int y = 0; y < truth.height; ++y)
      for (int d = 0; d < 4; ++d)
        if (map.wall(makeCell(x + c, y + c), d) != truth.wall(x, y, d)) ++wrong;
  CHECK(wrong == 0);
  CHECK(robot.cellsDriven() <= 4 * truth.width * truth.height);
}


void testExploration() {
  checkSolve(Truth::load(mazePath("impar_05.txt")), makeCell(2, 2));
  checkSolve(Truth::load(mazePath("clasico_01.txt")), makeCell(7, 7));
  checkMapTrack(Truth::load(mazePath("impar_05.txt")));
  checkMapTrack(Truth::load(mazePath("clasico_01.txt")));
  Truth competitionTruth;
  competitionTruth.width = competition::WIDTH;
  competitionTruth.height = competition::HEIGHT;
  for (const char* row : competition::MAP) competitionTruth.rows.emplace_back(row);
  const Cell competitionStart = makeCell(competition::START_X, competition::START_Y);
  checkSolve(competitionTruth, makeCell(competition::GOAL_X, competition::GOAL_Y), competitionStart);
  checkMapTrack(competitionTruth, competitionStart);
}

void testMission() {
  using namespace raton;
  OrquestadorMision m;
  EstadoMision e{};
  CHECK(m.decidir(e) == AccionMision::LeerSalida);
  e.salidaLeida = true;
  e.hayFrontera = true;
  CHECK(m.decidir(e) == AccionMision::Explorar);
  e.fallosSeguidos = 3;
  CHECK(m.decidir(e) == AccionMision::Recuperar);
  e.fallosSeguidos = 0;
  e.hayFrontera = false;
  CHECK(m.decidir(e) == AccionMision::Terminar);
  e.hayMeta = true;
  e.mapaAprendido = true;
  CHECK(m.decidir(e) == AccionMision::RutaCorta);
  e.abortado = true;
  CHECK(m.decidir(e) == AccionMision::Terminar);

}

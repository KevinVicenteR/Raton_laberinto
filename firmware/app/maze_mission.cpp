#include "app/maze_mission.h"

#include "app/competition_maze.h"
#include "comms/telemetry.h"
#include "config.h"

namespace {

static_assert(competition::WIDTH <= raton::CellMap::kMax && competition::HEIGHT <= raton::CellMap::kMax,
              "el mapa de la competencia no cabe en la rejilla");

raton::ExplorerParams explorerParams() {
  raton::ExplorerParams p;
  p.exploreSpeed = cfg::EXPLORE_SPEED;
  p.maxNoProgress = cfg::MAX_NO_PROGRESS;
  p.searchMaxTargets = cfg::SEARCH_MAX_TARGETS;
  p.frontWallFactor = cfg::FRONT_WALL_FACTOR;
  p.laserMaxMm = cfg::LASER_MAX_MM;
  return p;
}

bool competitionWall(int x, int y, int dir) {
  using namespace competition;
  const int r = HEIGHT - 1 - y;
  switch (dir) {
    case raton::NORTH: return MAP[2 * r][4 * x + 2] == '-';
    case raton::SOUTH: return MAP[2 * r + 2][4 * x + 2] == '-';
    case raton::WEST: return MAP[2 * r + 1][4 * x] == '|';
    default: return MAP[2 * r + 1][4 * x + 4] == '|';
  }
}

}  // namespace

MazeMission::MazeMission(RobotHW& robot, Reporter& log, Display& display)
    : robot_(robot), log_(log), display_(display), explorer_(robot, log, map_, explorerParams()) {}

raton::Cell MazeMission::goalCell() const {
  const OperatorRequests& req = robot_.requests();
  if (req.goalX >= 0) return raton::makeCell(req.goalX, req.goalY);
  return raton::makeCell(competition::GOAL_X, competition::GOAL_Y);
}

void MazeMission::reachedGoal(const char* how) {
  robot_.setState("META");
  display_.show("META!", how);
  telemetry::println("META");
}

void MazeMission::solveUnknown() {
  robot_.setGrid(true);
  raton::Dir heading = raton::NORTH;
  const raton::Cell center = raton::makeCell(raton::CellMap::kMax / 2, raton::CellMap::kMax / 2);
  const OperatorRequests& req = robot_.requests();
  if (req.goalX < 0) {
    // ENTRENAR: recorrer y mapear la pista desconocida. El mapa se guarda para la ruta mas corta.
    map_.clear(nullptr, raton::CellMap::kMax, raton::CellMap::kMax);
    learnedFrame_ = true;
    raton::Cell pos = center;
    display_.show("LABERINTO", "Mapeando pista", "desconocida");
    telemetry::println("MAPA_NUEVO");
    if (explorer_.mapTrack(pos, heading)) reachedGoal("mapa completo");
    robot_.setGrid(false);
    return;
  }
  if (learnedFrame_) {
    // RUTA MAS CORTA sobre el mapa aprendido: desde la salida (centro de la rejilla) hasta la celda
    // elegida en la web, por tramos rectos a velocidad de carrera. Si algo no coincide, explora.
    raton::Cell pos = center;
    const raton::Cell goal = raton::makeCell(req.goalX, req.goalY);
    if (!map_.inside(goal)) {
      log_.decide("la meta esta fuera del mapa aprendido");
      robot_.setGrid(false);
      return;
    }
    map_.setKeep(0, pos);
    map_.setKeep(1, goal);
    map_.setAvoidDead(true);
    char line[56];
    snprintf(line, sizeof(line), "RUTA mas corta a %d,%d por el mapa aprendido", goal.x, goal.y);
    log_.decide(line);
    telemetry::println("RUTA_INICIO");
    if (explorer_.runRoute(pos, heading, goal, cfg::COMP_SPEED) || explorer_.exploreTo(pos, heading, goal)) {
      reachedGoal("ruta mas corta");
    }
    robot_.setGrid(false);
    return;
  }
  raton::Cell pos = raton::makeCell(cfg::START_X, cfg::START_Y);
  const raton::Cell goal = goalCell();
  if (mapReady_) {
    display_.show("LABERINTO", "Carrera rapida", "mapa aprendido");
    log_.decide("carrera rapida por el mapa aprendido");
    if (explorer_.runRoute(pos, heading, goal, cfg::COMP_SPEED)) {
      reachedGoal("carrera rapida");
      robot_.setGrid(false);
      return;
    }
    log_.decide("el mapa aprendido no coincide: sigue explorando");
  } else {
    map_.clear(nullptr, cfg::MAZE_WIDTH, cfg::MAZE_HEIGHT);
    display_.show("LABERINTO", "Explorando", "por celdas");
  }
  if (explorer_.exploreTo(pos, heading, goal)) {
    mapReady_ = true;
    reachedGoal("explorando");
    if (cfg::ADACHI_SEARCH) explorer_.adachiSearch(pos, heading, raton::makeCell(cfg::START_X, cfg::START_Y), goal);
  }
  robot_.setGrid(false);
}

void MazeMission::runCompetition() {
  using namespace competition;
  robot_.setGrid(true);
  learnedFrame_ = false;
  map_.clear(competitionWall, cfg::MAZE_WIDTH, cfg::MAZE_HEIGHT);
  raton::Cell pos = raton::makeCell(START_X, START_Y);
  raton::Dir heading = raton::NORTH;
  const raton::Cell goal = raton::makeCell(GOAL_X, GOAL_Y);
  display_.show("RUTA COMPETENCIA", "Ruta mas corta", "por tramos");
  if (explorer_.runRoute(pos, heading, goal, cfg::COMP_SPEED)) {
    reachedGoal("Ruta competencia");
    robot_.setGrid(false);
    return;
  }
  if (robot_.aborted()) {
    robot_.setGrid(false);
    return;
  }
  char line[72];
  snprintf(line, sizeof(line), "el mapa no coincide cerca de %d,%d -> explora por celdas", pos.x, pos.y);
  log_.decide(line);
  if (explorer_.exploreTo(pos, heading, goal)) reachedGoal("explorando");
  robot_.setGrid(false);
}

// Adaptador para el simulador grafico mms (https://github.com/mackorone/mms).
// mms manda las respuestas por stdin y lee los comandos por stdout; los logs van a stderr.
//
// En mms: Mouse > "+"  ->  Directory: esta carpeta del repo
//   Build Command: cmake --build build --target raton_mms
//   Run Command:   build/raton_mms   (con Visual Studio: build/Debug/raton_mms.exe)

#include <iostream>
#include <stdexcept>
#include <string>

#include "solver.h"

namespace {

class MmsRobot : public raton::Robot {
 public:
  std::string command(const std::string& cmd) {
    std::cout << cmd << std::endl;
    std::string response;
    std::getline(std::cin, response);
    if (!response.empty() && response.back() == '\r') response.pop_back();
    return response;
  }

  void send(const std::string& cmd) { std::cout << cmd << '\n'; }

  raton::Walls readWalls() override {
    return raton::Walls{command("wallLeft") == "true", command("wallFront") == "true",
                        command("wallRight") == "true"};
  }

  void moveForward(int cells) override {
    if (command("moveForward " + std::to_string(cells)) != "ack") throw std::runtime_error("choque");
  }

  void turnLeft() override { command("turnLeft"); }
  void turnRight() override { command("turnRight"); }
  void turnAround() override {
    command("turnRight");
    command("turnRight");
  }

  void setRunMode(raton::RunMode mode) override { speed_ = mode == raton::RunMode::Speed; }

  void onStep(const raton::Maze& maze, const raton::FloodFill& flood, raton::Cell pos, raton::Dir) override {
    for (int d = 0; d < 4; ++d) {
      const raton::Dir dir = static_cast<raton::Dir>(d);
      if (maze.isKnown(pos, dir) && maze.hasWall(pos, dir)) {
        send("setWall " + std::to_string(pos.x) + " " + std::to_string(pos.y) + " " + raton::dirChar(dir));
      }
    }
    send("setColor " + std::to_string(pos.x) + " " + std::to_string(pos.y) + (speed_ ? " G" : " B"));
    for (int x = 0; x < maze.width(); ++x) {
      for (int y = 0; y < maze.height(); ++y) {
        const uint16_t dist = flood.distance(raton::Cell{static_cast<int8_t>(x), static_cast<int8_t>(y)});
        send("setText " + std::to_string(x) + " " + std::to_string(y) + " " +
             (dist == raton::INF_DIST ? std::string("-") : std::to_string(dist)));
      }
    }
    std::cout.flush();
  }

 private:
  bool speed_ = false;
};

}  // namespace

int main() {
  MmsRobot robot;
  const int width = std::stoi(robot.command("mazeWidth"));
  const int height = std::stoi(robot.command("mazeHeight"));
  if (width > raton::MAX_MAZE_SIZE || height > raton::MAX_MAZE_SIZE) {
    std::cerr << "Laberinto mayor de 16x16, no soportado\n";
    return 1;
  }

  raton::Solver solver(robot, width, height);
  try {
    std::cerr << "Explorando...\n";
    if (!solver.explore(8)) {
      std::cerr << "No se encontro camino a la meta\n";
      return 1;
    }
    std::cerr << "Exploracion: " << solver.stats().searchCells << " celdas, "
              << (solver.stats().pathProven ? "camino optimo probado" : "camino no probado") << '\n';
    robot.send("clearAllColor");
    std::cerr << "Carrera rapida...\n";
    solver.speedRun();
    std::cerr << "Meta en " << solver.stats().speedRunCells << " celdas, " << solver.stats().speedRunMoves
              << " tramos\n";
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << '\n';
    return 1;
  }
  return 0;
}

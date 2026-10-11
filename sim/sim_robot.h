// Robot virtual: lee paredes del laberinto "real" y estima el tiempo de cada movimiento.
#pragma once

#include <iosfwd>
#include <stdexcept>

#include "maze.h"
#include "robot.h"

namespace sim {

struct Crash : std::runtime_error {
  using std::runtime_error::runtime_error;
};

// Modelo de tiempos, aproximado a un N20 de 500 rpm con celdas de 180 mm.
struct MotionModel {
  double cellMm = 180.0;
  double searchSpeed = 250.0;  // mm/s
  double speedRunSpeed = 600.0;
  double accel = 1500.0;       // mm/s^2
  double turn90 = 0.35;        // s
  double turn180 = 0.60;
  double sensePause = 0.05;    // parada para leer los laseres al explorar
};

class SimRobot : public raton::Robot {
 public:
  explicit SimRobot(const raton::Maze& truth, MotionModel model = MotionModel());

  raton::Walls readWalls() override;
  void moveForward(int cells) override;
  void turnLeft() override;
  void turnRight() override;
  void turnAround() override;
  void setRunMode(raton::RunMode mode) override { mode_ = mode; }
  void onStep(const raton::Maze& known, const raton::FloodFill& flood, raton::Cell pos, raton::Dir heading) override;

  // Dibuja el laberinto en cada paso, esperando delayMs entre pasos.
  void animate(std::ostream* out, int delayMs) {
    animOut_ = out;
    animDelayMs_ = delayMs;
  }

  double seconds() const { return seconds_; }
  void resetClock() { seconds_ = 0; }

 private:
  const raton::Maze& truth_;
  MotionModel model_;
  raton::Cell pos_;
  raton::Dir heading_;
  raton::RunMode mode_ = raton::RunMode::Search;
  double seconds_ = 0;
  std::ostream* animOut_ = nullptr;
  int animDelayMs_ = 0;
};

// Dibujo ASCII. Paredes conocidas: --- y |. Paredes reales aun no vistas: . y :
// En cada celda: el raton (^ > v <) o la distancia del flood fill.
void render(std::ostream& out, const raton::Maze& known, const raton::Maze* truth, const raton::FloodFill* flood,
            raton::Cell mouse, raton::Dir heading);

}  // namespace sim

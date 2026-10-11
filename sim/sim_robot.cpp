#include "sim_robot.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <ostream>
#include <thread>

using raton::Cell;
using raton::Dir;

namespace sim {

SimRobot::SimRobot(const raton::Maze& truth, MotionModel model)
    : truth_(truth), model_(model), pos_{0, 0}, heading_(raton::NORTH) {}

raton::Walls SimRobot::readWalls() {
  seconds_ += model_.sensePause;
  return raton::Walls{truth_.hasWall(pos_, raton::leftOf(heading_)), truth_.hasWall(pos_, heading_),
                      truth_.hasWall(pos_, raton::rightOf(heading_))};
}

void SimRobot::moveForward(int cells) {
  for (int i = 0; i < cells; ++i) {
    if (truth_.hasWall(pos_, heading_)) {
      throw Crash("choque en (" + std::to_string(pos_.x) + "," + std::to_string(pos_.y) + ") hacia " +
                  raton::dirChar(heading_));
    }
    pos_ = raton::step(pos_, heading_);
  }
  // Perfil trapezoidal: acelera, crucero, frena
  const double d = cells * model_.cellMm;
  const double v = mode_ == raton::RunMode::Speed ? model_.speedRunSpeed : model_.searchSpeed;
  const double a = model_.accel;
  seconds_ += d < v * v / a ? 2.0 * std::sqrt(d / a) : d / v + v / a;
}

void SimRobot::turnLeft() {
  heading_ = raton::leftOf(heading_);
  seconds_ += model_.turn90;
}

void SimRobot::turnRight() {
  heading_ = raton::rightOf(heading_);
  seconds_ += model_.turn90;
}

void SimRobot::turnAround() {
  heading_ = raton::behind(heading_);
  seconds_ += model_.turn180;
}

void SimRobot::onStep(const raton::Maze& known, const raton::FloodFill& flood, Cell pos, Dir heading) {
  if (pos != pos_ || heading != heading_) throw Crash("el solver perdio la posicion del raton");
  if (!animOut_) return;
  *animOut_ << "\x1b[H\x1b[2J";  // limpia la terminal
  render(*animOut_, known, &truth_, &flood, pos, heading);
  *animOut_ << (mode_ == raton::RunMode::Speed ? "Carrera rapida" : "Explorando") << "   t = " << seconds_
            << " s\n";
  animOut_->flush();
  if (animDelayMs_ > 0) std::this_thread::sleep_for(std::chrono::milliseconds(animDelayMs_));
}

void render(std::ostream& out, const raton::Maze& known, const raton::Maze* truth, const raton::FloodFill* flood,
            Cell mouse, Dir heading) {
  const int w = known.width();
  const int h = known.height();
  auto c = [](int x, int y) { return Cell{static_cast<int8_t>(x), static_cast<int8_t>(y)}; };
  auto wallKind = [&](Cell cell, Dir d) {
    if (known.isKnown(cell, d)) return known.hasWall(cell, d) ? 2 : 0;
    return truth && truth->hasWall(cell, d) ? 1 : 0;
  };

  for (int y = h - 1; y >= -1; --y) {
    out << 'o';
    for (int x = 0; x < w; ++x) {
      const int k = y >= 0 ? wallKind(c(x, y), raton::NORTH) : wallKind(c(x, 0), raton::SOUTH);
      out << (k == 2 ? "---" : k == 1 ? " . " : "   ") << 'o';
    }
    out << '\n';
    if (y < 0) break;
    for (int x = 0; x < w; ++x) {
      const int k = wallKind(c(x, y), raton::WEST);
      out << (k == 2 ? '|' : k == 1 ? ':' : ' ');
      char buf[8];
      if (c(x, y) == mouse) {
        static const char kArrows[4] = {'^', '>', 'v', '<'};
        std::snprintf(buf, sizeof(buf), " %c ", kArrows[heading]);
      } else if (flood && flood->distance(c(x, y)) != raton::INF_DIST) {
        std::snprintf(buf, sizeof(buf), "%3u", static_cast<unsigned>(flood->distance(c(x, y))));
      } else if (known.isGoal(c(x, y))) {
        std::snprintf(buf, sizeof(buf), " G ");
      } else {
        std::snprintf(buf, sizeof(buf), "   ");
      }
      out << buf;
    }
    const int k = wallKind(c(w - 1, y), raton::EAST);
    out << (k == 2 ? '|' : k == 1 ? ':' : ' ') << '\n';
  }
}

}  // namespace sim

// Implementacion real de raton::Robot: perfiles de velocidad con encoders, rumbo con giroscopio
// y centrado con los laseres laterales.
#pragma once

#include "hw/display.h"
#include "hw/encoders.h"
#include "hw/gyro.h"
#include "hw/lasers.h"
#include "hw/motors.h"
#include "robot.h"

class RobotHW : public raton::Robot {
 public:
  RobotHW(Motors& motors, Encoders& encoders, Lasers& lasers, Gyro& gyro, Display& display);

  raton::Walls readWalls() override;
  void moveForward(int cells) override;
  void turnLeft() override { rotate(90.0f); }
  void turnRight() override { rotate(-90.0f); }
  void turnAround() override { rotate(180.0f); }
  void setRunMode(raton::RunMode mode) override { mode_ = mode; }
  void onStep(const raton::Maze& maze, const raton::FloodFill& flood, raton::Cell pos, raton::Dir heading) override;
  bool aborted() override { return aborted_; }

  void abort() { aborted_ = true; }
  void clearAbort() { aborted_ = false; }

 private:
  void rotate(float degrees);
  // Desplazamiento lateral respecto al centro del pasillo (mm, positivo = a la derecha).
  bool lateralOffset(float& offset) const;
  void waitPeriod(uint32_t& next);

  Motors& motors_;
  Encoders& encoders_;
  Lasers& lasers_;
  Gyro& gyro_;
  Display& display_;
  raton::RunMode mode_ = raton::RunMode::Search;
  bool aborted_ = false;
};

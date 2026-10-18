// Bitacora de decisiones: lineas DECIDE con las lecturas del momento y eventos para la web.
#pragma once

#include "exploracion/ports.h"
#include "seguridad/robot_hw.h"

class Reporter : public raton::MissionLog {
 public:
  explicit Reporter(RobotHW& robot) : robot_(robot) {}
  void decide(const char* why) override;
  void event(const char* line) override;

 private:
  RobotHW& robot_;
};

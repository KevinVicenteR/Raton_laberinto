// Platform de la placa real: lazo de control a 500 Hz (Params::dt) con sensores, motores y Bluetooth.
#pragma once

#include "hw/encoders.h"
#include "hw/imu.h"
#include "hw/lasers.h"
#include "hw/motors.h"
#include "raton/params.h"
#include "raton/platform.h"

class Board : public raton::Platform {
 public:
  Board(Motors& m, Encoders& e, Lasers& l, Imu& imu, const raton::Params& p)
      : motors_(m), enc_(e), lasers_(l), imu_(imu), p_(p) {}

  void setMotors(float left, float right) override;
  const raton::Sensors& tick() override;
  void report(const char* line, bool important) override;
  bool stopRequested() override { return stop_; }
  bool saveMap(const raton::Maze& maze) override;

  void requestStop() { stop_ = true; }
  // false en la practica: no se sobrescribe el mapa de la competencia guardado en flash.
  bool saveEnabled = true;
  void clearStop() { stop_ = false; }
  // Atender Bluetooth y Serie mientras se espera (se llama desde tick y desde el menu).
  void (*service)() = nullptr;
  // Ciclos que llegaron tarde (el control no pudo mantener 500 Hz).
  uint32_t lateTicks() const { return late_; }
  float maxLoadUs() const { return maxLoad_; }
  void resetLoad() { maxLoad_ = 0; late_ = 0; }
  const raton::Sensors& last() const { return s_; }

 private:
  Motors& motors_;
  Encoders& enc_;
  Lasers& lasers_;
  Imu& imu_;
  const raton::Params& p_;
  raton::Sensors s_;
  uint32_t next_ = 0;
  uint32_t t0_ = 0;
  bool started_ = false;
  int laser_ = 0;
  uint32_t imuTick_ = 0;
  float dutyL_ = 0, dutyR_ = 0;
  float stillT_ = 0;
  long lastL_ = 0, lastR_ = 0;
  volatile bool stop_ = false;
  uint32_t late_ = 0;
  float maxLoad_ = 0;
  uint32_t lastReturn_ = 0;
};

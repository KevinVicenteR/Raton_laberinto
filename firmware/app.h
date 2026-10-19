// Aplicacion: arranque del hardware, menu (con la mano delante del laser frontal, o por Bluetooth/Serie),
// modos y ordenes. La logica del laberinto esta en lib/raton_core (la misma que se prueba en el simulador).
#pragma once

#include "board.h"
#include "hw/display.h"
#include "hw/encoders.h"
#include "hw/imu.h"
#include "hw/lasers.h"
#include "hw/motors.h"
#include "raton/mouse.h"
#include "raton/params.h"

class App {
 public:
  App();
  void setup();
  void loop();
  // Bluetooth y Serie (tambien durante la marcha, para la orden de parar).
  void service();

 private:
  enum Mode { EXPLORE = 1, RACE, GOAL_ONLY, SENSORS, MOTOR_CAL, WHEELS, ERASE, kModes = ERASE };
  void fail(const char* msg);
  int menu();
  bool armStart();
  void runMode(int mode);
  void feed(int c);
  void command(char* line);
  void reply(const char* line);
  void sensorsMode();
  void motorCalibration();
  void wheelTest();
  void calibrateFront();
  void calibrateSides(float corridorMm);
  void printMap();
  void showResult(const raton::MissionResult& r, const char* title);

  raton::Params params_;
  Motors motors_;
  Encoders encoders_;
  Lasers lasers_;
  Imu imu_;
  Display display_;
  Board board_;
  raton::Mouse mouse_;
  int mode_ = EXPLORE;
  int requestedMode_ = 0;
  bool startRequested_ = false;
  bool busy_ = false;
  char line_[96];
  int lineLen_ = 0;
  uint16_t lastSeq_ = 0;
  bool seqValid_ = false;
};

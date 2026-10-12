// Implementacion real de raton::Robot: perfiles de velocidad con encoders, rumbo con giroscopio
// y centrado con los laseres laterales.
#pragma once

#include "hw/display.h"
#include "hw/encoders.h"
#include "hw/gyro.h"
#include "hw/lasers.h"
#include "hw/motors.h"
#include "config.h"
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

  // Seguidor de paredes (no depende del tamano de celda ni de la pista).
  enum class DriveEvent { Distance, FrontWall, SideOpen, CrashLeft, CrashRight, Aborted };
  // Avanza recto (rumbo con giroscopio, centrado con laseres) hasta recorrer maxMm, quedar a
  // FRONT_TURN_MM de una pared delante o, si watchSide != 0, ver abierto ese lado (+1 izq, -1 der).
  DriveEvent drive(float maxMm, int watchSide);
  void turnDegrees(float degrees) { rotate(degrees); }
  // Ancho del pasillo medido con los dos laterales mientras avanza (mm).
  float corridorMm() const { return corridorMm_; }
  // Marcha atras recta mm milimetros (para separarse de una pared antes de girar).
  void backUp(float mm);

  // Al arrancar: con el giroscopio comprueba si los motores o los laseres izquierdo/derecho estan
  // cruzados y lo corrige. Gira un poco en el sitio. Escribe el resultado en msg (>= 22 bytes).
  void selfCheck(char* msg);

  void abort() { aborted_ = true; }
  void clearAbort() { aborted_ = false; }

 private:
  void rotate(float degrees);
  // Un paso del control de velocidad y rumbo a la consigna v (mm/s).
  // useWalls = false ignora los laseres laterales (p. ej. cerca de una pared delante).
  void controlStep(float v, float speed, float& integral, float dt, bool useWalls = true);
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
  float corridorMm_ = cfg::DEFAULT_CORRIDOR_MM;
  float yawTarget_ = 0;  // rumbo objetivo suavizado para centrarse
};

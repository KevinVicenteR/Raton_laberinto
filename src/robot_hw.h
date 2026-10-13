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
  // frontTurnMm() de una pared delante o, si watchSide != 0, ver abierto ese lado (+1 izq, -1 der,
  // kWatchBoth = cualquiera de los dos; openSide() dice cual).
  static const int kWatchBoth = 2;
  DriveEvent drive(float maxMm, int watchSide);
  int openSide() const { return openSide_; }
  // La siguiente drive() que vigile huecos no mira los laterales durante los primeros mm (tras girar,
  // un lateral aun ve el pasillo del que viene y lo tomaria por un hueco nuevo).
  void ignoreSidesFor(float mm) { ignoreSideMm_ = mm; }
  void turnDegrees(float degrees) { rotate(degrees); }
  // Ancho del pasillo medido con los dos laterales mientras avanza (mm).
  float corridorMm() const { return corridorMm_; }
  // Lectura frontal a la que parar para que el eje de ruedas quede en el centro del cruce y pueda
  // girar sin rozar (con pasillos de 14 cm el raton apenas cabe girando).
  int frontTurnMm() const {
    const float v = 0.5f * corridorMm_ - cfg::FRONT_SENSOR_AHEAD_MM;
    return v > cfg::FRONT_TURN_MIN_MM ? static_cast<int>(v) : cfg::FRONT_TURN_MIN_MM;
  }
  // Distancia a la pared de delante (mm, -1 = nada). Si el laser frontal deja de medir con la pared
  // cerca (por debajo de ~3 cm el VL53L0X falla), la estima con lo avanzado desde la ultima medida.
  int frontMm();

  // Meta pedida desde la web ('G<x>,<y>'), o x = -1 si no hay.
  int goalX() const { return goalX_; }
  int goalY() const { return goalY_; }
  // Marcha atras recta mm milimetros (para separarse de una pared antes de girar).
  void backUp(float mm);

  // Al arrancar: con el giroscopio comprueba si los motores o los laseres izquierdo/derecho estan
  // cruzados y lo corrige, y mide la distancia entre ruedas. Gira un poco en el sitio. Escribe el
  // resultado en msg (>= 22 bytes).
  void selfCheck(char* msg);
  float trackWidthMm() const { return trackWidth_; }

  // Estado que se ve en la telemetria ("RECTA", "GIRO"...).
  void setState(const char* state) { state_ = state; }
  // Atiende Bluetooth, Serie y telemetria con el raton parado (en vez de delay).
  void idle(uint32_t ms);
  // Ajustables en marcha por Bluetooth o Serie.
  float speed() const { return speed_; }

  void abort() { aborted_ = true; }
  void clearAbort() { aborted_ = false; }
  // Orden de arranque recibida ('s' por Bluetooth o Serie). Se consume al leerla.
  bool takeStart() {
    const bool s = startRequested_;
    startRequested_ = false;
    return s;
  }
  // Modo pedido por Bluetooth ('m1'..'m3'), o 0 si no hay.
  int takeMode() {
    const int m = requestedMode_;
    requestedMode_ = 0;
    return m;
  }
  // Pone la posicion a cero (salida del mapa).
  void resetOdometry();

 private:
  void rotate(float degrees);
  // Un paso del control de velocidad y rumbo a la consigna v (mm/s).
  // useWalls = false ignora los laseres laterales (p. ej. cerca de una pared delante).
  void controlStep(float v, float speed, float& integral, float dt, bool useWalls = true);
  // Desplazamiento lateral respecto al centro del pasillo (mm, positivo = a la derecha).
  bool lateralOffset(float& offset) const;
  void waitPeriod(uint32_t& next);
  // Frena, suelta y espera SETTLE_MS corrigiendo el sesgo del giroscopio.
  void stopMotors();
  // Rumbo fusionado (giroscopio + encoders) desde el ultimo resetHeading().
  void resetHeading();
  void updateHeading();
  // Ordenes: x = parar, v<mm/s> = velocidad, k<valor> = KP_WALL, ? = ver valores.
  void handleInput();
  void runCommand(const char* cmd);
  void sendTelemetry();
  // Posicion del eje de ruedas en la pista (mm) y rumbo (grados) desde resetOdometry().
  void updateOdometry();

  Motors& motors_;
  Encoders& encoders_;
  Lasers& lasers_;
  Gyro& gyro_;
  Display& display_;
  raton::RunMode mode_ = raton::RunMode::Search;
  bool aborted_ = false;
  float corridorMm_ = cfg::DEFAULT_CORRIDOR_MM;
  float yawTarget_ = 0;  // rumbo objetivo suavizado para centrarse
  int openSide_ = 0;     // lado del ultimo hueco detectado (+1 izq, -1 der)
  float ignoreSideMm_ = 0;
  float speed_ = cfg::WALL_FOLLOW_SPEED;
  float kpWall_ = cfg::KP_WALL;
  float trackWidth_ = cfg::TRACK_WIDTH_MM;
  float heading_ = 0;
  float lastGyroYaw_ = 0;
  float lastSpeed_ = 0;
  const char* state_ = "INICIO";
  uint32_t lastTelemetry_ = 0;
  uint32_t lastPeriodUs_ = 0;  // para medir el ciclo de control mas lento
  uint32_t maxLagUs_ = 0;
  bool startRequested_ = false;
  int requestedMode_ = 0;
  int goalX_ = cfg::GOAL_X;
  int goalY_ = cfg::GOAL_Y;
  float odoX_ = 0;
  float odoY_ = 0;
  float odoYaw0_ = 0;
  float odoLastA_ = 0;  // rumbo del paso anterior (para integrar con el rumbo medio)
  long odoL_ = 0;
  long odoR_ = 0;
  float odoDist_ = 0;        // recorrido total hacia delante (mm, marcha atras resta)
  int frontMemMm_ = -1;      // ultima lectura frontal valida cercana
  float frontMemDist_ = 0;   // odoDist_ en ese momento
  char cmd_[16] = {};
  int cmdLen_ = 0;
};

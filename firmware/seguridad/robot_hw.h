// Adaptador compartido de placa. Estado comun; implementaciones por funcionalidad
// en paredes/, centrado/, velocidad/, giros/, odometria/, seguridad/ y comunicacion/.
#pragma once

#include "velocidad/adaptive_speed.h"
#include "seguridad/motion_result.h"
#include "seguridad/orquestador.h"
#include "exploracion/ports.h"
#include "paredes/track_geometry.h"
#include "paredes/cell_window.h"
#include "hardware/display.h"
#include "hardware/encoders.h"
#include "hardware/gyro.h"
#include "hardware/lasers.h"
#include "hardware/motors.h"
#include "comunicacion/command_handler.h"
#include "comunicacion/tuning.h"

struct Walls {
  bool left;
  bool front;
  bool right;
};

class RobotHW : public raton::CellDriver {
 public:
  RobotHW(Motors& motors, Encoders& encoders, Lasers& lasers, Gyro& gyro, Display& display);

  // ---- Puerto del explorador por celdas
  bool aborted() override { return aborted_; }
  void setState(const char* state) override { state_ = state; }
  raton::TrackGeometry& geometry() override { return geom_; }
  void turnInCell(float degrees) override { turnCorner(degrees, true); }
  int runCells(int cells, float vmax, bool* unexpected, raton::CellObserver* observer) override;
  bool cellSides(int i, bool& left, bool& right) const override;
  bool frontWallStopped() override;
  int stableFrontMm() override;
  bool calibrateStill(int& left, int& right, int& front) override;
  void alignToFront() override;

  const raton::TrackGeometry& geometry() const { return geom_; }
  Tuning& tuning() { return tuning_; }
  OperatorRequests& requests() { return requests_; }
  Lasers& lasers() { return lasers_; }

  // ---- Lecturas
  // Distancia del costado izquierdo/derecho a la pared (mm, -1 = nada).
  int sideMm(Lasers::Id id) const { return geom_.sideFromRay(lasers_.mm(id)); }
  // Distancia a la pared de delante (mm, -1 = nada). Si el laser frontal deja de medir con la pared
  // cerca (por debajo de ~3 cm el VL53L0X falla), la estima con lo avanzado desde la ultima medida.
  int frontMm();
  Walls readWalls();
  // Paredes de la celda leidas parado con varias medidas (izq, delante, der respecto al rumbo).
  Walls senseCell();

  // ---- Maniobras del seguidor de paredes y del modo 5
  enum class DriveEvent { Distance, FrontWall, SideOpen, CrashLeft, CrashRight, Aborted };
  // Avanza recto hasta recorrer maxMm, quedar a frontTurnMm() de una pared delante o, si watchSide != 0,
  // ver abierto ese lado (+1 izq, -1 der, kWatchBoth = cualquiera de los dos).
  static const int kWatchBoth = 2;
  DriveEvent drive(float maxMm, int watchSide);
  // La siguiente drive() que vigile huecos no mira los laterales durante los primeros mm.
  void ignoreSidesFor(float mm) { ignoreSideMm_ = mm; }
  void turnDegrees(float degrees) { rotate(degrees); }
  // Giro de 90 para entrar en un pasillo (en dos tiempos si TURN_TWO_STAGE). Con trackCells, el
  // siguiente avance de celdas descuenta lo que el giro ya adelanto.
  void turnCorner(float degrees, bool trackCells = false);
  // Marcha atras recta mm milimetros.
  void backUp(float mm);
  // Marcha atras vigilando los laterales hasta ver un hueco: +1 izquierda, -1 derecha, 0 ninguno.
  int backOut(float maxMm);
  // Rumbo absoluto de la cuadricula (modos 3 y 4): los tramos mantienen ese rumbo y los giros acaban
  // en el. Llamar mirando al norte de la pista.
  void setGrid(bool on);

  // ---- Memoria del modo 5 (rejilla de celdas del tamano del pasillo, centrada en la salida)
  float travelledMm() const { return odoDist_; }
  int visitsToward(float turnDeg) const;
  int exitsToward(float turnDeg) const;
  void markExit(float turnDeg);
  // Meta en coordenadas de odometria (mm desde el arranque): drive() se para al llegar.
  void setGoal(float x, float y);
  void clearGoal() { goalSet_ = goalReached_ = false; }
  bool goalReached() const { return goalReached_; }
  // Coseno del angulo entre la direccion turnDeg y la meta (1 = va directo); 0 sin meta.
  float goalAlignment(float turnDeg) const;
  void resetOdometry();

  // ---- Calibraciones
  // Comprueba si los motores estan cruzados y mide la distancia entre ruedas (msg >= 22 bytes).
  void selfCheck(char* msg);
  float trackWidthMm() const { return trackWidth_; }
  // Con el raton centrado en un pasillo de ancho conocido calcula el angulo real de los laterales.
  bool calibrateSideAngle(float corridorMm, int& left, int& right);
  // Prueba de ruedas con el raton levantado: cada rueda sola, encoders y giroscopio.
  void wheelTest();

  // ---- Ciclo y ordenes
  // Atiende Bluetooth, Serie y telemetria con el raton parado (en vez de delay).
  void idle(uint32_t ms);
  void abort() { aborted_ = true; }
  void clearAbort() { aborted_ = false; }
  void coast() { motors_.coast(); }
  bool takeStart();
  int takeMode();

 private:
  // Ciclo de control
  void waitPeriod(uint32_t& next);
  void service();
  void sendTelemetry();
  void recordMotion(const char* phase, float requested, const raton::MotionResult& result);
  void updateOdometry();
  int cellOf(float v) const;
  int worldDir(float turnDeg) const;  // 0 norte, 1 oeste, 2 sur, 3 este

  // Rumbo
  void resetHeading();
  void resetHeadingToGrid();  // igual, con el cero en el rumbo de la cuadricula (si esta activa)
  void updateHeading();
  float gridError() const;    // rumbo respecto a la cuadricula en (-45, 45]
  float gridTurnError(float degrees);

  // Movimiento
  void rotate(float degrees);
  float rotateOnce(float degrees);  // un intento; devuelve el error que queda (grados)
  float speedOutput(float v, float speed, float& integral, float dt);
  void controlStep(float v, float speed, float& integral, float dt, bool useWalls = true);
  bool lateralOffset(float& offset, float& rate, float& conf) const;
  void stopMotors();
  void waitFresh(int samples);
  void observeCellSides(float pos, int cells, float cellMm, float already, float& target,
                        raton::CellObserver*& observer, raton::CellWindow& window);
  void confirmCellFront(int frontNow, uint32_t& sample, int& previous, int& confirmed);
  bool postTurnFresh_ = false;
  raton::AdaptiveSpeed adaptiveSpeed_;
  bool adaptiveStarted_ = false;
  bool cachedCellFront_ = false;
  int cachedCellFrontMm_ = -1;
  float wheelDistance() const;
  static raton::ParamsMovimiento motionParams();

  Motors& motors_;
  Encoders& encoders_;
  Lasers& lasers_;
  Gyro& gyro_;
  Display& display_;
  CommandHandler commands_;
  raton::TrackGeometry geom_;
  Tuning tuning_;
  OperatorRequests requests_;
  bool aborted_ = false;
  const char* state_ = "INICIO";

  // Movimiento
  float yawTarget_ = 0;    // rumbo objetivo suavizado para centrarse
  float headingRef_ = 0;   // rumbo a mantener cuando no hay paredes para centrarse
  float ignoreSideMm_ = 0;
  float pendingAdvance_ = 0;  // cuanto va el centro del raton por delante del centro de la celda
  float lastSpeed_ = 0;
  uint32_t laserSamples_ = 0;  // lecturas nuevas de los laseres (para muestrear solo lo nuevo)

  // Paredes laterales del ultimo tramo por celdas
  static const int kMaxCellSides = cfg::MAX_CELL_SIDES;
  static constexpr float kSideWindowMm = cfg::SIDE_WINDOW_MM;  // ventana de lectura alrededor del punto de cada celda
  bool cellSideL_[kMaxCellSides] = {};
  bool cellSideR_[kMaxCellSides] = {};
  int numCellSides_ = 0;

  // Rumbo
  bool gridOn_ = false;
  float gridYaw_ = 0;      // totalYaw del norte de la cuadricula
  float heading_ = 0;
  float headingBase_ = 0;  // rumbo al empezar el tramo (para mezclar con los encoders)
  float lastGyroYaw_ = 0;
  float trackWidth_ = cfg::TRACK_WIDTH_MM;

  // Odometria y memoria del modo 5
  static const int kGrid = cfg::VISIT_GRID;
  uint8_t visits_[kGrid][kGrid] = {};
  uint8_t exits_[kGrid][kGrid][4] = {};
  int cellX_ = -1;
  int cellY_ = -1;
  float odoX_ = 0;
  float odoY_ = 0;
  float odoYaw0_ = 0;
  float odoLastA_ = 0;  // rumbo del paso anterior (para integrar con el rumbo medio)
  long odoL_ = 0;
  long odoR_ = 0;
  float odoDist_ = 0;   // recorrido total hacia delante (mm, marcha atras resta)
  float lastValidDist_[3] = {-1e9f, -1e9f, -1e9f};  // recorrido en la ultima lectura valida de cada laser
  int frontMemMm_ = -1;     // ultima lectura frontal valida cercana
  float frontMemDist_ = 0;  // odoDist_ en ese momento
  float goalX_mm_ = 0;
  float goalY_mm_ = 0;
  bool goalSet_ = false;
  bool goalReached_ = false;

  // Telemetria
  uint32_t lastTelemetry_ = 0;
  uint32_t lastObservationTelemetry_ = 0;
  uint32_t lastPeriodUs_ = 0;  // para medir el ciclo de control mas lento
  uint32_t maxLagUs_ = 0;
};

// Navegador de celdas: mueve el raton de celda en celda leyendo las paredes por delante.
//
// Vista de los sensores (sonda.h): cada lectura de los tres VL53L0X se compara con lo que la sonda dice
// que deberia leer con el cono real del sensor, el mapa y la pose. Como un radar:
//  - mapa: cada pared que el cono alcanza se prueba cerrada y abierta; se vota la hipotesis que explica la
//    lectura (si las dos la explican igual, no se vota nada);
//  - pose: con paredes ya confirmadas, la diferencia entre lectura y prediccion corrige el desplazamiento
//    lateral y el rumbo (filtro de Kalman) y el avance (postes, huecos, pared de frente).
// Los laterales a 45 grados ven las paredes de la celda SIGUIENTE cuando el eje aun esta en la actual, asi
// que la decision esta lista antes de llegar: no hay que pararse a mirar.
//
// Giros: en el sitio en el centro de la celda (robusto) o curva suave sin parar (carrera).
// Seguridad: si las ruedas no avanzan con orden de moverse (atasco) o el frontal ve una pared que
// no deberia estar, para, retrocede, se endereza a la cuadricula y vuelve a planificar.
#pragma once

#include "raton/lateral.h"
#include "raton/autonomy.h"
#include "raton/side_sensors.h"
#include "raton/maze.h"
#include "raton/motion.h"
#include "raton/params.h"
#include "raton/platform.h"
#include "raton/sonda.h"

namespace raton {

// Quien decide a donde ir (explorador o ruta de carrera).
class Brain {
 public:
  virtual ~Brain() {}
  // El raton va a estar en (x, y) mirando a h. Devuelve la direccion por la que salir, o -1 = parar
  // en esta celda (meta, o sin camino).
  virtual int decide(int x, int y, Dir h) = 0;
  // Celdas que se van a recorrer en linea recta despues de salir de (x, y) por d antes de la siguiente
  // parada o giro (para ir rapido en las rectas conocidas). 0 = no se sabe.
  virtual int straightAfter(int x, int y, Dir d) {
    (void)x; (void)y; (void)d;
    return 0;
  }
  // Se llama en cada ciclo; devolver true aborta la navegacion (tiempo agotado, etc.).
  virtual bool shouldAbort() { return false; }
  virtual bool diagonalPair(int x, int y, Dir h, Dir& next) {
    (void)x; (void)y; (void)h; (void)next; return false;
  }
};

enum class NavResult { Arrived, NoPath, Aborted, Failed };

class Navigator {
 public:
  Navigator(Platform& io, const Params& p, Maze& maze);

  // Prepara la pose: celda (x, y), rumbo h, quieto en el centro.
  void begin(int x, int y, Dir h);
  // Navega hasta que el cerebro pide parar (Arrived) o no hay camino / se aborta.
  NavResult run(Brain& brain, float vmax, bool smoothTurns);
  // Gira en el sitio hasta mirar a d.
  void face(Dir d);
  // Gira en el sitio a deg grados de la cuadricula (+ = izquierda) sin cambiar de rumbo de celda (para
  // calibrar: luego face() vuelve a la cuadricula).
  void aim(float deg);
  // Angulo real respecto a la cuadricula (grados, + = izquierda), por ejemplo tras aim().
  float aimAngle() const { return motion_.angle() - gridAngle_; }
  const Sonda& sonda() const { return sonda_; }
  float gyroScale() { return motion_.gyroScale(); }
  // Lo aprendido en marcha de los sensores: inclinacion de cada uno (grados) y reflejo del piso.
  float sensorTilt(int sensor) const { return sonda_.tiltDeg(sensor); }
  float floorReflect() const { return sonda_.floorReflect(); }
  float cellPitch() const { return cell_; }
  void invalidatePitch() { cellRef_ = 0; pitchSavedRef_ = -1; }
  bool pitchConfident() const { return pitchFits_ >= 3 && cellVar_ < 4.0f; }
  // Pista de medida incierta (practica): ajustar el paso de celda en marcha con las paredes de enfrente.
  void setLearnCell(bool on) { learnCell_ = on; }
  // Paso de celda que se supuso al calibrar los desfases de los sensores (0 = el de los parametros). Si la
  // pista mide otra cosa, las lecturas se corrigen con lo aprendido en marcha.
  void setOffsetsCell(float c) { offsetsCell_ = c; }
  float rangeBias() const;

  bool sensorFault() const { return rangeFault_ || motion_.sensorFault(); }
  void setAutonomous(bool on) { autonomous_ = on; }
  bool persistent() const { return autonomous_; }
  void setCautious(bool on) { cautious_=on; }
  bool repairRanges();
  bool resumeAfterStorage();
  void snapshot() { lastReport_=-1e9f; lastHealthReport_=-1e9f; report(); if(autonomous_)diagnose(); }

  int x() const { return cx_; }
  int y() const { return cy_; }
  Dir heading() const { return h_; }
  float now() const { return t_; }
  int recoveries() const { return recoveries_; }
  int crashes() const { return stalls_; }
  const Sensors& sensors() const { return s_; }
  Motion& motion() { return motion_; }

  // Medida en el sitio (calibracion): espera n lecturas nuevas de cada sensor y devuelve la media.
  void sample(int n, int& left, int& front, int& right);
  // Barrido para calibrar: gira en el sitio hasta deg grados de la cuadricula (+ = izquierda) a rate
  // grados/s y anota cada lectura nueva con el angulo (respecto a la cuadricula) al que se tomo.
  struct SweepLog {
    static const int kMax = 200;
    int n[3] = {0, 0, 0};
    float ang[3][kMax];
    int16_t mm[3][kMax];
  };
  void sweep(float deg, float rate, SweepLog& log);
  // Un ciclo de control con los motores segun Motion (para modos externos).
  void step();

 private:
  enum class Act : uint8_t { None, Forward, Left, Right, Back, Stop };

  void tick();
  void setMotorsFromMotion();
  void filterSensors();
  bool stopped() { return sensorFault() || io_.stopRequested(); }
  void report();
  float cellPos() const { return motion_.position() - centerOdo_; }
  float gridAngle() const { return gridAngle_; }
  float headingError() const { return motion_.angle() - gridAngle_; }
  void enterNextCell();
  void sense();
  void senseSides();
  SideEvidence sideEvidence_[2];
  float sideSeen_[2] = {-1, -1};
  void radar(int sensor, int mm);
  void frontCheck(int mm);
  // La lectura del frontal la explica la pared de la frontera k mejor que lo que hay sin ella (piso, lejos).
  bool frontSeesWall(int k, int mm);
  void learnVertical(int sensor, int mm, float h0);
  // Pose del sensor (0 izq, 1 frente, 2 der) en el marco del laberinto, con la pose del raton desplazada
  // fwd mm hacia delante, lat mm a la izquierda y dpsi rad (para los jacobianos).
  void sensorPose(int sensor, float fwd, float lat, float dpsi, float& x, float& y, float& ang) const;
  // Todas las paredes que el cono cruza hasta reach estan confirmadas (sirve para corregir la pose).
  bool confident(const Sonda::Scan& scan, float reach) const;
  void resetBeams();
  void steer();
  Act actionFor(int dir) const;
  void plan(Brain& brain);
  enum class Depart { Go, Stop, Retry, Fail };
  Depart depart(Brain& brain, float vmax);
  void centerOnFront(int k);
  float actCenterOdo() const;
  float planTarget() const;
  bool pivot(int quarterTurns);
  void learnGyroScale();
  void applyCorrection(float e, float gain);
  void startStraight();
  void addPitchPoint(float corr);
  void learnPitch();
  bool waitStopped(float timeout, float speedTol = 15.0f);
  float arcSpeed(float radius = 0) const;  // 0 = el de la curva elegida
  float cruiseSpeed(int nx, int ny) const;
  bool recover(Brain& brain);
  void checkStall();
  void computeSmoothGeometry();
  bool safeCurve(int x, int y, Dir incoming, Dir outgoing) const;
  bool diagonal(Dir next, Brain& brain);
  void diagnose();
  bool autonomous_ = false;
  bool cautious_ = false;
  bool trajectory_ = false;
  float steerCommand_ = 0, lastHealthReport_ = -1;
  float mapResumeAt_ = 0;
  int continuousRecenter(float y, Brain& brain);
  int repairAttempts_ = 0;
  RangeWatch health_[5];
  OscillationWatch oscillation_;
  ProgressWatch progress_;
  float dampingUntil_=0;
  int oscillationCount_=0, collisionCount_=0;
  uint8_t confirmedCells_[Maze::kMax*Maze::kMax/8]={};
  float lastExternalProgress_=0;
  bool wallAheadOpenConfirmed() const;
  bool checkFrontAfterPivot();
  float frontStopReading() const;

  Platform& io_;
  const Params& p_;
  Maze& maze_;
  Motion motion_;
  Sensors s_;
  float t_ = 0, tickDt_ = 0.002f;
  float rangeSeen_[3] = {};
  bool rangeLost_[3] = {};
  bool faultReported_ = false;
  bool rangeFault_ = false;

  int cx_ = 0, cy_ = 0;
  Dir h_ = NORTH;
  float centerOdo_ = 0;   // recorrido (mm) en el centro de la celda actual
  float gridAngle_ = 0;   // rumbo de la cuadricula (grados, acumulado)
  LateralFilter lateral_;  // desplazamiento lateral y error de rumbo
  float latAge_ = 1e9f;    // s desde la ultima medida lateral
  float lastOdo_ = 0;
  float gridCorr_ = 0;     // correccion de rumbo aplicada desde el ultimo giro (grados)
  float turnSince_ = 0;    // grados girados en el sitio desde la ultima estimacion
  int wallUpdates_ = 0;

  // Celda siguiente
  bool decided_ = false;
  Act act_ = Act::None;
  int nextDir_ = -1;
  bool arcStarted_ = false;
  bool arcPlanned_ = false;
  bool planActive_ = false;
  bool planArc_ = false;
  float planVEnd_ = 0;
  int planX_ = 0, planY_ = 0;
  int actX_ = -1, actY_ = -1;
  bool blocked_ = false;
  int blockedHits_ = 0;

  // Por sensor: lecturas seguidas muy cerca (si deja de medir, sigue habiendo pared) y perdidas seguidas.
  struct Beam {
    int near = 0;
    int lastValidMm = 1000;
    float lastValidT = -1;
    int misses = 0;
  } beam_[3];
  Sonda sonda_;
  Sonda::Scan scan_, scanJ_;
  Sonda::Combo combo_;
  float longVar_ = 25.0f;  // varianza del avance (mm^2)
  float vertKey_[7] = {};   // parametros verticales con los que se preparo la sonda (si cambian, de nuevo)

  float stallTimer_ = 0;
  float pushRefOdo_ = -1e9f;
  int pushRefFront_ = 0;
  bool stalled_ = false;
  bool recovering_ = false;
  int stalls_ = 0;
  int recoveries_ = 0;
  int recoverCellIdx_ = -1;
  int recoverCount_ = 0;
  float lastReport_ = -1;

  float smoothDf_ = 70.0f;  // avance de la curva (mm) por rumbo inicial
  float smoothDl_ = 70.0f;  // desplazamiento lateral
  float smoothArc_ = 100.0f;
  float smoothRadius_ = 50.0f;
  bool smoothOk_ = false;      // la curva cabe en la celda con margen
  float cell_ = 0;           // paso real de celda aprendido en marcha (mm)
  float cellRef_ = 0;        // el de los parametros (el ajuste no se aleja mas de un 8 %)
  bool learnCell_ = false;
  // Tramo recto actual, para el ajuste del paso: celdas avanzadas y correcciones aplicadas desde su inicio.
  int straightCells_ = 0;
  float straightCorr_ = 0;
  // Paso de celda: correccion acumulada frente a lo recorrido en la recta actual (regresion) y su duda.
  float straightStart_ = 0;
  int regN_ = 0;
  float regS_ = 0, regS2_ = 0, regC_ = 0, regSC_ = 0;
  int pitchFits_ = 0;
  float pitchSavedRef_ = -1, pitchWheelRef_ = -1, pitchCountsRef_ = -1;
  float cellVar_ = 0;
  float offsetsCell_ = 0;
  float halfCorr_ = 71.5f;   // medio pasillo
  bool smooth_ = false;
  float vmax_ = 300.0f;
  float requestedVmax_ = 300.0f;
};

}  // namespace raton


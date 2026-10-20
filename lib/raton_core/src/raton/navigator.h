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
  const Sonda& sonda() const { return sonda_; }
  float gyroScale() { return motion_.gyroScale(); }
  float cellPitch() const { return cell_; }
  // Pista de medida incierta (practica): ajustar el paso de celda en marcha con las paredes de enfrente.
  void setLearnCell(bool on) { learnCell_ = on; }

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
  // Un ciclo de control con los motores segun Motion (para modos externos).
  void step();

 private:
  enum class Act : uint8_t { None, Forward, Left, Right, Back, Stop };

  void tick();
  void setMotorsFromMotion();
  void report();
  float cellPos() const { return motion_.position() - centerOdo_; }
  float gridAngle() const { return gridAngle_; }
  float headingError() const { return motion_.angle() - gridAngle_; }
  void enterNextCell();
  void sense();
  void radar(int sensor, int mm);
  void frontCheck(int mm);
  // Pose del sensor (0 izq, 1 frente, 2 der) en el marco del laberinto, con la pose del raton desplazada
  // fwd mm hacia delante, lat mm a la izquierda y dpsi rad (para los jacobianos).
  void sensorPose(int sensor, float fwd, float lat, float dpsi, float& x, float& y, float& ang) const;
  // Todas las paredes que el cono cruza hasta reach estan confirmadas (sirve para corregir la pose).
  bool confident(const Sonda::Scan& scan, float reach) const;
  void resetBeams();
  void steer();
  Act actionFor(int dir) const;
  void plan(Brain& brain);
  enum class Depart { Go, Stop, Fail };
  Depart depart(Brain& brain, float vmax);
  void centerOnFront(int k);
  float actCenterOdo() const;
  float planTarget() const;
  bool pivot(int quarterTurns);
  void learnGyroScale();
  void applyCorrection(float e, float gain);
  void startStraight();
  bool waitStopped(float timeout);
  bool recover(Brain& brain);
  void checkStall();
  void computeSmoothGeometry();
  bool wallAheadOpenConfirmed() const;
  bool checkFrontAfterPivot();
  float frontStopReading() const;

  Platform& io_;
  const Params& p_;
  Maze& maze_;
  Motion motion_;
  Sensors s_;
  float t_ = 0;

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

  float stallTimer_ = 0;
  float pushRefOdo_ = -1e9f;
  int pushRefFront_ = 0;
  bool stalled_ = false;
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
  bool cellLearned_ = false;
  float halfCorr_ = 71.5f;   // medio pasillo
  bool smooth_ = false;
  float vmax_ = 300.0f;
};

}  // namespace raton

// Navegador de celdas: mueve el raton de celda en celda leyendo las paredes por delante.
//
// Geometria clave (sensores a 45 grados): el rayo lateral toca la pared ~70 mm por delante del eje. Por
// eso las paredes laterales de la celda SIGUIENTE se leen cuando el eje aun esta en la actual (entre
// 27 y 55 mm pasado su centro), y la decision de que hacer en la siguiente celda esta lista antes de
// entrar en ella: no hay que pararse a mirar ni a pensar (flood fill < 1 ms).
//
// Cada lectura se convierte en un voto sobre una pared concreta del mapa (la celda donde cae el rayo),
// descartando las que caen cerca de los postes. El frontal vota la pared que tiene delante y corrige
// la distancia recorrida; el paso de pared a hueco (el poste) corrige el avance en pasillos largos.
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
  float gyroScale() { return motion_.gyroScale(); }
  float cellPitch() const { return cell_; }

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
  void senseSides();
  void senseFront();
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
  int frontLastMm_ = 1000;
  int frontMisses_ = 0;
  float frontLastT_ = -1;
  bool blocked_ = false;
  int blockedHits_ = 0;

  // Transiciones pared/hueco por lado para corregir el avance
  struct SideTrack {
    bool valid = false;
    bool wall = false;
    float hitOdo = 0;
    int lastValidMm = 1000;
    float lastValidT = -1;
  } side_[2];

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
  float halfCorr_ = 71.5f;   // medio pasillo medido en marcha
  float sideLat_[2] = {0, 0};
  float sideLatOdo_[2] = {-1e9f, -1e9f};
  bool smooth_ = false;
  float vmax_ = 300.0f;
};

}  // namespace raton

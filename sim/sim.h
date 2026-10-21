// Simulador fisico del raton para probar el nucleo sin la placa.
//  - Laberinto real con paredes y postes de grosor wallMm.
//  - Ruedas: motor de primer orden (constante de tiempo, rozamiento estatico, asimetria) y limite de
//    adherencia (si se acelera de mas, la rueda patina: el encoder cuenta y el raton no avanza).
//  - Encoders con cuentas enteras y diametro real distinto del nominal.
//  - Giroscopio a 200 Hz con sesgo, deriva y ruido.
//  - VL53L0X: rayo hasta la pared, medida cada ~21 ms con latencia, ruido proporcional, lecturas perdidas,
//    falsas, fuera de alcance, y lectura por turnos (una por ciclo) como en la placa.
//  - Choques: el cuerpo redondo no atraviesa paredes; el contacto se cuenta.
#pragma once

#include <math.h>
#include <stdint.h>

#include <random>
#include <string>
#include <vector>

#include "raton/maze.h"
#include "raton/params.h"
#include "raton/platform.h"

namespace sim {

struct Physical {
  // Diferencias entre el raton real y lo que creen los parametros (para probar robustez).
  float cellMm = 155.0f;
  float wallMm = 12.0f;
  float wheelDiamL = 34.0f;
  float wheelDiamR = 34.0f;
  float trackMm = 75.0f;
  float bodyDiamMm = 90.0f;
  float diagAngleDeg = 45.0f;  // rayo unico (modelo antiguo, fovDeg = 0)
  // Modelo real del VL53L0X: cono de fovDeg; devuelve la media de las distancias pesada por la senal que
  // vuelve (mas fuerte de cerca y de frente). Los laterales montados a mountDeg.
  float fovDeg = 0.0f;
  float mountDeg = 45.0f;
  // En vertical: altura e inclinacion de los sensores, altura de las paredes, reflejo del piso y lo que hay
  // fuera del laberinto (gente, mesas) a outsideMm, que se ve por encima de las paredes (0 = nada).
  float sensorHeightMm = 25.0f;
  float sensorTiltDeg = 0.0f;
  float wallHeightMm = 50.0f;
  float floorReflect = 0.3f;
  float outsideMm = 0.0f;
  // Chasis (rectangulo respecto al eje de las ruedas) para los choques.
  bool rectBody = false;
  float bodyFrontMm = 45.0f, bodyRearMm = 45.0f, bodyWidthMm = 90.0f;
  float motorTau = 0.04f;     // s
  float vmaxFull = 750.0f;    // mm/s con duty 1
  float stictionDuty = 0.05f;
  float motorAsym = 0.0f;     // +x = rueda derecha x mas fuerte
  float grip = 5500.0f;       // mm/s^2 maximos por adherencia
  float gyroBiasDps = 0.05f; // residual tras calibrar en reposo al arrancar
  float gyroDriftDps = 0.02f; // por segundo (paseo aleatorio)
  float gyroNoiseDps = 0.08f;
  float gyroScale = 1.0f;
  float rangeNoise = 0.03f;   // fraccion de la distancia
  float rangeNoiseMm = 1.5f;
  float dropout = 0.02f;
  float outlier = 0.004f;
  int offsetF = 0, offsetL = 0, offsetR = 0;  // error de cada sensor (mm)
  // Inyeccion reproducible: -1 desactiva, segundos desde el arranque.
  float laserFailAt[3] = {-1, -1, -1};  // izq, frente, der: no llegan muestras
  float gyroFailAt = -1;
  bool gyroFrozen = false;  // true: publica el mismo angulo; false: no publica
  float startX = 0, startY = 0, startDeg = 0;  // error de colocacion en la salida
};

struct Rect {
  float x0, y0, x1, y1;
};

class World {
 public:
  World(const raton::Maze& truth, const Physical& ph);
  // Distancia hasta la primera caja; axis = 0 si choca con una cara vertical (normal en x), 1 si horizontal.
  float raycast(float x, float y, float ang, float maxd, int* axis = nullptr) const;
  // Lectura sin ruido de un VL53L0X con cono de fovDeg (2000 = sin senal).
  float coneReading(float x, float y, float ang, float fovDeg) const;
  bool collides(float x, float y, float r) const;
  // Chasis rectangular (front/rear respecto al eje, ancho w) con rumbo th: toca alguna pared o poste.
  bool collidesBody(float x, float y, float th, float front, float rear, float w) const;
  const raton::Maze& truth() const { return truth_; }

 private:
  raton::Maze truth_;
  Physical ph_;
  std::vector<Rect> rects_;
};

struct Event {
  float t;
  std::string text;
};

class SimPlatform : public raton::Platform {
 public:
  SimPlatform(const World& world, const raton::Params& p, const Physical& ph, uint32_t seed);

  // Coloca el raton en el centro de una celda mirando a dir (con el error de colocacion de Physical).
  void place(int x, int y, int dir);

  void setMotors(float left, float right) override;
  const raton::Sensors& tick() override;
  void report(const char* line, bool important) override;
  bool saveMap(const raton::Maze& maze) override;

  // Estado real
  float x() const { return x_; }
  float y() const { return y_; }
  float headingDeg() const;
  int cellX() const;
  int cellY() const;
  float time() const { return t_; }
  int contacts() const { return contacts_; }
  float contactTime() const { return contactTime_; }
  float maxAreaTime() const { return maxArea_; }
  bool verbose = false;
  bool trace = false;  // imprime la pose cada 0.1 s
  std::vector<Event> events;
  float timeLimit = 1200.0f;  // corta la simulacion (raton bloqueado en un bucle)
  bool stopRequested() override { return t_ > timeLimit; }
  bool saved = false;
  const raton::Maze* watch = nullptr;  // avisa cuando una pared del mapa del raton contradice la real
  int wrongEvents = 0;
  // Error de localizacion (estimada - real): rms y maximo del avance, lateral (mm) y rumbo (grados).
  float locRms(int which) const {
    const float v = which == 0 ? locLong2_ : (which == 1 ? locLat2_ : locHead2_);
    return locN_ > 0 ? sqrtf(v / locN_) : 0.0f;
  }
  int lostEvents() const { return lostEvents_; }
  // Real: rumbo respecto a la cuadricula (grados rms), descentrado (mm rms y maximo) en los tramos rectos.
  float wobbleRms() const { return locN_ > 0 ? sqrtf(wob2_ / locN_) : 0.0f; }
  float offsetRms() const { return locN_ > 0 ? sqrtf(off2_ / locN_) : 0.0f; }
  float offsetMax() const { return offMax_; }
  float locMax(int which) const { return which == 0 ? locLongMax_ : (which == 1 ? locLatMax_ : locHeadMax_); }

 private:
  void physics(float dt);
  void updateLasers();
  int readLaser(int i);

  const World& world_;
  const raton::Params& p_;
  Physical ph_;
  std::mt19937 rng_;
  raton::Sensors s_;
  float t_ = 0;
  float x_ = 0, y_ = 0, th_ = 0;  // mm, rad (0 = este, antihorario)
  float wl_ = 0, wr_ = 0;          // velocidad de la rueda (mm/s)
  float bl_ = 0, br_ = 0;          // velocidad sobre el suelo
  float dutyL_ = 0, dutyR_ = 0;
  double encL_ = 0, encR_ = 0;     // mm reales de giro de rueda (diametro real)
  float bias_ = 0;
  float nextGyro_ = 0;
  float gyroAcc_ = 0;
  float lastGyroRead_ = 0;
  float yawTrue_ = 0;
  // Laseres: siguiente medida lista y valor
  float laserReady_[3] = {0, 0, 0};
  int laserValue_[3] = {-1, -1, -1};
  bool laserPending_[3] = {false, false, false};
  int tickCount_ = 0;
  int contacts_ = 0;
  bool touching_ = false;
  float contactTime_ = 0;
  // Zona (regla de los 30 s) medida con la posicion real
  int ax_ = -1000, ay_ = -1000;
  float anchorT_ = 0;
  float maxArea_ = 0;
  uint8_t wrongState_[raton::Maze::kMax * raton::Maze::kMax] = {};
  bool lost_ = false;
  float wob2_ = 0, off2_ = 0, offMax_ = 0;
  float lastErr_[3] = {0, 0, 0};
  int lostEvents_ = 0;
  float locN_ = 0, locLong2_ = 0, locLat2_ = 0, locHead2_ = 0, locLongMax_ = 0, locLatMax_ = 0, locHeadMax_ = 0;
};

// Cono de los sensores para todos los casos (25 = VL53L0X real, 0 = rayo unico). --cono en el simulador.
extern float gFovDeg;
// Calibrar los sensores al arrancar (como el modo 1). --sincal lo quita.
extern bool gAutoCal;
// Objetos fuera del laberinto a esta distancia (se ven por encima de las paredes). --fuera.
extern float gOutsideMm;

// Laberintos de prueba
bool loadMazeFile(const std::string& path, raton::Maze& out, std::vector<std::string>* rows = nullptr);
// Laberinto aleatorio perfecto (un solo camino) de w x h, con loops extra paredes quitadas.
void randomMaze(int w, int h, int loops, uint32_t seed, raton::Maze& out);
// Salida como en la competencia: pared a los dos lados y abierta al frente (y todo sigue conectado).
void makeStartCell(raton::Maze& m, int x, int y, raton::Dir d, uint32_t seed);

}  // namespace sim

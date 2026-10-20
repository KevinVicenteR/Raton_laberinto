// Sonda: lo que deberia leer cada VL53L0X segun el mapa y la pose del raton.
//
// El VL53L0X no mide un rayo fino: ilumina un cono de ~25 grados y da una media de las distancias pesada
// por la luz que vuelve (mas de lo cercano y de lo que esta de frente). Con los laterales a 45 grados la
// mancha sobre la pared mide de 4 a 9 cm de largo segun el pasillo, y cerca de un hueco o de un poste la
// lectura mezcla las dos cosas. Por eso aqui se traza el cono entero (varios rayos) por la rejilla de
// paredes y postes, igual que en el simulador.
//
// El cono tambien se abre en vertical. Con el sensor a altura h y paredes de altura H:
//  - la parte de arriba pasa por encima de una pared a distancia t si su angulo supera atan((H - h) / t):
//    con el sensor alto, gran parte del cono mira fuera del laberinto;
//  - la parte de abajo toca el piso a h / tan(angulo): en un pasillo vacio el frontal lee el piso (con el
//    sensor a 40 mm, a unos 18 cm).
// Cada columna del cono se reparte entre piso, pared y "por encima" con esos angulos.
//
// Con la prediccion se hace todo lo demas, como un radar:
//  - mapa: para cada pared que el cono puede ver se comparan las dos hipotesis (pared / hueco) con la
//    lectura y se vota la que la explica (Navigator::radar);
//  - posicion: lo que cambia la prediccion al mover el raton (jacobiano) corrige el desplazamiento lateral,
//    el rumbo y el avance.
#pragma once

#include "raton/maze.h"

namespace raton {

class Sonda {
 public:
  static const int kRays = 9;      // columnas del cono
  static const int kMaxHits = 10;  // fronteras por columna
  static const int kSolid = -1;    // poste, borde o fuera del laberinto: siempre hay algo

  struct Hit {
    int seg;       // segmento de pared (Maze::hSeg / vSeg) o kSolid
    float t;       // distancia horizontal desde el sensor hasta la cara
    float cosInc;  // coseno del angulo de incidencia
  };
  struct Ray {
    int j;  // columna del cono
    int n;
    Hit hits[kMaxHits];
  };
  struct Scan {
    Ray rays[kRays];
    int sensor = 1;  // 0 izquierdo, 1 frontal, 2 derecho (cada uno con su inclinacion)
  };
  // Estado supuesto de algunas paredes (el resto, el del mapa).
  struct Override {
    int n = 0;
    int seg[8];
    bool closed[8];
    void set(int s, bool c);
    bool find(int s, bool& c) const;
  };

  void setup(float cellMm, float wallMm, float fovDeg, int maxMm);
  // Altura del sensor, su inclinacion (+ = arriba), altura de las paredes y reflejo del piso (0 = no se ve).
  // Sin llamarla, paredes infinitas y sin piso (modelo plano).
  void setVertical(float sensorHeightMm, float tiltDeg, float wallHeightMm, float floorReflect);
  // Inclinacion de un sensor (se afina en marcha) y reflejo del piso.
  void setTilt(int sensor, float deg) { tilt_[sensor] = deg / 57.2957795f; }
  float tiltDeg(int sensor) const { return tilt_[sensor] * 57.2957795f; }
  void setFloorReflect(float r) { floorR_ = r; }
  float floorReflect() const { return floorR_; }
  // Traza el cono del sensor desde (x, y) mm (marco del laberinto: x este, y norte) con eje en ang (rad,
  // 0 = este, antihorario).
  void trace(const Maze& m, float x, float y, float ang, Scan& out, int sensor = 1) const;
  // Lectura esperada; -1 = sin senal o mas lejos que el alcance. Las paredes con votos > 0 cuentan como
  // cerradas, el resto abiertas, salvo lo que diga ov.
  float predict(const Maze& m, const Scan& s, const Override* ov = nullptr) const;
  // Lo mismo y ademas cuanto de la lectura es piso: floorSigma = duda (mm) por lo mal que se conoce el piso
  // (reflejo, inclinacion; el piso real no es el del modelo).
  float predict(const Maze& m, const Scan& s, const Override* ov, float& floorSigma) const;
  // Prediccion rapida para todas las combinaciones de unas pocas paredes (cand[0..nc), nc <= 8): cada
  // columna queda como una lista corta de lo que puede tocar; las demas paredes, como esten en el mapa.
  struct Combo {
    struct R {
      int n;
      float w[kMaxHits], wt[kMaxHits];    // todo lo que vuelve si lo primero cerrado es esta frontera...
      float wf[kMaxHits], wtf[kMaxHits];  // ...y la parte que es piso
      int8_t cand[kMaxHits];  // indice en cand, o -1 = siempre hay algo
      float wEnd, wtEnd;      // si no toca nada: solo el piso
    } rays[kRays];
  };
  void prepare(const Maze& m, const Scan& s, const int* cand, int nc, Combo& out) const;
  // Lectura esperada con las paredes cand[k] cerradas segun el bit k de mask (y la duda por el piso).
  float predictMask(const Combo& c, int mask, float* floorSigma = nullptr) const;
  // Pared plana paralela al rayo central con angulo theta (rad) respecto a ella: lectura = distancia
  // perpendicular x factor.
  float parallelFactor(float theta) const;
  int maxMm() const { return maxMm_; }
  // Distancia a la que el cono del sensor empieza a tocar el piso (sin paredes delante); 0 si no lo toca.
  float floorStartMm(int sensor = 1) const;

 private:
  void traceRay(const Maze& m, float x0, float y0, float ux, float uy, Ray& r) const;
  // Peso y peso x distancia de la columna j si lo primero cerrado esta a t (any = false: no toca nada).
  // wf, wtf: la parte del piso.
  void column(int j, float tilt, float t, float cosInc, bool any, float& w, float& wt, float& wf, float& wtf) const;
  // Duda por el piso de una lectura d con suma de pesos sw, de los que swf (con swfd = peso x distancia) piso.
  static float floorDoubt(float d, float sw, float swf, float swfd);
  float cell_ = 155.0f, hw_ = 6.0f, halfFov_ = 0.218f;
  int maxMm_ = 600;
  float wRef_ = 0;  // senal minima (pared de frente a 1200 mm)
  float rayW_[kRays] = {}, rayCos_[kRays] = {}, raySin_[kRays] = {}, rayE_[kRays] = {};
  bool vertical_ = false;
  float h_ = 40.0f, wallH_ = 50.0f, floorR_ = 0.0f;
  float tilt_[3] = {0, 0, 0};  // rad
};

}  // namespace raton

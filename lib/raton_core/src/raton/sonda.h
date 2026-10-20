// Sonda: lo que deberia leer cada VL53L0X segun el mapa y la pose del raton.
//
// El VL53L0X no mide un rayo fino: ilumina un cono de ~25 grados y da una media de las distancias pesada
// por la luz que vuelve (mas de lo cercano y de lo que esta de frente). Con los laterales a 45 grados la
// mancha sobre la pared mide de 4 a 9 cm de largo segun el pasillo, y cerca de un hueco o de un poste la
// lectura mezcla las dos cosas. Por eso aqui se traza el cono entero (varios rayos) por la rejilla de
// paredes y postes, igual que en el simulador.
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
  static const int kRays = 9;  // rayos por cono
  static const int kMaxHits = 10;  // fronteras por rayo
  static const int kSolid = -1;    // poste, borde o fuera del laberinto: siempre hay algo

  struct Hit {
    int seg;       // segmento de pared (Maze::hSeg / vSeg) o kSolid
    float t;       // distancia desde el sensor hasta la cara
    float cosInc;  // coseno del angulo de incidencia
  };
  struct Ray {
    float w0;  // peso del rayo dentro del cono
    int n;
    Hit hits[kMaxHits];
  };
  struct Scan {
    Ray rays[kRays];
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
  // Traza el cono desde (x, y) mm (marco del laberinto: x este, y norte) con eje en ang (rad, 0 = este,
  // antihorario).
  void trace(const Maze& m, float x, float y, float ang, Scan& out) const;
  // Lectura esperada; -1 = sin senal o mas lejos que el alcance. Las paredes con votos > 0 cuentan como
  // cerradas, el resto abiertas, salvo lo que diga ov.
  float predict(const Maze& m, const Scan& s, const Override* ov = nullptr) const;
  // Prediccion rapida para todas las combinaciones de unas pocas paredes (cand[0..nc), nc <= 8): cada rayo
  // queda como una lista corta de lo que puede tocar; las demas paredes, como esten en el mapa.
  struct Combo {
    struct R {
      int n;
      float w[kMaxHits], wt[kMaxHits];
      int8_t cand[kMaxHits];  // indice en cand, o -1 = siempre hay algo
    } rays[kRays];
  };
  void prepare(const Maze& m, const Scan& s, const int* cand, int nc, Combo& out) const;
  // Lectura esperada con las paredes cand[k] cerradas segun el bit k de mask.
  float predictMask(const Combo& c, int mask) const;
  // Pared plana paralela al rayo central con angulo theta (rad) respecto a ella: lectura = distancia
  // perpendicular x factor.
  float parallelFactor(float theta) const;
  int maxMm() const { return maxMm_; }

 private:
  void traceRay(const Maze& m, float x0, float y0, float ux, float uy, Ray& r) const;
  float cell_ = 155.0f, hw_ = 6.0f, halfFov_ = 0.218f;
  int maxMm_ = 600;
  float wRef_ = 0;  // senal minima (pared de frente a 1200 mm)
  float rayW_[kRays] = {}, rayCos_[kRays] = {}, raySin_[kRays] = {};
};

}  // namespace raton

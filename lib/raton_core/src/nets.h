// Entradas de las redes de control y de logica (identicas a tools/nets.py).
#pragma once
#include "risk_predictor.h"

namespace raton {

constexpr int kControlInputs = 43;
constexpr int kLogicInputs = 36;
constexpr float kControlYawMax = 4.0f;   // grados de correccion maxima del rumbo objetivo
constexpr float kControlVMin = 0.3f;     // factor de velocidad minimo

// Red de control: historial de 3 lecturas de L/F/R, velocidad (mm/s), giro (grados/s), error respecto
// a la cuadricula (grados, + = izquierda), lo que falta del tramo (mm), rumbo objetivo del centrado
// (grados) y si va marcha atras.
void controlFeatures(const RiskFrame history[3], float speed, float yawRate, float headingError,
                     float remaining, float baseTarget, bool reverse, float out[kControlInputs]);
// Salida de la red -> correccion de rumbo (grados) y factor de velocidad (0.3..1).
void controlOutputs(const float out[2], float& yawDeg, float& speedScale);

// Red de logica: una entrada por direccion relativa al raton (0 delante, 1 derecha, 2 atras, 3 izquierda).
struct LogicDir {
  bool open, known, seen, allowed;
  int dist;      // pasos de inundacion del vecino (999 = pared o sin camino)
  int visits;
  float goalCos; // coseno entre esa direccion y la meta
};
void logicFeatures(const LogicDir dirs[4], int distHere, float seenFraction, float goalForward,
                   float goalRight, int heading, float out[kLogicInputs]);

}  // namespace raton

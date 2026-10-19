#pragma once
#include <stdint.h>

namespace cfg {


// ---------- Mecanica (CALIBRAR) ----------
const float WHEEL_DIAMETER_MM = 34.0f;
// N20: 7 pulsos por vuelta del motor x 4 flancos x reduccion 1:100 (medido: 30 cm empujando a mano)
const float ENCODER_COUNTS_PER_REV = 7.0f * 4.0f * 100.0f;
const float MM_PER_COUNT = 3.14159265f * WHEEL_DIAMETER_MM / ENCODER_COUNTS_PER_REV;
const bool INVERT_ENC_IZQ = true;
const bool INVERT_ENC_DER = false;
const bool ENCODERS_SWAPPED = true;  // MEDIDO con la prueba de ruedas (T): encoder izq y der cruzados

// ---------- Rumbo con encoders ----------
const float TRACK_WIDTH_MM = 75.0f;        // distancia entre ruedas; al arrancar se mide sola
const float ENC_YAW_WEIGHT = 0.0f;         // peso de los encoders en el rumbo (0 = solo giroscopio; si las

const int VISIT_GRID = 40;

}  // namespace cfg

#pragma once
#include <stdint.h>
#include "hardware/params.h"

namespace cfg {

const float CONF_MIN_SPEED = 0.6f;        // con confianza 0 la velocidad baja a este factor
const float SEARCH_SPEED = 100.0f;        // mm/s
const float SPEED_RUN_SPEED = 700.0f;     // mm/s (max teorico ~890 mm/s a 6 V)
const float ACCEL = 1000.0f;               // mm/s^2
// Velocidad adaptativa: confianza, rumbo, celdas desconocidas y margen frontal.
// Tras giro/recuperacion se recorren ADAPT_CAUTION_MM a consigna cauta; transiciones con ACCEL.
const float ADAPT_MIN_RATIO = 0.832f;
const float ADAPT_UNKNOWN_RATIO = 0.977f;
const float ADAPT_HEADING_DEG = 6.91f;
const float ADAPT_CAUTION_MM = 63.7f;
const float ADAPT_FRONT_MM = 60.0f;
const float MIN_SPEED = 40.0f;            // mm/s al final de la frenada

// ---------- Seguidor de paredes (CALIBRAR en la pista) ----------
const float WALL_FOLLOW_SPEED = SEARCH_SPEED;
// Pista conocida (modo 4): tramos rectos de una vez a esta velocidad (mm/s). Si una pared no coincide
// con el mapa, pasa a buscar la meta como en una pista desconocida.
const float COMP_SPEED = 368.8f;
const float EXPLORE_SPEED = 230.0f;        // margen N20: perfil trapezoidal y PI capaz de frenar
const float SLOW_NEAR_WALL_SPEED = 60.0f;   // con pared delante a menos de FRONT_WALL_MM, como mucho esto

const float KFF_SPEED = (PWM_MAX - PWM_MIN_RUN) / 500.0f; // PWM por mm/s, sumado a PWM_MIN_RUN
const float SPEED_FILTER = 0.85f;         // suavizado de la velocidad medida (0..1, mas = mas suave)
const float KP_SPEED = 0.0008f;
const float KI_SPEED = 0.004f;

}  // namespace cfg

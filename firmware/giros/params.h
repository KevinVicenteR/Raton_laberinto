#pragma once
#include <stdint.h>

namespace cfg {

const uint32_t SETTLE_MS = 15;             // pausa tras cada maniobra (corrige sesgo y estabiliza)
const float TURN_RATE_DPS = 220.0f;
const float TURN_ACCEL_DPS2 = 900.0f;
const int POST_TURN_SAMPLES = 2;
const float KD_TURN = 0.0010f;
const bool TURN_IN_PLACE = true;            // true = gira sobre si mismo (pista pequena), false = pivote en una rueda
const float TURN_PWM_MAX = 0.45f;
const float TURN_PWM_MIN = 0.28f;          // minimo para ARRANCAR a girar (parado); girando ya no se fuerza
const float TURN_TOLERANCE_DEG = 3.0f;     // giro dado por bueno
const float TURN_RETRY_DEG = 3.0f;         // reintentar solo fuera de la precision requerida
const int TURN_ATTEMPTS = 2;               // intentos por giro
const float TURN_FAIL_DEG = 3.0f;         // no continuar con un error >=3 grados
const uint32_t TURN_TIMEOUT_MS = 3000;

}  // namespace cfg

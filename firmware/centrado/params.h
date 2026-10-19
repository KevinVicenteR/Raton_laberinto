#pragma once
#include <stdint.h>

namespace cfg {

const float KP_YAW = 0.0250f;              // PWM por grado de error
const float KD_YAW = 0.00124f;             // PWM por grado/s
const float KP_WALL = 0.353f;              // cascada: grados de rumbo por mm descentrado (modo por defecto)
// PID de direccion sobre la desviacion lateral (mm respecto al centro del pasillo):
//   giro = KP_STEER * desviacion + KD_STEER * (cambio de la desviacion en un ciclo de control)
// La derivada sale de la velocidad que estima el Kalman de cada laser (no del valor crudo, que salta).
const float KP_STEER = 0.0014f;           // PWM entre ruedas por mm de desviacion
const float KD_STEER = 1.0f;              // PWM por mm de cambio de desviacion en un ciclo (5 ms)
// Modo de direccion:
//   0 = CASCADA (por defecto): la desviacion lateral fija el rumbo objetivo (KP_WALL, escalado por la
//       confianza de los laseres) y un PID rapido sobre el giroscopio lo sigue. En el simulador
//       llega en 14 de 15 casos de 14 cm.
//   1 = PID DIRECTO con KP_STEER y KD_STEER sobre la desviacion. En la simulacion con Kp 0.0014 y Kd 1.0
//       llega en 1 de 10 (el Kd amplifica el ruido de los laseres, que actualizan a 50 Hz).
// Se cambia en marcha desde la web (orden p0 / p1).
const int STEER_MODE = 0;
const float MAX_WALL_CORRECTION_DEG = 7.01f;
const float LATERAL_DEADBAND_MM = 3.47f;   // descentrado menor que esto no se corrige (evita zigzag)
const float YAW_TARGET_SMOOTH = 0.12f;    // 200 Hz: menos retardo en la cascada amortiguada
const float MAX_STEER = 0.14f;            // correccion maxima de PWM entre ruedas (evita bandazos)

const float WALL_RATE_DAMP = 0.0455f;       // grados por mm/s de deriva lateral (Kalman)

}  // namespace cfg

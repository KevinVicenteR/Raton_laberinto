#pragma once
#include <stdint.h>

namespace cfg {


// Cambia el signo si una rueda gira al reves o un encoder cuenta al reves
const bool INVERT_MOTOR_IZQ = true;
const bool INVERT_MOTOR_DER = false;

// Giroscopio: +1 si girar a la izquierda (antihorario visto desde arriba) da yaw positivo
const float GYRO_SIGN = 1.0f;

// ---------- Motores ----------
// Motores de 6 V con bateria 2S (8.4 V): limitar a ~70 % del PWM
const float PWM_MAX = 0.70f;      // 0..1
const float PWM_MIN_MOVE = 0.25f; // minimo para que el N20 arranque (parado)
const float PWM_MIN_RUN = 0.18f;  // minimo para que siga en marcha sin pararse
const uint32_t PWM_PERIOD_US = 2000; // 500 Hz como analogWrite: a 20 kHz los N20 1:100 no tienen fuerza

// ---------- Giroscopio ----------
const float GYRO_BIAS_TRACK_DPS = 2.0f;    // parado y girando menos que esto: se corrige el sesgo
const float GYRO_BIAS_TRACK_GAIN = 0.01f;  // velocidad de esa correccion por muestra

// ---------- Arranque automatico (sin arrancador) ----------
const uint32_t POWER_SETTLE_MS = 500;   // margen para que laseres, IMU y OLED se enciendan
const int SENSOR_INIT_TRIES = 5;        // reintentos de arranque de cada sensor

}  // namespace cfg

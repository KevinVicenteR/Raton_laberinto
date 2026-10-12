
#pragma once

#include <Arduino.h>

namespace cfg {

// ---------- Pines ----------
// Laseres VL53L0X (I2C en A4/A5, compartido con la OLED)
const uint8_t PIN_XSHUT_IZQ = A0;
const uint8_t PIN_XSHUT_FRO = A1;
const uint8_t PIN_XSHUT_DER = A2;

// DRV8833
const uint8_t PIN_AIN1 = 5;  // motor izquierdo
const uint8_t PIN_AIN2 = 6;
const uint8_t PIN_BIN1 = 9;  // motor derecho
const uint8_t PIN_BIN2 = 10;

// Encoders N20 (amarillo = A, verde = B). Alimentarlos a 3.3 V, NO a 5 V.
const uint8_t PIN_ENC_IZQ_A = 2;
const uint8_t PIN_ENC_IZQ_B = 3;
const uint8_t PIN_ENC_DER_A = 4;
const uint8_t PIN_ENC_DER_B = 7;

// ---------- Direcciones I2C ----------
const uint8_t I2C_LASER_IZQ = 0x30;
const uint8_t I2C_LASER_FRO = 0x31;
const uint8_t I2C_LASER_DER = 0x32;
const uint8_t I2C_OLED = 0x3C;

// ---------- Laberinto ----------
const int MAZE_WIDTH = 16;
const int MAZE_HEIGHT = 16;
const float CELL_MM = 180.0f;
const int MAX_SEARCH_RUNS = 4;  // viajes ida/vuelta maximos explorando

// ---------- Mecanica (CALIBRAR) ----------
const float WHEEL_DIAMETER_MM = 34.0f;
// N20 500 rpm: 7 pulsos por vuelta del motor x 4 flancos x reduccion 1:30
const float ENCODER_COUNTS_PER_REV = 7.0f * 4.0f * 30.0f;
const float MM_PER_COUNT = 3.14159265f * WHEEL_DIAMETER_MM / ENCODER_COUNTS_PER_REV;

// Cambia el signo si una rueda gira al reves o un encoder cuenta al reves
const bool INVERT_MOTOR_IZQ = false;
const bool INVERT_MOTOR_DER = true;
const bool INVERT_ENC_IZQ = false;
const bool INVERT_ENC_DER = true;

// Giroscopio: +1 si girar a la izquierda (antihorario visto desde arriba) da yaw positivo
const float GYRO_SIGN = 1.0f;

// ---------- Motores ----------
// Motores de 6 V con bateria 2S (8.4 V): limitar a ~70 % del PWM
const float PWM_MAX = 0.70f;      // 0..1
const float PWM_MIN_MOVE = 0.12f; // minimo para que el N20 arranque
const uint32_t PWM_PERIOD_US = 50; // 20 kHz, sin pitido

// ---------- Paredes (CALIBRAR, en mm medidos por cada laser) ----------
const int SIDE_WALL_MM = 110;        // lateral por debajo de esto = hay pared
const int FRONT_WALL_MM = 140;       // frontal por debajo de esto = hay pared
const int SIDE_CENTER_MM = 50;       // lectura lateral con el raton centrado en la celda
const int FRONT_STOP_MM = 45;        // lectura frontal con el raton en el centro de la celda

// ---------- Arranque automatico (sin arrancador) ----------
const uint32_t POWER_SETTLE_MS = 500;   // margen para que laseres, IMU y OLED se enciendan
const int SENSOR_INIT_TRIES = 5;        // reintentos de arranque de cada sensor
const uint32_t START_DELAY_MS = 0;      // espera tras encender antes de explorar (0 = al instante)
const uint32_t RUN_PAUSE_MS = 2000;     // pausa en la salida entre carreras rapidas

// ---------- Control ----------
const uint32_t CONTROL_PERIOD_US = 5000;  // 200 Hz
const float SEARCH_SPEED = 250.0f;        // mm/s
const float SPEED_RUN_SPEED = 500.0f;     // mm/s (max teorico ~890 mm/s a 6 V)
const float ACCEL = 1200.0f;              // mm/s^2
const float MIN_SPEED = 60.0f;            // mm/s al final de la frenada

const float KFF_SPEED = PWM_MAX / 800.0f; // PWM por mm/s (prealimentacion)
const float KP_SPEED = 0.0008f;
const float KI_SPEED = 0.004f;
const float KP_YAW = 0.012f;              // PWM por grado de error
const float KD_YAW = 0.0006f;             // PWM por grado/s
const float KP_WALL = 0.15f;              // grados de correccion por mm descentrado
const float MAX_WALL_CORRECTION_DEG = 6.0f;

const float KP_TURN = 0.010f;
const float KD_TURN = 0.0008f;
const float TURN_PWM_MAX = 0.45f;
const float TURN_TOLERANCE_DEG = 1.5f;
const uint32_t TURN_TIMEOUT_MS = 2000;

}  // namespace cfg

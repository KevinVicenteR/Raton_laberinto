
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
// N20: 7 pulsos por vuelta del motor x 4 flancos x reduccion 1:100 (medido: 30 cm empujando a mano)
const float ENCODER_COUNTS_PER_REV = 7.0f * 4.0f * 100.0f;
const float MM_PER_COUNT = 3.14159265f * WHEEL_DIAMETER_MM / ENCODER_COUNTS_PER_REV;

// Cambia el signo si una rueda gira al reves o un encoder cuenta al reves
const bool INVERT_MOTOR_IZQ = true;
const bool INVERT_MOTOR_DER = false;
const bool INVERT_ENC_IZQ = true;
const bool INVERT_ENC_DER = false;

// Giroscopio: +1 si girar a la izquierda (antihorario visto desde arriba) da yaw positivo
const float GYRO_SIGN = 1.0f;

// ---------- Motores ----------
// Motores de 6 V con bateria 2S (8.4 V): limitar a ~70 % del PWM
const float PWM_MAX = 0.70f;      // 0..1
const float PWM_MIN_MOVE = 0.25f; // minimo para que el N20 arranque
const uint32_t PWM_PERIOD_US = 2000; // 500 Hz como analogWrite: a 20 kHz los N20 1:100 no tienen fuerza

// ---------- Filtro de Kalman de los laseres ----------
const float LASER_KF_R = 64.0f;        // ruido de medida (mm^2, ~8 mm): mas alto = suaviza mas pero tarda mas
const float LASER_KF_ACCEL = 1500.0f;  // cuanto puede cambiar la velocidad de acercamiento (mm/s^2): mas alto = reacciona antes
const float LASER_JUMP_MM = 50.0f;     // salto mayor que esto: se descarta si es suelto, se acepta si se repite
const uint8_t LASER_JUMP_CONFIRM = 2;  // lecturas seguidas para aceptar un salto
const uint8_t LASER_MAX_MISS = 2;      // lecturas invalidas seguidas para dar el lado por libre (-1)

// ---------- Paredes (CALIBRAR, en mm medidos por cada laser) ----------
const int SIDE_WALL_MM = 110;        // lateral por debajo de esto = hay pared
const int FRONT_WALL_MM = 140;       // frontal por debajo de esto = hay pared
const int FRONT_STOP_MM = 45;        // lectura frontal con el raton en el centro de la celda

// ---------- Arranque automatico (sin arrancador) ----------
const uint32_t POWER_SETTLE_MS = 500;   // margen para que laseres, IMU y OLED se enciendan
const int SENSOR_INIT_TRIES = 5;        // reintentos de arranque de cada sensor
const uint32_t START_DELAY_MS = 0;      // espera tras encender antes de explorar (0 = al instante)
const uint32_t RUN_PAUSE_MS = 2000;     // pausa en la salida entre carreras rapidas

// ---------- Control ----------
const uint32_t CONTROL_PERIOD_US = 5000;  // 200 Hz
const float SEARCH_SPEED = 100.0f;        // mm/s
const float SPEED_RUN_SPEED = 700.0f;     // mm/s (max teorico ~890 mm/s a 6 V)
const float ACCEL = 800.0f;               // mm/s^2
const float MIN_SPEED = 40.0f;            // mm/s al final de la frenada

// ---------- Seguidor de paredes (CALIBRAR en la pista) ----------
const bool FOLLOW_RIGHT = true;             // true = regla de la mano derecha, false = izquierda
const float WALL_FOLLOW_SPEED = SEARCH_SPEED;
const int FRONT_TURN_MM = 50;               // se para y gira con la pared delante a esta distancia
const uint32_t STALL_MS = 500;              // atascado este tiempo = pared delante (retrocede y gira)
const float BACKUP_MM = 20.0f;              // marcha atras al encontrar pared delante, antes de girar
const float BACKUP_PWM = 0.40f;
const int SIDE_CRASH_MM = 30;               // lateral por debajo de esto = rozando la pared
const float NUDGE_BACK_MM = 15.0f;          // al rozar un lateral: marcha atras...
const float NUDGE_DEG = 15.0f;              // ...y enderezarse estos grados hacia el otro lado
const int SIDE_OPEN_MM = 130;               // lateral por encima de esto (o sin lectura) = hueco
const float OPEN_CONFIRM_MM = 5.0f;         // el hueco tiene que seguir abierto este recorrido
// Geometria del raton (CALIBRAR con una regla)
const float ROBOT_WIDTH_MM = 90.0f;         // entre las caras de los dos laseres laterales
const float SIDE_SENSOR_AHEAD_MM = 45.0f;   // cuanto van los laseres laterales por delante del eje de ruedas
const float DEFAULT_CORRIDOR_MM = 160.0f;   // ancho de pasillo inicial; luego lo mide solo

const float KFF_SPEED = PWM_MAX / 800.0f; // PWM por mm/s (prealimentacion)
const float KP_SPEED = 0.0008f;
const float KI_SPEED = 0.004f;
const float KP_YAW = 0.006f;              // PWM por grado de error
const float KD_YAW = 0.0004f;             // PWM por grado/s
const float KP_WALL = 0.08f;              // grados de correccion por mm descentrado
const float MAX_WALL_CORRECTION_DEG = 4.0f;
const float LATERAL_DEADBAND_MM = 5.0f;   // descentrado menor que esto no se corrige (evita zigzag)
const float YAW_TARGET_SMOOTH = 0.1f;     // 0..1: suavizado del rumbo objetivo por ciclo
const float MAX_STEER = 0.12f;            // correccion maxima de PWM entre ruedas (evita bandazos)

const float KP_TURN = 0.006f;
const float KD_TURN = 0.0010f;
const bool TURN_IN_PLACE = true;            // true = gira sobre si mismo (pista pequena), false = pivote en una rueda
const float TURN_PWM_MAX = 0.32f;
const float TURN_PWM_MIN = 0.28f;          // minimo para girar en el sitio
const float TURN_TOLERANCE_DEG = 3.0f;
const uint32_t TURN_TIMEOUT_MS = 3000;

}  // namespace cfg

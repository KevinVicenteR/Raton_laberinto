
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
// Reglamento: pista de ~1.5 x 3 m, pasillos de ~14 cm (celda ~15.5 cm con la pared). CALIBRAR con la pista real.
const int MAZE_WIDTH = 19;           // celdas a lo ancho (3 m)
const int MAZE_HEIGHT = 10;          // celdas a lo alto (1.5 m)
const float CELL_MM = 155.0f;        // de centro a centro de celda
// Salida: en el ejemplo del reglamento, "Inicio" esta abajo a la derecha, saliendo hacia arriba (norte).
const int START_X = MAZE_WIDTH - 1;
const int START_Y = 0;
// Meta: celda (x, y) contando desde abajo a la izquierda; -1 = el centro. Se puede cambiar desde la web.
const int GOAL_X = -1;
const int GOAL_Y = -1;
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
const float PWM_MIN_MOVE = 0.25f; // minimo para que el N20 arranque (parado)
const float PWM_MIN_RUN = 0.18f;  // minimo para que siga en marcha sin pararse
const uint32_t PWM_PERIOD_US = 2000; // 500 Hz como analogWrite: a 20 kHz los N20 1:100 no tienen fuerza

// ---------- Bluetooth ----------
const char* const BLE_NAME = "Raton";          // nombre que se ve desde el movil
const uint32_t TELEMETRY_PERIOD_MS = 50;       // una linea de datos cada 50 ms (20 por segundo)

// ---------- Arranque ----------
// true: al encender (y tras parar) espera la orden ARRANCAR de la web ('s' por Bluetooth o Serie).
// false: arranca solo al encender y la mano tambien sirve para arrancar.
const bool WAIT_FOR_WEB_START = true;

// ---------- Eleccion de modo con la mano ----------
// Al encender, cada pasada de la mano delante del laser frontal cambia de modo:
// 1 = seguir pared derecha, 2 = seguir pared izquierda, 3 = resolver laberinto (flood fill),
// 4 = ruta de la competencia (laberinto del reglamento en competition_maze.h), 5 = espacio libre.
// Sin pasadas en MODE_SELECT_MS arranca en el modo 1. 0 = sin eleccion, arranca al instante.
const uint32_t MODE_SELECT_MS = 2000;
const int HAND_NEAR_MM = 60;     // mano delante
const int HAND_FAR_MM = 150;     // mano retirada

// ---------- Giroscopio ----------
const float GYRO_BIAS_TRACK_DPS = 2.0f;    // parado y girando menos que esto: se corrige el sesgo
const float GYRO_BIAS_TRACK_GAIN = 0.01f;  // velocidad de esa correccion por muestra
const uint32_t SETTLE_MS = 40;             // pausa tras cada maniobra (corrige sesgo y estabiliza)

// ---------- Rumbo con encoders ----------
const float TRACK_WIDTH_MM = 75.0f;        // distancia entre ruedas; al arrancar se mide sola
const float ENC_YAW_WEIGHT = 0.0f;         // peso de los encoders en el rumbo (0 = solo giroscopio; si las
                                           // ruedas patinan, los encoders desvian el rumbo)

// ---------- Alcance de los laseres ----------
// Ajustables en marcha desde la web (r<mm>, g<MCPS>). El raton solo necesita ver hasta ~15 cm.
const int LASER_MAX_MM = 160;                 // mas lejos que esto = no hay nada (-1)
const float LASER_SIGNAL_LIMIT_MCPS = 0.6f;   // eco minimo aceptado (de fabrica 0.25): mas alto = ve menos

// Error fijo de cada laser (mm que se SUMAN a su lectura). CALIBRAR: con el raton a una distancia
// conocida de una pared (p. ej. 100 mm medidos con regla), offset = real - lectura de la web.
// Descartar las medidas que el propio VL53L0X marca como no validas (senal debil, sigma alta...).
const bool LASER_CHECK_STATUS = true;
const int LASER_OFFSET_IZQ = 0;
const int LASER_OFFSET_FRO = 0;
const int LASER_OFFSET_DER = 0;

// ---------- Confianza de los laseres (la usa la red para ajustar el PID y los filtros) ----------
const int LASER_STABLE_MM = 25;           // salto maximo entre lecturas para contar como estable
const float LASER_CONF_RATE = 0.05f;      // rapidez con que cambia la confianza (por lectura)
const float LASER_R_BY_CONF = 4.0f;       // Kalman: ruido de medida x (1 + esto * (1 - confianza))
const float CONF_MIN_SPEED = 0.6f;        // con confianza 0 la velocidad baja a este factor

// ---------- Filtro de Kalman de los laseres ----------
const float LASER_KF_R = 64.0f;        // ruido de medida (mm^2, ~8 mm): mas alto = suaviza mas pero tarda mas
const float LASER_KF_ACCEL = 1500.0f;  // cuanto puede cambiar la velocidad de acercamiento (mm/s^2): mas alto = reacciona antes
const float LASER_JUMP_MM = 50.0f;     // salto mayor que esto: se descarta si es suelto, se acepta si se repite
const uint8_t LASER_JUMP_CONFIRM = 2;  // lecturas seguidas para aceptar un salto
const uint8_t LASER_MAX_MISS = 2;      // lecturas invalidas seguidas para dar el lado por libre (-1)

// ---------- Paredes (CALIBRAR, en mm medidos por cada laser) ----------
// Un solo umbral lateral: distancia del costado del raton a la pared (ya convertida desde la lectura
// en diagonal, ver RobotHW::sideMm). Por debajo = pared; sin lectura o por encima = libre.
// Centrado en un pasillo de 14 cm, cada costado queda a (140 - 80) / 2 = 30 mm.
const int SIDE_WALL_MM = 80;         // costado a menos de esto de la pared = hay pared
const int FRONT_WALL_MM = 140;       // frontal por debajo de esto = hay pared
// Punto muerto: lo que marca el laser frontal con el morro del raton TOCANDO la pared (la pieza de
// delante sobresale del sensor). CALIBRAR: pegar el raton a una pared y leer "Frontal" en la web.
const int FRONT_DEADZONE_MM = 10;    // el laser frontal va en la nariz, casi a ras
const int FRONT_MARGIN_MM = 25;      // margen sobre el punto muerto: para con el frontal a 35 mm (a 30 el VL53L0X falla; simulado)
const int FRONT_STOP_MM = FRONT_DEADZONE_MM + FRONT_MARGIN_MM;  // parada ante una pared en los modos de celdas

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
const float WALL_FOLLOW_SPEED = SEARCH_SPEED;
const int FRONT_TURN_MIN_MM = FRONT_DEADZONE_MM + FRONT_MARGIN_MM;           // la parada ante una pared se calcula para girar en el centro
                                            // del cruce (pasillo/2 - FRONT_SENSOR_AHEAD_MM), nunca menos que esto
const int FRONT_MEMORY_MM = 100;            // con la pared a menos de esto, si el laser deja de medir se
                                            // estima la distancia con los encoders (no la da por desaparecida)
// Decision ponderada en los cruces (modo 5): puntuacion de cada direccion libre =
//   W_FREE * espacio (0..1) + W_NOVEL * novedad (1 = celda sin visitar) - W_TURN * giro (0, 1 o 2)
//   + B_STRAIGHT si es seguir recto. Gana la mayor. Pesos optimizados en tools/sim_extremos.py y
//   ajustables desde la web (orden n<free>,<novel>,<turn>,<bias>).
const float W_FREE = 1.0f;
const float W_NOVEL = 0.8f;
const float W_TURN = 0.3f;
const float B_STRAIGHT = 0.2f;
// Pista conocida (modo 4): tramos rectos de una vez a esta velocidad (mm/s). Si una pared no coincide
// con el mapa, pasa a buscar la meta como en una pista desconocida.
const float COMP_SPEED = 250.0f;
const float EXPLORE_SPEED = 250.0f;        // modo 3 (y respaldo del 4): tramos explorando por celdas
const float REVERSE_SHORT_MM = 25.0f;      // marcha atras de un callejon: parar antes si hay pared detras
                                           // (la cola barre 49 mm al girar y no cabria)
// Pistas desconocidas (modo 5): ademas del espacio y la novedad, la direccion hacia la meta (coseno del
// angulo, -1..1) y las veces que ya se salio por esa salida del cruce (memoria estilo Tremaux).
const float W_GOAL = 0.6f;
const float W_EXIT = 0.7f;
const float GOAL_RADIUS_MM = 70.0f;   // dentro de este radio del centro de la meta = llegado

// Puerta de confianza del mapa (modo 4): fraccion de paredes vistas que coinciden con el mapa. Si baja
// de esto tras MAP_CONF_STEPS celdas, el raton pasa a modo prudente (mas despacio).
const float MAP_CONF_MIN = 0.75f;
const int MAP_CONF_STEPS = 8;
const int LOOP_TURNS = 3;                   // modo 5: giros seguidos sin avanzar = atrapado en un cruce
const float LOOP_MIN_TRAVEL_MM = 60.0f;     // avance minimo entre giros para no contar como bucle
const float LOOP_ESCAPE_MM = 400.0f;        // tramo siguiendo la pared derecha para salir del bucle
// Giro en dos tiempos (70, avanzar, 20): solo para un raton con el eje muy atrasado. El raton real
// (8 x 8 cm, eje en el centro) barre como mucho ~57 mm al girar y cabe girando en el sitio en el
// centro de una celda de 14 cm, asi que gira en el sitio.
const bool TURN_TWO_STAGE = false;
const float TURN_SPLIT_DEG = 70.0f;
const float TURN_SPLIT_ADVANCE_MM = 75.0f;
// Al girar en dos tiempos el eje avanza ~70 mm en la direccion nueva; con el raton "centrado" en una
// celda el eje queda AXLE_BEHIND_CENTER_MM por detras del centro. El siguiente avance de celdas descuenta
// esa diferencia para no descuadrarse respecto a la rejilla.
const float AXLE_BEHIND_CENTER_MM = 0.0f;    // eje de ruedas en el centro del cuerpo
const int CELL_WALL_MM = 100;               // leyendo una celda parado: pared delante si el frontal mide menos
// Paredes laterales de una celda: se miden AL LLEGAR a ella, cuando el rayo en diagonal cae en el
// centro de su costado (el raton va SIDE_LOOK_AHEAD_MM antes del centro). Parado en el centro, el
// rayo a 45 grados apunta justo al poste de la esquina y no distingue pared de hueco.
const int CELL_SIDE_WALL_MM = 70;           // costado a menos de esto de la pared = pared en esa celda
const uint8_t MAX_NO_PROGRESS = 3;          // tramos seguidos sin avanzar antes de pararse (SIN_AVANCE)
const bool SELF_CHECK_ON_START = true;      // al encender: comprobacion corta de motores (sin giros de 90)
const int SIDE_CLOSE_MM = 40;               // lateral sin lectura tras una lectura menor que esto = demasiado
                                            // cerca (por debajo del minimo del laser), no un hueco
const float UNSTICK_BACK_MM = 25.0f;        // atascado con el frente libre: atras...
const float UNSTICK_DEG = 10.0f;            // ...y enderezarse hacia el lado con mas espacio
const uint32_t STALL_MS = 600;              // atascado este tiempo = pared delante (retrocede y gira)
const float BUMP_G = 0.45f;
const float BUMP_MIN_SPEED = 50.0f;         // solo es choque si iba al menos a esto y frena de golpe                 // pico de aceleracion horizontal (g) que se toma como choque
const uint32_t BUMP_IGNORE_MS = 250;        // al arrancar un movimiento el tiron no cuenta como choque
const float SLOW_NEAR_WALL_SPEED = 60.0f;   // con pared delante a menos de FRONT_WALL_MM, como mucho esto
const float BACKUP_MM = 5.0f;               // marcha atras al encontrar pared delante, antes de girar
const float BACKUP_PWM = 0.40f;
const int SIDE_CRASH_MM = 8;                // lateral por debajo de esto = rozando la pared
const int SIDE_CRASH_MARGIN_MM = 10;        // con los dos laterales cerca y parecidos = pasillo estrecho, no roce
const float NUDGE_BACK_MM = 15.0f;          // al rozar un lateral: marcha atras...
const float NUDGE_DEG = 15.0f;              // ...y enderezarse estos grados hacia el otro lado
const int SIDE_OPEN_MM = SIDE_WALL_MM;               // lateral por encima de esto (o sin lectura) = hueco
const float OPEN_CONFIRM_MM = 5.0f;         // el hueco tiene que seguir abierto este recorrido
// Geometria del raton (CALIBRAR con una regla). Raton de ~8 x 8 cm con el eje en el centro; los
// laseres laterales van en las esquinas biseladas del frente y miran en diagonal (45 grados).
const float BODY_LENGTH_MM = 80.0f;
const float BODY_WIDTH_MM = 80.0f;
const float ROBOT_WIDTH_MM = BODY_WIDTH_MM; // las distancias laterales se dan desde los costados
const float SIDE_SENSOR_AHEAD_MM = 30.0f;   // laseres laterales: por delante del eje...
const float SIDE_SENSOR_LATERAL_MM = 30.0f; // ...y separados 3 cm del centro hacia cada lado (medido)
const float SIDE_SENSOR_ANGLE_DEG = 45.0f;  // angulo del rayo lateral respecto a la marcha
const float SIDE_ANGLE_TOLERANCE_DEG = 5.0f;  // error de montaje medido (+-5): los umbrales valen de 40 a 50
const float FRONT_SENSOR_AHEAD_MM = 40.0f;  // cuanto va el laser frontal por delante del eje de ruedas
const float DEFAULT_CORRIDOR_MM = 140.0f;   // ancho de pasillo inicial; luego lo mide solo
// Donde toca la pared el rayo lateral (por delante del eje) con el raton centrado: ahi es donde "mira".
// Con el rayo a 45 grados (tan = 1) avanza lo mismo que se separa: 30 + (70 - 30) = 70 mm.
const float SIDE_LOOK_AHEAD_MM = SIDE_SENSOR_AHEAD_MM + (0.5f * DEFAULT_CORRIDOR_MM - SIDE_SENSOR_LATERAL_MM);

const float KFF_SPEED = (PWM_MAX - PWM_MIN_RUN) / 500.0f; // PWM por mm/s, sumado a PWM_MIN_RUN
const float SPEED_FILTER = 0.85f;         // suavizado de la velocidad medida (0..1, mas = mas suave)
const float KP_SPEED = 0.0008f;
const float KI_SPEED = 0.004f;
const float KP_YAW = 0.012f;              // PWM por grado de error
const float KD_YAW = 0.0002f;             // PWM por grado/s
const float KP_WALL = 0.15f;              // cascada: grados de rumbo por mm descentrado (modo por defecto)
// PID de direccion sobre la desviacion lateral (mm respecto al centro del pasillo):
//   giro = KP_STEER * desviacion + KD_STEER * (cambio de la desviacion en un ciclo de control)
// La derivada sale de la velocidad que estima el Kalman de cada laser (no del valor crudo, que salta).
const float KP_STEER = 0.0014f;           // PWM entre ruedas por mm de desviacion
const float KD_STEER = 1.0f;              // PWM por mm de cambio de desviacion en un ciclo (5 ms)
// Modo de direccion:
//   0 = CASCADA (por defecto): la desviacion lateral fija el rumbo objetivo (KP_WALL, escalado por la
//       confianza de los laseres) y un PID rapido sobre el giroscopio lo sigue. En tools/sim_extremos.py
//       llega en 14 de 15 casos de 14 cm.
//   1 = PID DIRECTO con KP_STEER y KD_STEER sobre la desviacion. En la simulacion con Kp 0.0014 y Kd 1.0
//       llega en 1 de 10 (el Kd amplifica el ruido de los laseres, que actualizan a 50 Hz).
// Se cambia en marcha desde la web (orden p0 / p1).
const int STEER_MODE = 0;
const float MAX_WALL_CORRECTION_DEG = 4.0f;
const float LATERAL_DEADBAND_MM = 5.0f;   // descentrado menor que esto no se corrige (evita zigzag)
const float YAW_TARGET_SMOOTH = 0.1f;     // 0..1: suavizado del rumbo objetivo por ciclo
const float MAX_STEER = 0.12f;            // correccion maxima de PWM entre ruedas (evita bandazos)

const float KP_TURN = 0.006f;
const float KD_TURN = 0.0010f;
const bool TURN_IN_PLACE = true;            // true = gira sobre si mismo (pista pequena), false = pivote en una rueda
const float TURN_PWM_MAX = 0.40f;
const float TURN_PWM_MIN = 0.28f;          // minimo para ARRANCAR a girar (parado); girando ya no se fuerza
const float TURN_TOLERANCE_DEG = 4.0f;     // giro dado por bueno
const float TURN_ACCEPT_DEG = 8.0f;        // parado y sin poder afinar: se acepta (lo corrige el rumbo de la cuadricula)
const float TURN_FAIL_DEG = 20.0f;         // error mayor al acabar el tiempo = giro fallido (para el modo)
const uint32_t TURN_TIMEOUT_MS = 3000;

}  // namespace cfg

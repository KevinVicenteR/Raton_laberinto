#pragma once
#include <stdint.h>

namespace cfg {

                                           // ruedas patinan, los encoders desvian el rumbo)

// ---------- Alcance de los laseres ----------
// Ajustables en marcha desde la web (r<mm>, g<MCPS>). El raton solo necesita ver hasta ~15 cm.
const int LASER_MAX_MM = 300;                 // mas lejos que esto = no hay nada (-1); en el laberinto de madera
                                              // (pasillo 25 cm) las paredes de la celda se leen a 100-220 mm
const float LASER_SIGNAL_LIMIT_MCPS = 0.35f;  // eco minimo aceptado (de fabrica 0.25): con 0.6 la madera a 15-20 cm
                                              // daba muchas lecturas vacias y paredes que aparecian y desaparecian

// Error fijo de cada laser (mm que se SUMAN a su lectura). CALIBRAR: con el raton a una distancia
// conocida de una pared (p. ej. 100 mm medidos con regla), offset = real - lectura de la web.
// Descartar las medidas que el propio VL53L0X marca como no validas (senal debil, sigma alta...).
const bool LASER_CHECK_STATUS = true;
const int LASER_OFFSET_IZQ = 0;
const int LASER_OFFSET_FRO = 0;
const int LASER_OFFSET_DER = 0;

// ---------- Confianza de los laseres (ajusta el PID y los filtros) ----------
const int LASER_STABLE_MM = 25;           // salto maximo entre lecturas para contar como estable
const float LASER_CONF_RATE = 0.05f;      // rapidez con que cambia la confianza (por lectura)
const float LASER_R_BY_CONF = 4.0f;       // Kalman: ruido de medida x (1 + esto * (1 - confianza))

// ---------- Filtro de Kalman de los laseres ----------
const float LASER_KF_R = 64.0f;        // ruido de medida (mm^2, ~8 mm): mas alto = suaviza mas pero tarda mas
const float LASER_KF_ACCEL = 1500.0f;  // cuanto puede cambiar la velocidad de acercamiento (mm/s^2): mas alto = reacciona antes
const float LASER_JUMP_MM = 50.0f;     // salto mayor que esto: se descarta si es suelto, se acepta si se repite
const uint8_t LASER_JUMP_CONFIRM = 2;  // lecturas seguidas para aceptar un salto
const uint8_t LASER_MAX_MISS = 2;      // lecturas invalidas seguidas para dar el lado por libre (-1)

// ---------- Paredes (CALIBRAR, en mm medidos por cada laser) ----------
// Un solo umbral lateral: distancia del costado del raton a la pared (ya convertida desde la lectura
// en diagonal, ver TrackGeometry::sideFromRay). Por debajo = pared; sin lectura o por encima = libre.
// Centrado en un pasillo de 14 cm, cada costado queda a (140 - 80) / 2 = 30 mm.
const int SIDE_WALL_MM = 80;         // costado a menos de esto de la pared = hay pared
const int FRONT_WALL_MM = 140;       // frontal por debajo de esto = hay pared
// Punto muerto: lo que marca el laser frontal con el morro del raton TOCANDO la pared (la pieza de
// delante sobresale del sensor). CALIBRAR: pegar el raton a una pared y leer "Frontal" en la web.
const int FRONT_DEADZONE_MM = 50;    // MEDIDO en la pista: con el morro tocando la pared el frontal marca
                                     // 44-50 mm (el sensor va metido en su hueco y tiene desfase)
const int FRONT_MARGIN_MM = 20;      // margen sobre el punto muerto: nunca parar con el morro a menos de esto
const int FRONT_MEMORY_MM = 100;            // con la pared a menos de esto, si el laser deja de medir se
const float SIDE_WALL_FACTOR = 0.42f;
const float FRONT_WALL_FACTOR = 0.28f;
const float WALL_MM = 12.0f;                 // grosor de pared: celda = pasillo medido + esto
const int SIDE_CLOSE_MM = 40;               // lateral sin lectura tras una lectura menor que esto = demasiado
const float OPEN_CONFIRM_MM = 5.0f;         // el hueco tiene que seguir abierto este recorrido
// Geometria del raton (CALIBRAR con una regla). Raton de ~8 x 8 cm con el eje en el centro; los
// laseres laterales van en las esquinas biseladas del frente y miran en diagonal (45 grados).
const float BODY_LENGTH_MM = 80.0f;
const float BODY_WIDTH_MM = 80.0f;
const float ROBOT_WIDTH_MM = BODY_WIDTH_MM; // las distancias laterales se dan desde los costados
const float SIDE_SENSOR_AHEAD_MM = 30.0f;   // laseres laterales: por delante del eje...
const float SIDE_SENSOR_LATERAL_MM = 30.0f; // ...y separados 3 cm del centro hacia cada lado (medido)
const float SIDE_SENSOR_ANGLE_DEG = 31.0f;  // angulo del rayo lateral respecto a la marcha. Nominal 45, pero
                                            // AJUSTADO con 13 lecturas reales (registros del 16/10): la mediana
                                            // da 31 grados (cuartiles 23-38). Recalibrable con la orden K<mm>.
const float FRONT_SENSOR_AHEAD_MM = 40.0f;  // cuanto va el laser frontal por delante del eje de ruedas
const float DEFAULT_CORRIDOR_MM = 140.0f;   // ancho de pasillo inicial; luego lo mide solo
const int FRONT_STABLE_SAMPLES = 5;
const int CALIBRATION_SAMPLES = 7;

const int MAX_CELL_SIDES = 20;
constexpr float SIDE_WINDOW_MM = 12.0f;

}  // namespace cfg

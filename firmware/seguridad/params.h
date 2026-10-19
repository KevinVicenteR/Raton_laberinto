#pragma once
#include <stdint.h>

namespace cfg {


// ---------- Control ----------
const uint32_t CONTROL_PERIOD_US = 5000;  // 200 Hz
const uint8_t MAX_NO_PROGRESS = 3;          // tramos seguidos sin avanzar antes de pararse (SIN_AVANCE)
                                            // cerca (por debajo del minimo del laser), no un hueco
const float UNSTICK_BACK_MM = 25.0f;        // atascado con el frente libre: atras...
const float UNSTICK_DEG = 10.0f;            // ...y enderezarse hacia el lado con mas espacio
const uint32_t STALL_MS = 600;              // atascado este tiempo = pared delante (retrocede y gira)
const float BUMP_G = 0.45f;                  // pico de aceleracion horizontal (g) que se toma como choque
const float BUMP_MIN_SPEED = 50.0f;         // solo es choque si iba al menos a esto y frena de golpe
const uint32_t BUMP_IGNORE_MS = 250;        // al arrancar un movimiento el tiron no cuenta como choque
const float BACKUP_MM = 5.0f;               // marcha atras al encontrar pared delante, antes de girar
const float BACKUP_PWM = 0.40f;
const int SIDE_CRASH_MM = 8;                // lateral por debajo de esto = rozando la pared
const float SKEW_LIMIT_DEG = 10.0f;         // tramo de celdas: mas torcido que esto respecto a la cuadricula = enderezar
const uint32_t RUB_MS = 60;                // roce o torcido durante esto antes de corregir
const float RUB_BACK_MM = 15.0f;            // al corregir un roce: marcha atras corta antes de enderezarse
const int MAX_RUBS = 2;                     // roces en un mismo tramo antes de darse por PEGADO
const float PUSH_CHECK_MM = 40.0f;          // empujando: avanza esto por encoders sin que el frontal se acerque
const int PUSH_MIN_CLOSE_MM = 10;           // ...al menos esto (si no, esta empujando una pared o patina)
const int SIDE_CRASH_MARGIN_MM = 10;        // con los dos laterales cerca y parecidos = pasillo estrecho, no roce
const float NUDGE_BACK_MM = 15.0f;          // al rozar un lateral: marcha atras...
const float NUDGE_DEG = 15.0f;              // ...y enderezarse estos grados hacia el otro lado

}  // namespace cfg

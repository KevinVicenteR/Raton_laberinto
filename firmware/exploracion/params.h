#pragma once
#include <stdint.h>

namespace cfg {

                                            // estima la distancia con los encoders (no la da por desaparecida)
// Decision ponderada en los cruces (modo 5): puntuacion de cada direccion libre =
//   W_FREE * espacio (0..1) + W_NOVEL * novedad (1 = celda sin visitar) - W_TURN * giro (0, 1 o 2)
//   + B_STRAIGHT si es seguir recto. Gana la mayor. Pesos optimizados con el simulador y
//   ajustables desde la web (orden n<free>,<novel>,<turn>,<bias>).
const float W_FREE = 1.0f;
const float W_NOVEL = 0.8f;
const float W_TURN = 0.3f;
const float B_STRAIGHT = 0.2f;
// Pistas desconocidas (modo 5): ademas del espacio y la novedad, la direccion hacia la meta (coseno del
// angulo, -1..1) y las veces que ya se salio por esa salida del cruce (memoria estilo Tremaux).
const float W_GOAL = 0.6f;
const float W_EXIT = 0.7f;
const float GOAL_RADIUS_MM = 70.0f;   // dentro de este radio del centro de la meta = llegado

const int LOOP_TURNS = 3;                   // modo 5: giros seguidos sin avanzar = atrapado en un cruce
const float LOOP_MIN_TRAVEL_MM = 60.0f;     // avance minimo entre giros para no contar como bucle
const float LOOP_ESCAPE_MM = 400.0f;        // tramo siguiendo la pared derecha para salir del bucle
// Paredes laterales de una celda: se miden AL LLEGAR a ella, cuando el rayo en diagonal cae en el
// centro de su costado (TrackGeometry::sideLookAhead antes del centro). Parado en el centro, el rayo
// apunta justo al poste de la esquina y no distingue pared de hueco. El raton (8 x 8 cm, eje en el
// centro) gira en el sitio en el centro de la celda.
// Factores de los umbrales de pared (ENTRENADOS con CMA-ES y curriculo en el simulador, ver
// docs/entrenamiento.md): pared lateral si el costado esta a menos de SIDE_WALL_FACTOR x pasillo; pared
// delante (raton parado) si el frontal lee menos que la parada + FRONT_WALL_FACTOR x celda.
// Busqueda tras la primera llegada (Adachi 1 + regreso explorando; ganadora del reentrenamiento con
// reglas, hojas de datos y estrategias de competidores, docs/entrenamiento.md).
const bool ADACHI_SEARCH = true;
const int SEARCH_MAX_TARGETS = 12;  // celdas sin ver de la mejor ruta que va a leer como maximo

}  // namespace cfg

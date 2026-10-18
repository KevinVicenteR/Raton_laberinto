#pragma once
#include <stdint.h>

namespace cfg {


// ---------- Arranque ----------
// true: al encender (y tras parar) espera la orden ARRANCAR de la web ('s' por Bluetooth o Serie).
// false: arranca solo al encender y la mano tambien sirve para arrancar.
const bool WAIT_FOR_WEB_START = true;

// ---------- Eleccion de modo con la mano ----------
// Al encender, cada pasada de la mano delante del laser frontal cambia de modo:
// 1 = seguir pared derecha, 2 = seguir pared izquierda, 3 = resolver laberinto (flood fill),
// 4 = ruta de la competencia, 5 = espacio libre, 6 = aprender pista y guardar en flash.
// Sin pasadas en MODE_SELECT_MS arranca en el modo 1. 0 = sin eleccion, arranca al instante.
const uint32_t MODE_SELECT_MS = 2000;
const int HAND_NEAR_MM = 60;     // mano delante
const int HAND_FAR_MM = 150;     // mano retirada
const uint32_t START_DELAY_MS = 0;      // espera tras encender antes de explorar (0 = al instante)
const bool SELF_CHECK_ON_START = false;     // al encender no gira (motores y encoders ya comprobados con la prueba T)

}  // namespace cfg

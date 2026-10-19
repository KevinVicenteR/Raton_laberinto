// Firmware del raton de laberinto (Arduino Nano 33 BLE Sense Rev2). Toda la logica esta en App y en el
// nucleo portable lib/raton_core (el mismo codigo que corre el simulador).
#include <Arduino.h>

#include "app.h"

namespace {
App app;
}

void setup() { app.setup(); }

void loop() { app.loop(); }

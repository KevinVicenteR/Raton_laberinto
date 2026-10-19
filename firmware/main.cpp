// Firmware del raton de laberinto (Arduino Nano 33 BLE Sense Rev2). Toda la logica vive en App
// (firmware/app) y en el nucleo portable (lib/raton_core); aqui solo se engancha a Arduino.
//
// Uso: pon el raton en un pasillo de la pista y enciendelo. Telemetria, mapa y ordenes por Bluetooth
// (web/) o Monitor Serie: x = parar, s = arrancar, m1..m5 = modo, ? = ver valores.
#include <Arduino.h>

#include "modos/app.h"

namespace {
App app;
}  // namespace

void setup() { app.setup(); }

void loop() { app.loop(); }

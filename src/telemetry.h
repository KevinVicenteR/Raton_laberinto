// Telemetria y ordenes por Bluetooth (servicio UART de Nordic). Se lee y se escribe desde el movil
// con apps como nRF Connect, Bluefruit Connect o Serial Bluetooth Terminal. Nombre: cfg::BLE_NAME.
#pragma once

#include <Arduino.h>

namespace telemetry {

bool begin();
// Atiende el Bluetooth y envia como mucho un trozo pendiente. Llamar seguido.
void poll();
// Pone una linea en la cola de envio (no espera; si la cola esta llena se descarta). No hace nada
// si no hay nadie conectado.
void println(const char* line);
// Whole lines dropped due to queue pressure; diagnostic capture must report loss.
uint32_t droppedLines();
// Envia lo que quede en la cola durante como mucho ms milisegundos (antes de reiniciar).
void flush(uint32_t ms);
// Siguiente caracter recibido, o -1.
int read();

}  // namespace telemetry

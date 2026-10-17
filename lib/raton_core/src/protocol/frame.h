// Protocolo de lineas entre la placa y la web: "$<seq>,<datos>*<CRC8 en hex>".
//  - seq: numero de linea (0..65535, vuelve a 0) para que el receptor cuente las perdidas.
//  - CRC8 (polinomio 0x07, como SMBus) de "<seq>,<datos>": una linea corrupta se descarta.
// Las ordenes de la web llevan el mismo formato y la placa contesta "ACK <seq>" o "NAK <seq>".
// Una linea sin '$' se sigue aceptando (Monitor Serie, apps BLE sencillas).
#pragma once
#include <stddef.h>
#include <stdint.h>

namespace raton {

uint8_t crc8(const char* data, size_t len);

// Escribe "$seq,payload*HH" en out (sin salto de linea). Devuelve la longitud, o 0 si no cabe.
size_t frameLine(uint16_t seq, const char* payload, char* out, size_t outSize);

// Comprueba una linea "$seq,payload*HH". Si es valida, copia el payload en out y pone seq.
enum class FrameResult { Plain, Ok, BadCrc, Malformed };
FrameResult parseFrame(const char* line, uint16_t& seq, char* out, size_t outSize);

}  // namespace raton

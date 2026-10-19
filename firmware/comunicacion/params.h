#pragma once
#include <stdint.h>

namespace cfg {


// ---------- Bluetooth ----------
const char* const BLE_NAME = "Raton";          // nombre que se ve desde el movil
const uint32_t TELEMETRY_PERIOD_MS = 100;      // una linea de estado cada 100 ms (10 por segundo): el
                                               // Bluetooth no daba abasto a 20/s y se perdian las importantes
const uint32_t OBS_PERIOD_MS = 500;            // linea de observacion detallada (OBS) cada 500 ms

}  // namespace cfg

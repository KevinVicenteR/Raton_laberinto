# comunicacion

Recibe órdenes y emite telemetría con tramas y colas.

Entradas/salidas: Bluetooth/Serie → peticiones, ACK/NAK y líneas de estado.
Parámetros: Nombre BLE y periodos de telemetría y observación.
Los valores de placa están en params.h; config.h los incluye sin duplicarlos.

Pruebas: `scripts/run.sh pruebas`.
Las pruebas se descubren desde la entrada común; no hay comandos por archivo.

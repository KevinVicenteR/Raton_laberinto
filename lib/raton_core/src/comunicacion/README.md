# comunicacion

Recibe órdenes y emite telemetría con tramas y colas.

Entradas/salidas: Bluetooth/Serie → peticiones, ACK/NAK y líneas de estado.
Parámetros: Nombre BLE y periodos de telemetría y observación.
Núcleo sin Arduino, compartido por CMake y PlatformIO.

Pruebas: `ctest --test-dir build --output-on-failure`.
Las pruebas se descubren desde la entrada común; no hay comandos por archivo.

# comunicacion

Recibe órdenes y emite telemetría con tramas y colas.

Entradas/salidas: Bluetooth/Serie → peticiones, ACK/NAK y líneas de estado.
Parámetros: Nombre BLE y periodos de telemetría y observación.
Consume configuración del firmware o los valores históricos del escenario.

Pruebas: `.venv/bin/python -m pytest tests/python -q`.
Las pruebas se descubren desde la entrada común; no hay comandos por archivo.

# giros

Ejecuta giros de esquina y rotaciones con perfil angular.

Entradas/salidas: Ángulo solicitado y giroscopio → giro completado o fallo.
Parámetros: Aceleración angular, tolerancias, tiempo límite y reintentos.
Consume configuración del firmware o los valores históricos del escenario.

Pruebas: `.venv/bin/python -m pytest tests/python -q`.
Las pruebas se descubren desde la entrada común; no hay comandos por archivo.

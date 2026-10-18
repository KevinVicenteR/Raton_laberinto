# giros

Ejecuta giros de esquina y rotaciones con perfil angular.

Entradas/salidas: Ángulo solicitado y giroscopio → giro completado o fallo.
Parámetros: Aceleración angular, tolerancias, tiempo límite y reintentos.
Los valores de placa están en params.h; config.h los incluye sin duplicarlos.

Pruebas: `scripts/run.sh pruebas`.
Las pruebas se descubren desde la entrada común; no hay comandos por archivo.

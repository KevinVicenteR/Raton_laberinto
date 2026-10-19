# prueba_hardware

Comprueba sensores y motores con un firmware de diagnóstico separado.

Entradas/salidas: Órdenes por Serie y sensores → diagnóstico y maniobras de prueba.
Parámetros: Usa los parámetros de hardware y pines de config.h.
Comparte los parámetros de hardware; no requiere cargar una placa para compilar.

Pruebas: `scripts/run.sh pruebas`.
Las pruebas se descubren desde la entrada común; no hay comandos por archivo.

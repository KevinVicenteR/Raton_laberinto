# paredes

Convierte rayos diagonales, detecta paredes y calibra el ángulo lateral K.

Entradas/salidas: Lecturas de láser y geometría → distancias, paredes y confianza.
Parámetros: Umbrales, filtros, memoria frontal y muestras de calibración.
Los valores de placa están en params.h; config.h los incluye sin duplicarlos.

Pruebas: `scripts/run.sh pruebas`.
Las pruebas se descubren desde la entrada común; no hay comandos por archivo.

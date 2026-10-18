# exploracion

Explora por celdas, busca con Adachi y ejecuta la carrera rápida.

Entradas/salidas: Mapa y puertos del robot → decisiones y misión terminada.
Parámetros: ADACHI_SEARCH, SEARCH_MAX_TARGETS y pesos de decisión.
Los valores de placa están en params.h; config.h los incluye sin duplicarlos.

Pruebas: `scripts/run.sh pruebas`.
Las pruebas se descubren desde la entrada común; no hay comandos por archivo.

# exploracion

Explora por celdas, busca con Adachi y ejecuta la carrera rápida.

Entradas/salidas: Mapa y puertos del robot → decisiones y misión terminada.
Parámetros: ADACHI_SEARCH, SEARCH_MAX_TARGETS y pesos de decisión.
Núcleo sin Arduino, compartido por CMake y PlatformIO.

Pruebas: `ctest --test-dir build --output-on-failure`.
Las pruebas se descubren desde la entrada común; no hay comandos por archivo.

# exploracion

Explora por celdas, busca con Adachi y ejecuta la carrera rápida.

Entradas/salidas: Mapa y puertos del robot → decisiones y misión terminada.
Parámetros: ADACHI_SEARCH, SEARCH_MAX_TARGETS y pesos de decisión.
Consume configuración del firmware o los valores históricos del escenario.

Pruebas: `.venv/bin/python -m pytest tests/python -q`.
Las pruebas se descubren desde la entrada común; no hay comandos por archivo.

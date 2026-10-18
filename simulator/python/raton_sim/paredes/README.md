# paredes

Convierte rayos diagonales, detecta paredes y calibra el ángulo lateral K.

Entradas/salidas: Lecturas de láser y geometría → distancias, paredes y confianza.
Parámetros: Umbrales, filtros, memoria frontal y muestras de calibración.
Consume configuración del firmware o los valores históricos del escenario.

Pruebas: `.venv/bin/python -m pytest tests/python -q`.
Las pruebas se descubren desde la entrada común; no hay comandos por archivo.

# velocidad

Controla la velocidad, la aceleración y la consigna adaptativa.

Entradas/salidas: Consigna, encoders y confianza → PWM base y límite de velocidad.
Parámetros: ADAPT_*, ACCEL, EXPLORE_SPEED, COMP_SPEED y ganancias PI.
Consume configuración del firmware o los valores históricos del escenario.

Pruebas: `.venv/bin/python -m pytest tests/python -q`.
Las pruebas se descubren desde la entrada común; no hay comandos por archivo.

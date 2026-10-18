# velocidad

Controla la velocidad, la aceleración y la consigna adaptativa.

Entradas/salidas: Consigna, encoders y confianza → PWM base y límite de velocidad.
Parámetros: ADAPT_*, ACCEL, EXPLORE_SPEED, COMP_SPEED y ganancias PI.
Los valores de placa están en params.h; config.h los incluye sin duplicarlos.

Pruebas: `scripts/run.sh pruebas`.
Las pruebas se descubren desde la entrada común; no hay comandos por archivo.

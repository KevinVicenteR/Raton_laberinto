# velocidad

Controla la velocidad, la aceleración y la consigna adaptativa.

Entradas/salidas: Consigna, encoders y confianza → PWM base y límite de velocidad.
Parámetros: ADAPT_*, ACCEL, EXPLORE_SPEED, COMP_SPEED y ganancias PI.
Núcleo sin Arduino, compartido por CMake y PlatformIO.

Pruebas: `ctest --test-dir build --output-on-failure`.
Las pruebas se descubren desde la entrada común; no hay comandos por archivo.

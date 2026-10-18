# seguridad

Coordina el movimiento y aplica las recuperaciones por roce, atasco y choque.

Entradas/salidas: Observaciones y avance → frenado, recuperación o parada.
Parámetros: BUMP_*, RUB_*, STALL_MS y distancias de recuperación.
Núcleo sin Arduino, compartido por CMake y PlatformIO.

Pruebas: `ctest --test-dir build --output-on-failure`.
Las pruebas se descubren desde la entrada común; no hay comandos por archivo.

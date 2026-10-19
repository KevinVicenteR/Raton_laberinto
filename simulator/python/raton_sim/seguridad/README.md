# seguridad

Coordina el movimiento y aplica las recuperaciones por roce, atasco y choque.

Entradas/salidas: Observaciones y avance → frenado, recuperación o parada.
Parámetros: BUMP_*, RUB_*, STALL_MS y distancias de recuperación.
Consume configuración del firmware o los valores históricos del escenario.

Pruebas: `.venv/bin/python -m pytest tests/python -q`.
Las pruebas se descubren desde la entrada común; no hay comandos por archivo.

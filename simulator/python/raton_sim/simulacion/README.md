# simulacion

Compone el simulador físico y los comportamientos por función.

Entradas/salidas: Escenario y semilla → trayectoria, contactos, llegada y fallo.
Parámetros: Modelos de cuerpo, sensores y fotografía de configuración del escenario.
Consume configuración del firmware o los valores históricos del escenario.

Pruebas: `.venv/bin/python -m pytest tests/python -q`.
Las pruebas se descubren desde la entrada común; no hay comandos por archivo.

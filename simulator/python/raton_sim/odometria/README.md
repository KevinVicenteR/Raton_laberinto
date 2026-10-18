# odometria

Integra encoders y rumbo y conserva visitas y salidas del modo 5.

Entradas/salidas: Pulsos y giroscopio → posición, rumbo y memoria de visitas.
Parámetros: Ruedas, cuentas por vuelta, ancho del eje y tamaño de rejilla.
Consume configuración del firmware o los valores históricos del escenario.

Pruebas: `.venv/bin/python -m pytest tests/python -q`.
Las pruebas se descubren desde la entrada común; no hay comandos por archivo.

# centrado

Mantiene el centro del pasillo y el rumbo con amortiguación.

Entradas/salidas: Distancias laterales, confianza y rumbo → corrección entre ruedas.
Parámetros: KP_WALL, KP_YAW, KD, banda muerta y límites.
Consume configuración del firmware o los valores históricos del escenario.

Pruebas: `.venv/bin/python -m pytest tests/python -q`.
Las pruebas se descubren desde la entrada común; no hay comandos por archivo.

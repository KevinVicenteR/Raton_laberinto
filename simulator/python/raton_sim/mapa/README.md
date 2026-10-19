# mapa

Mantiene paredes por celda, inundación, poda y persistencia.

Entradas/salidas: Celdas y paredes observadas → mapa, distancias y datos serializados.
Parámetros: Dimensiones, salida, meta y celda; geometría medida desde paredes.
Consume configuración del firmware o los valores históricos del escenario.

Pruebas: `.venv/bin/python -m pytest tests/python -q`.
Las pruebas se descubren desde la entrada común; no hay comandos por archivo.

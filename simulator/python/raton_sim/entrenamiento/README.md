# entrenamiento

Evalúa controladores y entrena con escenarios reproducibles.

Entradas/salidas: Escenarios, semillas y candidatos → métricas y selección de parámetros.
Parámetros: Dominios, población y presupuesto definidos por las órdenes existentes.
Consume configuración del firmware o los valores históricos del escenario.

Pruebas: `.venv/bin/python -m pytest tests/python -q`.
Las pruebas se descubren desde la entrada común; no hay comandos por archivo.

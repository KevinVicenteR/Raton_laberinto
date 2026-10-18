# publicacion

Construye catálogos y reproducciones para la web.

Entradas/salidas: Ejecuciones evaluadas → JSON y sesiones de reproducción.
Parámetros: Matriz de casos, controlador y límite de muestras.
Consume configuración del firmware o los valores históricos del escenario.

Pruebas: `.venv/bin/python -m pytest tests/python -q`.
Las pruebas se descubren desde la entrada común; no hay comandos por archivo.

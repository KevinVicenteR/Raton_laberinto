# paredes

Convierte rayos diagonales, detecta paredes y calibra el ángulo lateral K.

Entradas/salidas: Lecturas de láser y geometría → distancias, paredes y confianza.
Parámetros: Umbrales, filtros, memoria frontal y muestras de calibración.
Núcleo sin Arduino, compartido por CMake y PlatformIO.

Pruebas: `ctest --test-dir build --output-on-failure`.
Las pruebas se descubren desde la entrada común; no hay comandos por archivo.

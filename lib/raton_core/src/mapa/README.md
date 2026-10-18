# mapa

Mantiene paredes por celda, inundación, poda y persistencia.

Entradas/salidas: Celdas y paredes observadas → mapa, distancias y datos serializados.
Parámetros: Dimensiones, salida, meta y celda; geometría medida desde paredes.
Núcleo sin Arduino, compartido por CMake y PlatformIO.

Pruebas: `ctest --test-dir build --output-on-failure`.
Las pruebas se descubren desde la entrada común; no hay comandos por archivo.

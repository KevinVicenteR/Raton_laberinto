# odometria

Integra encoders y rumbo y conserva visitas y salidas del modo 5.

Entradas/salidas: Pulsos y giroscopio → posición, rumbo y memoria de visitas.
Parámetros: Ruedas, cuentas por vuelta, ancho del eje y tamaño de rejilla.
Núcleo sin Arduino, compartido por CMake y PlatformIO.

Pruebas: `ctest --test-dir build --output-on-failure`.
Las pruebas se descubren desde la entrada común; no hay comandos por archivo.

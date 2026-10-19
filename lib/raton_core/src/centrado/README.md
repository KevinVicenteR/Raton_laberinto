# centrado

Control lateral y de rumbo con banda muerta y amortiguación.

Entradas: observaciones y parámetros del adaptador de placa.
Salidas: corrección de dirección o tasa angular solicitada.
Parámetros: recibidos sin duplicar los valores de firmware/centrado/params.h.
No incluye Arduino ni accede a sensores o actuadores.
Pruebas: `ctest --test-dir build --output-on-failure`.
Los checks están en tests/cpp/centrado/ y usan los parámetros reales.

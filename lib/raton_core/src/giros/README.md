# giros

Perfil angular con frenado cerca del objetivo.

Entradas: observaciones y parámetros del adaptador de placa.
Salidas: corrección de dirección o tasa angular solicitada.
Parámetros: recibidos sin duplicar los valores de firmware/giros/params.h.
No incluye Arduino ni accede a sensores o actuadores.
Pruebas: `ctest --test-dir build --output-on-failure`.
Los checks están en tests/cpp/giros/ y usan los parámetros reales.

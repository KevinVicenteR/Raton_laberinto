# mapa

Mantiene paredes por celda, inundación, poda y persistencia.

Entradas/salidas: Celdas y paredes observadas → mapa, distancias y datos serializados.
Parámetros: Dimensiones, salida, meta y celda; geometría medida desde paredes.
Los valores de placa están en params.h; config.h los incluye sin duplicarlos.

Pruebas: `scripts/run.sh pruebas`.
Las pruebas se descubren desde la entrada común; no hay comandos por archivo.

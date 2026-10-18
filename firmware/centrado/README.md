# centrado

Mantiene el centro del pasillo y el rumbo con amortiguación.

Entradas/salidas: Distancias laterales, confianza y rumbo → corrección entre ruedas.
Parámetros: KP_WALL, KP_YAW, KD, banda muerta y límites.
Los valores de placa están en params.h; config.h los incluye sin duplicarlos.

Pruebas: `scripts/run.sh pruebas`.
Las pruebas se descubren desde la entrada común; no hay comandos por archivo.

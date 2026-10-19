# odometria

Integra encoders y rumbo y conserva visitas y salidas del modo 5.

Entradas/salidas: Pulsos y giroscopio → posición, rumbo y memoria de visitas.
Parámetros: Ruedas, cuentas por vuelta, ancho del eje y tamaño de rejilla.
Los valores de placa están en params.h; config.h los incluye sin duplicarlos.

Pruebas: `scripts/run.sh pruebas`.
Las pruebas se descubren desde la entrada común; no hay comandos por archivo.

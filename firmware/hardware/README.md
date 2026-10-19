# hardware

Conduce motores, encoders, láseres, giroscopio y pantalla.

Entradas/salidas: Pines, I2C y PWM → medidas físicas y actuadores.
Parámetros: Potencia PWM, inversión de motores y ajuste del giroscopio.
Los valores de placa están en params.h; config.h los incluye sin duplicarlos.

Pruebas: `scripts/run.sh pruebas`.
Las pruebas se descubren desde la entrada común; no hay comandos por archivo.

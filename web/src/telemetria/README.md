# telemetria

Interpreta, conserva y presenta las muestras y registros del robot.

Entradas/salidas: Líneas del robot → muestras, estado y grabaciones.
Parámetros: Contrato de muestras y estado inicial conservados.
Componentes, lógica y pruebas viven juntos; los hooks están en su función.

Pruebas: `cd web && npm test`.
Las pruebas se descubren desde la entrada común; no hay comandos por archivo.

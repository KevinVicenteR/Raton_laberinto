# conexion

Gestiona Web Bluetooth, tramas y cola de órdenes.

Entradas/salidas: Dispositivo y órdenes → conexión, mensajes y confirmaciones.
Parámetros: Protocolo CRC y plazos de la cola conservados.
Componentes, lógica y pruebas viven juntos; los hooks están en su función.
La conexión GATT reintenta fallos iniciales y se recupera automáticamente de desconexiones
inesperadas; una desconexión manual no inicia la reconexión.

Pruebas: `cd web && npm test`.
Las pruebas se descubren desde la entrada común; no hay comandos por archivo.

# mapa

Muestra mapas de celdas y ocupación en SVG y Three.js.

Entradas/salidas: Muestras, pose y paredes → mapas y escena 3D.
Parámetros: Geometría de pista y constantes visuales existentes.
Componentes, lógica y pruebas viven juntos; los hooks están en su función.

Pruebas: `cd web && npm test`.
Las pruebas se descubren desde la entrada común; no hay comandos por archivo.

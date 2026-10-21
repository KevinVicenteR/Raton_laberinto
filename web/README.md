# Web del micromouse

Aplicación independiente Vite + React 18 + TypeScript estricto. No modifica ni compila firmware.

```sh
npm install
npm run dev
```

Abre http://127.0.0.1:5173. Web Bluetooth UART Nordic requiere Chrome/Edge compatible y contexto seguro (localhost o HTTPS). Selecciona modo y pulsa ARRANCAR. PARAR envía `x` inmediatamente. Para la prueba de ruedas levanta el robot; para calibrar colócalo centrado en un pasillo.

## Capas

- `src/conexion`: BLE, protocolo, cola de órdenes, hooks y pruebas.
- `src/telemetria`: muestras, reducer, registros, sensores y componentes.
- `src/mapa`: ocupación, inundación, mapas SVG/3D y pruebas.
- `src/simulacion`: catálogo, sesiones, reproducción, hooks y pruebas.
- `src/parametros`, `src/seguridad`, `src/modos`: ajustes, diagnóstico y navegación.

Telemetría permite registrar TXT con segundos desde el inicio, importar TXT/JSON, pausar, buscar y cambiar velocidad. La reproducción reconstruye el estado desde el inicio y bloquea órdenes y supervisor. Three.js y sus controles se empaquetan desde npm.

Simulación carga `/sim/dashboard.json`; el simulador publica el catálogo y las sesiones en
`web/public/sim/`. Estos archivos se versionan para permitir la reproducción sin ejecutar el simulador.

## Cinco sensores y calibración

El panel muestra DI (diagonal izquierdo), F, DD (diagonal derecho), LI (lateral izquierdo a 90°) y LD
(lateral derecho a 90°), con tarjetas, radar y gráfico. Reconoce `NAV`, `SENS`, `LATERALES` y
`PRUEBA_MM`, también al importar registros TXT. Los mensajes de sensores no inventan posiciones del robot.
`-1` aparece como «Sin lectura»; tras 1,5 s sin recibir un canal aparece «Sin datos recientes». Esta edad
es de recepción de telemetría, no el indicador `fresh` interno del láser.

«Ver sensores» abre el monitor sin movimiento; «Probar sensores (8 s)» ejecuta el diagnóstico acotado.
Las lecturas laterales se muestran incluso con `sideEnabled=0`. Activar el control requiere medir su
geometría y offsets: XSHUT izquierdo A6, derecho A3; SDA A4 y SCL A5. El radar es un esquema.

Parámetros envía únicamente campos editados y distingue valores leídos de la placa de referencias
locales. «Leer valores de la placa» solicita `params`; enviar cambios y guardar son acciones separadas.

El mapa de competencia comparte `mazes/reglamento_2026.txt` con el simulador. La vista 3D respeta
coordenadas absolutas de NAV v2, paso aprendido y grosor de pared; conserva el marco relativo de registros
antiguos. La transcripción del mapa de ejemplo debe contrastarse con la pista montada de la competencia.

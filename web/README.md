# Web del micromouse

Aplicación independiente Vite + React 18 + TypeScript estricto. No modifica ni compila firmware.

```sh
npm install
npm run dev
```

Abre http://127.0.0.1:5173. Web Bluetooth UART Nordic requiere Chrome/Edge compatible y contexto seguro (localhost o HTTPS). Selecciona modo y pulsa ARRANCAR. PARAR envía `x` inmediatamente. Para la prueba de ruedas levanta el robot; para calibrar colócalo centrado en un pasillo.

## Capas

- `src/domain`: protocolo/CRC, validación de sesiones, pose, flood fill, ocupación y supervisor puros.
- `src/infrastructure`: BLE, cola serial ACK/NAK (800 ms, envío inicial + tres reintentos, misma secuencia), registros, catálogo y sesión de reproducción.
- `src/application`: reducer y hooks de telemetría, órdenes y reproducción.
- `src/ui`: componentes React, mapas SVG/Three.js, controles y páginas.

Telemetría permite registrar TXT con segundos desde el inicio, importar TXT/JSON, pausar, buscar y cambiar velocidad. La reproducción reconstruye el estado desde el inicio y bloquea órdenes y supervisor. Three.js y sus controles se empaquetan desde npm.

Simulación carga `/sim/dashboard.json`; el simulador publica el catálogo y las sesiones en
`web/public/sim/`. Estos archivos se versionan para permitir la reproducción sin ejecutar el simulador.

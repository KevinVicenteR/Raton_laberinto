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

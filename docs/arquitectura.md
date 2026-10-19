# Arquitectura por funcionalidad

La unidad de organización es lo que hace el ratón. Cada función reúne su lógica, sus parámetros y
un README corto; las pruebas se agrupan con el mismo nombre. La lógica portable sigue sin Arduino.

```text
firmware/
├── config.h                     pines y agregador de params.h
├── main.cpp                     entrada de PlatformIO
├── paredes/                     detección, memoria frontal, calibración K y lecturas parado
├── centrado/                    control lateral y rumbo
├── velocidad/                   PI de velocidad y parámetros de perfiles y adaptación
├── giros/                       rotate, turnCorner y perfil angular
├── seguridad/                   RobotHW, ciclo, tramos y recuperaciones
├── odometria/                   encoders, rumbo, posición, visitas y salidas
├── mapa/                        pista de competencia y persistencia flash
├── exploracion/                 MazeMission
├── modos/                       App, modo_1.cpp … modo_6.cpp, seguidores y espacio libre
├── comunicacion/                Bluetooth, órdenes, telemetría y ajustes remotos
├── hardware/                    motores, láseres, IMU, encoders y pantalla
└── prueba_hardware/             diagnóstico independiente
lib/raton_core/src/
├── paredes/                     TrackGeometry
├── centrado/                    control lateral y rumbo portable
├── giros/                       perfil angular portable
├── velocidad/                   AdaptiveSpeed y PI portable
├── odometria/                   progreso de tramos
├── seguridad/                   orquestador de movimiento y MotionResult
├── mapa/                        CellMap, rejilla, inundación y serialización
├── exploracion/                 CellExplorer, puertos y orquestador de misión
└── comunicacion/                tramas CRC y LineQueue
simulator/python/raton_sim/
├── simulacion/                  física, escenarios y composición Sim
├── paredes/ centrado/ velocidad/ giros/ odometria/
├── seguridad/                   tramos, orquestador y auditoría
├── mapa/                        pistas, mapa observado y persistencia
├── exploracion/ modos/          explorador y políticas de cada modo
├── entrenamiento/               evaluación, CMA-ES y selección robusta
├── publicacion/                 catálogo y reproducciones
└── comunicacion/                auditoría de registros
web/src/
├── conexion/                    BLE, protocolo, órdenes, hook y pruebas
├── telemetria/                  muestras, reducer, registro, sensores y pantalla
├── mapa/                        ocupación, inundación, mapas SVG/3D y pruebas
├── simulacion/                  catálogo, reproducción, hooks y pruebas
├── parametros/                  edición de ajustes
├── seguridad/                   supervisor, diagnóstico y pruebas
└── modos/                       App, navegación y estilos globales
 tests/{cpp,python}/<función>/    pruebas agrupadas por comportamiento
```

## Placa y núcleo portable

Se conservan `src_dir = firmware` y la biblioteca automática `lib/raton_core` de PlatformIO. El include
root es `firmware` (`build_flags = -I firmware`); el de la biblioteca es `lib/raton_core/src`.
CMake compila recursivamente ese mismo núcleo portable, y el puente de giros de `simulator/native`.
`prueba_hardware` compila los drivers y su entrada, excluyendo la calibración que necesita RobotHW.
No se duplican algoritmos de navegación para probarlos: `CellExplorer` usa el puerto `CellDriver`.

`RobotHW` conserva el estado compartido para mantener el comportamiento y el orden del ciclo; sus
métodos están implementados en la carpeta de la función correspondiente. `seguridad/movimiento.cpp`
orquesta los tramos completos y delega la captura de ventanas por celda a paredes, el control y los
sensores. No introduce tareas concurrentes ni cambia la frecuencia de lectura. `comunicacion/tuning.h`
es el agregado de ajustes remotos y peticiones de operador; sus valores vienen de cada params.h.
Los seis puntos de entrada de modo viven en archivos separados y reutilizan las políticas existentes.

Cada `firmware/<función>/params.h` mantiene el nombre, tipo, expresión y valor originales en `cfg`.
`config.h` conserva pines e I2C y agrega los módulos; `velocidad/params.h` declara su dependencia de
hardware para la compensación PWM. El lector Python `giros/native.configuration_text()` expande los
includes locales una sola vez; los textos históricos monolíticos siguen admitidos por `firmware_config`.

## Simulador y web

`Sim` compone clases de comportamiento por función, sin alterar los cuerpos de los métodos ni los
pasos de sus generadores. `simulacion/modelos.py` agrupa el cuerpo, sensor, escenario y la fotografía
`Firmware` de parámetros históricos de simulación; la configuración real se lee de los params.h.
Las clases derivadas de auditoría, mapa observado y entrenamiento siguen sobrescribiendo los mismos
métodos. La entrada pública de consola continúa siendo `python -m raton_sim`.

La web abandona las carpetas de capas técnicas. Cada función reúne componentes, hooks, funciones
puras y sus `__tests__`. Las dependencias se importan directamente desde la función propietaria.
Los tipos compartidos de muestras están en `telemetria/types.ts`; App queda en `modos/`.

## Pruebas y equivalencia

`tests/cpp/test_main.cpp` incluye los checks existentes por función y conserva una sola entrada CTest
(`nucleo`). Los fixtures de navegación están en `tests/cpp/exploracion/`. Pytest descubre recursivamente
`tests/python/<función>/`; su conftest raíz sigue compartido. Vitest descubre `src/**/__tests__`.
`scripts/run.sh pruebas` reconfigura CMake y ejecuta todas las comprobaciones y ambos firmwares.
Los resultados previos y `web/public/sim` no forman parte de esta reorganización.
La comparación de los siete escenarios y tamaños del firmware se documenta en
[equivalencia-organizacion.md](equivalencia-organizacion.md).

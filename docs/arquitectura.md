# Arquitectura

El proyecto separa la lógica (portable, probada en el PC) de los adaptadores de hardware, de la web y
del simulador. Las dependencias apuntan siempre hacia el núcleo.

```
            web/ (React)                 simulator/python (raton_sim)
                 │ Bluetooth (protocolo)            │ ctypes
                 ▼                                  ▼
firmware/  app/ ──► robot/ ──► hw/        simulator/native (barrido de giros)
            │         │
            └────┬────┘
                 ▼
        lib/raton_core   (sin Arduino: se compila igual en la placa y en el PC)
```

## Núcleo — `lib/raton_core/src`

| Carpeta | Contenido |
|---|---|
| `maze/` | `grid.h` (celdas y rumbos), `CellMap` (paredes, celdas vistas, visitas, callejones, inundación, frontera y celdas sin ver de la mejor ruta) |
| `geometry/` | `TrackGeometry`: pasillo y celda medidos, ángulo real de los laterales, punto muerto y todas las distancias de decisión |
| `exploration/` | `CellExplorer` (modos 3 y 4: explorar, mapear, Adachi, carrera rápida) y sus puertos `CellDriver`, `CellObserver` y `MissionLog` |
| `control/` | `OrquestadorMision`, `OrquestadorMovimiento` (frenado y seguridad), `motionProgress`, `MotionResult` |
| `protocol/` | Tramas `$seq,datos*CRC` y `LineQueue` (colas de telemetría) |

`CellExplorer` no conoce el hardware: habla con el robot por el puerto `CellDriver`. En el firmware lo
implementa `RobotHW`; en las pruebas (`tests/cpp`) un robot simulado que recorre laberintos de
`tests/mazes`. Así el algoritmo que corre en la placa es exactamente el que se prueba.

## Firmware — `firmware/`

| Carpeta | Clases |
|---|---|
| `app/` | `App` (arranque, elección de modo, despacho), `WallFollower` (modos 1-2), `FreeSpaceNavigator` (modo 5), `MazeMission` (modos 3-4 y 6, persistencia TDBStore en `map_storage`), `Maneuvers` (recuperaciones), `Reporter` (líneas DECIDE) |
| `robot/` | `RobotHW` (ciclo de control, odometría y rumbo en `robot_hw.cpp`; maniobras en `robot_motion.cpp`; calibraciones en `robot_calibration.cpp`), `CommandHandler` (órdenes y ACK/NAK), `Tuning` y `OperatorRequests` |
| `comms/` | `Telemetry` encapsula Bluetooth, recepción y colas prioritaria y normal |
| `hw/` | Drivers: `Motors` (DRV8833), `Encoders`, `Lasers` (3×VL53L0X con Kalman), `Gyro` (BMI270), `Display` (SSD1306) |
| `hw_test/` | Firmware de prueba de sensores y motores (`pio run -e prueba_hardware`) |
| `config.h` | Pines, calibración y parámetros (algunos entrenados, ver [entrenamiento.md](entrenamiento.md)) |

Los contadores de interrupciones de `Encoders` quedan encapsulados en un banco interno compartido
por los pines físicos. `HardwareTest` agrupa los drivers y el estado del firmware de calibración.

## Web — `web/`

React + TypeScript con capas `domain/` (protocolo, mapa, supervisor: funciones puras), `infrastructure/`
(transporte Web Bluetooth, cola de órdenes con ACK, registros y sesiones), `application/` (hooks y
estado) y `ui/` (componentes). `App` contiene la navegación, `Telemetry` coordina la pantalla de
telemetría y `Diagnostics` presenta las decisiones y el diagnóstico. Páginas: Telemetría y Simulación.

## Simulador — `simulator/`

`simulator/python/raton_sim`: simulador físico del ratón (sensores con ruido, geometría diagonal,
odometría), bancos de pistas, evaluación, publicación de resultados para la web y entrenamiento CMA-ES.
`simulator/native`: verificador de barrido de giros en C++ usado por el simulador.

## Pruebas — `tests/`

- `tests/cpp`: núcleo (CTest), incluido el explorador real resolviendo y mapeando laberintos.
- `tests/python`: simulador (pytest).
- `web/src/**/__tests__`: web (Vitest).

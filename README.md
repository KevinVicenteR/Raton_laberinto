# Ratón de laberinto

Firmware, simulador y web de telemetría de un micromouse con Arduino Nano 33 BLE Sense Rev2:
3 × VL53L0X (frontal y dos laterales en diagonal), DRV8833 con motores N20 1:100 y encoders, IMU
BMI270 y OLED SSD1306. Cuerpo de 80 × 80 mm con el eje centrado.

## Puesta en marcha

```bash
scripts/setup.sh
```

Instala lo que falte en el sistema (Python, CMake, Node; con Homebrew en macOS o apt en Linux), crea
`.venv`, instala la web, compila el núcleo y descarga la cadena del firmware.
Para trabajar sin la cadena de la placa, usa `scripts/setup.sh --sin-firmware` y
`scripts/run.sh pruebas --sin-firmware`. Las pruebas completas incluyen también el formato de Python y web.

```bash
scripts/run.sh
```

Pasa las pruebas rápidas y abre la web de telemetría en http://127.0.0.1:5173 (Bluetooth con Chrome o
Edge). Otras órdenes:

| Orden | Qué hace |
|---|---|
| `scripts/run.sh pruebas` | Todas las pruebas: núcleo C++, simulador, web y compilación del firmware |
| `scripts/run.sh subir [puerto]` | Compila y sube el firmware por USB |
| `scripts/run.sh monitor [puerto]` | Monitor Serie |
| `scripts/run.sh sim casos` | Casos extremos del simulador (`simulator/results/`) |
| `scripts/run.sh sim publicar` | Publica resultados y sesiones de simulación para la web |
| `scripts/run.sh sim entrenar` | Entrenamiento CMA-ES de parámetros |

## Modos del ratón

1. Seguir la pared derecha · 2. Seguir la pared izquierda · 3. Laberinto por celdas (mapea una pista
desconocida; con meta: explora, busca la ruta demostrada y hace la carrera rápida) · 4. Ruta más corta
del laberinto del reglamento · 5. Espacio libre · 6. Aprender pista (mapear, volver y guardar en flash).

## Estructura

```
firmware/        firmware del robot (PlatformIO): app/, robot/, comms/, hw/, hw_test/, config.h
lib/raton_core/  núcleo portable: mapa, geometría, explorador, orquestadores, protocolo
simulator/       simulador Python (raton_sim) y biblioteca nativa
web/             telemetría y laboratorio de simulación (React + TypeScript)
tests/           pruebas del núcleo (C++), del simulador (Python) y laberintos
data/registros/  registros reales de telemetría por Bluetooth
docs/            arquitectura, protocolo, entrenamiento y fuentes
scripts/         setup.sh y run.sh
```

Más detalle en [docs/arquitectura.md](docs/arquitectura.md), [docs/protocolo.md](docs/protocolo.md) y
[docs/entrenamiento.md](docs/entrenamiento.md).

## Integración continua

`.github/workflows/ci.yml` compila y prueba el núcleo (con sanitizers), el simulador, la web y el
firmware, y comprueba que `scripts/setup.sh` y `scripts/run.sh pruebas` funcionan en una máquina limpia.

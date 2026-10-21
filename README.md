# Ratón de laberinto — arquitectura v2

Firmware, núcleo portable y simulador de un micromouse para la categoría **Robot ratón** del Concurso de
Robótica SECIHTI Yucatán 2026:
- Laberinto de ~1.5 × 3 m, suelo negro.
- Pasillos de ~14 cm y paredes de 5 cm.
- Robot de 10 × 10 cm como máximo.
- 10 minutos para llegar a la meta.
- Descalificación si pasa más de 30 s en la misma zona.

Hardware:
- **Placa:** Arduino Nano 33 BLE Sense Rev2 (nRF52840 a 64 MHz, IMU BMI270).
- **Distancia:** 3 × VL53L0X, uno al frente y dos a 45° (no hay sensores a 90°).
- **Motores:** DRV8833 con N20 1:100 con encoders.
- **Pantalla:** OLED SSD1306 opcional.

## Por qué se rehízo

En los videos el ratón se pegaba a las paredes, se atoraba en sitios sencillos, iba en zigzag, era lento y volvía a sitios por donde ya había pasado. Dos causas de fondo:

1. **El VL53L0X ve un cono de 25°, no un rayo.** El simulador anterior usaba un rayo fino y por eso todo salía bien en el PC. Con el cono real en el simulador, el código anterior cumplía 34 de 42 casos y fallaba como en la pista de madera: paredes fantasma en los huecos, celdas mal contadas, vuelta a sitios ya vistos.
2. **Cada sensor tiene su propio desfase** (1–2 cm de fábrica). Sin calibrar, el ratón cree que está descentrado y se pega a una pared.

| Problema | Solución |
|---|---|
| Ve mal, mapea mal, vuelve a sitios ya vistos | **Sonda/radar:** cada lectura se compara con lo que el sensor debería leer con su cono real, el mapa y la posición. Se prueban todas las combinaciones de paredes que el cono alcanza y solo se vota lo que todas las explicaciones aceptan. |
| Se pega a las paredes | **Calibración opcional** de los desfases con paredes de referencia, aprendizaje de inclinación/piso en marcha y posición lateral corregida con la sonda. |
| Hay que calibrar a mano | Giroscopio en reposo validado, motores aprendidos por rueda y sentido, y paso efectivo del encoder guardado (ver [docs/calibracion.md](docs/calibracion.md)). La geometría inicial y los desfases de distancia requieren referencias conocidas. |
| Pierde la cuenta de celdas | El avance se corrige con postes, huecos y paredes de frente usando la sonda, comprobando cada corrección antes de aplicarla. |
| Zigzag, torcido tras girar | Filtro de Kalman lateral; cada giro en el sitio termina alineado con la cuadrícula. |
| Empuja la pared sin avanzar | Tres detectores de atasco y recuperación determinista: retroceder, enderezarse, recentrarse en "S" y replanificar. |
| Regla de los 30 s | Vigilante de zona: si lleva 10 s en la misma zona, fuerza un objetivo fuera de ella. |
| Lento | 400 mm/s explorando con 0.3 g de aceleración (−12 % de tiempo), decisión de la celda siguiente antes de llegar, 650 mm/s en carrera con curvas suaves. |

## Resultados en el simulador

El simulador ejecuta **exactamente el mismo código** que la placa (`lib/raton_core`) sobre una física con:
- motor con inercia y rozamiento, y ruedas que patinan;
- encoders con cuentas enteras;
- giroscopio con sesgo, deriva y error de escala;
- **VL53L0X con cono de 25°** (lectura = media pesada por la señal), latencia, ruido, lecturas perdidas y falsas, desfase propio de cada sensor;
- cuerpo redondo que no atraviesa paredes.

Errores del nivel 2: celda ±6 mm, pared ±4, ruedas ±2 %, eje ±4 %, motores, giroscopio, ruido de los láseres 4 %, desfases de cada láser ±16 mm, montaje de los laterales ±3°, colocación en la salida.

| Prueba (laberintos de 19 × 10) | Antes (con el cono real) | Ahora |
|---|---|---|
| 161 casos con errores: cumplen el reglamento | 119 | **161/161** |
| … sin ningún roce | 90 | **155** |
| … paredes mal en el mapa final (total) | 694 | **2** |
| 81 casos sin errores | — | **81/81 sin roce** |
| Pistas de práctica de 170–300 mm completas | 18 de 28 | **70 de 70** |
| Laberinto del reglamento | | meta explorando ~50 s, carrera ~12 s, 0 roces |

La tabla anterior es la referencia histórica previa a la autocalibración por rueda. La actualización del
21 de octubre se comprobó con `scripts/banco.sh 160`: **161/161**, 157 sin contacto, 3 paredes mal en total,
meta media 63,4 s, exploración 117,1 s y carrera 15,8 s. En la misma prueba, tu versión base del día daba
161/161, 158 sin contacto y 68 contactos en carrera; la nueva da 38 contactos en carrera.
También pasa **81/81 con motores un 20 % más débiles** y **81/81 con sensores a 25 mm**.
Véase [calibración](docs/calibracion.md) para el arranque validado, aprendizaje y límites físicos.
La política de averías está descrita en el [traspaso](docs/traspaso.md).

El simulador es un modelo: la pista real tiene diferencias. Por eso hay que **calibrar en la pista**
([docs/calibracion.md](docs/calibracion.md)) antes de competir.

## Uso rápido

```bash
scripts/run.sh pruebas              # compila y pasa las pruebas (364 comprobaciones, incluidas misiones)
scripts/run.sh sim                  # laberinto del reglamento con detalle y mapa final
scripts/run.sh sim --random 7 --perturb 2   # aleatorio con errores fisicos
scripts/run.sh bateria 20           # bateria completa
scripts/run.sh subir                # compila y sube el firmware (PlatformIO)
```

### En la placa

1. **Elegir modo:** al encender aparece el menú. Pasa la mano delante del sensor frontal (acercar y retirar) para cambiar de modo.
2. **Arrancar:** deja la mano delante más de 1 s y retírala. El ratón espera 0,7 s, calibra el giroscopio quieto y arranca.
3. **Bluetooth "Raton" o Monitor Serie:** `s` arranca, `x` para, `m<n>` elige modo y `help` lista las órdenes.

Si "Raton" no aparece, reinicia la placa y revisa el Monitor Serie a 115200: si BLE no pudo iniciar,
el firmware lo informa ahí. Conecta un solo cliente Bluetooth a la vez; después de una desconexión
inesperada, la web intenta recuperar el enlace automáticamente.

| Modo | Qué hace |
|---|---|
| 1 Resolver desconocido | Parte de un mapa vacío y busca la meta. Conserva el mapa guardado en flash. |
| 2 Mapear y guardar | Explora todo lo alcanzable, vuelve a la salida, guarda el mapa y calcula la ruta más corta en número de celdas. |
| 3 Resolver aprendido | Carga el mapa guardado y busca la ruta más corta por pasos conocidos hasta la meta. Si falta mapa o ruta, informa y permanece parado. |

Envía **`1`, `2` o `3`** para arrancar directamente (también sin salto de línea). Se mantienen `s1`–`s3` para arrancar y `m1`–`m3` para seleccionar. `x` detiene, incluso durante la preparación del arranque. `sensores` muestra lecturas sin mover el ratón; `x` vuelve al menú. `erase` borra el mapa; no es un cuarto modo.

Configura `cellMm`, tamaño, salida y meta para la pista real. Coloca el ratón en la salida con el rumbo configurado antes de cada intento. Los tres modos usan esa geometría; no miden automáticamente una pista de tamaño desconocido.

La calibración contra paredes es opcional mediante `calsonda`: ya no bloquea el arranque normal cuando no hay paredes visibles. Una lectura nueva fuera de alcance no se considera sensor desconectado. Se conserva la parada ante pérdida real del frontal, ambos laterales o giroscopio. El modo 2 guarda únicamente al completar la exploración y regresar; el modo 3 recalcula la ruta desde el mapa persistido.

## Estructura

```
lib/raton_core/src/raton/   nucleo portable (sin Arduino): mapa, planificador, control, navegador,
                            explorador, filtro lateral, parametros, tramas
firmware/                   placa: hw/ (motores, encoders, laseres, IMU, pantalla, Bluetooth), board
                            (lazo de 500 Hz), app (menu, ordenes, calibraciones), storage (flash)
sim/                        simulador fisico y bateria de laberintos
tests/                      pruebas (CTest) y stubs/ para comprobar el firmware en el PC
mazes/                      laberinto del reglamento en texto
docs/                       arquitectura, calibracion y fuentes
web/                        web de telemetria por Bluetooth (de la version anterior; lee NAV y CELL)
```

Más detalle en [docs/arquitectura.md](docs/arquitectura.md).

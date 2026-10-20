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
| Se pega a las paredes | **Calibración automática** de los desfases de los tres sensores al arrancar (gira en el sitio en la salida) y posición lateral corregida con la sonda. |
| Hay que calibrar a mano | Todo es automático: sensores en la salida, motores y giroscopio aprendidos en marcha y guardados solos (ver [docs/calibracion.md](docs/calibracion.md)). Solo hay que decirle dónde está la meta. |
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
| 161 casos con errores: cumplen el reglamento | 119 | **158–159** |
| … sin ningún roce | 90 | **~146** |
| … paredes mal en el mapa final (total) | 694 | **~30** |
| 81 casos sin errores | — | **81/81 sin roce** |
| Pistas de práctica de 170–300 mm completas | 18 de 28 | **67 de 70** |
| Laberinto del reglamento | | meta explorando ~50 s, carrera ~12 s, 0 roces |

El simulador es un modelo: la pista real tiene diferencias. Por eso hay que **calibrar en la pista**
([docs/calibracion.md](docs/calibracion.md)) antes de competir.

## Uso rápido

```bash
scripts/run.sh pruebas              # compila y pasa las pruebas (unas 110 comprobaciones + misiones)
scripts/run.sh sim                  # laberinto del reglamento con detalle y mapa final
scripts/run.sh sim --random 7 --perturb 2   # aleatorio con errores fisicos
scripts/run.sh bateria 20           # bateria completa
scripts/run.sh subir                # compila y sube el firmware (PlatformIO)
```

### En la placa

1. **Elegir modo:** al encender aparece el menú. Pasa la mano delante del sensor frontal (acercar y retirar) para cambiar de modo.
2. **Arrancar:** deja la mano delante más de 1 s y retírala. El ratón espera 0,7 s, calibra el giroscopio quieto y arranca.
3. **Bluetooth "Raton" o Monitor Serie:** `s` arranca, `x` para, `m<n>` elige modo y `help` lista las órdenes.

| Modo | Qué hace |
|---|---|
| 1 Explorar (competencia) | Geometría de la competencia (`cellMm`, tamaño, salida y meta guardados). Calibra solo los sensores en la salida, va a la meta, busca la ruta más corta, vuelve a la salida y guarda el mapa y lo aprendido en flash |
| 2 Carrera (competencia) | Ruta más rápida con el mapa guardado (curvas suaves, rectas a 650 mm/s) |
| 3 Práctica (cualquier medida) | Cualquier pista: calibra los laterales y mide el pasillo en la salida (gira a cada lado), afina el paso de celda en marcha, explora todo recordando por dónde pasó y vuelve a la salida. No toca la geometría ni el mapa de la competencia |
| 4 Sensores | Lecturas en pantalla y Bluetooth |
| 5 Cal. motor | Mide kV, kS y τ de los motores en un pasillo recto |
| 6 Ruedas | Prueba de sentido de motores y encoders (ratón levantado) |
| 7 Borrar mapa | Olvida el mapa guardado |

**Estrategia de competencia:**
- **1ª etapa** (4 oportunidades, 10 min): modo 1. Al llegar a la meta ya cuenta. Si lo dejan seguir, vuelve solo y deja el mapa guardado.
- **2ª etapa** (2 oportunidades, mejor tiempo): modo 2 con ese mapa.

Si el laberinto real no es el del ejemplo, configura tamaño, salida y meta antes (ver calibración).

**Modo práctica (3):** pon el ratón en el centro de una celda con pared a la izquierda y a la derecha y la salida libre delante. El mapa es de 32 × 32 con la salida en el centro, así que la pista puede extenderse hasta 15 celdas hacia cada lado. Sin meta: explora hasta conocer todo lo alcanzable (no vuelve a pasar por donde ya conoce salvo para llegar a lo que falta) y regresa. Al arrancar gira a cada lado para calibrar los laterales y medir el pasillo; el frontal tiene que venir calibrado de antes (`calsonda` en la pista de la competencia + `save`).

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

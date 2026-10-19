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

En los videos de la versión anterior el ratón:
- se pegaba a las paredes;
- se quedaba moviéndose sin avanzar en esquinas y callejones;
- dudaba girando de un lado a otro;
- quedaba torcido tras los giros.

La versión 2 ataca cada causa:

| Problema | Solución |
|---|---|
| Decide parado y "duda" | Las paredes de la celda **siguiente** se leen mientras avanza (el rayo a 45° cae ~70 mm por delante del eje). La decisión (inundación, < 1 ms) está lista antes de entrar en la celda. |
| Queda torcido y roza | Filtro de Kalman lateral: calcula el desplazamiento y el **error del giroscopio** con las paredes. Además aprende solo la escala del giroscopio y el ancho real del pasillo. |
| Empuja la pared sin avanzar | Tres detectores de atasco: ruedas que no siguen la consigna, encoders que avanzan sin que el frontal se acerque, y pared inesperada delante. Recuperación determinista: retroceder, enderezarse, recentrarse en "S" y replanificar. |
| Lecturas fantasma | Cada lectura es un **voto** sobre una pared concreta. Se descartan las que caen cerca de los postes. Una pared solo cuenta con 2 votos netos. |
| Regla de los 30 s | Vigilante de zona: si lleva 10 s en la misma zona, fuerza un objetivo fuera de ella. Si el mapa quedó sin camino, olvida las paredes dudosas y sigue. |
| Lento | Velocidad trapezoidal de 320 mm/s explorando y 650 mm/s en carrera, con curvas suaves sin parar. Si en la carrera hay un atasco, termina en modo seguro. |

## Resultados en el simulador

El simulador ejecuta **exactamente el mismo código** que la placa (`lib/raton_core`) sobre una física con:
- motor con inercia y rozamiento, y ruedas que patinan;
- encoders con cuentas enteras;
- giroscopio con sesgo, deriva y error de escala;
- VL53L0X con rayo real, latencia, ruido, lecturas perdidas y falsas;
- cuerpo redondo que no atraviesa paredes.

`scripts/run.sh bateria 40` corre el laberinto del reglamento y 40 laberintos aleatorios de 19 × 10, cada uno con física ideal y con errores. Los errores son: celda de 149–161 mm, ruedas ±2 %, ángulo de los sensores ±4°, giroscopio ±2 %, offsets de los sensores y colocación en la salida.

| Prueba | Resultado |
|---|---|
| Laberinto del reglamento | meta explorando en ~55 s, ruta más corta demostrada, vuelve a la salida; carrera en **~12 s**; 0 roces y 0 paredes mal leídas |
| 82 casos | todos llegan a la meta en las dos pasadas, ninguno pasa 30 s en una zona; ~80 sin ningún roce (los otros, roces leves en la física con errores) |
| Medias | meta explorando ~80 s, exploración completa ~150 s, carrera ~16 s |

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
| 1 Explorar | Va a la meta, busca la ruta más corta, vuelve a la salida y guarda el mapa en flash |
| 2 Carrera | Ruta más rápida con el mapa guardado (curvas suaves, rectas a 650 mm/s) |
| 3 Solo meta | Explora hasta la meta y para |
| 4 Sensores | Lecturas en pantalla y Bluetooth |
| 5 Cal. motor | Mide kV, kS y τ de los motores en un pasillo recto |
| 6 Ruedas | Prueba de sentido de motores y encoders (ratón levantado) |
| 7 Borrar mapa | Olvida el mapa guardado |

**Estrategia de competencia:**
- **1ª etapa** (4 oportunidades, 10 min): modo 1. Al llegar a la meta ya cuenta. Si lo dejan seguir, vuelve solo y deja el mapa guardado.
- **2ª etapa** (2 oportunidades, mejor tiempo): modo 2 con ese mapa.

Si el laberinto real no es el del ejemplo, configura tamaño, salida y meta antes (ver calibración).

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

> Actualización 2026-10-21: el menú tiene únicamente 1 Resolver desconocido, 2 Mapear/guardar/ruta mínima, 3 Resolver aprendido. Los números arrancan directamente; `s1`–`s3` y `m1`–`m3` siguen disponibles. La calibración de paredes es opcional (`calsonda`) y no forma parte del arranque. Las referencias posteriores a los siete modos o práctica corresponden al diseño anterior. Véase README para el funcionamiento actual.

# Arquitectura

```
          +-------------------+        +------------------------------+
 placa -> | firmware/board    |        | sim/sim.cpp (fisica)         | <- PC
          | (Platform real)   |        | (Platform simulada)          |
          +---------+---------+        +---------------+--------------+
                    \                                 /
                     v                               v
          +----------------------------------------------------------+
          | raton::Mouse      mision: explorar / carrera              |
          |  raton::Explorer  cerebro: fases, vigilante de zona       |
          |  raton::Navigator celdas: sensado, decision, giros,       |
          |                   atascos, correcciones                   |
          |   raton::Motion   control 500 Hz: perfiles + modelo + PID |
          |   raton::LateralFilter  centrado y error del giroscopio   |
          |  raton::Maze / planner  mapa con votos, inundacion,       |
          |                         Dijkstra con giros                |
          +----------------------------------------------------------+
```

## Platform

`raton::Platform` es la única frontera con el hardware:
- `tick()` espera al siguiente periodo de 2 ms y devuelve encoders (mm), giroscopio (grados acumulados) y la medida nueva de un láser (si la hay).
- `setMotors()` recibe el duty por rueda.
- `report()` envía telemetría.

La placa (`firmware/board.cpp`) atiende Bluetooth y Serie en el tiempo sobrante del ciclo. Lee un VL53L0X por ciclo, por turnos: cada sensor mide a ~50 Hz y el I2C del ciclo no pasa de ~0,6 ms. El giroscopio BMI270 se sube a 200 Hz escribiendo `GYR_CONF`, porque la librería de Arduino lo fija a 100 Hz.

## Control (`motion.cpp`)

- **Avance:** perfil trapezoidal hacia una posición final con velocidad final (0 = parar, `turnSpeed` = entrar en curva).
- **Rumbo:** perfil trapezoidal (giros) o rumbo de cuadrícula más corrección de centrado (rectas).
- **Salida por rueda:** `kS·signo + kV·v + kA·a` (modelo del motor, `kA = kV·τ`) más PID sobre el error de posición y de rumbo, con compensación de zona muerta (rozamiento estático).
- **Saturación:** se prioriza el giro. Con el duty al máximo, la consigna no se adelanta a lo medido, y la velocidad de crucero no pasa de lo que los motores dan de verdad (`Motion::topSpeed`, con el modelo aprendido). Antes, con batería baja la consigna se escapaba, el error crecía y parecía un atasco a toda velocidad.
- **Curvas de la carrera:** la velocidad de curva se limita para que la rueda de fuera no pida más de lo que da el motor (`Navigator::arcSpeed`); si no, la curva se abre y choca.
- **Rumbo medido:** giroscopio interpolado con los encoders entre muestras.

El DRV8833 se maneja en *slow decay* a 20 kHz: una entrada fija en alto y la otra con PWM invertido. Así la velocidad es casi proporcional al duty y el N20 conserva par a baja velocidad, según la hoja de datos del DRV8833.

## Sonda y radar (`sonda.cpp`, `navigator.cpp`)

El VL53L0X no mide un rayo fino: ilumina un **cono de 25°** y devuelve una media de las distancias pesada por la luz que vuelve (más peso a lo cercano y a lo que está de frente). Con los laterales a 45° la mancha sobre la pared mide de 4 a 9 cm según el pasillo, y cerca de un hueco o de un poste la lectura mezcla las dos cosas. El simulador antiguo usaba un rayo fino; con el cono real en el simulador el código anterior pasaba de 42/42 a 34/42 en la batería de la competencia, igual que los fallos vistos en la pista de madera.

**Sonda** (`sonda.h`): para una pose y un estado de paredes, traza el cono entero (9 rayos) por la rejilla de paredes con su grosor y los postes de las esquinas, y calcula la lectura esperada con el mismo modelo que el simulador. Coincide con el sensor simulado con 2 mm de error medio y sin sesgo, con el sensor entre 15 y 45 mm de altura.

**El cono también se abre en vertical** (sensor a `sensorHeightMm`, paredes de `wallHeightMm`). Cada columna del cono se reparte, integrando en el ángulo vertical, entre:
- **piso:** la parte de abajo lo toca a `h / tan(12,5°)` (a 40 mm, ~18 cm; a 25 mm, ~11 cm). En un pasillo vacío el frontal "ve algo delante": es el piso;
- **pared;**
- **por encima:** con el sensor alto, la parte de arriba pasa por encima de las paredes y mira fuera del laberinto.

Lo que vuelve del piso se modela, pero el piso real nunca es como el del modelo. Por eso cada predicción lleva una **duda por el piso**: la fracción de la lectura que es piso × lo que la movería un error del 50 % en su reflejo y del 10 % en su distancia. Esa duda ensancha la tolerancia del radar y resta peso a la lectura para la pose.

**Se aprende en marcha** (`learnVertical`, `Navigator::learnVertical`): la inclinación real de cada sensor (absorbe también el error de altura) y el reflejo del piso. Se ajustan con las lecturas que el mapa ya explica (paredes seguras, pose bien conocida, en recta) por gradiente normalizado: cada lectura mueve poco, así que el ruido se promedia. Al terminar la misión se guardan (`leftTiltDeg`, `frontTiltDeg`, `rightTiltDeg`, `floorReflect`).

**¿Es pared o es piso?** (`frontSeesWall`): antes de parar por una pared inesperada, de darse por atascado empujando una pared o de colocarse con el frontal, se compara con la sonda si la lectura se parece más a "pared en esa frontera" o a "sin pared" (piso y lo de detrás). El detector de "empuja una pared y el frontal no se acerca" además solo actúa por debajo de donde empieza el piso: el piso avanza con el ratón y su lectura no cambia, y antes eso parecía un atasco.

**Radar** (cada lectura de cada sensor, en `Navigator::radar`):
- **Mapa por hipótesis:** se toman las paredes que el cono puede alcanzar (hasta 6) y se prueban todas sus combinaciones, cerrada o abierta. Una pared se vota solo si todas las combinaciones que explican la lectura coinciden en ella. Si unas la explican con pared y otras sin pared, esa lectura no dice nada de ella. Si ninguna la explica, la pose está mal y no se vota.
- **Alcance:** los laterales votan paredes de esta celda y la siguiente; el frontal, hasta dos celdas. Sin medida (más de 60 cm o perdida dos veces) solo se deduce que lo cercano está libre.
- **Tolerancia:** ruido del sensor (~3 %), error del modelo (crece con la distancia) y lo que la duda de la pose mueve esa lectura (pendiente de la sonda × incertidumbre).
- **Pose:** con paredes ya confirmadas (±3 votos) y cerca, la diferencia entre lectura y predicción corrige el desplazamiento lateral (filtro de Kalman) y el avance (postes, huecos, pared de frente) usando la pendiente de la sonda. Cerca de un borde la relación no es lineal: antes de corregir el avance se comprueba con la sonda que el desplazamiento propuesto de verdad explica la lectura. El rumbo no se corrige con la forma de una sola lectura (un desfase de unos mm lo torcería grados); sale de cómo cambia el desplazamiento al avanzar.
- **Giros:** girando (más de 40°/s) no se usa ninguna lectura. Lo que se mueva el eje durante un giro en el sitio (las ruedas patinan algo) no se suma al desplazamiento lateral: antes, al acabar una media vuelta se sumaba con el rumbo nuevo y el ratón creía estar 2–3 cm descentrado, y votaba mal paredes clave.
- **Paso de celda en marcha (también en la competencia):** al acabar cada recta, la corrección de avance acumulada frente a lo recorrido. La pendiente es lo que la odometría se desvía por milímetro; lo que se arrastra del giro anterior queda en la ordenada y no molesta. Las rectas se juntan con un filtro de Kalman (las largas pesan mucho, las cortas casi nada), dentro de ±5 % del nominal (±8 % en práctica). Corrige a la vez que la pista no mida lo nominal y que las ruedas no midan lo previsto: en el simulador converge a ±0,5 mm del valor real. Sin esto, en una recta de 5 celdas el ratón empezaba las curvas de la carrera 3–4 cm antes y chocaba.
- **Desfases y pasillo real:** los desfases de los láseres se calibran en la salida suponiendo pasillo = `cellMm − wallMm`. Si el paso aprendido es otro, las tres lecturas se corrigen la mitad de la diferencia (`Navigator::rangeBias`).

Los laterales a 45° ven las paredes de la celda siguiente cuando el eje aún está en la actual: la decisión está lista antes de llegar, sin pararse a mirar.

**Calibración de los sensores** (`Mouse::calibrateSensors`, orden `calsonda`, automática al arrancar el modo 1 y el 3): en la salida, con pared a los dos lados, un **barrido continuo** (sin paradas, 120°/s) hasta 110° a cada lado y vuelta. En una pared plana la lectura más corta es la perpendicular: el frontal pasa por ella a ±90° y cada lateral a ±45°. Se ajusta una parábola a las lecturas de ±20° alrededor (sin las lecturas sueltas falsas); si sale demasiado plana, se usa la mediana de ±5°. Ida y vuelta a cada mano: el error de escala del giroscopio se anula. En el simulador: error de 2,0 / 2,3 / 2,2 mm (frontal / laterales) en unos 4 s. Donde cae el mínimo también diría lo torcido que se colocó el ratón y el ángulo real de los laterales, pero cerca de la perpendicular la lectura es tan plana que esas estimaciones salían peor que no hacerlas. Con la celda conocida, izquierda + derecha = pasillo da el desfase del frontal; con la distancia a cada pared, el desfase de cada lateral. En el simulador, con desfases de hasta 16 mm por sensor: 158/161 cumplen con calibración y 123/161 sin ella. Si con la celda de la competencia las paredes de la salida están a otra distancia, avisa `PISTA_DISTINTA` y no sale (usar el modo 3 o `set cellMm`).

Giros:
- **Explorando:** en el sitio, en el centro de la celda. Tras girar, el frontal confirma que la salida está libre.
- **En carrera (ruta conocida):** curvas suaves. De los radios que dejan al cuerpo redondo con margen del poste interior y de la esquina exterior, el más rápido con los motores reales: con un radio mayor la rueda de fuera va menos por encima del centro. Debe caber en media celda para enlazar dos curvas seguidas.
- **Explorando por celdas ya recorridas:** en recta, crucero a `knownSpeed` (650) en vez de `searchSpeed` (400). Los giros siguen siendo en el sitio: probadas las curvas en lo conocido, empeoraban el mapa.

Atascos:
- **Detección:** ruedas que no siguen a la consigna, encoders que avanzan sin que el frontal se acerque, o pared inesperada.
- **Respuesta:** parar, retroceder 35 mm enderezándose a la cuadrícula a la vez (un solo movimiento) y recentrarse en "S" si los laterales ven que está desplazado. Luego se coloca con el frontal y replanifica. El rozamiento de los motores se compensa en rampa (no a saltos), así que arrancar marcha atrás o corregir parado no da tirones.
- **Atascos repetidos:** si se atasca varias veces en la misma celda sin pared visible, esa salida se da por cerrada.
- **En carrera:** tras un atasco sigue en modo seguro.

## Filtro lateral (`lateral.h`)

Kalman de 2 estados: desplazamiento `y` y error de rumbo `b`.
- Cada lectura lateral entra con su pendiente de la sonda (`updateLinear`): cuánto cambia la lectura por mm de desplazamiento.
- El rumbo se observa porque `y` cambia con `v·sin(rumbo real)`.
- El error estimado se pasa a la cuadrícula, para que el control vaya recto de verdad.
- Lo que se corrige después de cada giro en el sitio sirve para aprender la **escala del giroscopio**, poco a poco (un 8 % de lo que dice cada giro): cada giro deja 2–4° de corrección por otras causas, que se cancelan entre izquierdas y derechas.

## Explorador (`explorer.cpp`)

1. **Meta:** inundación optimista (lo desconocido cuenta como abierto); empates: recto, celdas nuevas.
2. **Ruta más corta:** visita solo las celdas sin confirmar de la mejor ruta posible, con coste por giro, hasta que la ruta con paredes conocidas es tan buena como la optimista (`exploreBudgetS` como máximo).
3. **Vuelta** a la salida y mapa a flash.
4. **Carrera:** Dijkstra sobre (celda, rumbo) con coste por giro. En las rectas conocidas acelera hasta la siguiente curva.
5. **Explorar todo** (sin meta, o meta que no existe en esa pista): va siempre a la celda sin conocer más cercana hasta que no queda nada alcanzable. Lo ya conocido no se vuelve a recorrer salvo para llegar a lo que falta. Antes de darlo por terminado revisa una vez las paredes dudosas, y si al volver aparece algo nuevo alcanzable, da otra vuelta.
6. **Volver sin camino:** si ni olvidando las paredes dudosas hay camino a la salida, olvida todas (las celdas visitadas se conservan) y vuelve mirando de nuevo.

## Modo práctica (`Mouse::measureCell`, `App::practiceRun`)

- La competencia y la práctica están separadas: la práctica usa una copia de los parámetros y del mapa y al terminar los restaura; no guarda nada en flash.
- Mapa de 32 × 32 con la salida en (16, 16): cabe una pista de hasta 15 celdas hacia cada lado, sin saber de antemano su tamaño.
- **Medida del pasillo:** la misma calibración de sensores (gira a cada lado y vuelve). Pasillo = distancia a la pared izquierda + a la derecha; celda = pasillo + `wallMm`. El desfase del frontal tiene que venir calibrado de la pista de la competencia (`calsonda`); el de los laterales se calibra ahí mismo.
- **Paso de celda en marcha:** al ver de frente una pared confirmada tras al menos dos celdas rectas, el error acumulado del tramo, repartido por celdas, corrige el paso (máximo ±8 %).

Vigilante de zona: con más de `areaLimitS` (10 s) dentro de un radio de 1 celda, el objetivo pasa a ser una celda alcanzable fuera de la zona. Si no hay camino, olvida las paredes poco confirmadas y vuelve a planificar.

## Parámetros

Todos están en `lib/raton_core/src/raton/params.h` y tienen nombre en `param_table.cpp`:
- en la placa: `set kpRot 0.02`, `get kpRot`, `params`, `save`;
- en el simulador: `--set kpRot=0.02`.

Se guardan en flash con CRC.

## Pruebas

- `tests/test_main.cpp`: mapa con votos, inundación, rutas, texto ↔ mapa, tabla de parámetros, filtro lateral, tramas, 6 misiones completas en el simulador (reglamento ideal y con errores, y 4 aleatorios) y 3 pistas de práctica de otras medidas (170, 200 y 260 mm).
- `testSonda`: la sonda contra el sensor simulado en 4000 poses al azar.
- Simulador: `--bench 20` (competencia, 42 casos), `--nivel 2 --desde k` (solo los casos con errores, a partir del k: para repartir en varios procesos), `--practica 70` (pistas de 4×5 a 8×9 celdas de 170 a 300 mm), `--pista ancho,alto,celda,semilla,errores` (una pista de práctica con detalle), `--cono 0` (rayo fino del simulador antiguo), `--sincal` (sin calibrar los sensores al arrancar).
- El simulador mide además el error de la pose estimada (avance / lateral / rumbo), el zigzag real (rumbo respecto a la cuadrícula) y el descentrado real en los tramos rectos, y avisa `PERDIDO` si el ratón cree estar en otra celda.
- Errores del nivel 2: celda ±6 mm, pared ±4, ruedas ±2 %, eje ±4 %, motores, giroscopio (sesgo 0.25°/s, escala ±2 %), ruido de los láseres 4 % con lecturas perdidas y falsas, desfases de cada láser ±16 mm, montaje de los laterales ±3°, colocación en la salida.
- Resultados actuales: competencia con errores (161 casos) 158–159 cumplen, ~146 sin roce, ~30 paredes mal en total; sin errores 81/81 sin roce. Práctica 67/70.
- CMake también compila todo `firmware/` contra cabeceras mínimas (`tests/stubs`) para detectar errores sin la placa.
- La integración continua compila el firmware real con PlatformIO y corre la batería de laberintos.

### Laterales opcionales a 90 grados

`Sensors` conserva `left/front/right` y anade `sideLeft/sideRight`. El driver admite hasta cinco VL53L0X;
los XSHUT nuevos sin asignar se omiten y la placa sigue consultando tres sensores. Cuando se configuran
y responden, consulta los cinco por turnos. `sideEnabled` activa la fusion, independientemente de la
lectura de diagnostico. El formato `NAV` anterior no cambia: las lecturas nuevas van en `LATERALES`.

`side_sensors.h` comprueba geometria, validez, distancia a postes y persistencia de evidencia. El
navegador aporta votos solo a aristas desconocidas con pose precisa. Con ambas paredes confirmadas,
la diferencia entre distancias a izquierda y derecha corrige el filtro lateral, evitando convertir el
error del ancho de pasillo en un error de centrado. Con aperturas, muestras incoherentes o caducadas,
se mantienen las correcciones de los diagonales y el giroscopio. No se aprenden automaticamente los
offsets de los sensores nuevos; deben medirse antes de habilitarlos. La carga de parametros acepta
tambien el formato guardado antes de esta ampliacion.

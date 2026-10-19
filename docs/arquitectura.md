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
- **Saturación:** se prioriza el giro.
- **Rumbo medido:** giroscopio interpolado con los encoders entre muestras.

El DRV8833 se maneja en *slow decay* a 20 kHz: una entrada fija en alto y la otra con PWM invertido. Así la velocidad es casi proporcional al duty y el N20 conserva par a baja velocidad, según la hoja de datos del DRV8833.

## Navegador (`navigator.cpp`)

Geometría con sensores a 45° (montaje). El VL53L0X tiene un cono de 25° y devuelve el punto más cercano, así que el ángulo efectivo del rayo es ~32° respecto al avance (`diagAngleDeg = 32`):
- El rayo lateral toca la pared ~100 mm por delante del eje.
- Las paredes laterales de la celda siguiente se ven poco después de pasar el centro de la actual.
- Ahí se decide la acción para la celda siguiente, así no hay pausa para mirar ni pensar.

Qué hace con cada lectura:
- **Lateral:** se convierte en un voto para la pared donde cae el rayo, descartando ±20 mm alrededor de los postes (más en pasillos anchos). Si cae en la zona limpia de la pared, alimenta el filtro lateral. El cambio pared→hueco (el poste) corrige el avance. En pasillos anchos (práctica), una lectura entre la pared y el umbral se ignora: suele ser el rayo que cruzó un hueco y dio en una pared de la celda vecina.
- **Votos:** una pared es conocida con 2 votos netos que coinciden; una lectura suelta no decide nada.
- **Frontal:** vota la primera pared que encuentra y las fronteras que cruza. Con paredes confirmadas corrige el avance. Si ve una pared que no debería estar (delante y cerca), para.

Giros:
- **Explorando:** en el sitio, en el centro de la celda. Tras girar, el frontal confirma que la salida está libre.
- **En carrera (ruta conocida):** curvas suaves de radio calculado para que el cuerpo redondo pase con margen del poste interior y de la esquina exterior.

Atascos:
- **Detección:** ruedas que no siguen a la consigna, encoders que avanzan sin que el frontal se acerque, o pared inesperada.
- **Respuesta:** parar, retroceder 35 mm, enderezarse a la cuadrícula y recentrarse en "S" si los laterales ven que está desplazado. Luego se coloca con el frontal y replanifica.
- **Atascos repetidos:** si se atasca varias veces en la misma celda sin pared visible, esa salida se da por cerrada.
- **En carrera:** tras un atasco sigue en modo seguro.

## Filtro lateral (`lateral.h`)

Kalman de 2 estados: desplazamiento `y` y error de rumbo `b`.
- La suma izquierda + derecha de los sensores a 45° no depende del rumbo, así que sirve para medir el **ancho real del pasillo**.
- El rumbo se observa porque `y` cambia con `v·sin(rumbo real)`.
- El error estimado se pasa a la cuadrícula, para que el control vaya recto de verdad.
- Lo que se corrige después de cada giro en el sitio sirve para aprender la **escala del giroscopio**.

## Explorador (`explorer.cpp`)

1. **Meta:** inundación optimista (lo desconocido cuenta como abierto); empates: recto, celdas nuevas.
2. **Ruta más corta:** visita solo las celdas sin confirmar de la mejor ruta posible, con coste por giro, hasta que la ruta con paredes conocidas es tan buena como la optimista (`exploreBudgetS` como máximo).
3. **Vuelta** a la salida y mapa a flash.
4. **Carrera:** Dijkstra sobre (celda, rumbo) con coste por giro. En las rectas conocidas acelera hasta la siguiente curva.
5. **Explorar todo** (sin meta, o meta que no existe en esa pista): va siempre a la celda sin conocer más cercana hasta que no queda nada alcanzable. Lo ya conocido no se vuelve a recorrer salvo para llegar a lo que falta. Antes de darlo por terminado revisa una vez las paredes dudosas, y si al volver aparece algo nuevo alcanzable, da otra vuelta.

## Modo práctica (`Mouse::measureCell`, `App::practiceRun`)

- La competencia y la práctica están separadas: la práctica usa una copia de los parámetros y del mapa y al terminar los restaura; no guarda nada en flash.
- Mapa de 32 × 32 con la salida en (16, 16): cabe una pista de hasta 15 celdas hacia cada lado, sin saber de antemano su tamaño.
- **Medida del pasillo:** gira a la izquierda, mide con el frontal, vuelve; gira a la derecha, mide, vuelve (un cuarto de vuelta a cada mano para que el error de escala del giroscopio se anule). Pasillo = izquierda + derecha + 2 × `frontAheadMm`; celda = pasillo + `wallMm`. Si había `calfront`, se descuenta su desfase y se traslada a la pista nueva.
- **Paso de celda en marcha:** al ver de frente una pared confirmada tras al menos dos celdas rectas, el error acumulado del tramo, repartido por celdas, corrige el paso (máximo ±8 %).

Vigilante de zona: con más de `areaLimitS` (10 s) dentro de un radio de 1 celda, el objetivo pasa a ser una celda alcanzable fuera de la zona. Si no hay camino, olvida las paredes poco confirmadas y vuelve a planificar.

## Parámetros

Todos están en `lib/raton_core/src/raton/params.h` y tienen nombre en `param_table.cpp`:
- en la placa: `set kpRot 0.02`, `get kpRot`, `params`, `save`;
- en el simulador: `--set kpRot=0.02`.

Se guardan en flash con CRC.

## Pruebas

- `tests/test_main.cpp`: mapa con votos, inundación, rutas, texto ↔ mapa, tabla de parámetros, filtro lateral, tramas, 6 misiones completas en el simulador (reglamento ideal y con errores, y 4 aleatorios) y 3 pistas de práctica de otras medidas (170, 200 y 260 mm).
- Simulador: `--bench 20` (competencia, 42 casos), `--practica 70` (pistas de 4×5 a 8×9 celdas de 170 a 300 mm), `--pista ancho,alto,celda,semilla,errores` (una pista de práctica con detalle).
- CMake también compila todo `firmware/` contra cabeceras mínimas (`tests/stubs`) para detectar errores sin la placa.
- La integración continua compila el firmware real con PlatformIO y corre la batería de laberintos.

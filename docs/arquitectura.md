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

Geometría con sensores a 45°:
- El rayo lateral toca la pared ~70 mm por delante del eje.
- Las paredes laterales de la celda siguiente se ven cuando el eje está entre 27 y 55 mm pasado el centro de la actual.
- Ahí se decide la acción para la celda siguiente, así no hay pausa para mirar ni pensar.

Qué hace con cada lectura:
- **Lateral:** se convierte en un voto para la pared donde cae el rayo, descartando ±20 mm alrededor de los postes. Si cae en la zona limpia de la pared, alimenta el filtro lateral. El cambio pared→hueco (el poste) corrige el avance.
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

Vigilante de zona: con más de `areaLimitS` (10 s) dentro de un radio de 1 celda, el objetivo pasa a ser una celda alcanzable fuera de la zona. Si no hay camino, olvida las paredes poco confirmadas y vuelve a planificar.

## Parámetros

Todos están en `lib/raton_core/src/raton/params.h` y tienen nombre en `param_table.cpp`:
- en la placa: `set kpRot 0.02`, `get kpRot`, `params`, `save`;
- en el simulador: `--set kpRot=0.02`.

Se guardan en flash con CRC.

## Pruebas

- `tests/test_main.cpp`: mapa con votos, inundación, rutas, texto ↔ mapa, tabla de parámetros, filtro lateral, tramas y 6 misiones completas en el simulador (reglamento ideal y con errores, y 4 aleatorios).
- CMake también compila todo `firmware/` contra cabeceras mínimas (`tests/stubs`) para detectar errores sin la placa.
- La integración continua compila el firmware real con PlatformIO y corre la batería de laberintos.

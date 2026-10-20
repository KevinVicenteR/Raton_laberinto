# Traspaso: estado del trabajo y cómo seguir

Documento para quien continúe (persona, Codex u otro agente). Resume qué se pidió, qué está hecho, cómo se
mide y qué falta. **Hablar siempre en español con el usuario.**

## Contexto

- Ratón de laberinto para SECIHTI 2026 (categoría "Robot ratón"): Arduino Nano 33 BLE Sense Rev2, 3 × VL53L0X
  (frontal y dos a 45°), DRV8833 con motores N20, giroscopio BMI270.
- Rama de trabajo: **`arquitectura_v2`**. No tocar ni fusionar en `sin_arrancador` (lo pidió el usuario).
- Lo que pide el usuario: que el ratón sea **autónomo y autosuficiente** (que se calibre y ajuste solo, en la
  competencia no se puede tocar) y **rápido** resolviendo pistas. Quejas vistas en el ratón real: señales
  falsas (en un pasillo vacío los sensores "ven algo delante"), retroceso y giro de calibración poco fluidos,
  se pegaba a las paredes, zigzag y no reconocía por dónde ya pasó.
- Medidas aproximadas del ratón (por fotos): láseres a ~40–50 mm del piso, metidos detrás de agujeros en una
  carcasa negra impresa en 3D; ruedas cerca de las esquinas delanteras con cola larga; menos de 10 cm. Faltan
  las medidas reales (ver `docs/calibracion.md`, "Medir una vez con una regla").

## Cómo compilar y medir

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j4
build/raton_tests                      # 142 comprobaciones, deben dar 0 fallos (~10 s)
scripts/banco.sh 80                    # 81 casos de competencia con errores de nivel 2 (~2,5 min, 4 núcleos)
SIM_H=25 scripts/banco.sh 80 --set sensorHeightMm=25   # sensores bajos
SIM_MOTOR=0.8 scripts/banco.sh 80      # motores un 20 % más lentos (batería baja)
build/raton_sim --practica 70          # pistas de práctica de cualquier medida (~10 min, 1 núcleo)
pio run                                # firmware (PlatformIO, ver platformio.ini)
```

Un caso concreto del banco con todo el registro: el caso `k` del banco es
`build/raton_sim --random <1000+k> --perturb 2 --seed <77+k> --simseed <5+k>` (añadir `--trace` para la pose
cada 0,1 s). Variables de diagnóstico (solo en el simulador, `RATON_DEBUG`):
- `SIM_CONTACTLOG=1`: errores de pose en cada choque;
- `SIM_LOCLOG=1`: error de pose real;
- `RADARDBG=<t>`: correcciones de pose del radar desde t durante 1,5 s;
- `VOTEDBG=<seg>`: votos de una pared (seg = `y*64+x` horizontal, `4096+y*64+x` vertical; ver `maze.h`);
- `CELLDBG=1`: aprendizaje del paso de celda;
- `PIVDBG=1`: estado al acabar cada giro en el sitio.

Errores del simulador: `SIM_H` (altura real de los láseres), `SIM_MOTOR`, `SIM_FLOOR` (reflejo del piso),
`SIM_NOVERT` (sin errores en lo vertical), `SIM_SOLO=<grupo>`; opciones `--fuera <mm>` (objetos fuera del
laberinto), `--sincal`, `--cono <grados>`.

## Resultados de referencia

Banco grande (`scripts/banco.sh 160`, 161 casos, láseres a 40 mm), tras la tanda de rapidez: 161 cumplen,
149 sin roce, meta explorando 63,8 s, exploración completa 118 s, carrera 16,4 s. Con 81 casos: láseres a
25 mm 81/81 (77 sin roce); motores al 80 % 81/81 (74). **Con 81 casos el resultado varía bastante con
cualquier cambio (el sistema es caótico): para decidir, 161 casos.**

### Banco de 81 casos antes de la tanda de rapidez

| Configuración | Cumplen | Sin roce | Paredes mal | Error de pose (avance/lateral/rumbo) |
|---|---|---|---|---|
| Láseres a 40 mm | 81 | 77 | 3 | 4,6 / 2,7 mm / 0,84° |
| Láseres a 25 mm | 80 | 77 | 8 | 4,6 / 2,6 mm / 0,86° |
| Motores al 80 % | 81 | 78 | 5 | 4,3 / 2,6 mm / 0,85° |

Antes de los cambios de esta tanda (con el modelo 3D del sensor ya en el simulador): 78/72, 67/61 y 72/54.
Tiempos medios con láseres a 40 mm: meta explorando 69 s, exploración completa 127 s, carrera 17 s.
Práctica (`--practica 70`): 70 de 70 pistas exploradas completas.

### Rapidez: probado

Dónde se va el tiempo explorando hasta la meta (caso 1005, 78 s): 21 s girando en el sitio, 5 s parado, 15 s
acelerando y frenando. En la carrera (24 s), 14 s en curvas a ~290 mm/s: la manda la rueda de fuera de
la curva, que llega a lo que da el motor.

Se quedó:
- radio de curva elegido por tiempo con los motores reales y `turnSpeed` solo como tope: carrera −4 %;
- `knownSpeed` (650): crucero rápido por rectas de celdas ya recorridas: exploración completa −4 %;
- empezar el giro en el sitio con el ratón casi parado (< 40 mm/s): −1,5 %.

Descartado (en 161 casos):
- `searchSpeed 500`: meta −11 %, pero fallos y roces en carrera;
- `runSpeed 900`: no cambia nada (ya manda el tope real de los motores, ~600 mm/s);
- coste de giro de la ruta 12 o 16 (en vez de 8): igual o peor;
- 95 % del motor para la rueda de fuera en curva (en vez de 85 %): −3 % de carrera pero muchos más roces;
- giros en el sitio más rápidos (630–720°/s): −4 % pero más zigzag y paredes mal;
- curvas sin parar explorando por celdas ya conocidas (`--smooth` para todo, o solo en lo conocido):
  −6 % a −13 % de tiempo, pero el mapa empeora mucho (150–370 paredes mal) y fallan carreras;
- girar con el ratón la posición y sus dudas en cada giro (el avance pasa a lateral): en teoría correcto,
  en la práctica 158/161. La duda de rumbo proporcional al ángulo girado: 157/161;
- corregir los láseres con el paso de celda aprendido (`rangeBiasFromCell 1`): se realimenta con el
  aprendizaje del paso y supone el grosor de pared nominal.

## Hecho en esta tanda (sin subir aún si este documento no está en el último commit)

1. **Modelo 3D del VL53L0X** en el simulador y en la sonda: piso, paredes de altura finita, lo que pasa por
   encima. Integración exacta en el ángulo vertical (sin sesgo de 15 a 45 mm de altura).
2. **Duda por el piso** en cada predicción, que se usa en las tolerancias del radar y en la pose.
3. **Aprendizaje en marcha** de la inclinación de cada láser y del reflejo del piso; se guarda al terminar.
4. **¿Pared o piso?** (`frontSeesWall`) en paradas por pared inesperada, atascos, comprobación tras girar y
   colocación con el frontal. Detector de "empuja la pared" solo por debajo de donde empieza el piso.
5. **Paso de celda aprendido en todos los modos** (pista que no mide lo nominal, ruedas) y corrección de los
   desfases de los láseres con ese paso (`rangeBias`).
6. **Motores saturados:** la consigna no se escapa (ya no hay atascos falsos a toda velocidad), velocidad
   máxima real con el modelo aprendido, y curvas de carrera limitadas a lo que da la rueda de fuera.
7. **Error en los giros en el sitio:** lo que patinaban las ruedas al girar se sumaba al lateral con el rumbo
   nuevo (2–3 cm de error tras una media vuelta). Corregido.
8. `PISTA_DISTINTA` lo confirman los laterales (un frontal sin calibrar ya no la dispara).
9. Firmware: guarda la inclinación y el piso aprendidos; compila.

## Pendiente (por orden)

1. **Rapidez** (petición del usuario): lo que queda es menos paradas al explorar. Las curvas sin parar en
   lo conocido ganan mucho tiempo pero empeoran el mapa: habría que entender por qué (pose tras la curva,
   votos con la pose dudosa) antes de activarlas. También: rutas en diagonal en la carrera.
2. **Movimiento fluido:** compensación de zona muerta gradual en `Motion::update` (ahora es todo o nada con
   `kS`), retroceso de los atascos más suave, barrido continuo en la calibración en vez de 6 paradas.
3. **Autoprueba al arrancar** (motores, encoders, giroscopio, láseres) y seguir funcionando con un láser roto.
4. **Chasis rectangular con eje adelantado** en el simulador (`Physical::rectBody` ya existe) cuando el usuario
   dé las medidas; comprobar que las curvas y los giros caben.
5. Casos que aún fallan: con láseres a 25 mm el caso 1046 choca en la carrera; mirar con `SIM_CONTACTLOG`.
6. Actualizar la tabla de resultados del README con un banco completo (`build/raton_sim --bench 160`, ~10 min).

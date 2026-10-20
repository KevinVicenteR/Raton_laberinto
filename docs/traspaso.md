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

## Resultados de referencia (banco de 81 casos, nivel 2)

| Configuración | Cumplen | Sin roce | Paredes mal | Error de pose (avance/lateral/rumbo) |
|---|---|---|---|---|
| Láseres a 40 mm | 81 | 77 | 3 | 4,6 / 2,7 mm / 0,84° |
| Láseres a 25 mm | 80 | 77 | 8 | 4,6 / 2,6 mm / 0,86° |
| Motores al 80 % | 81 | 78 | 5 | 4,3 / 2,6 mm / 0,85° |

Antes de los cambios de esta tanda (con el modelo 3D del sensor ya en el simulador): 78/72, 67/61 y 72/54.
Tiempos medios con láseres a 40 mm: meta explorando 69 s, exploración completa 127 s, carrera 17 s.
Práctica (`--practica 70`): 70 de 70 pistas exploradas completas.

Probado y descartado como valor por defecto: `searchSpeed 500` (meta explorando 61 s, pero 79/81 y 253 roces
en carrera).

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

1. **Rapidez** (petición del usuario): carrera y vuelta a casa más rápidas por camino conocido, curvas suaves
   también explorando por celdas ya conocidas, menos paradas.
2. **Movimiento fluido:** compensación de zona muerta gradual en `Motion::update` (ahora es todo o nada con
   `kS`), retroceso de los atascos más suave, barrido continuo en la calibración en vez de 6 paradas.
3. **Autoprueba al arrancar** (motores, encoders, giroscopio, láseres) y seguir funcionando con un láser roto.
4. **Chasis rectangular con eje adelantado** en el simulador (`Physical::rectBody` ya existe) cuando el usuario
   dé las medidas; comprobar que las curvas y los giros caben.
5. Casos que aún fallan: con láseres a 25 mm el caso 1046 choca en la carrera; mirar con `SIM_CONTACTLOG`.
6. Actualizar la tabla de resultados del README con un banco completo (`build/raton_sim --bench 160`, ~10 min).

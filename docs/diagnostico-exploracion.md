# Diagnóstico de exploración — 17 de octubre de 2026

Se trabajó sin red, sin commit/push y sin subir firmware a la placa. No se cambió
`firmware/mapa/competition_maze.h`. El repositorio ya tenía una reorganización extensa
con cambios preparados y sin preparar; se conservaron esos cambios.

## Causas y evidencia

**Competencia y aleatorio 101 a 192 mm: presupuesto insuficiente para mapear toda la pista.**
El modo 3 publicado tiene `use_goal=False`: explora todas las fronteras, no se dirige a
(4,2). Esa coordenada solo mide la primera llegada física. La sesión publicada de
competencia tenía realmente 120 s, no 600 s (`evaluation.run_job`).

Competencia, semilla de ruido 1, controlador mapped, 192 mm, antes de cambiar nada:

| Límite (s) | Primera llegada (s) | Fin (s) | Celdas leídas | Decisiones | Giros | Contactos |
| --- | --- | --- | --- | --- | --- | --- |
| 120 | ninguna | 119.995 | 45 | 59 | 37 | 0 |
| 600 | ninguna | 599.995 | 175 | 245 | 178 | 0 |
| 1200 | 639.230 | 727.305 | 190 | 286 | 214 | 0 |

Con 1200 s: 67,320 m recorridos; 537,685 s en `run_cells`, 189,280 s en
`turn_corner_track` y 0,340 s de lectura inicial. Los 121,540 s de `rotate` están
incluidos en los giros, no deben sumarse otra vez. Se reconstruyen al menos 327
transiciones de celda entre decisiones (más el tramo final). Hay 189 posiciones
diferentes de decisión y 97 decisiones adicionales sobre posiciones repetidas;
la máxima frecuencia es 5 en (3,5). Las 190 celdas leídas incluyen la última,
donde ya no hace falta decidir.

No hay eventos de choque ni abortos. Hay 12 intentos con desplazamiento menor de
1 mm, que suman 0,080 s, nunca consecutivos: no se activa `maxNoProgress=3`.
De los 286 tramos, 272 figuran completos y 14 interrumpidos; interrumpido no
significa atasco permanente. Hay 35 decisiones de media vuelta. Los callejones
reales más repetidos, excluida la salida, aparecen dos veces: (5,7), (4,7), (7,0),
(2,1) y (3,3); no aparece un ciclo que bloquee el descubrimiento de fronteras.
La poda de `MissionOrchestrator.prune` no se ejecuta en esta rama de mapeo completo.

El aleatorio 101 a 192 mm lee 177 celdas a 600 s; con 1200 s llega a 625,865 s y
completa 190 celdas a 686,465 s, sin contactos. Hace 271 decisiones, 199 giros y
recorre 64,464 m. Sus ocho intentos sin desplazamiento suman 0,060 s y tampoco son
consecutivos. Avance: 515,300 s; giros: 170,825 s. La física y el tiempo de giro
explican el coste de recorrer toda la pista; no se necesita aumentar velocidades
ni relajar sensores o seguridad para llegar.

**Aleatorio 101 a 140 mm: divergencia geométrica Python/C++.**
Antes termina a 76,080 s declarando `mapping_complete=True`, sin llegada, con solo
28 celdas, 38 decisiones y 26 giros. Poner 1200 s da exactamente el mismo resultado:
no era falta de tiempo. En observaciones iniciales hay tres paredes falsas que
cierran pasos reales: oeste de (18,2) a 1,8 s, este de (17,6) a 21,5 s y oeste
de (15,4) a 66,0 s. También hay lecturas abiertas falsas, que luego pueden
corregirse al aproximarse al obstáculo frontal.

Python conservaba `side_look_ahead=139.842 mm` (calculado para 192 mm) y
`cell_side_wall=96 mm`. El núcleo calcula ambos con el ancho del pasillo:
`sideAhead + (corridor/2 - sideLateral)/tan(sideAngle)` y
`int(max(45, 0.42*corridor))`. A 140 mm corresponden aproximadamente 96,571 mm y
58 mm. Con solo corregir la anticipación, la prueba termina todavía antes,
con tres celdas: es necesario corregir ambos parámetros juntos. Con ambos,
llega a 400,680 s y completa 190 celdas a 563,585 s, sin contactos, también con
el límite original de 600 s. Las falsas paredes que cerraban esos tres accesos
desaparecen; no se afirma que desaparezca todo el ruido de sensores.

**Fallo independiente en el núcleo C++: falsa demostración de ruta Adachi.**
El robot ideal carga directamente el dibujo de `competition_maze.h`. Desde
(18,0), norte, `exploreTo` llega a (4,2) en 76 celdas y 46 decisiones.
`mapTrack` lee las 190 celdas y regresa al inicio en 350 desplazamientos,
con todas las paredes comprobadas contra el dibujo. No falla el mapeo ideal.

La prueba completa `checkSolve` sí fallaba después: la salida ya podada quedaba
fuera del flood con `avoidDead=true`; `unseenOnRoute` devolvía false y Adachi
anunciaba inmediatamente «ruta demostrada». Después del regreso quedaba una
ruta optimista de 28 celdas frente a una mínima real de 32, y la carrera fallaba.
La poda depende de los destinos temporales (`keep`); no puede usarse como prueba
de que no quedan incógnitas en la ruta salida-meta. Se excluye la poda del flood
para demostrar y ejecutar rutas. La exploración conserva su poda y su alternativa
sin poda para salir de callejones.

El límite operativo de 12 objetivos Adachi sigue siendo un límite explícito:
no garantiza demostrar cualquier pista. `checkSolve` usa un presupuesto de
ancho*alto para exigir prueba completa; con él, comprueba ausencia de celdas sin
ver en la ruta y carrera de exactamente 32 celdas. No se elevó ese límite en el
firmware para ocultar el fallo. La regresión de `unseenOnRoute` prueba por separado
que una salida podada no permite declarar la ruta demostrada.

Python y firmware aún no son simulaciones idénticas de misión: Python selecciona
fronteras por aristas observadas y termina al agotar fronteras; C++ usa su rejilla
optimista y además vuelve al inicio. Python no ejecuta la búsqueda Adachi de C++.
Por eso el fallo Adachi encontrado no causa los tiempos de los JSON de modo 3,
y los 350 desplazamientos C++ no deben interpretarse como un tiempo físico Python.
No se reescribió ninguna de estas misiones.

## Cambios

- `simulator/python/raton_sim/simulator.py`: anticipación y umbral lateral según
  geometría configurada, también al medir el pasillo; fórmulas del núcleo C++.
- `simulator/python/raton_sim/evaluation.py`: 1200 s para modo 3 de mapeo completo;
  publica `simulation_limit_s`, `mapping_complete` y `cells_mapped`.
- `lib/raton_core/src/mapa/cell_map.cpp`: prueba optimista de ruta sin excluir
  callejones podados para objetivos anteriores.
- `lib/raton_core/src/exploracion/cell_explorer.cpp`: carrera sin esa exclusión.
- `tests/cpp/test_main.cpp`: competencia desde la salida correcta, lectura directa
  del header, `checkSolve`, `mapTrack`, ruta mínima, paredes y regresión de falsa prueba.
- `tests/python/test_sim_cells.py`: geometría por ancho y tras medición.
- `web/public/sim/`: mismas 20 sesiones, regeneradas localmente con `publicar`.

No se cambian velocidades, pared de competencia, semillas ni protecciones de giro.

## Comparación de las sesiones publicadas

Tiempos en segundos. «Fin después» es fin del mapeo en modo 3 y primera llegada en
los otros modos. Antes: límites de publicación originales (120 s competencia,
600 s aleatorios); después: 1200 s solo para modo 3. Competencia 168 mm es una
prueba adicional cuyo antes usa 600 s. Todos los contactos eran y siguen siendo cero.

| Pista | Modo | Pasillo mm | Llegada antes | Llegada después | Fin después | Contactos después |
| --- | --- | --- | --- | --- | --- | --- |
| Competencia | 4 | 140 | sin llegada | sin llegada | 26.01 | 0 |
| Competencia | 4 | 168 | 41.11 | 41.11 | 41.11 | 0 |
| Competencia | 4 | 192 | 44.19 | 44.19 | 44.19 | 0 |
| Competencia | 4 | 220 | 47.71 | 47.71 | 47.71 | 0 |
| Competencia | 3 | 192 | sin llegada | 639.23 | 727.30 | 0 |
| Competencia (prueba adicional) | 3 | 168 | 595.14 | 583.97 | 653.06 | 0 |
| Competencia alterada | 4 | 192 | 61.74 | 61.74 | 61.74 | 0 |
| Aleatorio 100 | 3 | 140 | 429.31 | 365.61 | 503.02 | 0 |
| Aleatorio 100 | 3 | 168 | 408.85 | 409.28 | 562.54 | 0 |
| Aleatorio 100 | 3 | 192 | 445.53 | 445.53 | 613.04 | 0 |
| Aleatorio 100 | 3 | 220 | 489.60 | 485.43 | 709.74 | 0 |
| Aleatorio 101 | 3 | 140 | sin llegada | 400.68 | 563.58 | 0 |
| Aleatorio 101 | 3 | 168 | 449.09 | 482.80 | 659.32 | 0 |
| Aleatorio 101 | 3 | 192 | sin llegada | 625.86 | 686.46 | 0 |
| Aleatorio 101 | 3 | 220 | 535.79 | 550.61 | 805.22 | 0 |
| Aleatorio 102 | 3 | 192 | 511.60 | 511.60 | 705.34 | 0 |
| Aleatorio 103 | 3 | 192 | 508.56 | 508.56 | 738.34 | 0 |
| Aleatorio 104 | 3 | 192 | 451.01 | 451.01 | 618.02 | 0 |
| Aleatorio 105 | 3 | 192 | 244.52 | 244.52 | 682.48 | 0 |
| Aleatorio 106 | 5 | 192 | 74.51 | 74.51 | 74.51 | 0 |
| Aleatorio 107 | 5 | 192 | 83.23 | 83.23 | 83.23 | 0 |

A igual presupuesto de 600 s, todos los casos a 192 mm tienen idénticas trazas,
llegadas y celdas antes/después; su nueva finalización se debe exclusivamente al
presupuesto. Competencia a 168 mm mejora de 180 a 184 celdas a 600 s y llega
11,170 s antes. Después completa 190 celdas a 653,065 s.

Empeoramientos de tiempo de primera llegada: aleatorio 100/168 +0,430 s,
101/168 +33,705 s y 101/220 +14,825 s. Se mantienen sus llegadas sin contactos.
El resto no pierde llegadas ni gana contactos. El modo 4 a 140 mm mantiene su
fallo previo `barrido bloqueado por paredes observadas` a 26,015 s: no se corrigió
ni se ocultó ese bloqueo físico/geométrico del giro. Resultado publicado: 19 de
20 llegadas limpias; no se declara resuelto el conjunto completo.

## Verificación y reproducción

Variables: `PYTHONPATH=$PWD/simulator/python`,
`RATON_NATIVE_LIB=$PWD/build/libraton_navigation.dylib`, Python `.venv/bin/python`.

- `cmake --build build && ctest --test-dir build --output-on-failure`: 1/1 pasa.
- `.venv/bin/python -m pytest tests/python -q`: 43 pasan.
- `.venv/bin/ruff check simulator/python tests/python`: pasa, configuración raíz.
- `.venv/bin/ruff format --check simulator/python tests/python`: 26 archivos conformes.
- `.venv/bin/python -m raton_sim publicar`: 20 resultados y 20 sesiones locales.
- `cd web && npm test`: 6 archivos, 21 pruebas pasan, incluido catálogo publicado.
- `.venv/bin/pio run -e raton`: SUCCESS; RAM 97.096/262.144 bytes (37,0%),
  flash 410.204/983.040 bytes (41,7%). La invocación normal inicial falló al
  escribir `~/.platformio/platforms.lock` por las restricciones del entorno.
  Se compiló con `PLATFORMIO_CORE_DIR=/private/tmp/raton-pio-offline`, enlaces a
  plataformas/paquetes ya instalados, telemetría deshabilitada y `sitecustomize.py`
  temporal que bloquea conexiones. No se instalaron dependencias ni se subió nada.
  La última invocación exacta de `pio run -e raton` confirmó SUCCESS en 4,82 s;
  la compilación previa completa tardó 108,76 s.

Evidencia detallada local (ignorada por git): `simulator/results/diagnostico/`.
`before-summary.json` y `after-summary.json` guardan la matriz con iguales límites;
los JSON por caso contienen decisiones, observaciones, maniobras y eventos completos.
`dashboard-before.json` conserva los resultados publicados originales.
`run.py` reproduce la matriz: `.venv/bin/python simulator/results/diagnostico/run.py <etiqueta>`.
`after-competition-0-3-168-1200.json` completa la prueba adicional. Los logs
`publicar.log`, `cpp-before.log`, `cpp-search-full.log` y `firmware-final.log`
conservan evidencia de publicación, fallo/regresión C++ y compilación.

Comprobación final automática: las 13 ejecuciones a 192 mm con iguales límites
tienen JSON de diagnóstico idénticos antes/después, incluidas decisiones,
observaciones y maniobras. Se verificaron los mismos 20 nombres de sesión y
190 celdas con mapeo completo en todos los modos 3 publicados.

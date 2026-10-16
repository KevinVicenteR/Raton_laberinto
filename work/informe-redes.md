Se generaron datos, se entrenaron seis candidatos y se evaluaron dos redes únicamente en simulación. No se modificó `tools/nets.py`; no fue necesario corregirlo. Se usó la biblioteca nativa existente, sin recompilar. No hubo cargas a hardware, commits ni push.

**Datos y selección.** Cada red contiene 25 000 ejemplos: 20 000 de entrenamiento (80 laberintos, semillas 1000–1079) y 5 000 de validación (20 laberintos, 1100–1119). Se verificó que los conjuntos son disjuntos y excluyen las semillas de prueba. Todos los candidatos se entrenaron durante 150 épocas, semilla 42, conservando los pesos de menor pérdida de validación. La selección usó exclusivamente validación; dentro del 3 % de la mejor pérdida se prefirió la red menor.

| Red | Capas ocultas | Pérdida de validación | Precisión lógica | MAE rumbo (°) | MAE factor de velocidad |
|---|---:|---:|---:|---:|---:|
| Lógica | 24×12 (elegida) | 0.621977 | 63.72 % | — | — |
| Lógica | 32×16 | 0.615212 | 68.56 % | — | — |
| Lógica | 48×24 | 0.621262 | 67.24 % | — | — |
| Control | 32×16 (elegida) | 0.569706 | — | 2.207094 | 0.251854 |
| Control | 48×24 | 0.573624 | — | 2.194170 | 0.246442 |
| Control | 64×32 | 0.572026 | — | 2.212950 | 0.252207 |

Se eligieron **lógica 36→24→12→4** y **control 43→32→16→2**. La pérdida de lógica 24×12 supera la mejor en solo 1,10 %; control 32×16 tiene la menor pérdida. El MAE de velocidad mide el factor adimensional aplicado a la velocidad (rango 0,3–1,0), no mm/s. Los modelos están en `tools/models/logic_model.json` y `tools/models/control_model.json`, con sus encabezados exportados en `lib/raton_core/src/`; ambos conservan `enabled=false`.

**Evaluación lógica sin física.** En 120 ejecuciones (60 laberintos, ruido de lectura 0 % y 2 %), la base llegó 108 veces y la red 110. En las 107 ejecuciones donde ambas llegaron, el tiempo estimado medio fue 95.51 s para la base y 91.88 s para la red; esta fue más rápida en 38, más lenta en 28 e igual en 41. Son costes discretos de acciones, sin física ni medición de contactos.

**Evaluación con física.** Llegada limpia = meta alcanzada y cero contactos. El tiempo medio incluye únicamente las llegadas de cada controlador (no necesariamente los mismos casos); contactos suma todos los casos. `mapped-ctrl` activa solo control, `mapped-logic` solo lógica y `mapped-both` ambas para estas pruebas, aunque los artefactos exportados permanezcan deshabilitados.

**Suite 21** (84 ejecuciones completas).

| Controlador | Ejecuciones | Llegadas limpias | Llegadas totales | Tiempo medio llegadas (s) | Contactos totales |
|---|---:|---:|---:|---:|---:|
| mapped | 21 | 11 | 17 | 70.37 | 12 |
| mapped-ctrl | 21 | 9 | 9 | 68.07 | 10 |
| mapped-logic | 21 | 7 | 13 | 73.27 | 12 |
| mapped-both | 21 | 9 | 9 | 73.13 | 8 |

**Suite reserved** (720 ejecuciones completas).

| Controlador | Ejecuciones | Llegadas limpias | Llegadas totales | Tiempo medio llegadas (s) | Contactos totales |
|---|---:|---:|---:|---:|---:|
| mapped | 180 | 80 | 114 | 70.09 | 88 |
| mapped-ctrl | 180 | 74 | 94 | 78.37 | 94 |
| mapped-logic | 180 | 48 | 86 | 81.00 | 98 |
| mapped-both | 180 | 44 | 66 | 85.30 | 110 |

**Suite fresh** (720 ejecuciones completas).

| Controlador | Ejecuciones | Llegadas limpias | Llegadas totales | Tiempo medio llegadas (s) | Contactos totales |
|---|---:|---:|---:|---:|---:|
| mapped | 180 | 60 | 108 | 79.34 | 140 |
| mapped-ctrl | 180 | 72 | 92 | 90.83 | 100 |
| mapped-logic | 180 | 48 | 74 | 76.51 | 140 |
| mapped-both | 180 | 46 | 58 | 85.40 | 118 |

Las suites `reserved` y `fresh` usan respectivamente semillas 200–229 y 400–429, tres semillas de ruido y modos 3 y 5: 180 ejecuciones por controlador. La suite 21 combina competición, pista alterada y laberintos 100–107. Los hashes de los cuatro archivos del simulador registrados al inicio de cada suite coinciden con los actuales. Los resultados completos están en `work/eval-logic.json` y `work/eval-nets-{21,reserved,fresh}.json`.

**Recomendación por red.**

- **Control: mantener deshabilitada.** En `reserved` reduce las llegadas limpias de 80 a 74 y aumenta los contactos de 88 a 94. En `fresh` mejora las limpias de 60 a 72 y reduce contactos de 140 a 100, pero no cumple el criterio en ambos conjuntos.
- **Lógica: mantener deshabilitada.** En `reserved` reduce las limpias de 80 a 48 y aumenta contactos de 88 a 98; en `fresh` reduce las limpias de 60 a 48 con los mismos 140 contactos. La mejora discreta de `eval-logic` no se traduce en mejora física.

La combinación tampoco justifica habilitarlas: en `reserved` produce 44 llegadas limpias y 110 contactos. Estos resultados proceden exclusivamente de simulaciones y no demuestran rendimiento en hardware.

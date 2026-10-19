# Entrenamiento y parámetros

Los parámetros de control y los umbrales de pared de `firmware/<función>/params.h` se ajustan en el simulador
(`python -m raton_sim entrenar`, CMA-ES con currículo y validación reservada). Los resultados son del
modelo, no una garantía del robot real: cada cambio se confirma después en la pista.

## Historial resumido

| Ronda | Qué se entrenó | Resultado en validación reservada | Se llevó al firmware |
|---|---|---|---|
| Geometría real | Umbral lateral, anticipación y parada frontal con el cuerpo 80×80 y laterales en diagonal | 7/21 llegadas limpias (suite 21) | Sí: geometría y paradas |
| Entrenamiento robusto | 10 candidatos en anchos de 120 a 300 mm y factores extremos | Mejora parcial; persisten abortos | No |
| CMA-ES + dos orquestadores | 12 parámetros (margen, velocidad, KP_WALL, KP_YAW, banda muerta, roce, torcido, reintentos…) | ghost 0→9/14, stale 0→12/14, start_wall 0→12/14; combinado 0→1/14 | Sí: los 12 parámetros |
| Conocimiento externo | 17 parámetros + estrategia (coste de giro, Adachi, parada con ruta demostrada, regreso, poda) | 100→101/136 limpias, búsqueda 231→196 s, contactos 13→17 | Solo la estrategia: Adachi 1 + regreso explorando (con poda) |

Los informes completos, curvas y checkpoints de cada ronda están archivados fuera del repositorio
(`../Raton_laberinto_archivo/work_2026-10-17.tar.gz`).

## Parámetros de la ronda histórica

- `FRONT_MARGIN_MM=20`, `EXPLORE_SPEED=180`, `COMP_SPEED=300`, `ACCEL=1000`, `KP_WALL=0.12`, `KP_YAW=0.0119`, `LATERAL_DEADBAND_MM=8.5`,
  `RUB_MS=60`, `SKEW_LIMIT_DEG=10`, `MAX_RUBS=2`, `TURN_ATTEMPTS=2`, `MAX_NO_PROGRESS=3`.
- `SIDE_WALL_FACTOR=0.42`, `FRONT_WALL_FACTOR=0.28`.
- `ADACHI_SEARCH=true`, `SEARCH_MAX_TARGETS=12`.

Los parámetros ganadores de la última ronda (velocidad 158,7 mm/s, margen 27,5 mm…) no se portaron
porque aumentaban los contactos.

## Ajuste de movimiento de 275b705 (17/10/2026)

`KD_YAW=0.0012`, `WALL_RATE_DAMP=0.035`, `YAW_TARGET_SMOOTH=0.12`,
`MAX_WALL_CORRECTION_DEG=3`, `MAX_STEER=0.14`. Se mantiene la banda muerta de 8.5 mm.
La tasa lateral filtrada amortigua la cascada y la ganancia lateral disminuye al subir velocidad.
Se corrigió también el signo del error inicial de rumbo respecto a la rejilla y el límite inferior
del PI que impedía frenar.

Giros: `TURN_RATE_DPS=220`, `TURN_ACCEL_DPS2=900`, `TURN_PWM_MAX=0.45`,
`TURN_TOLERANCE_DEG=3`, `TURN_RETRY_DEG=3`, `TURN_FAIL_DEG=3`. El mínimo de arranque solo actúa
en los primeros 200 ms, a más de 12° del objetivo; la compensación de fricción decae cerca de él.
Se mantiene el freno al terminar. `SETTLE_MS=15`, `POST_TURN_SAMPLES=2`,
`FRONT_STABLE_SAMPLES=5`, `CALIBRATION_SAMPLES=7`; la calibración solo se repite si hubo que centrar.

[Auditoría y tabla completa](diagnostico-movimiento.md): diez llegadas nominales sin contactos en
192/168 mm. **No aprobado en perturbaciones combinadas: contactos 10→11, llegadas 7/10→3/10.**
Son parámetros propuestos, sin validación física ni margen adverso demostrado; no se ha tocado la placa.
El sobrepaso nominal simulado sube de 0 a ~0.1°. No se presentan estos resultados como una mejora
global de seguridad. Los resultados están archivados en `simulator/results/motion-audit/`.

## Redes neuronales

Las redes de control, de lógica y de riesgo se retiraron del firmware y del simulador en la
reorganización: estaban desactivadas y en simulación empeoraban las llegadas (p. ej. lógica 80→48
llegadas limpias en la suite reservada). Siguen en el historial de git. Las opciones que sí tienen
sentido (clasificador de paredes con probabilidades, modelo del robot real para el simulador) se
discuten antes de implementarlas.

## Fuentes

Reglamentos, hojas de datos y estrategias de competidores: [fuentes.md](fuentes.md).

## Reentrenamiento de robustez posterior a 275b705

El ejecutable reproducible es:

```sh
export RATON_NATIVE_LIB="$PWD/build/libraton_navigation.dylib"
.venv/bin/python -m raton_sim.entrenamiento.robust_motion ablate
.venv/bin/python -m raton_sim.entrenamiento.robust_motion train --minutes 70 --generations 24 --population 8 --workers 6
.venv/bin/python -m raton_sim.entrenamiento.robust_motion select
# Portar los parametros de selected.json a firmware/<función>/params.h antes de comparar.
.venv/bin/python -m raton_sim.entrenamiento.robust_motion compare
.venv/bin/python -m raton_sim.entrenamiento.robust_motion check
```

Las referencias se extraen con `git show ea39fae:firmware/config.h` y
`git show 275b705:firmware/config.h`; no se cambia de checkout. El PD antiguo y
sus esperas se reconstruyen en la misma planta angular que el controlador nuevo.
`motion_audit --legacy` también lee ahora la configuración histórica, de modo que
un KP_YAW nuevo no contamine accidentalmente la referencia antigua.

CMA-ES ajusta 13 parámetros de velocidades, control de rumbo/centrado y adaptación.
Usa semilla sensorial 11001 y laberintos aleatorios 11000–11002 para entrenar el
combinado; la selección usa semilla 22001 y laberintos 22000–22002. Los casos
nominales de validación también usan esas semillas reservadas. El conjunto nominal
original (sensor 1, laberintos 100–102) forma parte de las restricciones de entrenamiento.
La validación selecciona el checkpoint, pero no alimenta las actualizaciones de CMA.
La selección final vuelve a exigir factibilidad tanto en entrenamiento como en
validación y descarta `ADAPT_UNKNOWN_RATIO=1`: entrar en terreno nuevo debe reducir
la consigna aun con observaciones estables. Las pruebas de selección rechazan
explícitamente un mutante rápido inseguro y un mutante que elimina esa reducción.

Una violación nunca se compensa con tiempo: combinado con menos de 7/10 llegadas
o más de 10 contactos, o cualquier contacto/fallo nominal, recibe una penalización
superior a cualquier tiempo admisible. Entre candidatos factibles se minimiza el
tiempo medio, asignando 600 s a cada no llegada para no premiar abandonar las rutas
lentas. Se conserva también el candidato inicial si supera a todos los mutantes.
Los JSON de parámetros, curvas, protocolo, observaciones y maniobras están en
`simulator/results/motion-audit/robust/`.

El criterio duro corresponde al banco combinado original que se está reparando.
La ablación ampliada informa además de limitaciones preexistentes (encoder perdido,
frontal retenido, lateral muerto, etc.); no sustituye silenciosamente ese banco por
otro más fácil. Véase [el diagnóstico](diagnostico-movimiento.md) para distinguir
las dos inyecciones de error de giro y la corrección del ángulo lateral oculto.

## Velocidad adaptativa y reentreno robusto (17-10-2026)

Ronda para corregir la regresión del commit 275b705 (más rápido, pero con perturbaciones combinadas
llegaba 3/10). Se añadió la velocidad adaptativa (`lib/raton_core/src/velocidad/adaptive_speed.h`, constantes
`ADAPT_*`) y se reentrenaron 13 parámetros con CMA-ES (13 generaciones, 70 min) con restricciones duras:
con perturbaciones combinadas ≥ 7/10 llegadas y ≤ 10 contactos, y 0 contactos en los casos nominales.
Se eligió el checkpoint de la generación 10 por la validación reservada
(`python -m raton_sim.entrenamiento.robust_motion select`).

| Banco (10 casos cada uno) | ea39fae | 275b705 | Nuevo |
|---|---|---|---|
| Nominal, llegadas / contactos / meta media | 10 / 0 / 182 s | 10 / 0 / 94 s | 10 / 0 / 85 s |
| Nominal reservado | 10 / 0 / 209 s | 10 / 0 / 107 s | 10 / 0 / 97 s |
| Perturbaciones combinadas | 7 / 6 / 331 s | 3 / 9 / 455 s | 10 / 0 / 89 s |
| Perturbaciones combinadas reservadas | 6 / 16 / 371 s | 5 / 6 / 356 s | 10 / 0 / 101 s |

Parámetros portados: `EXPLORE_SPEED=230`, `COMP_SPEED=368.8`, `KP_WALL=0.353`, `KP_YAW=0.025`,
`KD_YAW=0.00124`, `MAX_WALL_CORRECTION_DEG=7.01`, `LATERAL_DEADBAND_MM=3.47`, `WALL_RATE_DAMP=0.0455`,
`ADAPT_MIN_RATIO=0.832`, `ADAPT_UNKNOWN_RATIO=0.977`, `ADAPT_HEADING_DEG=6.91`, `ADAPT_CAUTION_MM=63.7`,
`ADAPT_FRONT_MM=60`. Pendiente de confirmar en la pista real.

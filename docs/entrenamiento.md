# Entrenamiento y parámetros

Los parámetros de control y los umbrales de pared de `firmware/config.h` se ajustan en el simulador
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

## Parámetros vigentes

- `FRONT_MARGIN_MM=20`, `EXPLORE_SPEED=132`, `KP_WALL=0.185`, `KP_YAW=0.0119`, `LATERAL_DEADBAND_MM=8.5`,
  `RUB_MS=60`, `SKEW_LIMIT_DEG=10`, `MAX_RUBS=2`, `TURN_ATTEMPTS=3`, `MAX_NO_PROGRESS=3`.
- `SIDE_WALL_FACTOR=0.42`, `FRONT_WALL_FACTOR=0.28`.
- `ADACHI_SEARCH=true`, `SEARCH_MAX_TARGETS=12`.

Los parámetros ganadores de la última ronda (velocidad 158,7 mm/s, margen 27,5 mm…) no se portaron
porque aumentaban los contactos.

## Redes neuronales

Las redes de control, de lógica y de riesgo se retiraron del firmware y del simulador en la
reorganización: estaban desactivadas y en simulación empeoraban las llegadas (p. ej. lógica 80→48
llegadas limpias en la suite reservada). Siguen en el historial de git. Las opciones que sí tienen
sentido (clasificador de paredes con probabilidades, modelo del robot real para el simulador) se
discuten antes de implementarlas.

## Fuentes

Reglamentos, hojas de datos y estrategias de competidores: [fuentes.md](fuentes.md).

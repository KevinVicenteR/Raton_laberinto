# Simulador del ratón

Paquete Python 3.11 con simulación física, sensores, navegación determinista,
mapa observado y entrenamiento CMA-ES. No requiere instalación ni red:

```sh
export PYTHONPATH="$PWD/simulator/python"
export RATON_NATIVE_LIB="$PWD/build/libraton_navigation.dylib" # .so en Linux
.venv/bin/python -m raton_sim --help
.venv/bin/python -m raton_sim casos
.venv/bin/python -m raton_sim pistas
.venv/bin/python -m raton_sim evaluar --controllers corrected,mapped
.venv/bin/python -m raton_sim publicar
.venv/bin/python -m raton_sim pesos 4
.venv/bin/python -m raton_sim entrenar --smoke --generations 1 --population 2 --workers 2
```

`casos`, `evaluar` y `entrenar` aceptan `--out`; sus destinos por defecto son
`simulator/results/casos/`, `simulator/results/evaluacion.json` y
`simulator/results/entrenamiento/`. `pistas` y `pesos` imprimen resultados.
Las salidas nunca se guardan dentro del paquete. Solo entrenar necesita NumPy
(extra `training` en `pyproject.toml`).

`publicar` reemplaza los JSON, HTML y SVG de `web/public/sim/` por el catálogo
y sus sesiones. Sin argumentos evalúa tres ejemplos con `mapped`. Para publicar
una evaluación completa: `publicar --evaluation <archivo> --controller mapped`.
El catálogo enlaza sesiones como `sim/<archivo>.json`.

Controladores: `baseline` (`original` es su alias), `corrected`, `precision`,
`hybrid`, `survey`, `mapped` y `mapped-raw`. El alias original usa la física
actual con el control histórico básico, sin cargar versiones por rutas.
`mapped` usa únicamente paredes observadas, encoders y giroscopio; la geometría
real se emplea para simular contactos y para medir los resultados.

La configuración se lee de `firmware/config.h` expandiendo los `params.h` por función; se mantienen los valores antiguos
70°, 75 mm y 25 mm de giro dividido y retroceso, y los valores de respaldo de
geometría y umbrales que ya no existen en el firmware. La biblioteca nativa solo
necesita `raton_turn`. No se cargan redes ni modelos de riesgo.

Entrenamiento: `entrenamiento/benchmark.py` define los dominios aleatorizados;
`cmaes.py` contiene CMA-ES con covarianza completa y el currículo de cuatro
etapas; `knowledge.py` añade búsqueda, regreso y carrera con mapa observado;
`entrenamiento/train.py` ejecuta el currículo de cinco etapas y conserva validación y test
separados. Conserva en `train.INITIAL_PARAMS` los parámetros del checkpoint inicial histórico,
sin depender de resultados de ejecuciones anteriores.
`--generations` limita el total y `--per-stage` el máximo por etapa. El humo
reduce los dominios, por lo que no constituye una evaluación de rendimiento.

Pruebas y estilo, desde la raíz del repositorio:

```sh
RATON_NATIVE_LIB=build/libraton_navigation.dylib .venv/bin/python -m pytest tests/python -q
.venv/bin/ruff check --no-cache --config simulator/python/pyproject.toml simulator/python tests/python
.venv/bin/ruff format --no-cache --check --config simulator/python/pyproject.toml simulator/python tests/python
```

#!/bin/sh
# Ejecutar desde la raiz del repositorio, despues de calibrar.py y refinar.py.
set -eu
RATON_CELL_SIDE_WALL_MM=$(python3 -c 'import json;print(json.load(open("work/geometria/calibracion_final.json"))["selected"]["threshold"])')
RATON_SIDE_LOOK_AHEAD_MM=$(python3 -c 'import json;print(json.load(open("work/geometria/calibracion_final.json"))["selected"]["look"])')
RATON_CELL_FRONT_STOP_MM=$(python3 -c 'import json;print(json.load(open("work/geometria/calibracion_final.json"))["selected"]["effective_stop"])')
RATON_NATIVE_LIB="$PWD/work/geometria/libraton_navigation.dylib"
export RATON_CELL_SIDE_WALL_MM RATON_SIDE_LOOK_AHEAD_MM RATON_CELL_FRONT_STOP_MM RATON_NATIVE_LIB
python3 -B tools/sim_pistas.py > work/geometria/sim_pistas.log 2>&1
python3 -B tools/navigation_experiments.py evaluate --baseline tools/sim_extremos.py --controllers mapped --suite 21 --out work/geometria/suite21.json --workers 4 > work/geometria/suite21.log 2>&1
python3 -B tools/navigation_experiments.py evaluate --baseline tools/sim_extremos.py --controllers mapped --suite reserved --out work/geometria/reserved.json --workers 4 > work/geometria/reserved.log 2>&1
python3 -B -m unittest discover -s tests -p 'test_sim_*.py' > work/geometria/tests.log 2>&1

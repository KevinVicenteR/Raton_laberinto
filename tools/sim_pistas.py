#!/usr/bin/env python3
"""Banco de pruebas de pistas completas (19 x 10, pasillos de 14 cm) con la logica del firmware:

  - Pista de la competencia con el mapa correcto (modo 4: ruta por tramos).
  - Pista de la competencia con una pared distinta de la del mapa (modo 4 debe pasar a buscar la meta).
  - Laberintos aleatorios que el raton no conoce: modo 3 (explorar por celdas con inundacion), comparado
    con modo 5 con meta (espacio libre) y modo 1 (seguir pared).

Uso:  python3 tools/sim_pistas.py
Requiere compilar raton_navigation con CMake. Si no esta en build/, indicar RATON_NATIVE_LIB.
"""

import os
import sys
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sim_extremos as S  # noqa: E402

from navigation import firmware_config
FW = firmware_config(S)
START, GOAL = (18, 0), (4, 2)


def altered_comp():
    """Pista de la competencia con un pasillo de la ruta cerrado (el mapa del raton no lo sabe)."""
    m = [list(l) for l in S.comp_maze()]
    # Cierra el paso hacia el oeste desde la celda (16,3), en mitad de la ruta mas corta.
    r = 10 - 1 - 3
    m[2 * r + 1][4 * 16] = '|'
    return [''.join(l) for l in m]


def job(args):
    name, maze, mode, use_goal, known, seed, secs = args
    case = S.Case(name, maze, START, GOAL, mode=mode, use_goal=use_goal, known=known, fw=FW, seconds=secs, controller="corrected")
    sim = S.Sim(case, seed=seed).run()
    return name, sim.reached, sim.collisions, sim.turns, sim.failure


def main():
    comp = S.comp_maze()
    jobs = []
    for seed in (1, 2, 3):
        jobs.append((f"Competencia, modo 4 (semilla {seed})", comp, 4, False, None, seed, 120))
    for seed in (1, 2):
        jobs.append((f"Competencia con pared cambiada, modo 4 (semilla {seed})", altered_comp(), 4, False, comp, seed, 300))
    for k in range(8):
        maze = S.random_maze(seed=100 + k)
        jobs.append((f"Aleatorio {k}: modo 3 celdas", maze, 3, False, None, 1, 600))
        jobs.append((f"Aleatorio {k}: modo 5 con meta", maze, 5, True, None, 1, 600))
    with ProcessPoolExecutor() as ex:
        for name, reached, col, turns, failure in ex.map(job, jobs):
            res = f"llega en {reached:5.1f} s" if reached else "NO llega     "
            print(f"{name:52s} {res}  choques {col:3d}  giros {turns:3d}"
                  + (f"  motivo: {failure}" if failure else ""), flush=True)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Optimiza los pesos de la decision en los cruces (modo 5) con el simulador.

Prueba combinaciones de [espacio, novedad, giro, sesgo recto] en varios laberintos y se queda con la que
llega antes, con menos choques y, si no llega, mas cerca de la meta. Uso:
    python -m raton_sim pesos [numero_de_candidatos]
Copia el resultado a W_FREE, W_NOVEL, W_TURN y B_STRAIGHT en firmware/config.h (o envialo desde la web con n).
"""

import argparse
import random
from collections import deque
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace

from . import simulator as S

BASE_FW = replace(
    S.Firmware(), cfg_side_ahead=56, cfg_front_ahead=33, turn_mode="dos_tiempos", unstick=True, weighted=True
)


def maze_dist(maze, goal):
    """Distancia en celdas a la meta para cada celda (BFS sobre el dibujo)."""
    rows, cols = len(maze) // 2, (len(maze[0]) - 1) // 4

    def wall(x, y, d):
        r = rows - 1 - y
        if d == "N":
            return maze[2 * r][4 * x + 2] == "-"
        if d == "S":
            return maze[2 * r + 2][4 * x + 2] == "-"
        if d == "W":
            return maze[2 * r + 1][4 * x] == "|"
        return maze[2 * r + 1][4 * x + 4] == "|"

    dist = {tuple(goal): 0}
    q = deque([tuple(goal)])
    while q:
        x, y = q.popleft()
        for d, (dx, dy) in (("N", (0, 1)), ("S", (0, -1)), ("E", (1, 0)), ("W", (-1, 0))):
            n = (x + dx, y + dy)
            if wall(x, y, d) or n in dist or not (0 <= n[0] < cols and 0 <= n[1] < rows):
                continue
            dist[n] = dist[(x, y)] + 1
            q.append(n)
    return dist


SEEDS = (1, 2, 3)  # varias semillas: unos pesos que solo funcionan con una no sirven


def cases(w):
    fw = replace(BASE_FW, w=tuple(w))
    comp = S.comp_maze()
    small = [
        ("T", S.M_T, (2, 0), (0, 2), {}),
        ("callejon", S.M_DEAD, (1, 0), (0, 2), {}),
        ("L 14", S.M_L, (2, 0), (0, 2), {}),
        ("L ancha", S.M_L, (2, 0), (0, 2), dict(corridor=200)),
    ]
    out = [
        (S.Case(n, m, st, g, fw=fw, seconds=40, **kw), seed, True)
        for n, m, st, g, kw in small
        for seed in SEEDS
    ]
    out.append((S.Case("reglamento", comp, (18, 0), (4, 2), fw=fw, seconds=120), 1, False))
    return out


def evaluate(w):
    total, detail = 0.0, {}
    for case, seed, must_arrive in cases(w):
        sim = S.Sim(case, seed=seed).run()
        if sim.reached is not None:
            cost = sim.reached + 4 * sim.collisions
        else:
            dist = maze_dist(case.maze, case.goal)
            cx, cy = int(sim.x // sim.pitch), int(sim.y // sim.pitch)
            cost = (
                case.seconds + 15 * dist.get((cx, cy), 40) + 4 * sim.collisions + (200 if must_arrive else 0)
            )
        total += cost
        ok, n = detail.get(case.name, (0, 0))
        detail[case.name] = (ok + (sim.reached is not None), n + 1)
    return total, w, [f"{k}:{a}/{n} llega" for k, (a, n) in detail.items()]


def main(argv=None):
    parser = argparse.ArgumentParser(description="Optimiza los pesos de decisión de los cruces.")
    parser.add_argument("candidatos", type=int, nargs="?", default=24, help="número de candidatos")
    n = parser.parse_args(argv).candidatos
    rnd = random.Random(7)
    cands = [
        (1.0, 0.8, 0.3, 0.2),
        (1.0, 0.0, 0.0, 0.0),
        (0.27, 0.87, 0.07, -0.2),
    ]  # iniciales, solo espacio, 1a optimizacion
    while len(cands) < n:
        cands.append(
            (
                round(rnd.uniform(0.2, 2.0), 2),
                round(rnd.uniform(0.0, 2.0), 2),
                round(rnd.uniform(0.0, 1.0), 2),
                round(rnd.uniform(-0.3, 0.8), 2),
            )
        )
    with ProcessPoolExecutor() as ex:
        results = sorted(ex.map(evaluate, cands))
    for cost, w, detail in results[:8]:
        print(f"coste {cost:7.1f}  pesos {w}  " + "  ".join(detail))
    base = [r for r in results if r[1] == cands[0]][0]
    old = [r for r in results if r[1] == cands[1]][0]
    first = [r for r in results if r[1] == cands[2]][0]
    print(
        f"\niniciales (1.0, 0.8, 0.3, 0.2): {base[0]:.1f} | solo espacio: {old[0]:.1f} | 1a optimizacion: {first[0]:.1f}"
    )
    best = results[0][1]
    print(f"MEJORES PESOS: W_FREE={best[0]} W_NOVEL={best[1]} W_TURN={best[2]} B_STRAIGHT={best[3]}")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Red de CONTROL y red de LOGICA del raton: entradas, maestros, datos, entrenamiento y exportacion.

- Control (43 -> H1 -> H2 -> 2, salida tanh): en cada muestra de los tramos por celdas (run_cells)
  corrige el rumbo objetivo del centrado (dyaw, +-CTRL_YAW_MAX grados) y la velocidad (factor 0.3..1).
  Se SUMA al control en cascada, que sigue de respaldo. Entradas solo observables (laseres, giroscopio,
  encoders). Etiquetas: un maestro del simulador que conoce la posicion real (destilacion).
- Logica (36 -> H1 -> H2 -> 4, puntuacion por direccion): en cada celda del explorador reordena las
  direcciones que la inundacion ya permite (nunca elige una pared ni una que no acerque a la meta).
  Etiquetas: la direccion de menor tiempo real hasta la meta con el mapa completo (giro y marcha atras
  cuestan mas que recto).

Uso (numpy solo para entrenar):
  python3 tools/nets.py collect-logic   --samples 25000 --out work/logic-data.json
  python3 tools/nets.py collect-control --samples 25000 --out work/control-data.json
  python3 tools/nets.py train --kind logic   --data work/logic-data.json   --out tools/models/logic_model.json \
      --export lib/raton_core/src/logic_model.h
  python3 tools/nets.py train --kind control --data work/control-data.json --out tools/models/control_model.json \
      --export lib/raton_core/src/control_model.h
  python3 tools/nets.py eval-logic --model tools/models/logic_model.json      (rapido, sin fisica)
La evaluacion con fisica va por navigation_experiments.py evaluate --controllers mapped,mapped-ctrl,...
Separacion por laberintos: entrenamiento 1000-1079, validacion 1100-1119; los de prueba 200-229 y 400-429
nunca entran en los datos.
"""
import argparse
from concurrent.futures import ProcessPoolExecutor
import hashlib
import heapq
import json
import math
from pathlib import Path
import random
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parent))

CTRL_IN, CTRL_OUT = 43, 2
LOGIC_IN, LOGIC_OUT = 36, 4
CTRL_YAW_MAX = 4.0          # grados de correccion maxima del rumbo objetivo
CTRL_VMIN = 0.3             # factor de velocidad minimo
TRAIN_MAZES = range(1000, 1080)
VAL_MAZES = range(1100, 1120)
TEST_MAZES = set(range(200, 230)) | set(range(400, 430)) | set(range(100, 108))
# Costes en segundos medidos en las trazas del simulador (tramo recto, giro en dos tiempos, marcha atras).
COST_STRAIGHT, COST_TURN, COST_REVERSE = 0.9, 1.9, 1.1
MV = [(0, 1), (1, 0), (0, -1), (-1, 0)]


# ----------------------------------------------------------------------------- inferencia
class Mlp:
    """Perceptron de dos capas ocultas ReLU y salida lineal (tanh o softmax las aplica quien la usa)."""

    def __init__(self, model):
        self.m = model
        self.n_in = model['architecture'][0]

    def __call__(self, x):
        if len(x) != self.n_in or not all(math.isfinite(v) for v in x):
            return None
        a = x
        for w, b in ((self.m['w1'], self.m['b1']), (self.m['w2'], self.m['b2'])):
            a = [max(0.0, bi + sum(wi * v for wi, v in zip(row, a))) for row, bi in zip(w, b)]
        out = [bi + sum(wi * v for wi, v in zip(row, a)) for row, bi in zip(self.m['w3'], self.m['b3'])]
        return out if all(math.isfinite(v) for v in out) else None


def load(path):
    return Mlp(json.loads(Path(path).read_text())) if path and Path(path).exists() else None


# ----------------------------------------------------------------------------- red de control
def control_vector(history, speed, yaw_rate, heading_error, remaining, base_target, reverse):
    """Igual que raton::controlFeatures (lib/raton_core/src/nets.cpp)."""
    out = []
    for frame in history[-3:]:
        for mm, valid, conf, age in frame:
            out.extend((min(300, max(0, mm)) / 300 if valid else 0, float(valid),
                        min(1, max(0, conf)), min(1000, max(0, age)) / 1000))
    out.extend((speed / 300, yaw_rate / 180, heading_error / 90,
                max(-1.0, min(3.0, remaining / 155)), base_target / CTRL_YAW_MAX,
                float(not reverse), float(reverse)))
    return out


def control_features(sim, base_target, remaining, reverse):
    history = list(sim.history)
    while len(history) < 3:
        history.insert(0, [(-1, False, 0, 1000)] * 3)
    err = math.degrees(sim.th - sim.grid_th) if sim.grid_th is not None else 0.0
    err = (err + 180) % 360 - 180
    return control_vector(history, sim.observed_speed, sim.observed_yaw, err, remaining, base_target, reverse)


def control_teacher(sim, base_target):
    """Maestro con la posicion real: rumbo que centra el cuerpo y velocidad segun la holgura real."""
    r = sim.c.robot
    th0 = sim.grid_th if sim.grid_th is not None else sim.th
    axle = (r.length_front - r.length_back) / 2
    bx, by = sim.x + axle * math.cos(sim.th), sim.y + axle * math.sin(sim.th)
    nx, ny = -math.sin(th0), math.cos(th0)              # normal a la izquierda del rumbo de la cuadricula
    perp = bx * nx + by * ny
    p = sim.pitch
    dev = perp - (math.floor(perp / p) + 0.5) * p        # + = a la izquierda del centro del pasillo
    err = (math.degrees(sim.th - th0) + 180) % 360 - 180
    target = max(-1.5 * CTRL_YAW_MAX, min(1.5 * CTRL_YAW_MAX, -0.25 * dev))
    dyaw = max(-CTRL_YAW_MAX, min(CTRL_YAW_MAX, target - base_target))
    half_free = sim.c.corridor / 2
    margin = half_free - abs(dev) - r.width / 2 - r.length_front * abs(math.sin(math.radians(err)))
    vscale = max(CTRL_VMIN, min(1.0, (margin - 2) / 12))
    return [dyaw / CTRL_YAW_MAX, (vscale - (1 + CTRL_VMIN) / 2) / ((1 - CTRL_VMIN) / 2)]


def control_apply(out):
    """Salida de la red -> (dyaw en grados, factor de velocidad)."""
    if out is None:
        return 0.0, 1.0
    dyaw = math.tanh(out[0]) * CTRL_YAW_MAX
    vscale = (1 + CTRL_VMIN) / 2 + math.tanh(out[1]) * (1 - CTRL_VMIN) / 2
    return dyaw, vscale


# ----------------------------------------------------------------------------- red de logica
def logic_features(pos, heading, wall, known, seen, visits, dist, goal, cols, rows):
    """wall(x,y,d) -> bool; known(x,y,d) -> bool (pared leida o del mapa previo); dist: dict celda->pasos."""
    here = dist.get(pos, 999)
    gx, gy = goal[0] - pos[0], goal[1] - pos[1]
    gn = math.hypot(gx, gy) or 1.0
    dirs = []
    for k in range(4):                                  # delante, derecha, atras, izquierda
        d = (heading + k) % 4
        n = (pos[0] + MV[d][0], pos[1] + MV[d][1])
        is_open = not wall(pos[0], pos[1], d)
        dn = dist.get(n, 999) if is_open else 999
        dirs.append((is_open, known(pos[0], pos[1], d), n in seen, is_open and dn < here, dn,
                     visits.get(n, 0), (MV[d][0] * gx + MV[d][1] * gy) / gn))
    fx, fy = MV[heading]
    return logic_vector(dirs, here, len(seen) / (cols * rows), (fx * gx + fy * gy) / cols,
                        (fy * gx - fx * gy) / rows, heading)


def logic_vector(dirs, here, seen_fraction, goal_forward, goal_right, heading):
    """Igual que raton::logicFeatures (lib/raton_core/src/nets.cpp)."""
    out = []
    for is_open, known, seen, allowed, dn, visits, goal_cos in dirs:
        out.extend((float(is_open), float(known), float(seen), float(allowed),
                    min(1.5, (dn if is_open else 999) / 60), min(1.0, visits / 4), goal_cos))
    out.extend((min(1.5, here / 60), seen_fraction, goal_forward, goal_right))
    out.extend(float(heading == i) for i in range(4))
    return out


def logic_pick(scores, heading, opts):
    """Entre las direcciones permitidas, la de mayor puntuacion (robot-relativa)."""
    return max(opts, key=lambda d: scores[(d - heading) % 4])


def maze_walls(maze):
    rows, cols = len(maze) // 2, (len(maze[0]) - 1) // 4

    def wall(x, y, d):
        if not (0 <= x + MV[d][0] < cols and 0 <= y + MV[d][1] < rows):
            return True
        r = rows - 1 - y
        if d == 0: return maze[2 * r][4 * x + 2] == '-'
        if d == 2: return maze[2 * r + 2][4 * x + 2] == '-'
        if d == 3: return maze[2 * r + 1][4 * x] == '|'
        return maze[2 * r + 1][4 * x + 4] == '|'
    return wall, cols, rows


def time_to_goal(maze, goal):
    """Tiempo minimo real desde cada (celda, rumbo) hasta la meta (Dijkstra hacia atras)."""
    wall, cols, rows = maze_walls(maze)
    best = {}
    heap = [(0.0, goal[0], goal[1], h) for h in range(4)]
    while heap:
        c, x, y, h = heapq.heappop(heap)
        if (x, y, h) in best:
            continue
        best[(x, y, h)] = c
        # Estados previos que llegan a (x,y,h): recto desde atras con rumbo h, girando, o marcha atras.
        for hp in range(4):
            for kind in ('fwd', 'rev'):
                d = h if kind == 'fwd' else (h + 2) % 4      # direccion del movimiento
                if kind == 'fwd':
                    cost = COST_STRAIGHT + (0 if hp == h else (COST_TURN if (h - hp) % 4 in (1, 3) else 99))
                    if hp != h and (h - hp) % 4 == 2:
                        continue
                else:
                    if hp != h:
                        continue
                    cost = COST_REVERSE
                px, py = x - MV[d][0], y - MV[d][1]
                if not (0 <= px < cols and 0 <= py < rows) or wall(px, py, d):
                    continue
                if (px, py, hp) not in best:
                    heapq.heappush(heap, (c + cost, px, py, hp))
    return best


def action_cost(heading, d):
    diff = (d - heading) % 4
    return COST_REVERSE if diff == 2 else COST_STRAIGHT + (COST_TURN if diff else 0)


def grid_explore(maze, start, goal, rng, policy=None, flip=0.02, record=None, epsilon=0.0, limit=2000):
    """Explorador por celdas sin fisica (mismo algoritmo que exploreTo del firmware). Devuelve el tiempo
    estimado (costes de accion) o None si no llega. record: lista donde guardar (x, etiqueta)."""
    true_wall, cols, rows = maze_walls(maze)
    walls, seen, visits = {}, set(), {}
    cost_to_go = time_to_goal(maze, goal) if record is not None else None

    def setw(x, y, d, v):
        walls[(x, y, d)] = v
        walls[(x + MV[d][0], y + MV[d][1], (d + 2) % 4)] = v

    def wall(x, y, d):
        if not (0 <= x + MV[d][0] < cols and 0 <= y + MV[d][1] < rows):
            return True
        return walls.get((x, y, d), False)

    def known(x, y, d):
        return (x, y, d) in walls or not (0 <= x + MV[d][0] < cols and 0 <= y + MV[d][1] < rows)
    pos, heading, total = tuple(start), 0, 0.0
    for _ in range(limit):
        if pos == tuple(goal):
            return total
        if pos not in seen:
            for d in (heading, (heading + 1) % 4, (heading + 3) % 4):
                v = true_wall(pos[0], pos[1], d)
                setw(pos[0], pos[1], d, (not v) if rng.random() < flip else v)
            seen.add(pos)
        visits[pos] = visits.get(pos, 0) + 1
        dist, q = {tuple(goal): 0}, [tuple(goal)]
        for c in q:
            for d in range(4):
                n = (c[0] + MV[d][0], c[1] + MV[d][1])
                if not wall(c[0], c[1], d) and n not in dist:
                    dist[n] = dist[c] + 1
                    q.append(n)
        if pos not in dist:
            return None
        opts = [d for d in range(4) if not wall(pos[0], pos[1], d)
                and dist.get((pos[0] + MV[d][0], pos[1] + MV[d][1]), 999) < dist[pos]]
        if not opts:
            return None
        base = min(opts, key=lambda d: ((0 if d == heading else (2 if (d - heading) % 4 == 2 else 1)),
                                        visits.get((pos[0] + MV[d][0], pos[1] + MV[d][1]), 0)))
        x = None
        if record is not None or policy is not None:
            x = logic_features(pos, heading, wall, known, seen, visits, dist, goal, cols, rows)
        if record is not None and len(opts) > 1:
            # Etiqueta: la permitida de menor tiempo real (accion + lo que queda con el mapa completo).
            def real(d):
                n = (pos[0] + MV[d][0], pos[1] + MV[d][1])
                nh = heading if (d - heading) % 4 == 2 else d
                if true_wall(pos[0], pos[1], d):
                    return 1e9
                return action_cost(heading, d) + cost_to_go.get((n[0], n[1], nh), 1e6)
            label = (min(opts, key=real) - heading) % 4
            record.append(dict(x=x, y=label, mask=[float(((heading + k) % 4) in opts) for k in range(4)]))
        if policy is not None:
            out = policy(x)
            d = logic_pick(out, heading, opts) if out is not None else base
        else:
            d = base
        if epsilon and rng.random() < epsilon:
            d = rng.choice(opts)
        n = (pos[0] + MV[d][0], pos[1] + MV[d][1])
        if true_wall(pos[0], pos[1], d):
            setw(pos[0], pos[1], d, True)          # choca con una pared mal leida: la apunta
            total += 0.5
            continue
        total += action_cost(heading, d)
        if (d - heading) % 4 != 2:
            heading = d
        pos = n
    return None


def _maze_case(seed):
    import sim_extremos as S
    rng = random.Random(seed * 7 + 3)
    if rng.random() < 0.5:
        start, goal = (18, 0), (4, 2)
    else:
        start = (rng.randrange(19), 0)
        goal = (rng.randrange(19), rng.randrange(3, 10))
    return S.random_maze(seed=seed, start=start, braid=rng.choice((0.05, 0.12, 0.2))), start, goal


def _logic_job(args):
    seed, rep, quota = args
    maze, start, goal = _maze_case(seed)
    rng = random.Random(seed * 1000 + rep)
    rec = []
    grid_explore(maze, start, goal, rng, flip=rng.choice((0.0, 0.02, 0.05)), record=rec,
                 epsilon=rng.choice((0.0, 0.1, 0.25)))
    return seed, rec[:quota]


def collect_logic(args):
    start = time.monotonic()
    out = []
    for split, seeds, share in (('train', TRAIN_MAZES, 0.8), ('validation', VAL_MAZES, 0.2)):
        need, rep = int(args.samples * share), 0
        got = []
        with ProcessPoolExecutor(max_workers=args.workers) as ex:
            while len(got) < need:
                jobs = [(s, rep, 10 ** 6) for s in seeds]
                for seed, rec in ex.map(_logic_job, jobs):
                    for r in rec:
                        got.append(dict(r, maze=seed, split=split))
                rep += 1
        out.extend(got[:need])
        print(f'{split}: {min(len(got), need)} decisiones ({rep} pasadas)', flush=True)
    assert not ({r['maze'] for r in out} & TEST_MAZES)
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(dict(kind='logic', examples=out, wall_seconds=time.monotonic() - start)))


# ----------------------------------------------------------------------------- datos de control (con fisica)
def _control_job(args):
    seed, noise, quota, every = args
    import sim_extremos as S
    import sim_pistas as P
    from dataclasses import replace
    from navigation import firmware_config
    from navigation_candidates import ObservedMapSim
    rng = random.Random(seed * 10 + noise)
    fw = replace(firmware_config(S), precision_turns=True, goal_grid=True)
    sensor = replace(S.Sensor(), noise=rng.choice((1., 3., 6.)), dropout=rng.choice((0., .02, .08)),
                     latency_ms=rng.choice((0, 20, 40)))
    maze = S.random_maze(seed=seed)
    case = S.Case(str(seed), maze, P.START, P.GOAL, mode=3, fw=fw, seconds=300, sensor=sensor,
                  controller='corrected', off_x=rng.uniform(-8, 8), off_deg=rng.uniform(-3, 3),
                  drift_dps=rng.uniform(-1.5, 1.5))
    sim = ObservedMapSim(case, seed=noise)
    sim.ctrl_log, sim.ctrl_every = [], every
    sim.run()
    return seed, sim.ctrl_log[:quota]


def collect_control(args):
    start = time.monotonic()
    out = []
    for split, seeds, share in (('train', TRAIN_MAZES, 0.8), ('validation', VAL_MAZES, 0.2)):
        need, noise = int(args.samples * share), 1
        got = []
        with ProcessPoolExecutor(max_workers=args.workers) as ex:
            while len(got) < need:
                quota = max(50, need // len(seeds) + 1)
                for seed, rec in ex.map(_control_job, [(s, noise, quota, args.every) for s in seeds]):
                    got.extend(dict(r, maze=seed, split=split) for r in rec)
                print(f'{split}: {len(got)}/{need} muestras', flush=True)
                noise += 1
        out.extend(got[:need])
    assert not ({r['maze'] for r in out} & TEST_MAZES)
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(dict(kind='control', examples=out, wall_seconds=time.monotonic() - start)))


# ----------------------------------------------------------------------------- entrenamiento
def train(args):
    import numpy as np
    start = time.monotonic()
    data = json.loads(Path(args.data).read_text())
    kind = data['kind']
    n_in, n_out = (CTRL_IN, CTRL_OUT) if kind == 'control' else (LOGIC_IN, LOGIC_OUT)
    h1, h2 = args.hidden
    rng = np.random.default_rng(args.seed)
    tr = [r for r in data['examples'] if r['split'] == 'train']
    va = [r for r in data['examples'] if r['split'] == 'validation']
    if {r['maze'] for r in tr} & {r['maze'] for r in va}:
        raise RuntimeError('laberintos repetidos entre entrenamiento y validacion')
    X = np.array([r['x'] for r in tr], 'f'); VX = np.array([r['x'] for r in va], 'f')
    if kind == 'control':
        Y = np.array([r['y'] for r in tr], 'f'); VY = np.array([r['y'] for r in va], 'f')
    else:
        Y = np.array([r['y'] for r in tr]); VY = np.array([r['y'] for r in va])
        M = np.array([r['mask'] for r in tr], 'f'); VM = np.array([r['mask'] for r in va], 'f')
    P = [rng.normal(0, np.sqrt(2 / n_in), (n_in, h1)).astype('f'), np.zeros(h1, 'f'),
         rng.normal(0, np.sqrt(2 / h1), (h1, h2)).astype('f'), np.zeros(h2, 'f'),
         rng.normal(0, np.sqrt(1 / h2), (h2, n_out)).astype('f'), np.zeros(n_out, 'f')]
    mom = [np.zeros_like(p) for p in P]; sq = [np.zeros_like(p) for p in P]

    def fwd(x):
        a = np.maximum(0, x @ P[0] + P[1]); b = np.maximum(0, a @ P[2] + P[3])
        return a, b, b @ P[4] + P[5]

    def loss_grad(x, y, m=None):
        a, b, z = fwd(x)
        if kind == 'control':
            t = np.tanh(z); err = t - y
            loss = float((err ** 2).mean())
            dz = 2 * err * (1 - t ** 2) / err.size
        else:
            z = np.where(m > 0, z, -1e4)                 # solo compiten las direcciones permitidas
            z = z - z.max(1, keepdims=True); p = np.exp(z); p /= p.sum(1, keepdims=True)
            loss = float(-np.log(p[np.arange(len(y)), y] + 1e-9).mean())
            dz = p.copy(); dz[np.arange(len(y)), y] -= 1; dz /= len(y)
        db = (dz @ P[4].T) * (b > 0); da = (db @ P[2].T) * (a > 0)
        return loss, [x.T @ da, da.sum(0), a.T @ db, db.sum(0), b.T @ dz, dz.sum(0)]

    def metric(x, y, m=None):
        z = fwd(x)[2]
        if kind == 'control':
            t = np.tanh(z)
            return dict(mse=float(((t - y) ** 2).mean()),
                        yaw_mae_deg=float(np.abs(t[:, 0] - y[:, 0]).mean() * CTRL_YAW_MAX),
                        speed_mae=float(np.abs(t[:, 1] - y[:, 1]).mean() * (1 - CTRL_VMIN) / 2))
        z = np.where(m > 0, z, -1e9)
        return dict(acc=float((z.argmax(1) == y).mean()))
    best, saved, step, epochs = None, None, 0, 0
    lr = args.lr
    for epoch in range(args.epochs):
        order = rng.permutation(len(X))
        for idx in np.array_split(order, max(1, len(X) // 128)):
            step += 1
            _, g = loss_grad(X[idx], Y[idx], M[idx] if kind == 'logic' else None)
            for i in range(6):                           # Adam
                mom[i] = 0.9 * mom[i] + 0.1 * g[i]; sq[i] = 0.999 * sq[i] + 0.001 * g[i] ** 2
                mh = mom[i] / (1 - 0.9 ** step); vh = sq[i] / (1 - 0.999 ** step)
                P[i] -= lr * mh / (np.sqrt(vh) + 1e-8)
        vloss = loss_grad(VX, VY, VM if kind == 'logic' else None)[0]
        epochs = epoch + 1
        if best is None or vloss < best:
            best, saved = vloss, [p.copy() for p in P]
        if epoch % 20 == 0:
            print(f'epoca {epoch} val {vloss:.5f}', flush=True)
    P[:] = saved
    model = dict(kind=kind, architecture=[n_in, h1, h2, n_out],
                 w1=P[0].T.tolist(), b1=P[1].tolist(), w2=P[2].T.tolist(), b2=P[3].tolist(),
                 w3=P[4].T.tolist(), b3=P[5].tolist(), enabled=False, seed=args.seed, epochs=epochs,
                 validation_loss=best,
                 train_metrics=metric(X, Y, M if kind == 'logic' else None),
                 validation_metrics=metric(VX, VY, VM if kind == 'logic' else None),
                 train_examples=len(X), validation_examples=len(VX),
                 train_mazes=sorted({r['maze'] for r in tr}), validation_mazes=sorted({r['maze'] for r in va}),
                 data_sha256=hashlib.sha256(Path(args.data).read_bytes()).hexdigest(),
                 wall_seconds=time.monotonic() - start)
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(model))
    if args.export:
        export(model, args.export)
    print(json.dumps({k: v for k, v in model.items() if k[0] not in 'wb' or k in ('architecture',)}))


def export(model, path, enabled=False):
    def cpp(v):
        if isinstance(v, list):
            return '{' + ','.join(cpp(x) for x in v) + '}'
        t = format(v, '.9g')
        if '.' not in t and 'e' not in t and 'n' not in t:
            t += '.0'
        return t + 'f'
    name = 'Control' if model['kind'] == 'control' else 'Logic'
    n_in, h1, h2, n_out = model['architecture']
    init = '{' + ','.join(cpp(model[k]) for k in ('w1', 'b1', 'w2', 'b2', 'w3', 'b3')) + '}'
    Path(path).write_text(
        '// Generado por tools/nets.py: no editar a mano.\n#pragma once\n#include "mlp.h"\nnamespace raton {\n'
        f'using {name}Model = Mlp<{n_in}, {h1}, {h2}, {n_out}>;\n'
        f'constexpr bool k{name}ModelTrained = true;\n'
        f'constexpr bool k{name}ModelEnabled = {"true" if enabled else "false"};\n'
        f'static const {name}Model k{name}Model = {init};\n}}  // namespace raton\n')


def export_cmd(args):
    export(json.loads(Path(args.model).read_text()), args.out, enabled=args.enabled)


# ----------------------------------------------------------------------------- evaluacion rapida de la logica
def _eval_logic_job(args):
    seed, model_path = args
    maze, start, goal = _maze_case(seed)
    net = load(model_path)
    res = []
    for flip in (0.0, 0.02):
        a = grid_explore(maze, start, goal, random.Random(seed), flip=flip)
        b = grid_explore(maze, start, goal, random.Random(seed), flip=flip, policy=net)
        res.append((a, b))
    return seed, res


def eval_logic(args):
    seeds = list(range(200, 230)) + list(range(400, 430))
    with ProcessPoolExecutor(max_workers=args.workers) as ex:
        rows = list(ex.map(_eval_logic_job, [(s, args.model) for s in seeds]))
    pairs = [p for _, r in rows for p in r]
    base = [a for a, b in pairs]; net = [b for a, b in pairs]
    both = [(a, b) for a, b in pairs if a is not None and b is not None]
    summary = dict(runs=len(pairs), base_arrivals=sum(a is not None for a in base),
                   net_arrivals=sum(b is not None for b in net),
                   base_time=sum(a for a, _ in both) / max(1, len(both)),
                   net_time=sum(b for _, b in both) / max(1, len(both)),
                   net_faster=sum(b < a - 1e-6 for a, b in both), net_slower=sum(b > a + 1e-6 for a, b in both))
    print(json.dumps(summary))
    if args.out:
        Path(args.out).write_text(json.dumps(dict(summary=summary, rows=rows)))


def main():
    p = argparse.ArgumentParser()
    sub = p.add_subparsers(dest='cmd', required=True)
    for name in ('collect-logic', 'collect-control'):
        c = sub.add_parser(name)
        c.add_argument('--samples', type=int, default=25000)
        c.add_argument('--out', required=True)
        c.add_argument('--workers', type=int, default=8)
        c.add_argument('--every', type=int, default=4, help='control: una muestra de cada N lecturas')
    t = sub.add_parser('train')
    t.add_argument('--kind', choices=('control', 'logic'))
    t.add_argument('--data', required=True); t.add_argument('--out', required=True); t.add_argument('--export')
    t.add_argument('--hidden', type=lambda s: tuple(int(v) for v in s.split(',')), default=(32, 16))
    t.add_argument('--epochs', type=int, default=150); t.add_argument('--lr', type=float, default=1e-3)
    t.add_argument('--seed', type=int, default=42)
    e = sub.add_parser('export'); e.add_argument('--model', required=True); e.add_argument('--out', required=True)
    e.add_argument('--enabled', action='store_true')
    v = sub.add_parser('eval-logic'); v.add_argument('--model', required=True); v.add_argument('--out')
    v.add_argument('--workers', type=int, default=8)
    a = p.parse_args()
    {'collect-logic': collect_logic, 'collect-control': collect_control, 'train': train,
     'export': export_cmd, 'eval-logic': eval_logic}[a.cmd](a)


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Simulador 2D de casos extremos del raton (sin dependencias).

Reproduce la logica del firmware de los modos 1 (seguir pared derecha) y 5 (espacio libre) con:
  - cuerpo de 80 x 80 mm, eje central y laterales diagonales a 31 grados,
  - un modelo de VL53L0X (rayo efectivo identificado, minimo fiable 30 mm, alcance maximo, ruido, perdidas),
  - laberintos de prueba en el mismo formato de dibujo que firmware/mapa/competition_maze.h.

Uso:  python -m raton_sim casos      -> tabla de resultados y simulator/results/casos/index.html
Los parametros siguen a firmware/config.h; los casos y las variantes estan al final del archivo.
"""

import math
import os
import random
from collections import deque
from dataclasses import replace

from raton_sim.centrado.sim import CentradoSim
from raton_sim.exploracion.sim import ExploracionSim
from raton_sim.giros import native as N
from raton_sim.giros.sim import GirosSim
from raton_sim.mapa.pistas import M_DEAD as M_DEAD
from raton_sim.mapa.pistas import M_L as M_L
from raton_sim.mapa.pistas import M_T as M_T
from raton_sim.mapa.pistas import comp_maze as comp_maze
from raton_sim.mapa.pistas import random_maze as random_maze
from raton_sim.modos.sim import ModosSim
from raton_sim.odometria.sim import OdometriaSim
from raton_sim.paredes.sim import ParedesSim
from raton_sim.seguridad import orchestrators as O
from raton_sim.seguridad.sim import SeguridadSim
from raton_sim.simulacion.modelos import Case as Case
from raton_sim.simulacion.modelos import Firmware as Firmware
from raton_sim.simulacion.modelos import MotionFailure as MotionFailure

# ---------------------------------------------------------------- parametros (como config.h)
from raton_sim.simulacion.modelos import Robot as Robot
from raton_sim.simulacion.modelos import Sensor as Sensor
from raton_sim.simulacion.modelos import build_walls as build_walls
from raton_sim.simulacion.modelos import seg_hit as seg_hit
from raton_sim.simulacion.modelos import segs_cross as segs_cross


class Sim(ParedesSim, CentradoSim, GirosSim, OdometriaSim, SeguridadSim, ExploracionSim, ModosSim):
    DT = 0.005

    def __init__(self, case, seed=1):
        if case.fw.geometry is not None and case.robot == Robot():
            case = replace(case, robot=case.fw.geometry)
        # Igual que TrackGeometry en el nucleo: la ventana de lectura depende
        # del pasillo configurado, no del ancho historico de 192 mm.
        robot = case.robot
        side_lateral = robot.width / 2 - robot.side_inset
        case = replace(
            case,
            fw=replace(
                case.fw,
                side_look_ahead=robot.side_ahead
                + (case.corridor / 2 - side_lateral) / math.tan(math.radians(robot.side_angle)),
                cell_side_wall=max(45, int(0.42 * case.corridor)),
            ),
        )
        self.c = case
        self.mission = O.MissionOrchestrator()
        self.motion = O.MotionOrchestrator()
        self.fresh_front = False
        self.rnd = random.Random(seed)
        self.mount_errors = {
            k: self.rnd.uniform(-case.sensor.mounting_error, case.sensor.mounting_error) for k in ("L", "R")
        }
        self.explore_v = 0.0
        self.segs, self.pitch, self.rows, self.cols = build_walls(case)
        sx, sy = case.start
        # Eje de ruedas colocado para que el raton quede centrado en la celda de salida.
        center_off = (case.robot.length_front - case.robot.length_back) / 2
        self.x = (sx + 0.5) * self.pitch + case.off_x
        self.y = (sy + 0.5) * self.pitch - center_off
        self.th = math.radians(90 + case.off_deg)  # 90 = mirando al norte (+y)
        self.t = 0.0
        self.trail = [(self.x, self.y)]
        self.collisions = 0
        self.in_contact = False
        self.turns = 0
        self.loops = 0
        self.travelled = 0.0
        self.reached = None
        self.failure = None
        self.readings = {"L": -1, "F": -1, "R": -1}
        self.hist = {"L": [], "F": [], "R": []}
        self.nsamples = 0
        self.edge_prev = {}
        self.edge_log = []
        self.decisions = []
        self.sensed = []  # ultimas 3 muestras (la mediana del firmware)
        self.miss = {"L": 0, "F": 0, "R": 0}
        self.next_sample = 0.0
        self.cell_mm = case.corridor + case.wall
        self.cells_mapped = 0
        self.mapping_complete = False
        self.recoveries = 0
        self.corridor_est = case.corridor
        self.events = []
        self._near = []
        self._near_at = None
        self.visits = {}
        self.exits = {}
        self.pending = 0.0
        self.along = None  # eje respecto al centro de la celda en el rumbo (None = centrado)
        self.grid_th = None  # rumbo absoluto de la cuadricula (modo 4): los errores de giro no se acumulan
        self.last_valid = {"L": (1000, 0), "F": (1000, 0), "R": (1000, 0)}
        self.last_valid_at = {"L": -1e9, "F": -1e9, "R": -1e9}  # recorrido (mm) en la ultima valida
        self.cell = None
        self.raw_readings = {k: -1 for k in ("L", "F", "R")}
        self.sensor_conf = {k: 1.0 for k in ("L", "F", "R")}
        self.prev_raw = {k: -1 for k in ("L", "F", "R")}
        self.delayed_samples = {k: deque() for k in ("L", "F", "R")}
        self.trace = []
        self.motions = []
        self.native = N.NativeNavigation() if case.controller != "baseline" else None
        self.observed_speed = self.observed_yaw = 0.0
        self._sample_distance, self._sample_heading, self._sample_time = self.travelled, self.th, self.t

    def near(self):
        """Paredes a menos de 300 mm (se recalcula al moverse 40 mm): acelera mucho el laberinto grande."""
        if self._near_at is None or math.hypot(self.x - self._near_at[0], self.y - self._near_at[1]) > 40:
            self._near_at = (self.x, self.y)
            R = 300.0
            self._near = [
                sg
                for sg in self.segs
                if min(sg[0][0], sg[1][0]) - R < self.x < max(sg[0][0], sg[1][0]) + R
                and min(sg[0][1], sg[1][1]) - R < self.y < max(sg[0][1], sg[1][1]) + R
            ]
        return self._near + self.hand_segs()

    # --- mundo
    def body(self, x=None, y=None, th=None):
        x = self.x if x is None else x
        y = self.y if y is None else y
        th = self.th if th is None else th
        r = self.c.robot
        c, s = math.cos(th), math.sin(th)
        pts = []
        for fx, lx in (
            (r.length_front, r.width / 2),
            (r.length_front, -r.width / 2),
            (-r.length_back, -r.width / 2),
            (-r.length_back, r.width / 2),
        ):
            pts.append((x + fx * c - lx * s, y + fx * s + lx * c))
        return pts

    def hand_segs(self):
        hd = self.c.hand
        if not hd or not (hd[2] <= self.t <= hd[3]):
            return []
        x, y = hd[0], hd[1]
        return [
            ((x - 30, y - 30), (x + 30, y - 30)),
            ((x + 30, y - 30), (x + 30, y + 30)),
            ((x + 30, y + 30), (x - 30, y + 30)),
            ((x - 30, y + 30), (x - 30, y - 30)),
        ]

    def collides(self, x, y, th):
        pts = self.body(x, y, th)
        edges = [(pts[i], pts[(i + 1) % 4]) for i in range(4)]
        for a, b in self.near():
            for e in edges:
                if segs_cross(e[0], e[1], a, b):
                    return True
        return False

    def try_move(self, nx, ny, nth):
        if self.collides(nx, ny, nth):
            if not self.in_contact:
                self.collisions += 1
                self.events.append((self.t, "choque", self.x, self.y))
            self.in_contact = True
            return False
        self.in_contact = False
        self.travelled += math.hypot(nx - self.x, ny - self.y) * (1 if self._fwd(nx, ny) else -1)
        self.x, self.y, self.th = nx, ny, nth
        if len(self.trail) == 0 or math.hypot(self.trail[-1][0] - nx, self.trail[-1][1] - ny) > 5:
            self.trail.append((nx, ny))
        return True

    def _fwd(self, nx, ny):
        return (nx - self.x) * math.cos(self.th) + (ny - self.y) * math.sin(self.th) >= 0

    # --- sensores
    def raycast(self, ox, oy, ang):
        best = None
        half = math.radians(self.c.sensor.cone_deg / 2)
        for k in (
            (0,) if half == 0 else (-1, -0.5, 0, 0.5, 1)
        ):  # el cono: gana lo mas cercano (como el VL53L0X)
            a = ang + k * half
            dx, dy = math.cos(a), math.sin(a)
            for (ax, ay), (bx, by) in self.near():
                d = seg_hit(ox, oy, dx, dy, ax, ay, bx, by)
                if d is not None and (best is None or d < best):
                    best = d
        return best

    def tick(self):
        self.t += self.DT
        c = self.odo_cell()
        if c != self.cell:
            self.cell = c
            self.visits[c] = self.visits.get(c, 0) + 1
        if self.t >= self.next_sample:
            self.sample()
            self.next_sample += 0.020
        cell = (int(self.x // self.pitch), int(self.y // self.pitch))
        if self.reached is None and cell == tuple(self.c.goal):
            self.reached = self.t

    # --- primitivas del firmware (generadores: un paso por yield)

    # --- modos (igual que firmware/app)

    # --- modo 4: ruta del mapa conocido por tramos (igual que competitionRun del firmware)

    # --- modo 3: explorar por celdas con inundacion (igual que exploreCells del firmware)

    def run(self):
        gen = {6: self.mode3, 5: self.mode5, 4: self.mode4, 3: self.mode3}.get(self.c.mode, self.mode1)()
        steps = int(self.c.seconds / self.DT)
        for _ in range(steps):
            try:
                next(gen)
            except MotionFailure as exc:
                self.failure = str(exc)
                break
            except StopIteration:
                if self.reached is None and not self.mapping_complete:
                    self.failure = "navegacion terminada sin llegada fisica"
                break
            if self.reached is not None and not (
                self.c.mode == 6 or (self.c.mode == 3 and not self.c.use_goal)
            ):
                break
        if (
            not self.mapping_complete
            and self.failure is None
            and (self.reached is None or self.c.mode == 6 or (self.c.mode == 3 and not self.c.use_goal))
        ):
            self.failure = "limite de simulacion"
        gen.close()
        self.mission.decide(finished=self.failure is None, recover=self.failure is not None)
        return self


# ---------------------------------------------------------------- dibujo


def svg(sim, path):
    pitch, rows, cols = sim.pitch, sim.rows, sim.cols
    W, H = cols * pitch, rows * pitch
    sc = 560 / max(W, H)

    def P(x, y):
        return f"{(x + 20) * sc:.1f},{(H - y + 20) * sc:.1f}"

    out = [
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {(W + 40) * sc:.0f} {(H + 40) * sc:.0f}" '
        f'width="{(W + 40) * sc:.0f}" height="{(H + 40) * sc:.0f}" style="background:#f6f2ea">'
    ]
    for a, b in sim.segs:
        out.append(
            f'<line x1="{(a[0] + 20) * sc:.1f}" y1="{(H - a[1] + 20) * sc:.1f}" x2="{(b[0] + 20) * sc:.1f}" '
            f'y2="{(H - b[1] + 20) * sc:.1f}" stroke="#8a6a3a" stroke-width="1.5"/>'
        )
    gx, gy = sim.c.goal
    out.append(
        f'<rect x="{(gx * pitch + 20) * sc:.1f}" y="{(H - (gy + 1) * pitch + 20) * sc:.1f}" '
        f'width="{pitch * sc:.1f}" height="{pitch * sc:.1f}" fill="#2b9a66" opacity="0.25"/>'
    )
    out.append(
        '<polyline fill="none" stroke="#2f6fdf" stroke-width="1.5" points="'
        + " ".join(P(x, y) for x, y in sim.trail)
        + '"/>'
    )
    for t, kind, x, y in sim.events:
        out.append(f'<circle cx="{(x + 20) * sc:.1f}" cy="{(H - y + 20) * sc:.1f}" r="4" fill="#c92a2a"/>')
    out.append(f'<polygon points="{" ".join(P(x, y) for x, y in sim.body())}" fill="#1d2330" opacity="0.8"/>')
    out.append("</svg>")
    with open(path, "w") as f:
        f.write("\n".join(out))


# ---------------------------------------------------------------- casos


def cases():
    real = Robot()
    centred = replace(real, length_front=50, length_back=50, side_ahead=25, front_ahead=10)
    inset = replace(real, side_inset=15)
    base_fw = Firmware()
    fixed_fw = replace(base_fw, cfg_side_ahead=56, cfg_front_ahead=33, turn_mode="dos_tiempos", unstick=True)
    out = []
    for fw_name, fw in (("actual", base_fw), ("dos tiempos", fixed_fw)):

        def tag(n):
            return f"{n} [{fw_name}]"

        out += [
            Case(tag("L estrecha 14 cm"), M_L, (2, 0), (0, 2), fw=fw),
            Case(tag("Cruce en T 14 cm"), M_T, (2, 0), (0, 2), fw=fw),
            Case(tag("Callejon sin salida"), M_DEAD, (1, 0), (0, 2), fw=fw),
            Case(
                tag("Pasillo ancho 22 cm, torcido 10°"),
                M_L,
                (2, 0),
                (0, 2),
                corridor=220,
                off_x=25,
                off_deg=10,
                fw=fw,
            ),
            Case(
                tag("Ruido alto + 15% perdidas"),
                M_L,
                (2, 0),
                (0, 2),
                sensor=Sensor(noise=8, dropout=0.15),
                fw=fw,
            ),
            Case(tag("Laser izquierdo roto"), M_T, (2, 0), (0, 2), sensor=Sensor(blind_left=True), fw=fw),
            Case(tag("Rueda floja (deriva 6°/s)"), M_L, (2, 0), (0, 2), drift_dps=6, fw=fw),
            Case(tag("Mano a 8 cm durante 3 s"), M_L, (2, 0), (0, 2), hand=(2.5 * 155, 240, 1.0, 4.0), fw=fw),
            Case(tag("Laterales hundidos 15 mm"), M_L, (2, 0), (0, 2), robot=inset, fw=fw),
            Case(
                tag("Eje de ruedas centrado"),
                M_L,
                (2, 0),
                (0, 2),
                robot=centred,
                fw=replace(
                    fw,
                    cfg_side_ahead=25 if fw_name == "corregido" else fw.cfg_side_ahead,
                    cfg_front_ahead=10 if fw_name == "corregido" else fw.cfg_front_ahead,
                ),
            ),
            Case(
                tag("Laberinto reglamento, modo 1"), comp_maze(), (18, 0), (4, 2), mode=1, seconds=240, fw=fw
            ),
            Case(
                tag("Laberinto reglamento, modo 5"), comp_maze(), (18, 0), (4, 2), mode=5, seconds=240, fw=fw
            ),
        ]
    return out


def main(out=None):
    outdir = str(out or N.ROOT / "simulator/results/casos")
    os.makedirs(outdir, exist_ok=True)
    rows = []
    for i, case in enumerate(cases()):
        sim = Sim(case).run()
        name = f"caso{i:02d}.svg"
        svg(sim, os.path.join(outdir, name))
        ok = sim.reached is not None
        rows.append((case.name, ok, sim.reached, sim.collisions, sim.turns, sim.loops, name))
        print(
            f"{case.name:48s} {'LLEGA' if ok else 'NO   '} "
            f"t={sim.reached or case.seconds:6.1f}s choques={sim.collisions:3d} giros={sim.turns:3d} bucles={sim.loops}"
        )
    html = [
        '<!doctype html><meta charset="utf-8"><title>Casos extremos</title>',
        "<style>body{font:14px system-ui;margin:20px;background:#fff;color:#1d2330}"
        ".g{display:grid;grid-template-columns:repeat(auto-fill,minmax(320px,1fr));gap:16px}"
        ".c{border:1px solid #ddd;border-radius:10px;padding:10px}.ok{color:#2b9a66}.no{color:#c92a2a}"
        "img{width:100%}</style><h1>Simulación de casos extremos</h1>"
        "<p>Línea azul = recorrido · puntos rojos = choques · verde = meta · rectángulo oscuro = ratón al final.</p>"
        '<div class="g">',
    ]
    for name, ok, t, col, turns, loops, img in rows:
        html.append(
            f'<div class="c"><b>{name}</b><br><span class="{"ok" if ok else "no"}">'
            f"{'Llega en %.1f s' % t if ok else 'No llega'}</span> · choques {col} · giros {turns} · bucles {loops}"
            f'<img src="{img}"></div>'
        )
    html.append("</div>")
    with open(os.path.join(outdir, "index.html"), "w") as f:
        f.write("\n".join(html))
    print("\nInforme:", os.path.join(outdir, "index.html"))


if __name__ == "__main__":
    main()

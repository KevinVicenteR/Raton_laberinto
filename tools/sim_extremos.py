#!/usr/bin/env python3
"""Simulador 2D de casos extremos del raton (sin dependencias).

Reproduce la logica del firmware de los modos 1 (seguir pared derecha) y 5 (espacio libre) con:
  - la geometria del raton (eje de ruedas atras, laseres por delante del eje),
  - un modelo de VL53L0X (cono de 25 grados, minimo fiable 30 mm, alcance maximo, ruido, perdidas),
  - laberintos de prueba en el mismo formato de dibujo que src/competition_maze.h.

Uso:  python3 tools/sim_extremos.py      -> tabla de resultados y tools/sim_resultados/index.html
Los parametros siguen a src/config.h; los casos y las variantes estan al final del archivo.
"""

import math
import os
import random
from dataclasses import dataclass, field, replace

# ---------------------------------------------------------------- parametros (como config.h)


@dataclass
class Robot:
    length_front: float = 83.0   # del eje de ruedas al morro (mm)
    length_back: float = 17.0    # del eje de ruedas a la trasera
    width: float = 92.0          # ancho total
    side_ahead: float = 56.0     # laseres laterales por delante del eje (real, medido en la foto)
    side_inset: float = 0.0      # cuanto van hundidos los laterales hacia dentro del borde
    front_ahead: float = 33.0    # laser frontal por delante del eje (real)
    deadzone: float = 50.0       # lectura frontal con el morro tocando (FRONT_DEADZONE_MM)


@dataclass
class Sensor:
    max_mm: float = 160.0        # LASER_MAX_MM
    min_mm: float = 30.0         # por debajo, el VL53L0X da basura o nada
    noise: float = 3.0           # desviacion tipica (mm)
    dropout: float = 0.02        # probabilidad de lectura perdida
    cone_deg: float = 25.0
    blind_left: bool = False     # laser izquierdo roto (nunca mide)


@dataclass
class Firmware:
    speed: float = 100.0
    side_wall: float = 150.0     # SIDE_WALL_MM = SIDE_OPEN_MM
    front_wall: float = 140.0    # FRONT_WALL_MM
    front_margin: float = 15.0
    cfg_side_ahead: float = 45.0  # lo que cree el firmware (SIDE_SENSOR_AHEAD_MM)
    cfg_front_ahead: float = 50.0
    robot_width_cfg: float = 90.0
    kp_wall: float = 0.08
    max_wall_deg: float = 4.0
    deadband: float = 5.0
    side_crash: float = 8.0
    turn_mode: str = "sitio"     # "sitio" = gira sobre el eje; "seguro" = retrocede antes;
                                 # "dos_tiempos" = gira 70, avanza, gira 20 (y sale marcha atras de callejones)
    split_deg: float = 70.0
    split_advance: float = 75.0
    close_mm: float = 60.0       # -1 tras una lectura por debajo de esto = demasiado cerca
    unstick: bool = False        # desatasco: atrás y enderezarse si se atasca con el frente libre
    weighted: bool = False       # decision ponderada en los cruces (como el firmware)
    w: tuple = (1.0, 0.8, 0.3, 0.2)  # espacio, novedad, giro, sesgo recto


@dataclass
class Case:
    name: str
    maze: list
    start: tuple                 # celda (x, y) contando desde abajo a la izquierda
    goal: tuple
    corridor: float = 140.0      # ancho libre del pasillo (mm)
    wall: float = 15.0           # grosor de pared
    mode: int = 5
    off_x: float = 0.0           # descentrado inicial (mm)
    off_deg: float = 0.0         # torcido inicial (grados)
    drift_dps: float = 0.0       # deriva del rumbo (rueda mas floja), grados/s
    robot: Robot = field(default_factory=Robot)
    sensor: Sensor = field(default_factory=Sensor)
    fw: Firmware = field(default_factory=Firmware)
    hand: tuple = None           # obstaculo movil: (x, y, desde_s, hasta_s) en mm
    seconds: float = 90.0


# ---------------------------------------------------------------- geometria


def seg_hit(ox, oy, dx, dy, ax, ay, bx, by):
    """Distancia del rayo (o + t d) al segmento ab, o None."""
    ex, ey = bx - ax, by - ay
    den = dx * ey - dy * ex
    if abs(den) < 1e-9:
        return None
    t = ((ax - ox) * ey - (ay - oy) * ex) / den
    u = ((ax - ox) * dy - (ay - oy) * dx) / den
    if t >= 0 and 0 <= u <= 1:
        return t
    return None


def segs_cross(p1, p2, p3, p4):
    def orient(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    d1, d2 = orient(p3, p4, p1), orient(p3, p4, p2)
    d3, d4 = orient(p1, p2, p3), orient(p1, p2, p4)
    return (d1 * d2 < 0) and (d3 * d4 < 0)


def build_walls(case):
    """Segmentos de pared (cada pared es un rectangulo de grosor `wall`) a partir del dibujo."""
    rows = len(case.maze) // 2
    cols = (len(case.maze[0]) - 1) // 4
    pitch = case.corridor + case.wall
    h = case.wall / 2
    rects = []
    for r in range(rows + 1):
        line = case.maze[2 * r]
        y = (rows - r) * pitch
        for x in range(cols):
            if line[4 * x + 2] == '-':
                rects.append((x * pitch - h, y - h, (x + 1) * pitch + h, y + h))
    for r in range(rows):
        line = case.maze[2 * r + 1]
        y0, y1 = (rows - r - 1) * pitch, (rows - r) * pitch
        for x in range(cols + 1):
            if line[4 * x] == '|':
                rects.append((x * pitch - h, y0 - h, x * pitch + h, y1 + h))
    segs = []
    for (x0, y0, x1, y1) in rects:
        segs += [((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)), ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))]
    return segs, pitch, rows, cols


# ---------------------------------------------------------------- simulacion


class Sim:
    DT = 0.005

    def __init__(self, case, seed=1):
        self.c = case
        self.rnd = random.Random(seed)
        self.segs, self.pitch, self.rows, self.cols = build_walls(case)
        sx, sy = case.start
        # Eje de ruedas colocado para que el raton quede centrado en la celda de salida.
        center_off = (case.robot.length_front - case.robot.length_back) / 2
        self.x = (sx + 0.5) * self.pitch + case.off_x
        self.y = (sy + 0.5) * self.pitch - center_off
        self.th = math.radians(90 + case.off_deg)   # 90 = mirando al norte (+y)
        self.t = 0.0
        self.trail = [(self.x, self.y)]
        self.collisions = 0
        self.in_contact = False
        self.turns = 0
        self.loops = 0
        self.travelled = 0.0
        self.reached = None
        self.readings = {"L": -1, "F": -1, "R": -1}
        self.miss = {"L": 0, "F": 0, "R": 0}
        self.next_sample = 0.0
        self.corridor_est = 140.0
        self.events = []
        self._near = []
        self._near_at = None
        self.visits = {}
        self.last_valid = {"L": (1000, 0), "F": (1000, 0), "R": (1000, 0)}
        self.cell = None

    def near(self):
        """Paredes a menos de 300 mm (se recalcula al moverse 40 mm): acelera mucho el laberinto grande."""
        if self._near_at is None or math.hypot(self.x - self._near_at[0], self.y - self._near_at[1]) > 40:
            self._near_at = (self.x, self.y)
            R = 300.0
            self._near = [sg for sg in self.segs
                          if min(sg[0][0], sg[1][0]) - R < self.x < max(sg[0][0], sg[1][0]) + R
                          and min(sg[0][1], sg[1][1]) - R < self.y < max(sg[0][1], sg[1][1]) + R]
        return self._near + self.hand_segs()

    # --- mundo
    def body(self, x=None, y=None, th=None):
        x = self.x if x is None else x
        y = self.y if y is None else y
        th = self.th if th is None else th
        r = self.c.robot
        c, s = math.cos(th), math.sin(th)
        pts = []
        for fx, lx in ((r.length_front, r.width / 2), (r.length_front, -r.width / 2),
                       (-r.length_back, -r.width / 2), (-r.length_back, r.width / 2)):
            pts.append((x + fx * c - lx * s, y + fx * s + lx * c))
        return pts

    def hand_segs(self):
        hd = self.c.hand
        if not hd or not (hd[2] <= self.t <= hd[3]):
            return []
        x, y = hd[0], hd[1]
        return [((x - 30, y - 30), (x + 30, y - 30)), ((x + 30, y - 30), (x + 30, y + 30)),
                ((x + 30, y + 30), (x - 30, y + 30)), ((x - 30, y + 30), (x - 30, y - 30))]

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
        for k in (-1, -0.5, 0, 0.5, 1):     # el cono: gana lo mas cercano (como el VL53L0X)
            a = ang + k * half
            dx, dy = math.cos(a), math.sin(a)
            for (ax, ay), (bx, by) in self.near():
                d = seg_hit(ox, oy, dx, dy, ax, ay, bx, by)
                if d is not None and (best is None or d < best):
                    best = d
        return best

    def sample(self):
        r, sn = self.c.robot, self.c.sensor
        c, s = math.cos(self.th), math.sin(self.th)
        half = r.width / 2 - r.side_inset
        mounts = {
            "F": (self.x + r.front_ahead * c, self.y + r.front_ahead * s, self.th),
            "L": (self.x + r.side_ahead * c - half * s, self.y + r.side_ahead * s + half * c, self.th + math.pi / 2),
            "R": (self.x + r.side_ahead * c + half * s, self.y + r.side_ahead * s - half * c, self.th - math.pi / 2),
        }
        for key, (ox, oy, a) in mounts.items():
            d = self.raycast(ox, oy, a)
            # El frontal va por detras del morro: su lectura incluye el punto muerto calibrado.
            if d is not None and key == "F":
                d += r.deadzone - (r.length_front - r.front_ahead)
            val = -1
            if d is not None and d <= sn.max_mm and not (key == "L" and sn.blind_left):
                if d < sn.min_mm:
                    val = -1 if self.rnd.random() < 0.5 else d + self.rnd.uniform(0, 40)  # basura
                elif self.rnd.random() >= sn.dropout:
                    val = max(0, d + self.rnd.gauss(0, sn.noise))
            if val < 0:   # LASER_MAX_MISS = 2: se mantiene la ultima medida un par de lecturas
                self.miss[key] += 1
                if self.miss[key] >= 2:
                    self.readings[key] = -1
            else:
                self.miss[key] = 0
                self.readings[key] = val
                self.last_valid[key] = (val, self.t)

    def odo_cell(self, x=None, y=None):
        pitch = self.corridor_est + 15
        x = self.x if x is None else x
        y = self.y if y is None else y
        return (math.floor(x / pitch + 0.5), math.floor(y / pitch + 0.5))

    def visits_toward(self, turn_deg):
        h = math.degrees(self.th) - 90 + turn_deg      # 0 = norte, positivo a la izquierda
        d = int(round(h / 90)) % 4
        dx, dy = [(0, 1), (-1, 0), (0, -1), (1, 0)][d]
        cx, cy = self.odo_cell()
        return self.visits.get((cx + dx, cy + dy), 0)

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

    def front_turn(self):
        fw = self.c.fw
        return max(self.corridor_est / 2 - fw.cfg_front_ahead, self.c.robot.deadzone + fw.front_margin)

    # --- primitivas del firmware (generadores: un paso por yield)
    def drive(self, max_mm, watch, ignore=0.0):
        fw, r = self.c.fw, self.c.robot
        v, start, moving_at, open_l, open_r = 0.0, self.travelled, self.t, None, None
        last_ok = {"L": 100.0, "R": 100.0}   # ultima lectura valida de cada lateral
        th0 = self.th
        yaw_t = 0.0
        while True:
            yield
            self.tick()
            pos = self.travelled - start
            L, F, R = self.readings["L"], self.readings["F"], self.readings["R"]
            if pos >= max_mm - 1:
                return "dist"
            front_near = 0 <= F < fw.front_wall
            if 0 <= F and F - self.front_turn() <= 0:
                return "pared"
            narrow = L >= 0 and R >= 0 and abs(L - R) < 10
            if 0 <= L < fw.side_crash and not narrow and (R < 0 or L < R):
                return "roce_izq"
            if 0 <= R < fw.side_crash and not narrow and (L < 0 or R < L):
                return "roce_der"
            if not front_near and 0 <= L <= fw.side_wall and 0 <= R <= fw.side_wall:
                self.corridor_est = min(400, max(80, 0.98 * self.corridor_est + 0.02 * (L + R + fw.robot_width_cfg)))
            if watch and not front_near and pos >= ignore:
                sides = []
                if watch in (1, 2):
                    sides.append(("L", L))
                if watch in (-1, 2):
                    sides.append(("R", R))
                for key, val in sides:
                    if val >= 0:
                        last_ok[key] = val
                    # Sin lectura justo despues de una lectura muy corta = demasiado cerca, no hueco.
                    too_close = val < 0 and last_ok[key] < fw.close_mm
                    is_open = not (0 <= val <= fw.side_wall) and not too_close
                    if key == "L":
                        open_l = (open_l if open_l is not None else pos) if is_open else None
                        if open_l is not None and pos - open_l >= 5:
                            self.open_side = 1
                            return "hueco"
                    else:
                        open_r = (open_r if open_r is not None else pos) if is_open else None
                        if open_r is not None and pos - open_r >= 5:
                            self.open_side = -1
                            return "hueco"
            # velocidad
            remaining = max_mm - pos
            if F >= 0:
                remaining = min(remaining, F - self.front_turn())
            vmax = min(fw.speed, 60) if front_near else fw.speed
            v = min(v + 800 * self.DT, vmax, max(math.sqrt(max(0, 2 * 800 * remaining)), 40))
            # centrado
            offset = None
            hasL, hasR = 0 <= L < fw.side_wall, 0 <= R < fw.side_wall
            center = (self.corridor_est - fw.robot_width_cfg) / 2
            if not front_near:
                if hasL and hasR:
                    offset = (L - R) / 2
                elif hasL:
                    offset = L - center
                elif hasR:
                    offset = center - R
            target = 0.0
            if offset is not None and abs(offset) > fw.deadband:
                offset -= math.copysign(fw.deadband, offset)
                target = max(-fw.max_wall_deg, min(fw.max_wall_deg, fw.kp_wall * offset))
            yaw_t += 0.1 * (target - yaw_t)
            err = math.radians(yaw_t) - (self.th - th0)
            omega = max(-1.5, min(1.5, 6 * err)) + math.radians(self.c.drift_dps)
            nth = self.th + omega * self.DT
            nx = self.x + v * self.DT * math.cos(nth)
            ny = self.y + v * self.DT * math.sin(nth)
            if self.try_move(nx, ny, nth):
                if v > 20:
                    moving_at = self.t
            if self.t - moving_at > 0.6:
                return "atasco"

    def rotate(self, deg):
        self.turns += 1
        target = self.th + math.radians(deg) + math.radians(self.rnd.gauss(0, 1.5))
        t0 = self.t
        while abs(target - self.th) > math.radians(0.5) and self.t - t0 < 3:
            yield
            self.tick()
            step = math.copysign(min(abs(target - self.th), math.radians(180) * self.DT), target - self.th)
            self.try_move(self.x, self.y, self.th + step)

    def back(self, mm):
        start = self.travelled
        t0 = self.t
        while start - self.travelled < mm and self.t - t0 < 1.0:
            yield
            self.tick()
            nx = self.x - 120 * self.DT * math.cos(self.th)
            ny = self.y - 120 * self.DT * math.sin(self.th)
            self.try_move(nx, ny, self.th)

    def back_out(self, max_mm):
        """Marcha atras vigilando los laterales hasta ver un hueco. Devuelve +1 izq, -1 der o 0."""
        fw = self.c.fw
        start, t0 = self.travelled, self.t
        since = {"L": None, "R": None}
        while start - self.travelled < max_mm and self.t - t0 < 4:
            yield
            self.tick()
            pos = start - self.travelled
            for key in ("L", "R"):
                v = self.readings[key]
                is_open = not (0 <= v <= fw.side_wall)
                since[key] = (since[key] if since[key] is not None else pos) if is_open else None
                if since[key] is not None and pos - since[key] >= 5:
                    # seguir hasta que el laser quede en el centro del hueco
                    extra = self.corridor_est / 2
                    s2 = self.travelled
                    while s2 - self.travelled < extra and self.t - t0 < 5:
                        yield
                        self.tick()
                        self.try_move(self.x - 100 * self.DT * math.cos(self.th),
                                      self.y - 100 * self.DT * math.sin(self.th), self.th)
                    return 1 if key == "L" else -1
            self.try_move(self.x - 100 * self.DT * math.cos(self.th), self.y - 100 * self.DT * math.sin(self.th), self.th)
        return 0

    def safe_turn(self, deg):
        """Variantes de giro segun fw.turn_mode."""
        fw = self.c.fw
        if fw.turn_mode == "dos_tiempos":
            if abs(deg) == 90:
                sgn = 1 if deg > 0 else -1
                yield from self.rotate(sgn * fw.split_deg)
                yield from self.drive(fw.split_advance, 0)
                yield from self.rotate(sgn * (90 - fw.split_deg))
                return
            if abs(deg) == 180:
                side = yield from self.back_out(3 * self.pitch)
                if side:
                    # El hueco queda detras del laser: entrar con el giro en dos tiempos hacia ese lado.
                    yield from self.safe_turn(90 * side)
                    return
        if fw.turn_mode == "seguro":
            for _ in range(12):
                if not self._turn_hits(deg):
                    break
                yield from self.back(8)
        yield from self.rotate(deg)

    def _turn_hits(self, deg):
        for k in range(1, 7):
            if self.collides(self.x, self.y, self.th + math.radians(deg) * k / 6):
                return True
        return False

    # --- modos (igual que src/main.cpp)
    def mode1(self, max_mm=None, side=-1):
        fw = self.c.fw
        start = self.travelled
        ignore = 0.0
        while max_mm is None or self.travelled - start < max_mm:
            ev = yield from self.drive(1e9, side, ignore)
            ignore = 0.0
            half = self.corridor_est / 2
            if ev == "hueco":
                yield from self.drive(max(fw.cfg_side_ahead + half - 5, 0), 0)
                yield from self.safe_turn(90 * side)
                yield from self.drive(max(half - fw.cfg_side_ahead + 15, 15), 0)
                ignore = half + fw.cfg_side_ahead
            elif ev in ("roce_izq", "roce_der"):
                yield from self.back(15)
                yield from self.rotate(-15 if ev == "roce_izq" else 15)
            elif ev == "atasco" and fw.unstick and not (0 <= self.readings["F"] < fw.front_wall):
                L, R = self.readings["L"], self.readings["R"]
                yield from self.back(25)
                yield from self.rotate(10 if (10000 if L < 0 else L) > (10000 if R < 0 else R) else -10)
            elif ev in ("pared", "atasco"):
                L, R = self.readings["L"], self.readings["R"]
                right_open = not (0 <= R < fw.side_wall)
                left_open = not (0 <= L < fw.side_wall)
                follow_open, other_open = (right_open, left_open) if side < 0 else (left_open, right_open)
                F = self.readings["F"]
                clear = self.front_turn() - F if F >= 0 else 0
                yield from self.back(max(10, clear))
                if follow_open:
                    yield from self.safe_turn(90 * side)
                elif other_open:
                    yield from self.safe_turn(-90 * side)
                else:
                    yield from self.safe_turn(180)
                ignore = half + fw.cfg_side_ahead

    def mode5(self):
        fw = self.c.fw
        space = lambda v: 10000 if v < 0 else v
        last_turn_at, in_place, ignore, stuck = self.travelled, 0, 0.0, 0
        while True:
            if in_place >= 3:
                self.loops += 1
                yield from self.mode1(max_mm=400)
                in_place, last_turn_at = 0, self.travelled
                continue
            ev = yield from self.drive(1e9, 2, ignore)
            ignore = 0.0
            if ev in ("roce_izq", "roce_der"):
                yield from self.back(15)
                yield from self.rotate(-15 if ev == "roce_izq" else 15)
                continue
            if ev not in ("hueco", "pared", "atasco"):
                continue
            if ev == "atasco" and fw.unstick and not (0 <= self.readings["F"] < fw.front_wall):
                stuck += 1
                if stuck < 3:
                    L, R = space(self.readings["L"]), space(self.readings["R"])
                    yield from self.back(25)
                    yield from self.rotate(10 if L > R else -10)
                    continue
                stuck = 0
                side = yield from self.back_out(3 * self.pitch)
                if side:
                    yield from self.safe_turn(90 * side)
                continue
            stuck = 0
            half = self.corridor_est / 2
            if ev == "hueco":
                yield from self.drive(max(fw.cfg_side_ahead + half - 5, 0), 0)
            for _ in range(8):   # parado: medidas nuevas
                yield
                self.tick()
            def side(key):
                v = self.readings[key]
                lv, lt = self.last_valid[key]
                if v < 0 and fw.weighted and lv < fw.close_mm and self.t - lt < 0.3:
                    return lv   # demasiado cerca, no libre
                return space(v)
            L, F, R = side("L"), space(self.readings["F"]), side("R")
            if fw.weighted:
                w = fw.w
                def score(mm, td):
                    return (w[0] * min(mm, 300) / 300 + w[1] / (1 + self.visits_toward(td))
                            - w[2] * abs(td) / 90 + (w[3] if td == 0 else 0))
                cand = []
                if F > fw.front_wall: cand.append((score(F, 0), 0))
                if L > fw.side_wall: cand.append((score(L, 90), 90))
                if R > fw.side_wall: cand.append((score(R, -90), -90))
                turn = max(cand)[1] if cand else 180
            else:
                turn, best = 0, (F if F > fw.front_wall else -1)
                if L > fw.side_wall and L > best:
                    best, turn = L, 90
                if R > fw.side_wall and R > best:
                    best, turn = R, -90
                if best < 0:
                    turn = 180
            if turn == 0:
                yield from self.drive(half + fw.cfg_side_ahead, 0)
                ignore = half
                continue
            Fr = self.readings["F"]
            if Fr >= 0 and self.front_turn() - Fr > 0:
                yield from self.back(self.front_turn() - Fr)
            in_place = in_place + 1 if self.travelled - last_turn_at < 60 else 1
            last_turn_at = self.travelled
            yield from self.safe_turn(turn)
            if turn != 180:
                yield from self.drive(max(half - fw.cfg_side_ahead + 15, 15), 0)
            ignore = half + fw.cfg_side_ahead

    def run(self):
        gen = self.mode5() if self.c.mode == 5 else self.mode1()
        steps = int(self.c.seconds / self.DT)
        for _ in range(steps):
            try:
                next(gen)
            except StopIteration:
                break
            if self.reached is not None:
                break
        return self


# ---------------------------------------------------------------- dibujo


def svg(sim, path):
    pitch, rows, cols = sim.pitch, sim.rows, sim.cols
    W, H = cols * pitch, rows * pitch
    sc = 560 / max(W, H)
    def P(x, y):
        return f"{(x + 20) * sc:.1f},{(H - y + 20) * sc:.1f}"
    out = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {(W + 40) * sc:.0f} {(H + 40) * sc:.0f}" '
           f'width="{(W + 40) * sc:.0f}" height="{(H + 40) * sc:.0f}" style="background:#f6f2ea">']
    for (a, b) in sim.segs:
        out.append(f'<line x1="{(a[0] + 20) * sc:.1f}" y1="{(H - a[1] + 20) * sc:.1f}" x2="{(b[0] + 20) * sc:.1f}" '
                   f'y2="{(H - b[1] + 20) * sc:.1f}" stroke="#8a6a3a" stroke-width="1.5"/>')
    gx, gy = sim.c.goal
    out.append(f'<rect x="{(gx * pitch + 20) * sc:.1f}" y="{(H - (gy + 1) * pitch + 20) * sc:.1f}" '
               f'width="{pitch * sc:.1f}" height="{pitch * sc:.1f}" fill="#2b9a66" opacity="0.25"/>')
    out.append('<polyline fill="none" stroke="#2f6fdf" stroke-width="1.5" points="' +
               " ".join(P(x, y) for x, y in sim.trail) + '"/>')
    for t, kind, x, y in sim.events:
        out.append(f'<circle cx="{(x + 20) * sc:.1f}" cy="{(H - y + 20) * sc:.1f}" r="4" fill="#c92a2a"/>')
    out.append(f'<polygon points="{" ".join(P(x, y) for x, y in sim.body())}" fill="#1d2330" opacity="0.8"/>')
    out.append("</svg>")
    with open(path, "w") as f:
        f.write("\n".join(out))


# ---------------------------------------------------------------- casos

M_L = [            # sube 3 celdas y gira a la izquierda hasta la meta (0,2)
    "+---+---+---+",
    "|           |",
    "+---+---+   +",
    "|       |   |",
    "+---+---+   +",
    "|       |   |",
    "+---+---+---+",
]
M_T = [            # sube y llega a una T: a la derecha callejon, a la izquierda la meta
    "+---+---+---+---+---+",
    "|                   |",
    "+---+---+   +---+---+",
    "|       |   |       |",
    "+---+---+   +---+---+",
    "|       |   |       |",
    "+---+---+---+---+---+",
]
M_DEAD = [         # cruce: a la derecha callejon sin salida, la meta arriba a la izquierda
    "+---+---+---+",
    "|   |   |   |",
    "+   +---+---+",
    "|           |",
    "+---+   +---+",
    "|   |   |   |",
    "+---+---+---+",
]


def comp_maze():
    here = os.path.dirname(os.path.abspath(__file__))
    src = open(os.path.join(here, "..", "src", "competition_maze.h")).read()
    return [l.split('"')[1] for l in src.splitlines() if l.strip().startswith('"+') or l.strip().startswith('"|')]


def cases():
    real = Robot()
    centred = replace(real, length_front=50, length_back=50, side_ahead=25, front_ahead=10)
    inset = replace(real, side_inset=15)
    base_fw = Firmware()
    fixed_fw = replace(base_fw, cfg_side_ahead=56, cfg_front_ahead=33, turn_mode="dos_tiempos", unstick=True)
    out = []
    for fw_name, fw in (("actual", base_fw), ("dos tiempos", fixed_fw)):
        tag = lambda n: f"{n} [{fw_name}]"
        out += [
            Case(tag("L estrecha 14 cm"), M_L, (2, 0), (0, 2), fw=fw),
            Case(tag("Cruce en T 14 cm"), M_T, (2, 0), (0, 2), fw=fw),
            Case(tag("Callejon sin salida"), M_DEAD, (1, 0), (0, 2), fw=fw),
            Case(tag("Pasillo ancho 22 cm, torcido 10°"), M_L, (2, 0), (0, 2), corridor=220, off_x=25, off_deg=10, fw=fw),
            Case(tag("Ruido alto + 15% perdidas"), M_L, (2, 0), (0, 2), sensor=Sensor(noise=8, dropout=0.15), fw=fw),
            Case(tag("Laser izquierdo roto"), M_T, (2, 0), (0, 2), sensor=Sensor(blind_left=True), fw=fw),
            Case(tag("Rueda floja (deriva 6°/s)"), M_L, (2, 0), (0, 2), drift_dps=6, fw=fw),
            Case(tag("Mano a 8 cm durante 3 s"), M_L, (2, 0), (0, 2), hand=(2.5 * 155, 240, 1.0, 4.0), fw=fw),
            Case(tag("Laterales hundidos 15 mm"), M_L, (2, 0), (0, 2), robot=inset, fw=fw),
            Case(tag("Eje de ruedas centrado"), M_L, (2, 0), (0, 2), robot=centred,
                 fw=replace(fw, cfg_side_ahead=25 if fw_name == "corregido" else fw.cfg_side_ahead,
                            cfg_front_ahead=10 if fw_name == "corregido" else fw.cfg_front_ahead)),
            Case(tag("Laberinto reglamento, modo 1"), comp_maze(), (18, 0), (4, 2), mode=1, seconds=240, fw=fw),
            Case(tag("Laberinto reglamento, modo 5"), comp_maze(), (18, 0), (4, 2), mode=5, seconds=240, fw=fw),
        ]
    return out


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    outdir = os.path.join(here, "sim_resultados")
    os.makedirs(outdir, exist_ok=True)
    rows = []
    for i, case in enumerate(cases()):
        sim = Sim(case).run()
        name = f"caso{i:02d}.svg"
        svg(sim, os.path.join(outdir, name))
        ok = sim.reached is not None
        rows.append((case.name, ok, sim.reached, sim.collisions, sim.turns, sim.loops, name))
        print(f"{case.name:48s} {'LLEGA' if ok else 'NO   '} "
              f"t={sim.reached or case.seconds:6.1f}s choques={sim.collisions:3d} giros={sim.turns:3d} bucles={sim.loops}")
    html = ['<!doctype html><meta charset="utf-8"><title>Casos extremos</title>',
            '<style>body{font:14px system-ui;margin:20px;background:#fff;color:#1d2330}'
            '.g{display:grid;grid-template-columns:repeat(auto-fill,minmax(320px,1fr));gap:16px}'
            '.c{border:1px solid #ddd;border-radius:10px;padding:10px}.ok{color:#2b9a66}.no{color:#c92a2a}'
            'img{width:100%}</style><h1>Simulación de casos extremos</h1>'
            '<p>Línea azul = recorrido · puntos rojos = choques · verde = meta · rectángulo oscuro = ratón al final.</p>'
            '<div class="g">']
    for name, ok, t, col, turns, loops, img in rows:
        html.append(f'<div class="c"><b>{name}</b><br><span class="{"ok" if ok else "no"}">'
                    f'{"Llega en %.1f s" % t if ok else "No llega"}</span> · choques {col} · giros {turns} · bucles {loops}'
                    f'<img src="{img}"></div>')
    html.append("</div>")
    with open(os.path.join(outdir, "index.html"), "w") as f:
        f.write("\n".join(html))
    print("\nInforme:", os.path.join(outdir, "index.html"))


if __name__ == "__main__":
    main()

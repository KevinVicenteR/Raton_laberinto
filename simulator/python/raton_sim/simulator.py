#!/usr/bin/env python3
"""Simulador 2D de casos extremos del raton (sin dependencias).

Reproduce la logica del firmware de los modos 1 (seguir pared derecha) y 5 (espacio libre) con:
  - cuerpo de 80 x 80 mm, eje central y laterales diagonales a 31 grados,
  - un modelo de VL53L0X (rayo efectivo identificado, minimo fiable 30 mm, alcance maximo, ruido, perdidas),
  - laberintos de prueba en el mismo formato de dibujo que firmware/app/competition_maze.h.

Uso:  python -m raton_sim casos      -> tabla de resultados y simulator/results/casos/index.html
Los parametros siguen a firmware/config.h; los casos y las variantes estan al final del archivo.
"""

import math
import os
import random
from collections import deque
from dataclasses import dataclass, field, replace

from . import native as N
from . import orchestrators as O

# ---------------------------------------------------------------- parametros (como config.h)


@dataclass
class Robot:
    length_front: float = 40.0
    length_back: float = 40.0
    width: float = 80.0
    side_ahead: float = 30.0
    side_inset: float = 10.0
    side_angle: float = 31.0
    front_ahead: float = 40.0
    deadzone: float = 50.0

    @classmethod
    def preset(cls, name="real"):
        if name == "antiguo":
            return cls(83, 17, 92, 56, 0, 90, 33, 50)
        if name != "real":
            raise ValueError(name)
        return cls()


@dataclass
class Sensor:
    max_mm: float = 300.0  # LASER_MAX_MM
    min_mm: float = 30.0  # por debajo, el VL53L0X da basura o nada
    noise: float = 3.0  # desviacion tipica (mm)
    dropout: float = 0.02  # probabilidad de lectura perdida
    cone_deg: float = 0.0  # identified effective ray, not nearest of an ideal cone
    distance_dropout: bool = True  # wood: 2% below 100, 30% from 200 mm
    mounting_error: float = 5.0  # independent fixed error per side, degrees
    latency_ms: float = 0.0  # additional simulated delivery latency; not a measured calibration
    blind_left: bool = False  # laser izquierdo roto (nunca mide)


@dataclass
class Firmware:
    geometry: object = None
    cell_side_wall: float = 96.0
    side_look_ahead: float = 139.84
    explore_speed: float = 150.0
    front_factor: float = 0.35
    kp_yaw: float = 0.014  # firmware scale; linearized plant 6 rad/s/rad at .014
    speed: float = 100.0
    side_wall: float = 80.0  # SIDE_WALL_MM = SIDE_OPEN_MM
    front_wall: float = 140.0  # FRONT_WALL_MM
    front_margin: float = 15.0
    cfg_side_ahead: float = 30.0  # lo que cree el firmware (SIDE_SENSOR_AHEAD_MM)
    cfg_front_ahead: float = 40.0
    robot_width_cfg: float = 80.0
    kp_wall: float = 0.15
    max_wall_deg: float = 4.0
    deadband: float = 8.0
    side_crash: float = 8.0
    raw_survey: bool = False  # experimental: classify stationary walls from acquired samples
    survey_samples: int = 5  # fresh stationary observations per new cell
    goal_grid: bool = False  # experimental mode 5 with a goal uses persistent cell exploration
    cell_min_samples: int = 2  # valid side samples required in the cell window
    rub_seconds: float = 0.1
    skew_limit: float = 10.0
    max_rubs: int = 3
    max_no_progress: int = 3
    turn_attempts: int = 3
    cell_turn_retry: bool = True
    cell_scrape_recovery: bool = True
    precision_turns: bool = False  # experimental gyro refinement before each diagonal
    turn_timeout: float = 4.0  # legacy baseline; main bank imports TURN_TIMEOUT_MS
    turn_mode: str = "sitio"  # "sitio" = gira sobre el eje; "seguro" = retrocede antes;
    # "dos_tiempos" = gira 70, avanza, gira 20 (y sale marcha atras de callejones)
    split_deg: float = 70.0
    split_advance: float = 75.0
    close_mm: float = 60.0  # -1 tras una lectura por debajo de esto = demasiado cerca
    comp_speed: float = 250.0  # modo 4: velocidad de los tramos
    corner_front: float = 65.0  # CORNER_FRONT_MM: parar con el laser a esto de la pared para poder girar
    turn_lateral: float = (
        -10.0
    )  # TURN_LATERAL_MM: eje al acabar el tramo del giro, respecto al centro del pasillo nuevo
    adaptive_split: bool = False  # angulo del primer tiempo segun donde esta el eje (si no, split_deg)
    edge_fix: bool = False  # corregir la posicion con los finales de pared laterales
    edge_bias_open: float = 12.0  # EDGE_OPEN_MM: laser pasado el borde de celda al ver que se acaba la pared
    edge_bias_wall: float = 12.0  # EDGE_WALL_MM: laser antes del borde al ver que empieza la pared
    turn_lat_comp: bool = False  # alargar/acortar el tramo del giro segun el descentrado lateral
    robust_lateral: bool = False  # centrado: si L+R no cuadran con el pasillo, solo la lectura corta
    corner_leg: bool = True  # tramo del giro sin roces ni centrado
    fused_stop: bool = False  # parada ante pared: mitad encoders, mitad laser
    align_front: bool = False  # ajustar con el frontal antes de girar si hay pared delante
    reverse_short: float = 25.0  # REVERSE_SHORT_MM: marcha atras, parar antes para que la cola quepa al girar
    w_goal: float = 0.6  # modo 5: peso de la direccion hacia la meta
    w_exit: float = 0.7  # modo 5: peso de las salidas ya usadas (Tremaux)
    unstick: bool = False  # desatasco: atrás y enderezarse si se atasca con el frente libre
    weighted: bool = False  # decision ponderada en los cruces (como el firmware)
    w: tuple = (1.0, 0.8, 0.3, 0.2)  # espacio, novedad, giro, sesgo recto


@dataclass
class Case:
    name: str
    maze: list
    start: tuple  # celda (x, y) contando desde abajo a la izquierda
    goal: tuple
    corridor: float = field(default_factory=lambda: float(os.environ.get("RATON_CORRIDOR_MM", 192)))  # mm
    wall: float = 12.0  # grosor de pared
    mode: int = 5
    off_x: float = 0.0  # descentrado inicial (mm)
    off_deg: float = 0.0  # torcido inicial (grados)
    drift_dps: float = 0.0  # deriva del rumbo (rueda mas floja), grados/s
    robot: Robot = field(default_factory=Robot)
    sensor: Sensor = field(default_factory=Sensor)
    fw: Firmware = field(default_factory=Firmware)
    hand: tuple = None  # obstaculo movil: (x, y, desde_s, hasta_s) en mm
    seconds: float = 90.0
    known: list = None  # mapa que cree conocer el raton (modo 4); None = el real
    controller: str = "baseline"  # baseline, corrected
    sweep_guard: bool = True
    trace: bool = False
    use_goal: bool = False  # modo 5 con meta (direccion hacia la meta + memoria de cruces)


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
            if line[4 * x + 2] == "-":
                rects.append((x * pitch - h, y - h, (x + 1) * pitch + h, y + h))
    for r in range(rows):
        line = case.maze[2 * r + 1]
        y0, y1 = (rows - r - 1) * pitch, (rows - r) * pitch
        for x in range(cols + 1):
            if line[4 * x] == "|":
                rects.append((x * pitch - h, y0 - h, x * pitch + h, y1 + h))
    segs = []
    for x0, y0, x1, y1 in rects:
        segs += [((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)), ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))]
    return segs, pitch, rows, cols


# ---------------------------------------------------------------- simulacion


class MotionFailure(N.MotionFailureBase):
    """Movimiento incompleto: no continuar actualizando la posicion logica."""


class Sim:
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

    def sample(self):
        r, sn = self.c.robot, self.c.sensor
        c, s = math.cos(self.th), math.sin(self.th)
        half = r.width / 2 - r.side_inset
        mounts = {
            "F": (self.x + r.front_ahead * c, self.y + r.front_ahead * s, self.th),
            "L": (
                self.x + r.side_ahead * c - half * s,
                self.y + r.side_ahead * s + half * c,
                self.th + math.radians(r.side_angle + self.mount_errors["L"]),
            ),
            "R": (
                self.x + r.side_ahead * c + half * s,
                self.y + r.side_ahead * s - half * c,
                self.th - math.radians(r.side_angle + self.mount_errors["R"]),
            ),
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
                elif self.rnd.random() >= (
                    min(
                        0.95,
                        sn.dropout + (0.28 * min(1, max(0, (d - 100) / 100)) if sn.distance_dropout else 0),
                    )
                ):
                    val = max(0, d + self.rnd.gauss(0, sn.noise))
            if sn.latency_ms > 0:
                self.delayed_samples[key].append((self.t + sn.latency_ms / 1000, val))
                val = -1
                while self.delayed_samples[key] and self.delayed_samples[key][0][0] <= self.t + 1e-9:
                    _, val = self.delayed_samples[key].popleft()
            self.raw_readings[key] = val
            self.hist[key] = (self.hist[key] + [val])[-3:]
            if self.c.controller != "baseline":
                h = sorted(100000 if v < 0 else v for v in self.hist[key])
                val = h[len(h) // 2]
                if val == 100000:
                    val = -1
            prev = self.prev_raw[key]
            stable = (prev < 0) == (val < 0) and (val < 0 or abs(val - prev) <= 25)
            self.sensor_conf[key] += 0.05 * (float(stable) - self.sensor_conf[key])
            self.prev_raw[key] = val
            if val < 0:  # LASER_MAX_MISS = 2: se mantiene la ultima medida un par de lecturas
                self.miss[key] += 1
                if self.miss[key] >= 2:
                    self.readings[key] = -1
            else:
                self.miss[key] = 0
                self.readings[key] = val
                self.last_valid[key] = (val, self.t)
                self.last_valid_at[key] = self.travelled
        self.nsamples += 1
        elapsed = max(self.t - self._sample_time, self.DT)
        self.observed_speed = (self.travelled - self._sample_distance) / elapsed
        self.observed_yaw = math.degrees(self.th - self._sample_heading) / elapsed
        self._sample_distance, self._sample_heading, self._sample_time = self.travelled, self.th, self.t
        frame = self.observation()
        if self.c.trace:
            self.trace.append(
                dict(
                    t=self.t,
                    raw=self.raw_readings.copy(),
                    filtered=self.readings.copy(),
                    sensors=frame,
                    encoder_mm=self.travelled,
                    heading_deg=math.degrees(self.th),
                    truth=(self.x, self.y, self.th),
                )
            )

    def observation(self):
        return [
            (
                self.readings[k],
                self.readings[k] >= 0,
                self.sensor_conf[k],
                max(0, (self.t - self.last_valid[k][1]) * 1000),
            )
            for k in ("L", "F", "R")
        ]

    def prepare_turn(self, degrees):
        if self.c.robot.side_angle != 90 or self.c.controller == "baseline" or not self.c.sweep_guard:
            return
        entry = self.native.turn(self.observation(), self.c.robot, self.c.fw, degrees)
        if entry is None:
            raise MotionFailure("obstaculo observado en el barrido del giro")
        start = self.travelled
        if entry < 0:
            yield from self.back(-entry)
        elif entry > 0:
            yield from self.drive(entry, 0)
        if abs(self.travelled - start - entry) > 3:
            raise MotionFailure("ajuste de entrada incompleto")

    def odo_cell(self, x=None, y=None):
        pitch = self.corridor_est + 15
        x = self.x if x is None else x
        y = self.y if y is None else y
        return (math.floor(x / pitch + 0.5), math.floor(y / pitch + 0.5))

    def world_dir(self, turn_deg):
        h = math.degrees(self.th) - 90 + turn_deg
        return int(round(h / 90)) % 4  # 0 norte, 1 oeste, 2 sur, 3 este

    def goal_alignment(self, turn_deg):
        if not self.c.use_goal:
            return 0.0
        gx = (self.c.goal[0] + 0.5) * self.pitch - self.x
        gy = (self.c.goal[1] + 0.5) * self.pitch - self.y
        n = math.hypot(gx, gy) or 1
        dx, dy = [(0, 1), (-1, 0), (0, -1), (1, 0)][self.world_dir(turn_deg)]
        return (dx * gx + dy * gy) / n

    def exits_toward(self, turn_deg):
        return self.exits.get((self.odo_cell(), self.world_dir(turn_deg)), 0)

    def mark_exit(self, turn_deg):
        k = (self.odo_cell(), self.world_dir(turn_deg))
        self.exits[k] = self.exits.get(k, 0) + 1

    def visits_toward(self, turn_deg):
        h = math.degrees(self.th) - 90 + turn_deg  # 0 = norte, positivo a la izquierda
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
        return max(
            self.corridor_est / 2 - fw.cfg_front_ahead + self.c.robot.deadzone,
            self.c.robot.deadzone + fw.front_margin,
        )

    # --- primitivas del firmware (generadores: un paso por yield)
    @N.instrument
    def drive(self, max_mm, watch, ignore=0.0, corner=False):
        """corner=True: tramo recto en mitad de un giro (en diagonal): sin roces ni centrado."""
        fw = self.c.fw
        v, start, moving_at, open_l, open_r = 0.0, self.travelled, self.t, None, None
        last_ok = {"L": 100.0, "R": 100.0}  # ultima lectura valida de cada lateral
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
            narrow = corner or (L >= 0 and R >= 0 and abs(L - R) < 10)
            if 0 <= L < fw.side_crash and not narrow and (R < 0 or L < R):
                return "roce_izq"
            if 0 <= R < fw.side_crash and not narrow and (L < 0 or R < L):
                return "roce_der"
            if not corner and not front_near and 0 <= L <= fw.side_wall and 0 <= R <= fw.side_wall:
                self.corridor_est = min(
                    400, max(80, 0.98 * self.corridor_est + 0.02 * (L + R + fw.robot_width_cfg))
                )
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
            v = min(v + 800 * self.DT, vmax, max(40, math.sqrt(max(0, 1600 * remaining))))
            # centrado
            offset = None
            hasL, hasR = 0 <= L < fw.side_wall, 0 <= R < fw.side_wall
            center = (self.corridor_est - fw.robot_width_cfg) / 2
            if not front_near and not corner:
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
            err = (math.radians(yaw_t) - (self.th - th0) + math.pi) % (2 * math.pi) - math.pi
            omega = self.motion.turn(err, fw.kp_yaw) + math.radians(self.c.drift_dps)
            nth = self.th + omega * self.DT
            nx = self.x + v * self.DT * math.cos(nth)
            ny = self.y + v * self.DT * math.sin(nth)
            if self.try_move(nx, ny, nth):
                if v > 20:
                    moving_at = self.t
            if (
                self.motion.safety(0, fw.rub_seconds, 0, fw.max_rubs, stalled=self.t - moving_at > 0.6)
                == "veto"
            ):
                return "atasco"

    @N.instrument
    def rotate(self, deg):
        self.turns += 1
        target = self.th + math.radians(deg) + math.radians(self.rnd.gauss(0, 1.5))
        t0, blocked_since, retries = self.t, None, 0
        while (
            abs(target - self.th) > math.radians(0.5 if self.c.controller == "baseline" else 3)
            and self.t - t0 < self.c.fw.turn_timeout
        ):
            yield
            self.tick()
            step = math.copysign(min(abs(target - self.th), math.radians(180) * self.DT), target - self.th)
            if self.try_move(self.x, self.y, self.th + step):
                blocked_since = None
            else:
                blocked_since = blocked_since if blocked_since is not None else self.t
                # Giro bloqueado (una esquina toca la pared): atras 15 mm y seguir girando.
                if (
                    self.c.fw.unstick
                    and self.c.controller == "baseline"
                    and self.t - blocked_since > 0.3
                    and retries < 3
                ):
                    retries += 1
                    blocked_since = None
                    yield from self.back(15)

        if abs(target - self.th) > math.radians(3):
            raise MotionFailure("giro incompleto")

    def rotate_precise_to(self, target):
        """Coarse move retains actuator error; gyro feedback then corrects the observed error.

        Only gyro heading enters the fine loop. Collision tests and sensor noise remain active.
        This models an additional control loop, not a quieter gyro or a smaller robot.
        """

        def error():
            return (target - self.th + math.pi) % (2 * math.pi) - math.pi

        yield from self.rotate(math.degrees(error()))
        start = self.t
        while abs(error()) > math.radians(0.5) and self.t - start < 1.0:
            yield
            self.tick()
            step = math.copysign(min(abs(error()), math.radians(30) * self.DT), error())
            # Fine-stage actuator disturbance; feedback does not get access to any walls.
            step += math.radians(self.rnd.gauss(0, 1.5)) * self.DT
            self.try_move(self.x, self.y, self.th + step)
        if abs(error()) > math.radians(0.5):
            raise MotionFailure("refinamiento angular incompleto")

    @N.instrument
    def back(self, mm):
        start = self.travelled
        t0 = self.t
        while start - self.travelled < mm and self.t - t0 < 1.0:
            yield
            self.tick()
            nx = self.x - 120 * self.DT * math.cos(self.th)
            ny = self.y - 120 * self.DT * math.sin(self.th)
            self.try_move(nx, ny, self.th)

    @N.instrument
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
                        self.try_move(
                            self.x - 100 * self.DT * math.cos(self.th),
                            self.y - 100 * self.DT * math.sin(self.th),
                            self.th,
                        )
                    return 1 if key == "L" else -1
            self.try_move(
                self.x - 100 * self.DT * math.cos(self.th),
                self.y - 100 * self.DT * math.sin(self.th),
                self.th,
            )
        return 0

    @N.instrument
    def safe_turn(self, deg):
        """Variantes de giro segun fw.turn_mode."""
        fw = self.c.fw
        if fw.turn_mode == "dos_tiempos":
            if abs(deg) == 90:
                yield from self.prepare_turn(deg)
                sgn = 1 if deg > 0 else -1
                heading = self.th
                if fw.precision_turns:
                    yield from self.rotate_precise_to(heading + math.radians(sgn * fw.split_deg))
                else:
                    yield from self.rotate(sgn * fw.split_deg)
                yield from self.drive(fw.split_advance, 0, corner=fw.corner_leg)
                if fw.precision_turns:
                    yield from self.rotate_precise_to(heading + math.radians(sgn * 90))
                else:
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

    # --- modos (igual que firmware/app)
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
        if self.c.fw.goal_grid and self.c.use_goal:
            yield from self.explore(tuple(self.c.start), 0)
            return
        fw = self.c.fw

        def space(v):
            return 10000 if v < 0 else v

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
            for _ in range(8):  # parado: medidas nuevas
                yield
                self.tick()

            def side(key):
                v = self.readings[key]
                lv, lt = self.last_valid[key]
                if v < 0 and fw.weighted and lv < fw.close_mm and self.t - lt < 0.3:
                    return lv  # demasiado cerca, no libre
                return space(v)

            L, F, R = side("L"), space(self.readings["F"]), side("R")
            if fw.weighted:
                w = fw.w

                def score(mm, td):
                    return (
                        w[0] * min(mm, 300) / 300
                        + w[1] / (1 + self.visits_toward(td))
                        - w[2] * abs(td) / 90
                        + (w[3] if td == 0 else 0)
                        + fw.w_goal * self.goal_alignment(td)
                        - fw.w_exit * self.exits_toward(td)
                    )

                cand = []
                if F > fw.front_wall:
                    cand.append((score(F, 0), 0))
                if L > fw.side_wall:
                    cand.append((score(L, 90), 90))
                if R > fw.side_wall:
                    cand.append((score(R, -90), -90))
                turn = max(cand)[1] if cand else 180
                if turn != 180:
                    self.mark_exit(turn)
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

    # --- modo 4: ruta del mapa conocido por tramos (igual que competitionRun del firmware)
    @N.instrument
    def run_cells(self, n, vmax, reverse=False, short=0.0, on_cell=None):
        """Avanza (o retrocede) n celdas. short: marcha atras, parar antes (pared detras en la celda
        de llegada: la cola no cabria al girar). Deja en self.along donde queda el eje."""
        fw, r = self.c.fw, self.c.robot
        cell = self.cell_mm if self.c.mode == 3 else self.pitch
        sgn = -1 if reverse else 1
        axle_back = (r.length_front - r.length_back) / 2
        # pending: cuanto va el centro del raton por delante del centro de la celda (en su rumbo).
        ofs = self.pending
        target = max(n * cell + ofs - short if reverse else n * cell - ofs, 0.0)
        start, v, moving_at, unexpected = self.travelled, self.explore_v, self.t, False
        th0, yaw_t = (self.grid_th if self.grid_th is not None else self.th), 0.0
        # Final de pared lateral (poste): el laser lateral pasa por el borde de la celda en una
        # posicion conocida -> corrige el error acumulado de los encoders.
        shift, edge, last_n, _last_pos = 0.0, {"L": None, "R": None}, self.nsamples, 0.0
        self.approach_sides = {}
        next_sample = 1
        side_samples = [[], []]
        brake_cell = False
        scrape_since, recoveries = None, 0
        lead = fw.cfg_side_ahead - axle_back  # laser lateral por delante del centro
        while True:
            yield
            self.tick()
            pos = sgn * (self.travelled - start)
            fresh_sample = self.nsamples != last_n
            if not reverse and fresh_sample:
                last_n = self.nsamples
                for k in ("L", "R"):
                    val = self.readings[k]
                    cls = "w" if 30 <= val < 100 else ("o" if val < 0 or val >= 130 else None)
                    st = edge[k]
                    if cls is None:
                        continue
                    if st is None or st[0] != cls:
                        edge[k] = (cls, 1, pos)
                        continue
                    edge[k] = (cls, st[1] + 1, st[2])
                    need = 3 if cls == "o" else 2  # muy cerca el laser da huecos falsos sueltos
                    if st[1] + 1 == need and self.edge_prev.get(k) is not None and self.edge_prev[k] != cls:
                        # Confirmado el cambio: ocurrio entre la muestra anterior al primer cambio y esta.
                        at = st[2] - 2.5
                        if cls == "o" and self.last_valid[k][0] < 32:
                            self.edge_prev[k] = cls
                            continue  # venia pegado a la pared: no es fiable
                        laser = ofs + at + shift + lead
                        bias = fw.edge_bias_open if cls == "o" else -fw.edge_bias_wall
                        kc = round((laser - bias - cell / 2) / cell)
                        err = (kc + 0.5) * cell + bias - laser
                        hx, hy = math.cos(th0), math.sin(th0)
                        lw = (self.x + fw.cfg_side_ahead * math.cos(self.th)) * hx + (
                            self.y + fw.cfg_side_ahead * math.sin(self.th)
                        ) * hy
                        lw -= 2.5 + (pos - at)  # donde estaba al ocurrir el cambio
                        true_ofs = (lw + cell / 2) % cell - cell / 2
                        self.edge_log.append((cls, round(err, 1), round(true_ofs, 1)))
                        if fw.edge_fix and abs(err) < 35:
                            shift += err
                    if st[1] + 1 >= need:
                        self.edge_prev[k] = cls
            heading_error = abs(math.degrees((self.th - th0 + math.pi) % (2 * math.pi) - math.pi))
            scrape = (
                any(0 <= self.side_face(self.readings[k]) < fw.side_crash for k in ("L", "R"))
                or heading_error > fw.skew_limit
            )
            scrape_since = (self.t if scrape_since is None else scrape_since) if scrape else None
            if (
                fw.cell_scrape_recovery
                and not reverse
                and scrape_since is not None
                and self.motion.safety(self.t - scrape_since, fw.rub_seconds, recoveries, fw.max_rubs)
                != "run"
            ):
                recoveries += 1
                self.recoveries += 1
                if recoveries >= fw.max_rubs:
                    raise MotionFailure("PEGADO")
                yield from self.back(15)
                yield from self.cell_rotate_to(th0)
                v, yaw_t, moving_at, scrape_since = 0, 0, self.t, None
                continue
            pos += shift
            at = next_sample * cell - fw.side_look_ahead
            if not reverse and next_sample <= n and pos + ofs >= at - 12:
                for i, key in enumerate(("L", "R")):
                    face = self.side_face(self.readings[key])
                    if face >= 0:
                        if fresh_sample and pos + ofs <= at + 12:
                            side_samples[i].append(face)
                if pos + ofs >= at + 12:
                    sides = tuple(
                        self.motion.classify(v, fw.cell_side_wall, fw.cell_min_samples) for v in side_samples
                    )
                    self.approach_sides[next_sample] = sides
                    if on_cell is not None:
                        brake_cell = (
                            on_cell(
                                next_sample,
                                sides,
                                self.readings["F"]
                                if self.motion.confirmed_front(self.readings["F"], self.nsamples)
                                else -1,
                            )
                            is False
                        )
                    side_samples = [[], []]
                    next_sample += 1
            remaining = target - pos
            F = self.readings["F"]
            confirmed = self.motion.confirmed_front(F, self.nsamples)
            stop = self.front_turn()
            if not reverse and confirmed and 0 <= F < fw.front_wall:
                if remaining > F + cell / 2:
                    unexpected, remaining = True, F - max(35, stop, self.front_turn())
                elif remaining < cell:
                    # Ultima celda con pared: mitad encoders, mitad laser (el cono del laser ve cosas
                    # de lado y acorta), y nunca mas cerca que la zona muerta.
                    remaining = (
                        min(0.5 * remaining + 0.5 * (F - fw.corner_front), F - stop)
                        if fw.fused_stop
                        else F - max(35, stop, self.front_turn())
                    )
            if remaining <= 1:
                break
            offset = None if reverse else self.lateral(self.readings["L"], self.readings["R"])
            target_deg, v = self.motion.tick(offset, fw, v, vmax, remaining, self.DT, brake_cell)
            yaw_t += 0.1 * (sgn * target_deg - yaw_t)  # marcha atras: la correccion va al reves
            err = (math.radians(yaw_t) - (self.th - th0) + math.pi) % (2 * math.pi) - math.pi
            omega = self.motion.turn(err, fw.kp_yaw) + math.radians(self.c.drift_dps)
            nth = self.th + omega * self.DT
            vs = sgn * (v if not reverse else min(v, 150))
            if (
                self.try_move(
                    self.x + vs * self.DT * math.cos(nth), self.y + vs * self.DT * math.sin(nth), nth
                )
                and v > 20
            ):
                moving_at = self.t
            if (
                self.motion.safety(0, fw.rub_seconds, 0, fw.max_rubs, stalled=self.t - moving_at > 0.6)
                == "veto"
            ):
                unexpected = True
                break
        self.explore_v = v
        done = sgn * (self.travelled - start) + shift
        self.edge_prev = {}
        moved = max(0, min(n, int(math.floor((done - ofs if reverse else done + ofs) / cell + 0.5))))
        # Donde queda el centro del raton respecto al de la celda: con pared delante lo mide el laser;
        # si no, encoders (lo que se paso o le falto).
        F = self.readings["F"] if self.motion.confirmed_front(self.readings["F"], self.nsamples) else -1
        if not reverse and 0 <= F < fw.front_wall:
            self.pending = self.corridor_est / 2 - F + r.deadzone - fw.cfg_front_ahead + axle_back
        elif reverse:
            self.pending = moved * cell + ofs - done
        else:
            self.pending = done + ofs - moved * cell
        self.along = self.pending - axle_back
        return min(moved, n), unexpected

    def side_face(self, raw):
        r = self.c.robot
        return raw * math.sin(math.radians(r.side_angle)) - r.side_inset if raw >= 0 else -1

    def lateral(self, L, R):
        """Desviacion lateral (+ = hay que ir a la izquierda). Muy cerca (<30 mm) el VL53L0X mide de
        mas: si las dos paredes no suman el ancho del pasillo, solo se cree la lectura mas corta."""
        fw = self.c.fw
        validL, validR = L >= 0, R >= 0
        L, R = self.side_face(L), self.side_face(R)
        hasL, hasR = validL and L < fw.side_wall, validR and R < fw.side_wall
        center = (self.corridor_est - fw.robot_width_cfg) / 2
        if not fw.robust_lateral:
            center = (self.corridor_est - fw.robot_width_cfg) / 2
            return (L - R) / 2 if hasL and hasR else (L - center if hasL else (center - R if hasR else None))
        if hasL and hasR:
            if abs(L + R + fw.robot_width_cfg - self.corridor_est) <= 20:
                return (L - R) / 2
            return L - center if L < R else center - R
        if hasL:
            return L - center
        if hasR:
            return center - R
        return None

    def align_front(self):
        """Align a visible front wall to the nominal cell stop distance before turning."""
        fw = self.c.fw
        for _ in range(3):
            got = []
            for _ in range(3):
                for _ in range(4):
                    yield
                    self.tick()
                got.append(self.readings["F"])
            valid = [v for v in got if v >= 0]
            if not valid or min(valid) >= fw.front_wall:
                return False
            stop_distance = self.front_turn()
            delta = sorted(valid)[len(valid) // 2] - stop_distance
            if abs(delta) <= 3:
                break
            delta = max(-30.0, min(30.0, delta))
            start, t0 = self.travelled, self.t
            th0 = self.grid_th if self.grid_th is not None else self.th
            while abs(self.travelled - start) < abs(delta) - 0.5 and self.t - t0 < 1.0:
                yield
                self.tick()
                v = math.copysign(60.0, delta)
                err = (th0 - self.th + math.pi) % (2 * math.pi) - math.pi
                nth = self.th + max(-1.5, min(1.5, 6 * err)) * self.DT
                self.try_move(self.x + v * self.DT * math.cos(nth), self.y + v * self.DT * math.sin(nth), nth)
        axle_back = (self.c.robot.length_front - self.c.robot.length_back) / 2
        self.along = self.corridor_est / 2 + self.c.robot.deadzone - stop_distance - fw.cfg_front_ahead
        self.pending = self.along + axle_back
        return True

    @N.instrument
    def turn_corner_track(self, deg):
        self.explore_v = 0.0
        yield from self.prepare_turn(deg)
        target = (self.grid_th if self.grid_th is not None else self.th) + math.radians(deg)
        yield from self.cell_rotate_to(target)
        self.grid_th = target
        self.pending = self.along = 0.0

    def wait_fresh_front(self):
        # Eight independent deliveries AND 240 ms: delayed/held values must expire.
        count, until = self.nsamples + 8, self.t + 0.24
        while self.nsamples < count or self.t < until:
            yield
            self.tick()
        self.readings["F"] = self.raw_readings["F"]
        self.fresh_front = True
        self.motion.front_count = 0

    def cell_rotate_to(self, target):
        yield from self._cell_rotate_to(target)
        yield from self.wait_fresh_front()

    def _cell_rotate_to(self, target):
        if not self.c.fw.cell_turn_retry:
            # Legacy acceptance: a stopped turn within twenty degrees proceeds.
            try:
                yield from self.rotate(math.degrees(target - self.th))
            except MotionFailure:
                if abs(math.degrees(target - self.th)) > 20:
                    raise
            return
        # Keep the established fine feedback, retry a stopped coarse/fine turn twice.
        for attempt in self.mission.attempts(self.c.fw.turn_attempts):
            try:
                yield from self.rotate_precise_to(target)
            except MotionFailure:
                pass
            error = abs(math.degrees((target - self.th + math.pi) % (2 * math.pi) - math.pi))
            if error <= 5:
                break
        error = abs(math.degrees((target - self.th + math.pi) % (2 * math.pi) - math.pi))
        if error > 10:
            raise MotionFailure("giro incompleto")
        if error > 3:
            try:
                yield from self.rotate_precise_to(target)
            except MotionFailure:
                if abs(math.degrees(target - self.th)) > 10:
                    raise

    def measure_cell(self):
        readings = []
        for degrees in (90, -180, 90):
            yield from self.turn_corner_track(degrees)
            values = []
            for _ in range(40):
                yield
                self.tick()
                value = self.readings["F"]
                if value >= 0:
                    values.append(value)
            readings.append(sorted(values)[len(values) // 2] if values else -1)
        left, right = readings[:2]
        measured = left + right - 2 * self.c.robot.deadzone + 2 * self.c.fw.cfg_front_ahead + self.c.wall
        if left >= 0 and right >= 0 and 140 <= measured <= 400:
            self.cell_mm = measured
            self.corridor_est = measured - self.c.wall
        else:
            self.cell_mm = 155
            self.corridor_est = 155 - self.c.wall
        half = self.corridor_est / 2
        self.c = replace(
            self.c,
            fw=replace(
                self.c.fw,
                side_look_ahead=self.c.robot.side_ahead
                + (half - self.c.robot.width / 2 + self.c.robot.side_inset)
                / math.tan(math.radians(self.c.robot.side_angle)),
                cell_side_wall=max(45, int(0.42 * self.corridor_est)),
                front_wall=self.cell_mm / 2 + 30,
                corner_front=max(35, half - self.c.fw.cfg_front_ahead),
            ),
        )
        self.events.append((self.t, "celda medida %.1f" % self.cell_mm, self.x, self.y))

    def mode4(self):
        known = self.c.known or self.c.maze
        rows, cols = len(known) // 2, (len(known[0]) - 1) // 4

        def wall(x, y, d):  # d: 0 N, 1 E, 2 S, 3 W (como raton::Dir)
            r = rows - 1 - y
            if d == 0:
                return known[2 * r][4 * x + 2] == "-"
            if d == 2:
                return known[2 * r + 2][4 * x + 2] == "-"
            if d == 3:
                return known[2 * r + 1][4 * x] == "|"
            return known[2 * r + 1][4 * x + 4] == "|"

        mv = [(0, 1), (1, 0), (0, -1), (-1, 0)]
        goal = tuple(self.c.goal)
        dist, q = {goal: 0}, [goal]
        while q:
            c = q.pop(0)
            for d in range(4):
                n = (c[0] + mv[d][0], c[1] + mv[d][1])
                if not wall(c[0], c[1], d) and 0 <= n[0] < cols and 0 <= n[1] < rows and n not in dist:
                    dist[n] = dist[c] + 1
                    q.append(n)
        pos, heading, mismatch = tuple(self.c.start), 0, False
        self.grid_th = self.th
        while dist.get(pos, 99) != 0:
            d = next(
                d
                for d in range(4)
                if not wall(pos[0], pos[1], d)
                and dist.get((pos[0] + mv[d][0], pos[1] + mv[d][1]), 999) + 1 == dist[pos]
            )
            cells, c = 0, pos
            while dist[c] != 0 and not wall(c[0], c[1], d):
                n = (c[0] + mv[d][0], c[1] + mv[d][1])
                if dist.get(n, 999) + 1 != dist[c]:
                    break
                c, cells = n, cells + 1
            diff = (d - heading) % 4
            if diff in (1, 3):
                for _ in range(10):  # parado: lecturas nuevas (como readWalls en el firmware)
                    yield
                    self.tick()
                yield from self.turn_corner_track(-90 if diff == 1 else 90)
            elif diff == 2:
                yield from self.turn_corner_track(180)
            heading = d
            moved, unexpected = yield from self.run_cells(cells, self.c.fw.comp_speed)
            for _ in range(moved):
                pos = (pos[0] + mv[d][0], pos[1] + mv[d][1])
            if moved:
                self.cell_sides = self.approach_sides.get(moved, (False, False))
            F = self.readings["F"]
            real_wall = unexpected and 0 <= F < self.c.fw.front_wall
            if real_wall or moved < cells:
                mismatch = True
                break
        if mismatch:
            self.events.append((self.t, "mapa", self.x, self.y))
            # El mapa no coincide: explorar por celdas desde aqui, con el mapa como punto de partida
            # (lo que vea en cada celda manda sobre el mapa).
            prior = {(x, y, d): wall(x, y, d) for x in range(cols) for y in range(rows) for d in range(4)}
            yield from self.explore(pos, heading, prior)
        self.grid_th = None

    # --- modo 3: explorar por celdas con inundacion (igual que exploreCells del firmware)
    def explore(self, pos, heading, prior=None):
        """Exploracion por celdas, anticipacion lateral y giros en el centro de 90/180 grados."""
        fw = self.c.fw
        cols, rows = self.cols, self.rows
        full_map = self.c.mode == 3 and not self.c.use_goal
        mv = [(0, 1), (1, 0), (0, -1), (-1, 0)]
        walls = {}  # (x, y, d) -> bool ; d 0 N, 1 E, 2 S, 3 W

        def setw(x, y, d, v):
            walls[(x, y, d)] = v
            n = (x + mv[d][0], y + mv[d][1])
            walls[(n[0], n[1], (d + 2) % 4)] = v

        def wall(x, y, d):
            n = (x + mv[d][0], y + mv[d][1])
            inside = (
                (-128 <= n[0] - self.c.start[0] < 128 and -128 <= n[1] - self.c.start[1] < 128)
                if full_map
                else (0 <= n[0] < cols and 0 <= n[1] < rows)
            )
            if not inside:
                return True
            return walls.get((x, y, d), False)

        if prior:
            for (x, y, d), v in prior.items():
                walls[(x, y, d)] = v
        goal = tuple(self.c.goal)
        seen = set()
        visits = {}
        pruned = self.mission.pruned
        no_progress = 0
        blocked_front = None
        self.grid_th = self.th if self.grid_th is None else self.grid_th
        while full_map or pos != goal:
            mission_action = self.mission.decide(start=not seen, route=self.c.mode == 4)
            if pos not in seen:
                if mission_action == "read_start":
                    # Only the start is surveyed stationary; later reads use the
                    # samples delivered during motion, without advancing time.
                    for _ in range(fw.survey_samples * 4):
                        yield
                        self.tick()
                front_value = self.raw_readings["F"] if self.fresh_front else self.readings["F"]
                self.fresh_front = False
                front = 0 <= front_value < self.front_turn() + fw.front_factor * self.cell_mm
                left, right = getattr(self, "cell_sides", (False, False))
                if blocked_front is not None:
                    # No nueva aproximacion: las laterales almacenadas pertenecen
                    # al rumbo de llegada, no al rumbo del intento que fallo.
                    left = wall(*pos, (heading + 3) % 4)
                    right = wall(*pos, (heading + 1) % 4)
                    front = True
                self.sensed.append((round(self.t, 1), pos, heading, left, front, right, {}, ()))
                for d, value in ((heading, front), ((heading + 3) % 4, left), ((heading + 1) % 4, right)):
                    setw(*pos, d, value)
                if blocked_front is not None:
                    setw(*pos, blocked_front, True)
                    blocked_front = None
                seen.add(pos)
                self.cells_mapped = len(seen)
            if not full_map:
                pruned = self.mission.prune(seen, goal, pos, wall, mv)
            visits[pos] = visits.get(pos, 0) + 1
            if full_map:
                frontier = deque([pos])
                reachable = {pos}
                goal = None
                while frontier:
                    current = frontier.popleft()
                    if current not in seen:
                        goal = current
                        break
                    for direction in range(4):
                        neighbor = (current[0] + mv[direction][0], current[1] + mv[direction][1])
                        if walls.get((*current, direction)) is False and neighbor not in reachable:
                            reachable.add(neighbor)
                            frontier.append(neighbor)
                if goal is None:
                    self.mapping_complete = True
                    self.mission.decide(finished=True)
                    return True
            dist, q = {goal: 0}, [goal]
            while q:
                c = q.pop(0)
                for d in range(4):
                    n = (c[0] + mv[d][0], c[1] + mv[d][1])
                    if (
                        n not in pruned
                        and not wall(c[0], c[1], d)
                        and (not full_map or walls.get((*c, d)) is False)
                        and n not in dist
                    ):
                        dist[n] = dist[c] + 1
                        q.append(n)
            if pos not in dist:
                self.mission.unreachable.add(pos)
                self.mission.decide(finished=True)
                return False  # encerrado: no hay camino posible
            opts = [
                d
                for d in range(4)
                if not wall(pos[0], pos[1], d)
                and dist.get((pos[0] + mv[d][0], pos[1] + mv[d][1]), 999) < dist[pos]
            ]
            if len(seen) == 1 and front and pos == tuple(self.c.start):
                side_opts = [d for d in opts if (d - heading) % 4 in (1, 3)]
                if side_opts:
                    opts = side_opts
            if not opts:
                raise MotionFailure("sin salidas disponibles")
            # Empates: recto, luego giros, y marcha atras lo ultimo.
            ordered_options = sorted(
                opts,
                key=lambda d: (
                    0 if d == heading else (2 if (d - heading) % 4 == 2 else 1),
                    visits.get((pos[0] + mv[d][0], pos[1] + mv[d][1]), 0),
                ),
            )
            d = self.mission.choose(ordered_options)
            diff = (d - heading) % 4
            self.decisions.append(
                (
                    round(self.t, 1),
                    pos,
                    heading,
                    d,
                    {k: round(v) for k, v in self.readings.items()},
                    (int(self.x // self.pitch), int((self.y + 33) // self.pitch)),
                    self.collisions,
                )
            )
            # Celdas seguidas por terreno ya visto (se recorren de una vez, mas rapido).
            cells, c = 1, (pos[0] + mv[d][0], pos[1] + mv[d][1])
            while (
                c in seen
                and c != goal
                and not wall(c[0], c[1], d)
                and dist.get((c[0] + mv[d][0], c[1] + mv[d][1]), 999) < dist[c]
            ):
                c, cells = (c[0] + mv[d][0], c[1] + mv[d][1]), cells + 1
            if diff == 2:
                yield from self.turn_corner_track(180)
            elif diff in (1, 3):
                yield from self.turn_corner_track(-90 if diff == 1 else 90)
            heading = d

            def on_cell(index, sides, front):
                arriving = (pos[0] + mv[d][0] * index, pos[1] + mv[d][1] * index)
                # Classify before reaching the centre, compensating the remaining
                # encoder distance. This is the per-cell callback used in motion.
                forward_wall = (
                    0 <= front < self.front_turn() + fw.front_factor * cell_distance + fw.side_look_ahead - 12
                )
                for direction, value in ((d, forward_wall), ((d + 3) % 4, sides[0]), ((d + 1) % 4, sides[1])):
                    setw(*arriving, direction, value)
                if arriving not in seen:
                    self.sensed.append(
                        (round(self.t, 1), arriving, d, sides[0], forward_wall, sides[1], {}, ())
                    )
                    seen.add(arriving)
                    self.cells_mapped = len(seen)
                # A front obstruction or a goal requires braking in this cell.
                # Other direction decisions are made at the next boundary with
                # no stationary tick; straight motion retains its velocity.
                return not forward_wall and arriving != goal

            cell_distance = self.cell_mm if self.c.mode == 3 else self.pitch
            moved, unexpected = yield from self.run_cells(cells, fw.explore_speed, on_cell=on_cell)
            for _ in range(moved):
                pos = (pos[0] + mv[d][0], pos[1] + mv[d][1])
            if moved:
                no_progress = 0
                self.mission.progress(True, pos, fw.max_no_progress)
                self.cell_sides = self.approach_sides.get(moved, (False, False))
            else:
                no_progress += 1
                blocked_front = heading
                setw(*pos, heading, True)
                seen.discard(pos)
                self.mission.decide(recover=True)
                if not self.mission.progress(False, pos, fw.max_no_progress):
                    raise MotionFailure("sin avance")
            if unexpected:
                setw(*pos, heading, True)
        return True

    def mode3(self):
        yield from self.wait_fresh_front()
        # Align only along the measured front wall; first run centres laterally.
        heading = 0
        if 0 <= self.raw_readings["F"] < self.c.fw.front_wall:
            yield from self.align_front()
            # Diagonal rays at rest can both hit the front wall even with an open
            # side. Inspect the more distant side first; verify with fresh front.
            right_first = self.readings["R"] >= self.readings["L"]
            degrees = -90 if right_first else 90
            yield from self.turn_corner_track(degrees)
            heading = 1 if right_first else 3
            if 0 <= self.raw_readings["F"] < self.c.fw.front_wall:
                yield from self.turn_corner_track(180)
                heading = (heading + 2) % 4
                if 0 <= self.raw_readings["F"] < self.c.fw.front_wall:
                    yield from self.turn_corner_track(-degrees)
                    heading = 2
        yield from self.explore(tuple(self.c.start), heading)

    def run(self):
        gen = {5: self.mode5, 4: self.mode4, 3: self.mode3}.get(self.c.mode, self.mode1)()
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
            if self.reached is not None and not (self.c.mode == 3 and not self.c.use_goal):
                break
        if (
            not self.mapping_complete
            and self.failure is None
            and (self.reached is None or (self.c.mode == 3 and not self.c.use_goal))
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

M_L = [  # sube 3 celdas y gira a la izquierda hasta la meta (0,2)
    "+---+---+---+",
    "|           |",
    "+---+---+   +",
    "|       |   |",
    "+---+---+   +",
    "|       |   |",
    "+---+---+---+",
]
M_T = [  # sube y llega a una T: a la derecha callejon, a la izquierda la meta
    "+---+---+---+---+---+",
    "|                   |",
    "+---+---+   +---+---+",
    "|       |   |       |",
    "+---+---+   +---+---+",
    "|       |   |       |",
    "+---+---+---+---+---+",
]
M_DEAD = [  # cruce: a la derecha callejon sin salida, la meta arriba a la izquierda
    "+---+---+---+",
    "|   |   |   |",
    "+   +---+---+",
    "|           |",
    "+---+   +---+",
    "|   |   |   |",
    "+---+---+---+",
]


def random_maze(cols=19, rows=10, seed=0, braid=0.12, start=(18, 0)):
    """Laberinto aleatorio (DFS) con un porcentaje de paredes quitadas para crear bucles."""
    rnd = random.Random(seed)
    H = [[True] * cols for _ in range(rows + 1)]  # H[j][i]: pared horizontal en la linea j (desde arriba)
    V = [[True] * rows for _ in range(cols + 1)]  # V[i][j]: pared vertical en la linea i (desde la izquierda)
    seen = {start}
    stack = [start]
    while stack:
        x, y = stack[-1]
        nb = [
            (x + dx, y + dy)
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1))
            if 0 <= x + dx < cols and 0 <= y + dy < rows and (x + dx, y + dy) not in seen
        ]
        if not nb:
            stack.pop()
            continue
        nx, ny = rnd.choice(nb)
        r, nr = rows - 1 - y, rows - 1 - ny
        if nx != x:
            V[max(x, nx)][r] = False
        else:
            H[max(r, nr)][x] = False
        seen.add((nx, ny))
        stack.append((nx, ny))
    for j in range(1, rows):
        for i in range(cols):
            if H[j][i] and rnd.random() < braid:
                H[j][i] = False
    # Como en la competencia, la salida mira a un pasillo abierto (al norte).
    sx, sy = start
    if sy + 1 < rows:
        H[rows - 1 - sy][sx] = False
    for i in range(1, cols):
        for j in range(rows):
            if V[i][j] and rnd.random() < braid:
                V[i][j] = False
    lines = []
    for j in range(rows + 1):
        lines.append("+" + "+".join("---" if H[j][i] else "   " for i in range(cols)) + "+")
        if j < rows:
            lines.append(
                "".join(("|" if V[i][j] else " ") + "   " for i in range(cols)) + ("|" if V[cols][j] else " ")
            )
    return lines


def comp_maze():
    with open(N.ROOT / "firmware/app/competition_maze.h") as source:
        src = source.read()
    return [
        l.split('"')[1] for l in src.splitlines() if l.strip().startswith('"+') or l.strip().startswith('"|')
    ]


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

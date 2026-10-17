"""Planta aleatorizada y misión de búsqueda, regreso y carrera por paredes observadas."""

import heapq
import math
import random
from collections import Counter, deque

from .. import orchestrators as O
from .. import simulator as S
from ..native import ROOT
from . import benchmark as B

MOVES = B.MOVES
WIDTHS = (140, 160, 168, 192, 220, 260, 300)
ISOLATED = (
    "noise",
    "dropout",
    "offset",
    "crosstalk",
    "gyro",
    "asymmetry",
    "deadzone",
    "backlash",
    "diameter",
    "battery",
    "angle",
    "construction",
)
EXTREMES = (
    "ghost",
    "stale",
    "start_wall",
    "start_offset",
    "dead_left",
    "dead_right",
    "combined",
    "post_echo",
)


def setwall(m, x, y, d, value):
    row = len(m) // 2 - 1 - y
    j, i = ((2 * row, 4 * x + 2), (2 * row + 1, 4 * x + 4), (2 * row + 2, 4 * x + 2), (2 * row + 1, 4 * x))[d]
    line = list(m[j])
    line[i] = "-" if value and d % 2 == 0 else ("|" if value else " ")
    if d % 2 == 0:
        line[i - 1] = line[i + 1] = line[i]
    m[j] = "".join(line)


def legal_maze(seed, n=16):
    # DFS: callejones largos. Bucles sólo si ambos postes conservan otra pared.
    m = S.random_maze(n, n, seed, 0, (0, 0))
    r = random.Random(seed + 31)
    setwall(m, 0, 0, 1, True)
    setwall(m, 0, 0, 0, False)
    center = {(n // 2 - 1, n // 2 - 1), (n // 2, n // 2 - 1), (n // 2 - 1, n // 2), (n // 2, n // 2)}
    for x, y in center:
        for d, (dx, dy) in enumerate(MOVES):
            if (x + dx, y + dy) in center:
                setwall(m, x, y, d, False)

    def attached(px, py):
        return any(
            0 <= x < n and 0 <= y < n and B.wall(m, x, y, d)
            for x, y, d in ((px - 1, py, 2), (px, py, 2), (px, py - 1, 3), (px, py, 3))
        )

    # Repara postes aislados; añadir pared puede desconectar: se comprueba conectividad.
    def connected():
        q = [(0, 0)]
        seen = set(q)
        for x, y in q:
            for d, (dx, dy) in enumerate(MOVES):
                p = x + dx, y + dy
                if 0 <= p[0] < n and 0 <= p[1] < n and p not in seen and not B.wall(m, x, y, d):
                    seen.add(p)
                    q.append(p)
        return len(seen) == n * n

    for py in range(1, n):
        for px in range(1, n):
            if (px, py) == (n // 2, n // 2) or attached(px, py):
                continue
            for x, y, d in ((px - 1, py, 2), (px, py, 2), (px, py - 1, 3), (px, py, 3)):
                if (x, y) in center and (x + MOVES[d][0], y + MOVES[d][1]) in center:
                    continue
                setwall(m, x, y, d, True)
                if connected():
                    break
                setwall(m, x, y, d, False)
    if seed % 2:
        for _ in range(n * n // 3):
            x, y = r.randrange(n), r.randrange(n)
            d = r.choice((0, 1))
            p = (x + MOVES[d][0], y + MOVES[d][1])
            if not (0 <= p[0] < n and 0 <= p[1] < n) or (x, y) == (0, 0) or p == (0, 0):
                continue
            if not B.wall(m, x, y, d):
                continue
            setwall(m, x, y, d, False)
            if any(not attached(px, py) for px, py in ((x, y + 1), (x + 1, y + 1)) if d == 0) or any(
                not attached(px, py) for px, py in ((x + 1, y), (x + 1, y + 1)) if d == 1
            ):
                setwall(m, x, y, d, True)
    return m


def scenario(width, seed, factor, competition=False):
    all_extremes = factor == "all_combined"
    if all_extremes:
        factor = "combined"
        competition = True
    d, m = B.scenario(width, seed, "nominal")
    r = random.Random(seed + 523)
    if competition or width == 168:
        m = legal_maze(seed)
        d.update(cols=16, rows=16, start=(0, 0), goal=(7, 7), wall=12)
    elif width == 192 and seed % 3 == 0:
        m = S.comp_maze()
        d.update(cols=19, rows=10, start=(18, 0), goal=(4, 2), wall=12)
    elif width == 192 and seed % 3 == 1:
        m = (ROOT / "tests/mazes/clasico_01.txt").read_text().splitlines()
        d.update(cols=16, rows=16, start=(0, 0), goal=(7, 7), wall=12)

    def active(name):
        return factor in (name, "combined")

    d.update(
        factor=factor,
        competition=competition or width == 168,
        wall=12,
        noise=0,
        drop=0,
        offsets=[r.uniform(-10, 10) if active("offset") else 0 for _ in range(3)],
        asym=r.choice((-1, 1)) * 0.035 if active("asymmetry") else 0,
        gyro=r.uniform(-0.5, 0.5) if active("gyro") else 0,
        gyro_scale=r.uniform(0.97, 1.03) if active("gyro") else 1,
        diameter=r.uniform(0.97, 1.03) if active("diameter") else 1,
        angle=r.uniform(26, 36) if active("angle") else 31,
        reflectance=r.uniform(0.17, 0.88),
        cross=r.uniform(0, 0.03) if active("crosstalk") else 0,
        off=r.uniform(-20, 20) if active("start_offset") else 0,
        rotation=r.uniform(-10, 10) if active("start_offset") else 0,
        dead="L" if factor == "dead_left" else ("R" if factor == "dead_right" else ""),
        ghost=0.03 if active("ghost") else 0,
        stale=0.24 if active("stale") else 0,
        pwm_dead=0.25 if active("deadzone") else 0,
        backlash=r.uniform(0, 2) if active("backlash") else 0,
        battery_end=r.uniform(0.8, 0.95) if active("battery") else 1,
    )
    if active("gyro") and r.random() < 0.02:
        d["gyro"] = r.uniform(-3, 3)
    if factor == "start_wall":
        d["start"] = next(
            ((x, y) for y in range(d["rows"]) for x in range(d["cols"]) if B.wall(m, x, y, 0)), d["start"]
        )
    if all_extremes:
        d["all_extremes"] = True
        d["dead"] = random.Random(seed + 17 * int(width)).choice(("L", "R"))
        # Rotar el laberinto mantiene la salida de esquina de tres paredes; mira al norte cerrado.
        m = [line[::-1] for line in m[::-1]]
        d["start"] = (d["cols"] - 1, d["rows"] - 1)
    return d, m


class Mission(O.MissionOrchestrator):
    def __init__(self, p, sim):
        super().__init__()
        self.p = p
        self.sim = sim
        self.phase = "out"
        self.proven = False
        self.returned = False
        self.finished = False

    def prune(self, seen, goal, pos, wall, moves):
        if not self.p["prune"]:
            return set()
        # Nunca excluir salida ni ninguna meta central.
        protected = self.sim.goals | {tuple(self.sim.c.start)}
        super().prune(seen - protected, goal, pos, wall, moves)
        return self.pruned

    def path(self, start, heading, targets, wall, known=None):
        q = [(0, start, heading, [])]
        best = {}
        turn = self.p["turn_cost"]
        while q:
            cost, pos, h, path = heapq.heappop(q)
            if (pos, h) in best:
                continue
            best[pos, h] = cost
            if pos in targets:
                return path, cost
            for d, (dx, dy) in enumerate(MOVES):
                nxt = pos[0] + dx, pos[1] + dy
                if (
                    not (0 <= nxt[0] < self.sim.cols and 0 <= nxt[1] < self.sim.rows)
                    or wall(*pos, d)
                    or nxt in self.pruned
                ):
                    continue
                if known is not None and known.get((*pos, d)) is not False:
                    continue
                turns = min((d - h) % 4, (h - d) % 4)
                heapq.heappush(q, (cost + 1 + turn * turns, nxt, d, path + [(nxt, d)]))
        return [], float("inf")

    def target(self, pos, h, seen, wall, walls):
        start = tuple(self.sim.c.start)
        path, cost = self.path(start, 0, self.sim.goals, wall)
        self.proven = (
            math.isfinite(cost)
            and all(n in seen for n, d in path)
            and all(
                walls.get((*(start if i == 0 else path[i - 1][0]), d)) is False
                for i, (n, d) in enumerate(path)
            )
        )
        self.sim.route_proven = self.proven
        if self.phase == "out" and pos in self.sim.goals:
            self.phase = "search"
        if self.phase == "search":
            if (self.proven and self.p["stop_on_proven_route"]) or not self.p["adachi_method"]:
                self.phase = "return"
            elif self.p["adachi_method"]:
                unknown = {n for n, d in path if n not in seen}
                if unknown:
                    route, _ = self.path(pos, h, unknown, wall)
                    if route:
                        return route[-1][0]
                self.phase = "return"
        if self.phase == "return":
            if pos == start:
                self.returned = True
                if self.p["stop_on_proven_route"] and not self.proven:
                    self.phase = "out"
                else:
                    self.finished = True
                    return pos
            if self.phase == "return" and self.p["return_explore"] and not self.proven:
                unknown = {n for n, d in path if n not in seen}
                if unknown:
                    route, _ = self.path(pos, h, unknown, wall)
                    if route:
                        return route[-1][0]
            if self.phase == "return":
                return start
        if self.p["adachi_method"] and not self.proven:
            unknown = {n for n, d in path if n not in seen}
            if unknown:
                route, _ = self.path(pos, h, unknown, wall)
                if route:
                    return route[-1][0]
        route, _ = self.path(pos, h, self.sim.goals, wall)
        return route[-1][0] if route else next(iter(self.sim.goals))

    def options(self, pos, h, goal, wall):
        route, cost = self.path(pos, h, {goal}, wall)
        return [route[0][1]] if route else []


class KnowledgeSim(B.RandomizedSim):
    def __init__(self, case, domain, seed, params):
        self.params = params
        self.route_proven = False
        self.first_arrival = None
        self.last_reverse = 0
        self.last_sign = 0
        self.encoder_fraction = 0
        super().__init__(case, domain, seed)
        self.mission = Mission(params, self)
        self.next_sample = 0.020
        self.goals = {(7, 7), (7, 8), (8, 7), (8, 8)} if domain["competition"] else {tuple(case.goal)}
        if domain["factor"] in ("combined", "construction"):
            # Desplaza cada pared física, nunca el mapa lógico. Tolerancia <= 5% de celda.
            r = random.Random(seed + 17)
            segs = []
            for i in range(0, len(self.segs), 4):
                block = self.segs[i : i + 4]
                xs = [p[0] for a in block for p in a]
                ys = [p[1] for a in block for p in a]
                x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)
                horizontal = x1 - x0 > y1 - y0
                delta = r.uniform(-min(0.05 * self.pitch, 20), min(0.05 * self.pitch, 20))
                # Extremos originales unidos al poste; desplazamiento sólo en mitad de pared.
                # Evita crear huecos de 18 mm desplazando paredes enteras independientemente.
                if horizontal:
                    mid = (x0 + x1) / 2
                    pts = [(x0, y0), (mid, y0 + delta), (x1, y0), (x1, y1), (mid, y1 + delta), (x0, y1)]
                else:
                    mid = (y0 + y1) / 2
                    pts = [(x0, y0), (x1, y0), (x1 + delta, mid), (x1, y1), (x0, y1), (x0 + delta, mid)]
                segs.extend((pts[j], pts[(j + 1) % 6]) for j in range(6))
            self.segs = segs

    def collides(self, x, y, th):
        # Caché exacta: radio circunscrito + desplazamiento máximo de caché.
        # Se reconstruye al superar 20 mm; las paredes no se mueven durante una ejecución.
        center = getattr(self, "_collision_center", None)
        if center is None or math.hypot(x - center[0], y - center[1]) > 20:
            r = self.c.robot
            radius = math.hypot(max(r.length_front, r.length_back), r.width / 2) + 20 + 1e-6
            self._collision_center = (x, y)
            self._collision_local = []
            for a, b in self.segs:
                lo, hi, bot, top = min(a[0], b[0]), max(a[0], b[0]), min(a[1], b[1]), max(a[1], b[1])
                if hi >= x - radius and lo <= x + radius and top >= y - radius and bot <= y + radius:
                    self._collision_local.append((a, b, lo, hi, bot, top))
        pts = self.body(x, y, th)
        edges = list(zip(pts, pts[1:] + pts[:1]))
        x0, x1 = min(p[0] for p in pts), max(p[0] for p in pts)
        y0, y1 = min(p[1] for p in pts), max(p[1] for p in pts)
        moving = [
            (a, b, min(a[0], b[0]), max(a[0], b[0]), min(a[1], b[1]), max(a[1], b[1]))
            for a, b in self.hand_segs()
        ]
        for a, b, lo, hi, bot, top in self._collision_local + moving:
            if hi < x0 or lo > x1 or top < y0 or bot > y1:
                continue
            if any(S.segs_cross(c, d, a, b) for c, d in edges):
                return True
        return False

    def side_face(self, raw):
        return raw * math.sin(math.radians(31)) - self.c.robot.side_inset if raw >= 0 else -1

    def raycast(self, ox, oy, ang):
        d = super().raycast(ox, oy, ang)
        f = self.domain["factor"]
        if f in ("post_echo", "combined") and self.rnd.random() < 0.05:
            echoes = [super(KnowledgeSim, self).raycast(ox, oy, ang + math.radians(a)) for a in (-12.5, 12.5)]
            valid = [x for x in echoes if x is not None]
            if valid:
                d = min(valid)
        if d is None:
            return None
        if f in ("noise", "combined"):
            sigma = max(3, 0.05 * d)
            d += self.rnd.gauss(0, sigma * (3 if self.rnd.random() < 0.05 else 1)) / math.sqrt(1.4)
        if f in ("dropout", "combined"):
            p = 0.06 * min(1, max(0, d / 300)) * (0.5 + 0.5 * (0.88 - self.domain["reflectance"]) / 0.71)
            if self.rnd.random() < p:
                return None
        return max(0, d * (1 - self.domain["cross"]))

    def try_move(self, x, y, heading):
        d = self.domain
        old = self.travelled
        th = self.th
        nav_before = (self.nav_x, self.nav_y)
        # Modelo equivalente de planta: escala rueda, caída batería, zona muerta y juego al invertir.
        dx, dy = x - self.x, y - self.y
        dist = math.hypot(dx, dy)
        battery = 1 - (1 - d["battery_end"]) * min(1, self.t / 600)
        gain = battery * d["diameter"]
        speed = dist / self.DT
        if d["pwm_dead"] and speed > 0:
            pwm = 0.18 + (0.70 - 0.18) * speed / 500
            # El arranque aplica el mínimo configurado; bajo batería cae el esfuerzo real.
            pwm = max(d["pwm_dead"], pwm) if self.observed_speed < 1 else pwm
            if pwm * battery < (0.25 if self.observed_speed < 1 else 0.18):
                gain = 0
        sign = 1 if dx * math.cos(th) + dy * math.sin(th) >= 0 else -1
        if dist and sign != self.last_sign:
            self.last_reverse = d["backlash"]
            self.last_sign = sign
        take = min(dist, self.last_reverse)
        self.last_reverse -= take
        gain *= max(0, 1 - take / dist) if dist else 1
        turn = heading - th
        # IMU independiente ausente en base: error equivalente de giro, explícito en informe.
        heading = (
            th
            + turn / d["gyro_scale"]
            + math.radians(self.rnd.gauss(0, 0.008 / math.sqrt(2 * self.DT))) * self.DT
            if d["factor"] in ("gyro", "combined")
            else heading
        )
        moved = super().try_move(self.x + dx * gain, self.y + dy * gain, heading)
        mm_count = math.pi * 34 / 2800
        encoded = (self.travelled - old) / d["diameter"] + self.encoder_fraction
        counts = math.floor(encoded / mm_count)
        self.encoder_fraction = encoded - counts * mm_count
        self.travelled = old + counts * mm_count
        yaw = (th + self.th) / 2 - self.nav_yaw0 + math.pi / 2
        self.nav_x = nav_before[0] + counts * mm_count * math.cos(yaw)
        self.nav_y = nav_before[1] + counts * mm_count * math.sin(yaw)
        return moved

    def tick(self):
        previous = self.nsamples
        super().tick()
        if self.nsamples != previous:
            self.next_sample = self.t + 0.020 - 1e-10
        if (
            int(self.x // self.pitch),
            int(self.y // self.pitch),
        ) in self.goals and self.first_arrival is None:
            self.first_arrival = self.t

    def fast_run(self):
        known = self.c.known

        def wall(x, y, d):
            return B.wall(known, x, y, d)

        pos = tuple(self.c.start)
        heading = 0
        self.grid_th = self.th
        route, cost = self.mission.path(pos, heading, self.goals, wall)
        if not math.isfinite(cost):
            raise S.MotionFailure("sin ruta observada")
        for nxt, d in route:
            diff = (d - heading) % 4
            if diff:
                yield from self.turn_corner_track(180 if diff == 2 else (-90 if diff == 1 else 90))
            moved, unexpected = yield from self.run_cells(1, self.c.fw.comp_speed)
            if not moved:
                raise S.MotionFailure("sin avance en run")
            pos = nxt
            heading = d

    def run(self):
        gen = self.fast_run() if self.c.mode == 4 else self.mode3()
        done = False
        try:
            for _ in range(int(self.c.seconds / self.DT)):
                try:
                    next(gen)
                except StopIteration:
                    done = True
                    break
                if self.c.mode == 4 and self.first_arrival is not None:
                    done = True
                    break
        except S.MotionFailure as exc:
            self.failure = str(exc)
        finally:
            gen.close()
        if not done and self.failure is None:
            self.failure = "limite de simulacion"
        if done and self.failure is None and self.first_arrival is None:
            self.failure = "navegacion terminada sin llegada fisica"
        return self

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
        while True:
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
            goal = self.mission.target(pos, heading, seen, wall, walls)
            if self.mission.finished:
                return True
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
            opts = self.mission.options(pos, heading, goal, wall)
            if len(seen) == 1 and front and pos == tuple(self.c.start):
                side_opts = [d for d in opts if (d - heading) % 4 in (1, 3)]
                if side_opts:
                    opts = side_opts
            if not opts:
                raise S.MotionFailure("sin salidas disponibles")
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
                    raise S.MotionFailure("sin avance")
            if unexpected:
                setw(*pos, heading, True)
        return True


def job(args):
    params, width, seed, factor, competition = args
    d, maze = scenario(width, seed, factor, competition)
    fw = S.Firmware(
        cell_side_wall=width * params["side"],
        front_factor=params["front"],
        front_margin=params["margin"],
        explore_speed=params["speed"],
        kp_wall=params["wall"],
        kp_yaw=params["yaw"],
        deadband=params["deadband"],
        side_look_ahead=30 + (width / 2 - 30) / math.tan(math.radians(31)),
        robust_lateral=True,
        cell_min_samples=2,
        rub_seconds=params["rub"],
        skew_limit=params["skew"],
        max_rubs=params["rubs"],
        turn_attempts=params["retry"],
        max_no_progress=params["progress"],
        front_wall=width / 2 + 50,
    )
    case = S.Case(
        str(seed),
        maze,
        tuple(d["start"]),
        tuple(d["goal"]),
        corridor=width,
        wall=12,
        mode=3,
        use_goal=True,
        controller="corrected",
        seconds=600,
        fw=fw,
        off_x=d["off"],
        off_deg=d["rotation"],
        robot=S.Robot(side_angle=d["angle"]),
        sensor=S.Sensor(noise=0, dropout=0, distance_dropout=False, mounting_error=0),
    )
    sim = KnowledgeSim(case, d, seed + 900000, params).run()
    learned = B.learned_maze(sim)
    runs = []
    # Replay independiente desde salida; no teletransporte computado como retorno. Sólo aristas observadas abiertas.
    if sim.mission.returned and sim.mission.finished:
        for goal in sorted(sim.goals):
            if B.path_length(learned, case.start, goal) is None:
                continue
            run_case = B.replace(case, mode=4, goal=goal, known=learned, seconds=300)
            run = KnowledgeSim(run_case, d, seed + 910000, params).run()
            if run.first_arrival is not None:
                runs.append((run.first_arrival, run.collisions))
            break  # una run por misión, sin selección múltiple de meta sobre verdad
    fastest = min(runs, default=None)
    counts = Counter(tuple(z[1]) for z in sim.decisions)
    return dict(
        width=width,
        seed=seed,
        factor=factor,
        competition=d["competition"],
        arrival=sim.first_arrival is not None,
        clean=sim.first_arrival is not None and sim.collisions == 0,
        cells=sim.cells_mapped,
        coverage=min(1, sim.cells_mapped / (d["cols"] * d["rows"])),
        contacts=sim.collisions,
        time=sim.t,
        search_time=sim.t,
        first_arrival=sim.first_arrival,
        competition_score=(fastest[0] + sim.t / 30)
        if fastest and fastest[1] == 0 and sim.collisions == 0
        else None,
        best_run=fastest[0] if fastest else None,
        run_contacts=fastest[1] if fastest else None,
        returned=sim.mission.returned,
        proven=sim.route_proven,
        failure=sim.failure,
        stuck=sim.failure in ("sin avance", "PEGADO", "limite de simulacion"),
        longest_scrape=sim.max_contact,
        dead_revisits=sum(max(0, n - 1) for n in counts.values()),
        domain=d,
    )


def error(rows):
    worst = min(
        sum(
            float(x["clean"]) + 0.1 * x["coverage"] - 0.1 * x["contacts"] - 0.1 * (x["longest_scrape"] > 1)
            for x in rows
            if x["width"] == w
        )
        / sum(x["width"] == w for x in rows)
        for w in {x["width"] for x in rows}
    )
    bonus = (
        0.05
        * sum(
            max(0, 1 - x["best_run"] / 300) if x["best_run"] is not None and x["run_contacts"] == 0 else 0
            for x in rows
        )
        / len(rows)
    )
    return 1 - worst - bonus

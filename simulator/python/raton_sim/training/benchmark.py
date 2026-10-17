"""Deterministic domain randomization. Run from repository root; stdlib only."""

import math
import random
from collections import Counter, deque
from dataclasses import replace

from .. import simulator as S
from ..native import ROOT
from ..observed_map import ObservedMapSim

OUT = ROOT / "simulator/results/entrenamiento"
WIDTHS = (120, 140, 160, 192, 220, 260, 300)
BASE = dict(
    side=0.5,
    front=0.35,
    margin=15,
    speed=150,
    wall=0.15,
    yaw=0.014,
    deadband=8,
    samples=1,
    rub=0.1,
    skew=10,
    rubs=3,
    retry=3,
    progress=3,
)
FACTORS = (
    "nominal",
    "combined",
    "asymmetry",
    "slip",
    "encoder",
    "gyro",
    "turn",
    "brake",
    "battery",
    "angle",
    "offset",
    "noise",
    "dropout",
    "ghost",
    "dead_left",
    "dead_right",
    "stale",
    "start_wall",
    "start_dead",
    "start_offset",
    "mapping",
)
MOVES = ((0, 1), (1, 0), (0, -1), (-1, 0))


def wall(maze, x, y, d):
    r = len(maze) // 2 - 1 - y
    return (
        maze[2 * r][4 * x + 2] == "-",
        maze[2 * r + 1][4 * x + 4] == "|",
        maze[2 * r + 2][4 * x + 2] == "-",
        maze[2 * r + 1][4 * x] == "|",
    )[d]


def scenario(width, seed, factor):
    r = random.Random(seed)
    cols, rows = r.choice(((5, 5), (7, 7), (10, 10), (19, 10)))
    start = (r.randrange(cols), r.randrange(rows))
    braid = r.choice((0, 0.08, 0.15, 0.25))
    maze = S.random_maze(cols, rows, seed, braid, start)
    if factor == "start_wall":
        start = next(((x, y) for y in range(rows) for x in range(cols) if wall(maze, x, y, 0)), start)
    if factor == "start_dead":
        start = next(
            (
                (x, y)
                for y in range(rows)
                for x in range(cols)
                if sum(wall(maze, x, y, d) for d in range(4)) == 3
            ),
            start,
        )
    goal = (cols - 1 - start[0], rows - 1 - start[1])
    if goal == start:
        goal = (0, 0) if start != (0, 0) else (cols - 1, rows - 1)

    def active(name):
        return factor in ("combined", name)

    d = dict(
        width=width,
        seed=seed,
        factor=factor,
        cols=cols,
        rows=rows,
        braid=braid,
        start=start,
        goal=goal,
        wall=r.uniform(10, 18),
        off=r.uniform(-40, 40) if active("start_offset") else 0,
        rotation=r.uniform(-15, 15) if active("start_offset") else 0,
        asym=r.uniform(-0.1, 0.1) if active("asymmetry") else 0,
        slip=active("slip"),
        loss=r.uniform(0, 0.05) if active("encoder") else 0,
        gyro=r.uniform(-2, 2) if active("gyro") else 0,
        turn=r.uniform(-8, 8) if active("turn") else 0,
        brake=r.uniform(0.6, 1.6) if active("brake") else 1,
        battery=0.7 if active("battery") else 1,
        angle=r.uniform(23, 38) if active("angle") else 31,
        offsets=[r.uniform(-15, 15) for _ in range(3)] if active("offset") else [0] * 3,
        noise=8 if active("noise") else 3,
        drop=0.12 if active("dropout") else 0.02,
        ghost=0.03 if active("ghost") else 0,
        dead=r.choice(("L", "R"))
        if factor == "combined"
        else ("L" if factor == "dead_left" else ("R" if factor == "dead_right" else "")),
        stale=0.24 if active("stale") else 0,
    )
    return d, maze


class RandomizedSim(ObservedMapSim):
    """Perturb physics separately from encoder distance; no truth supplied to planner."""

    def __init__(self, case, domain, seed):
        self.domain = domain
        self.stale_until = 0
        self.stale_value = -1
        self.contact_seconds = 0.0
        self.contact_run = 0.0
        self.max_contact = 0.0
        super().__init__(case, seed)

    def sample(self):
        super().sample()
        # Delivery faults after the existing median filter: intentionally severe.
        for i, k in enumerate(("L", "F", "R")):
            v = self.readings[k]
            if v >= 0:
                v = max(0, v + self.domain["offsets"][i])
            if self.rnd.random() < self.domain["ghost"]:
                v = self.rnd.uniform(30, 300)
            if k == self.domain["dead"]:
                v = -1
            if k == "F" and self.t < self.stale_until:
                v = self.stale_value
            self.readings[k] = v

    def rotate(self, deg):
        self.stale_value = self.readings["F"]
        yield from super().rotate(deg + self.domain["turn"])
        self.stale_until = self.t + self.domain["stale"]

    def try_move(self, x, y, heading):
        d = self.domain
        old = self.travelled
        ox, oy, th = self.x, self.y, self.th
        dx, dy = x - ox, y - oy
        dist = math.hypot(dx, dy)
        if dist:
            slip = 0.35 if d["slip"] and (int(self.t / 0.4) % 13 == 4) else 1
            # Braking uncertainty is approximated by displacement gain, not identified inertia.
            scale = d["battery"] * slip * (1 + (d["brake"] - 1) * min(1, dist / (150 * self.DT)))
            heading += 2 * d["asym"] * dist / 65 + math.radians(d["gyro"]) * self.DT
            angle = heading - th
            dx, dy = (
                (dx * math.cos(angle) - dy * math.sin(angle)) * scale,
                (dx * math.sin(angle) + dy * math.cos(angle)) * scale,
            )
        moved = S.Sim.try_move(self, ox + dx, oy + dy, heading)
        actual = self.travelled - old
        encoder = actual * (1 - d["loss"])
        self.travelled = old + encoder
        yaw = (th + heading) / 2 - self.nav_yaw0 + math.pi / 2
        self.nav_x += encoder * math.cos(yaw)
        self.nav_y += encoder * math.sin(yaw)
        # Gyro disturbance here affects heading dynamics; an independent IMU state is absent.
        self.contact_run = self.contact_run + self.DT if self.in_contact else 0
        self.contact_seconds += self.DT if self.in_contact else 0
        self.max_contact = max(self.max_contact, self.contact_run)
        return moved


def learned_maze(sim):
    """Only explicitly observed open edges may be run; unknown edges stay closed."""
    c, r = sim.cols, sim.rows
    h = [[True] * c for _ in range(r + 1)]
    v = [[True] * r for _ in range(c + 1)]
    for _, (x, y), heading, l, f, rr, *_ in sim.sensed:
        if not (0 <= x < c and 0 <= y < r):
            continue
        for d, closed in (((heading + 3) % 4, l), (heading, f), ((heading + 1) % 4, rr)):
            nx, ny = x + MOVES[d][0], y + MOVES[d][1]
            if not (0 <= nx < c and 0 <= ny < r):
                continue
            if d in (0, 2):
                h[r - y - (d == 0)][x] = closed
            else:
                v[x + (d == 1)][r - 1 - y] = closed
    lines = []
    for j in range(r + 1):
        lines.append("+" + "+".join("---" if w else "   " for w in h[j]) + "+")
        if j < r:
            lines.append(
                "".join(("|" if v[i][j] else " ") + "   " for i in range(c)) + ("|" if v[c][j] else " ")
            )
    return lines


def path_length(maze, start, goal):
    q = deque([(start, 0)])
    seen = {start}
    cols = (len(maze[0]) - 1) // 4
    rows = len(maze) // 2
    while q:
        (x, y), n = q.popleft()
        if (x, y) == goal:
            return n
        for d, (dx, dy) in enumerate(MOVES):
            p = x + dx, y + dy
            if 0 <= p[0] < cols and 0 <= p[1] < rows and p not in seen and not wall(maze, x, y, d):
                seen.add(p)
                q.append((p, n + 1))
    return None


def job(args):
    params, width, seed, factor, replay = args
    d, maze = scenario(width, seed, factor)
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
        cell_min_samples=params["samples"],
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
        wall=d["wall"],
        mode=3,
        use_goal=factor != "mapping",
        controller="corrected",
        seconds=180,
        fw=fw,
        off_x=d["off"],
        off_deg=d["rotation"],
        robot=S.Robot(side_angle=d["angle"]),
        sensor=S.Sensor(noise=d["noise"], dropout=d["drop"], mounting_error=0),
    )
    sim = RandomizedSim(case, d, seed + 900000).run()
    counts = Counter(tuple(z[1]) for z in sim.decisions)
    dead_revisit = sum(
        max(0, n - 1)
        for (x, y), n in counts.items()
        if 0 <= x < d["cols"]
        and 0 <= y < d["rows"]
        and sum(wall(maze, x, y, k) for k in range(4)) == 3
        and (x, y) != case.start
    )
    result = dict(
        width=width,
        seed=seed,
        factor=factor,
        arrival=sim.reached is not None,
        clean=sim.reached is not None and sim.collisions == 0,
        cells=sim.cells_mapped,
        coverage=sim.cells_mapped / (d["cols"] * d["rows"]),
        contacts=sim.collisions,
        time=sim.t,
        failure=sim.failure,
        stuck=sim.failure in ("sin avance", "PEGADO", "limite de simulacion"),
        contact_seconds=sim.contact_seconds,
        longest_scrape=sim.max_contact,
        dead_revisits=dead_revisit,
        complete=sim.mapping_complete,
        domain=d,
    )
    if replay:
        learned = learned_maze(sim)
        length = path_length(learned, case.start, case.goal)
        result["route_length"] = length
        result["route_arrival"] = False
        if length is not None:
            run = RandomizedSim(replace(case, mode=4, use_goal=True, known=learned), d, seed + 900000).run()
            result.update(
                route_arrival=run.reached is not None,
                route_contacts=run.collisions,
                route_failure=run.failure,
            )
    return result


def summary(rows):
    n = len(rows)
    return dict(
        n=n,
        clean=sum(x["clean"] for x in rows),
        arrivals=sum(x["arrival"] for x in rows),
        cells=sum(x["cells"] for x in rows) / n,
        coverage=sum(x["coverage"] for x in rows) / n,
        contacts=sum(x["contacts"] for x in rows),
        stuck=sum(x["stuck"] for x in rows),
        timeouts=sum(x["failure"] == "limite de simulacion" for x in rows),
        scrapes=sum(x["longest_scrape"] > 1 for x in rows),
        dead_revisits=sum(x["dead_revisits"] for x in rows),
    )


def score(rows):
    ss = [summary([x for x in rows if x["width"] == w]) for w in WIDTHS]
    values = [
        (
            s["clean"] / s["n"]
            + 0.1 * s["coverage"]
            - 0.1 * s["contacts"] / s["n"]
            - 0.1 * s["scrapes"] / s["n"]
        )
        for s in ss
    ]
    return min(values), sum(values) / len(values)

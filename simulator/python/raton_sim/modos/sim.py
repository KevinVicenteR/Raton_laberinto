import math

from raton_sim.simulacion.modelos import (
    MotionFailure,
)


class ModosSim:
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
        prior = None
        if self.c.mode == 3 and self.c.use_goal and self.c.learned_path:
            from raton_sim.mapa import map_storage

            try:
                walls, seen, geometry = map_storage.load(self.c.learned_path)
            except (OSError, ValueError):
                self.events.append((self.t, "MAPA_NO_DISPONIBLE", self.x, self.y))
            else:
                ox, oy = self.c.start[0] - 20, self.c.start[1] - 20
                prior = {
                    (x + ox, y + oy, d): bool(bits & (1 << d))
                    for (x, y), bits in walls.items()
                    for d in range(4)
                }
                self.loaded_seen = {(x + ox, y + oy) for x, y in seen}
                self.corridor_est, self.cell_mm = geometry[:2]
                self.events.append((self.t, "MAPA_CARGADO", self.x, self.y))
        yield from self.explore(tuple(self.c.start), heading, prior)
        if self.mapping_complete:
            yield from self.cell_rotate_to(math.pi / 2)
            if self.c.learned_path:
                from raton_sim.mapa import map_storage

                ox, oy = 20 - self.c.start[0], 20 - self.c.start[1]
                bits = {}
                for (x, y, d), closed in self.learned_walls.items():
                    key = x + ox, y + oy
                    if 0 <= key[0] < 40 and 0 <= key[1] < 40 and closed:
                        bits[key] = bits.get(key, 0) | (1 << d)
                seen = {(x + ox, y + oy) for x, y in self.learned_seen}
                if any(x in (0, 39) or y in (0, 39) for x, y in seen):
                    self.mapping_complete = False
                    raise MotionFailure("MAPA_LIMITE")
                map_storage.save(
                    self.c.learned_path,
                    bits,
                    seen,
                    self.corridor_est,
                    self.cell_mm,
                    math.sin(math.radians(self.c.robot.side_angle)),
                    int(self.c.robot.deadzone),
                )
                self.events.append((self.t, "MAPA_GUARDADO", self.x, self.y))

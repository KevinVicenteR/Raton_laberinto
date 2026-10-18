from collections import deque

from raton_sim.simulacion.modelos import (
    MotionFailure,
)


class ExploracionSim:
    def explore(self, pos, heading, prior=None):
        """Exploracion por celdas, anticipacion lateral y giros en el centro de 90/180 grados."""
        fw = self.c.fw
        cols, rows = self.cols, self.rows
        full_map = self.c.mode == 6 or (self.c.mode == 3 and not self.c.use_goal)
        learning = full_map
        mv = [(0, 1), (1, 0), (0, -1), (-1, 0)]
        walls = {}  # (x, y, d) -> bool ; d 0 N, 1 E, 2 S, 3 W

        def setw(x, y, d, v):
            walls[(x, y, d)] = v
            n = (x + mv[d][0], y + mv[d][1])
            walls[(n[0], n[1], (d + 2) % 4)] = v

        def wall(x, y, d):
            n = (x + mv[d][0], y + mv[d][1])
            inside = (
                (-20 < n[0] - self.c.start[0] < 19 and -20 < n[1] - self.c.start[1] < 19)
                if full_map
                else (0 <= n[0] < cols and 0 <= n[1] < rows)
            )
            if not inside:
                return True
            return walls.get((x, y, d), False)

        self.learned_walls = walls
        if prior:
            for (x, y, d), v in prior.items():
                walls[(x, y, d)] = v
        goal = tuple(self.c.goal)
        seen = set(getattr(self, "loaded_seen", set()))
        self.learned_seen = seen
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
                    deliveries = self.nsamples + fw.calibration_samples + fw.front_stable_samples
                    while self.nsamples < deliveries:
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
                    # Volver a la salida antes de declarar completo y guardar.
                    goal = tuple(self.c.start)
                    full_map = False
                    if pos == goal:
                        break
            dist, q = {goal: 0}, deque([goal])
            while q:
                c = q.popleft()
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
            route_speed = fw.comp_speed if getattr(self, "loaded_seen", None) else fw.explore_speed
            speed_seen = seen.copy()
            self._speed_known = lambda index: (
                (pos[0] + mv[d][0] * index, pos[1] + mv[d][1] * index) in speed_seen
            )
            moved, unexpected = yield from self.run_cells(cells, route_speed, on_cell=on_cell)
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
        if learning:
            self.mapping_complete = pos == tuple(self.c.start) and not self.failure
            self.learned_heading = heading
        return True

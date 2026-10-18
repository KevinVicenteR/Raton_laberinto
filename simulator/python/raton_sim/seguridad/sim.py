import math

from raton_sim.giros import native as N
from raton_sim.simulacion.modelos import (
    MotionFailure,
)


class SeguridadSim:
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
            v = min(v + fw.accel * self.DT, vmax, max(40, math.sqrt(max(0, 2 * fw.accel * remaining))))
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
            target = self.lateral_target(target, offset, v)
            yaw_t, omega = self.steering(target, yaw_t, self.th - th0, v)
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
    def run_cells(self, n, vmax, reverse=False, short=0.0, on_cell=None):
        """Avanza (o retrocede) n celdas. short: marcha atras, parar antes (pared detras en la celda
        de llegada: la cola no cabria al girar). Deja en self.along donde queda el eje."""
        fw, r = self.c.fw, self.c.robot
        from raton_sim.seguridad.orchestrators import AdaptiveSpeed

        if not hasattr(self, "adaptive_speed"):
            self.adaptive_speed = AdaptiveSpeed()
            self.adaptive_speed.caution(fw.adaptive_caution_mm)
        speed_pos = 0.0
        cell = self.cell_mm if self.c.mode == 3 else self.pitch
        sgn = -1 if reverse else 1
        axle_back = (r.length_front - r.length_back) / 2
        # pending: cuanto va el centro del raton por delante del centro de la celda (en su rumbo).
        ofs = self.pending
        target = max(n * cell + ofs - short if reverse else n * cell - ofs, 0.0)
        start, v, moving_at, unexpected = self.travelled, 0.0, self.t, False
        self._steer_rate = 0.0
        self._lateral_previous = None
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
                self.adaptive_speed.caution(fw.adaptive_caution_mm)
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
            known = self.c.mode == 4 or (
                hasattr(self, "_speed_known") and self._speed_known(int((pos + ofs) / cell) + 1)
            )
            confidence = min(self.sensor_conf["F"], max(self.sensor_conf["L"], self.sensor_conf["R"]))
            cap = self.adaptive_speed.step(
                fw,
                vmax,
                confidence,
                heading_error,
                known,
                max(0, F - stop) if F >= 0 else -1,
                pos - speed_pos,
                self.DT,
            )
            speed_pos = pos
            target_deg, v = self.motion.tick(offset, fw, v, cap, remaining, self.DT, brake_cell)
            target_deg = self.lateral_target(target_deg, offset, v)
            yaw_t, omega = self.steering(sgn * target_deg, yaw_t, self.th - th0, v)
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
        self.explore_v = 0
        stop_until = self.t + fw.stop_seconds
        while self.t < stop_until:
            yield
            self.tick()
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

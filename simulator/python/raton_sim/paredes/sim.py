import math
from dataclasses import replace

from raton_sim.simulacion.modelos import (
    Robot,
)


class ParedesSim:
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

    def front_turn(self):
        fw = self.c.fw
        return max(
            self.corridor_est / 2 - fw.cfg_front_ahead + self.c.robot.deadzone,
            self.c.robot.deadzone + fw.front_margin,
        )

    def side_face(self, raw):
        r = self.c.robot
        configured = self.c.fw.geometry or Robot()
        return raw * math.sin(math.radians(configured.side_angle)) - r.side_inset if raw >= 0 else -1

    def wait_fresh_front(self):
        # Solo entregas nuevas; no se añade una espera fija al tiempo del sensor.
        count, until = self.nsamples + self.c.fw.fresh_samples, self.t
        while self.nsamples < count or self.t < until:
            yield
            self.tick()
        self.readings["F"] = self.raw_readings["F"]
        self.fresh_front = True
        self.motion.front_count = 0

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

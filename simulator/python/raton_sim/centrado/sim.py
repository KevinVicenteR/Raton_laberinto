import math


class CentradoSim:
    def steering(self, target, yaw_target, heading_error, speed):
        fw = self.c.fw
        yaw_target += fw.yaw_smooth * (target - yaw_target)
        error = (math.radians(yaw_target) - heading_error + math.pi) % (2 * math.pi) - math.pi
        rate = getattr(self, "_steer_rate", 0.0)
        desired = self.motion.turn(error, fw.kp_yaw) / (1 + 6 / 0.014 * fw.kd_yaw)
        rate += min(1, self.DT / 0.04) * (desired - rate)
        self._steer_rate = rate
        return yaw_target, rate + math.radians(self.c.drift_dps)

    def lateral_target(self, target, offset, speed):
        fw = self.c.fw
        previous = getattr(self, "_lateral_previous", None)
        if previous is None or self.nsamples != previous[0]:
            derivative = (
                0
                if previous is None or offset is None or previous[1] is None
                else (offset - previous[1]) / max(self.t - previous[2], self.DT)
            )
            self._lateral_rate = 0.7 * getattr(self, "_lateral_rate", 0) + 0.3 * max(-40, min(40, derivative))
            self._lateral_previous = (self.nsamples, offset, self.t)
        if fw.wall_rate_damp == 0:
            return target
        scale = max(0.5, min(1, 132 / max(speed, 80)))
        value = scale * (target + fw.wall_rate_damp * self._lateral_rate) if offset is not None else 0
        return max(-fw.max_wall_deg, min(fw.max_wall_deg, value))

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

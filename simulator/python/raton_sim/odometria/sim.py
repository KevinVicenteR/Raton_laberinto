import math


class OdometriaSim:
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

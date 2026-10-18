"""Auditoría reproducible: python -m raton_sim.seguridad.motion_audit salida.json."""

import json
import math
import subprocess
import sys
from dataclasses import replace
from statistics import mean

from raton_sim.giros.native import ROOT, firmware_config
from raton_sim.simulacion import simulator as S


class AuditSim(S.Sim):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.phase = "idle"
        self.idle_seconds = 0
        self.straights = []
        self.overshoots = []

    def tick(self):
        if self.phase == "idle":
            self.idle_seconds += self.DT
        super().tick()

    def run_cells(self, *args, **kwargs):
        previous = self.phase
        self.phase = "straight"
        x, y, heading = self.x, self.y, self.th
        values = []
        iterator = super().run_cells(*args, **kwargs)
        while True:
            try:
                next(iterator)
            except StopIteration as done:
                if values:
                    self.straights.append(values)
                self.phase = previous
                return done.value
            lateral = -(self.x - x) * math.sin(heading) + (self.y - y) * math.cos(heading)
            yaw = math.degrees((self.th - heading + math.pi) % (2 * math.pi) - math.pi)
            values.append((self.t, lateral, yaw))
            yield

    def rotate(self, degrees):
        previous = self.phase
        self.phase = "turn"
        start = self.th
        target = abs(degrees)
        overshoot = 0
        for value in super().rotate(degrees):
            overshoot = max(overshoot, math.copysign(1, degrees) * math.degrees(self.th - start) - target)
            yield value
        self.overshoots.append(overshoot)
        self.phase = previous


def metrics(sim):
    segments = [v for v in sim.straights if v[-1][0] - v[0][0] >= 0.5]
    yaw = [max(z[2] for z in v) - min(z[2] for z in v) for v in segments]
    lateral = [max(z[1] for z in v) - min(z[1] for z in v) for v in segments]
    periods = []
    for v in segments:
        crossings = [b[0] for a, b in zip(v, v[1:]) if a[2] <= 0 < b[2]]
        periods.extend(b - a for a, b in zip(crossings, crossings[1:]) if b - a > 0.2)
    return dict(
        yaw_pp_deg=mean(yaw) if yaw else None,
        lateral_pp_mm=mean(lateral) if lateral else None,
        period_s=mean(periods) if periods else None,
        idle_per_decision_s=(sim.idle_seconds + len(sim.straights) * sim.c.fw.stop_seconds)
        / max(1, len(sim.straights)),
        speed_mm_s=sim.travelled / max(sim.t, 0.001),
        overshoot_deg=mean(sim.overshoots) if sim.overshoots else 0,
        goal_s=sim.reached,
        elapsed_s=sim.t,
        contacts=sim.collisions,
        failure=sim.failure,
    )


def main():
    results = []
    fw = firmware_config(S)
    if "--legacy" in sys.argv:
        historical = subprocess.check_output(
            ["git", "show", "ea39fae:firmware/config.h"], cwd=ROOT, text=True
        )
        fw = replace(firmware_config(S, config_text=historical), legacy_turn=True, stop_seconds=0.1)
    for width in (192, 168):
        cases = [("competencia4", S.comp_maze(), 4), ("competencia3", S.comp_maze(), 3)]
        cases += [(f"aleatorio{i}", S.random_maze(seed=100 + i), 3) for i in range(3)]
        for name, maze, mode in cases:
            case = S.Case(
                name,
                maze,
                (18, 0),
                (4, 2),
                corridor=width,
                mode=mode,
                use_goal=True,
                fw=fw,
                controller="corrected",
                seconds=600,
            )
            if "--margins" in sys.argv:
                case = replace(
                    case,
                    off_x=12,
                    off_deg=2,
                    drift_dps=3,
                    turn_motor_gain=0.75,
                    turn_tau_s=0.08,
                    sensor=replace(case.sensor, noise=6, dropout=0.08),
                )
            sim = AuditSim(case, seed=1).run()
            row = dict(width=width, case=name, **metrics(sim))
            results.append(row)
            print(json.dumps(row), flush=True)
    with open(sys.argv[1], "w") as output:
        json.dump(results, output, indent=2)
        output.write("\n")


if __name__ == "__main__":
    main()

"""Mediciones de telemetría real; conserva tiempos de recepción Bluetooth."""

import json
import math
import re
from statistics import mean, median

from raton_sim.giros.native import ROOT

STATE = re.compile(
    r",([A-Z_]+) I(-?\d+) F(-?\d+) D(-?\d+) v(-?\d+) r(-?\d+) p(-?\d+) x(-?\d+) y(-?\d+) a(-?\d+) t(-?\d+)"
)


def metrics(path):
    rows, decisions, errors, ages = [], [], [], []
    accepted, rejected = 0, 0
    for line in path.read_text().splitlines():
        try:
            time = float(line.split("\t")[0])
        except ValueError:
            continue
        frames = []
        for candidate in re.finditer(r"\$([^$*]+)\*([0-9A-Fa-f]{2})", line):
            body = candidate[1]
            crc = 0
            for byte in body.encode():
                crc ^= byte
                for _ in range(8):
                    crc = ((crc << 1) ^ (7 if crc & 128 else 0)) & 255
            if crc == int(candidate[2], 16):
                frames.append("," + body.split(",", 1)[1])
                accepted += 1
            else:
                rejected += 1
        line = " ".join(frames)
        for match in STATE.finditer(line):
            rows.append((time, match[1], *map(int, match.groups()[1:])))
        if "DECIDE:" in line:
            decisions.append(time)
        for match in re.finditer(r"MOTION .*?phase=rotate .*?error_cdeg=(-?\d+)", line):
            errors.append(abs(int(match[1])) / 100)
        for match in re.finditer(r"age=(\d+),(\d+),(\d+)", line):
            ages.append(max(map(int, match.groups())))
    segments, current = [], []
    for row in rows:
        if row[1] in ("CELDA", "RECTA") and row[5] > 20:
            if current and row[0] - current[-1][0] > 1:
                segments.append(current)
                current = []
            current.append(row)
        elif current:
            segments.append(current)
            current = []
    if current:
        segments.append(current)
    yaw, lateral, period = [], [], []
    for values in segments:
        if len(values) < 5 or values[-1][0] - values[0][0] < 0.5:
            continue
        angles = [v[10] for v in values]
        angles = [(a - angles[0] + 180) % 360 - 180 for a in angles]
        yaw.append(max(angles) - min(angles))
        # Laterales diagonales: proyeccion perpendicular; solo dos lecturas validas.
        offsets = [(v[2] - v[4]) * math.sin(math.radians(31)) / 2 for v in values if v[2] >= 0 and v[4] >= 0]
        if offsets:
            lateral.append(max(offsets) - min(offsets))
        middle = median(angles)
        crossings = [
            b[0] for a, b, ya, yb in zip(values, values[1:], angles, angles[1:]) if ya <= middle < yb
        ]
        period.extend(b - a for a, b in zip(crossings, crossings[1:]) if b - a >= 0.2)
    pauses = []
    for decision in decisions:
        following = next(
            (r for r in rows if r[0] > decision and r[1] in ("CELDA", "RECTA") and r[5] > 20), None
        )
        if following and following[0] - decision < 15:
            pauses.append(following[0] - decision)
    speeds = [r[5] for r in rows if r[1] in ("CELDA", "RECTA") and r[5] > 20]
    stationary_yaw = [
        abs(b[10] - a[10]) for a, b in zip(rows, rows[1:]) if a[1] == b[1] == "PARADO" and b[0] - a[0] < 0.5
    ]
    return dict(
        file=path.name,
        valid_frames=accepted,
        bad_crc_frames=rejected,
        moving_samples=len(speeds),
        straight_segments=len(yaw),
        yaw_pp_deg=median(yaw) if yaw else None,
        lateral_pp_mm=median(lateral) if lateral else None,
        yaw_period_s=median(period) if period else None,
        moving_speed_mm_s=mean(speeds) if speeds else None,
        decision_to_motion_s=median(pauses) if pauses else None,
        turn_final_error_mean_deg=mean(errors) if errors else None,
        turn_final_error_max_deg=max(errors) if errors else None,
        idle_heading_step_max_deg=max(stationary_yaw) if stationary_yaw else None,
        sensor_age_p50_ms=median(ages) if ages else None,
    )


def main():
    rows = [metrics(p) for p in sorted((ROOT / "data/registros").glob("*.txt"))]
    output = ROOT / "simulator/results/motion-audit"
    output.mkdir(parents=True, exist_ok=True)
    (output / "real.json").write_text(json.dumps(rows, indent=2) + "\n")
    print(json.dumps(rows, indent=2))


if __name__ == "__main__":
    main()

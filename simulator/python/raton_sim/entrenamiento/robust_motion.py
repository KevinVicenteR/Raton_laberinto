"""Ablación y CMA-ES de movimiento, con restricciones y selección reservada.

python -m raton_sim.entrenamiento.robust_motion {ablate,train,select,compare,check} --out DIRECTORIO
Las referencias se leen con git show; nunca se cambia el checkout ni se usa la placa.
"""

import argparse
import json
import subprocess
import time
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict, replace
from pathlib import Path

import numpy as np

from raton_sim.entrenamiento.benchmark import RandomizedSim, scenario
from raton_sim.entrenamiento.cmaes import CMA
from raton_sim.giros.native import ROOT, configuration_text, firmware_config
from raton_sim.seguridad.motion_audit import AuditSim, metrics
from raton_sim.simulacion import simulator as S

SPACE = {
    "explore_speed": (155, 230),
    "comp_speed": (270, 380),
    "kp_wall": (0.15, 0.42),
    "kp_yaw": (0.012, 0.025),
    "max_wall_deg": (4, 9),
    "deadband": (3, 9),
    "kd_yaw": (0.0006, 0.0016),
    "wall_rate_damp": (0.01, 0.07),
    "adaptive_min_ratio": (0.5, 0.85),
    "adaptive_unknown_ratio": (0.8, 1),
    "adaptive_heading_deg": (3, 9),
    "adaptive_caution_mm": (40, 180),
    "adaptive_front_mm": (60, 180),
}
FACTORS = (
    "nominal",
    "combined",
    "noise",
    "dropout",
    "ghost",
    "dead_left",
    "dead_right",
    "asymmetry",
    "slip",
    "encoder",
    "gyro",
    "turn",
    "motor",
    "battery",
    "angle",
    "offset",
    "stale",
    "start_offset",
    "domain_combined",
    "domain_nominal",
    "without_noise",
    "without_dropout",
    "without_gyro",
    "without_motor",
    "without_start_offset",
)
OUT = ROOT / "simulator/results/motion-audit/robust"


def config(version):
    text = (
        configuration_text()
        if version == "new"
        else subprocess.check_output(["git", "show", f"{version}:firmware/config.h"], cwd=ROOT, text=True)
    )
    fw = firmware_config(S, config_text=text)
    if version == "ea39fae":
        fw = replace(fw, legacy_turn=True, stop_seconds=0.1)
    return fw, text


def domain(kind, pool="original"):
    seed = {"original": 1, "train": 11001, "validation": 22001}[pool]
    return [(kind, w, i, seed, pool) for w in (192, 168) for i in range(5)]


def job(args):
    values, factor, width, index, seed, seed_pool = args
    fw = S.Firmware(**{**values, "geometry": S.Robot(**values["geometry"])})
    maze_seed = {"original": 100, "train": 11000, "validation": 22000}[seed_pool]
    maze = S.comp_maze() if index < 2 else S.random_maze(seed=maze_seed + index - 2)
    case = S.Case(
        str(index),
        maze,
        (18, 0),
        (4, 2),
        corridor=width,
        mode=4 if index == 0 else 3,
        use_goal=True,
        fw=fw,
        controller="corrected",
        seconds=600,
    )
    if factor.startswith("without_") or factor in (
        "combined",
        "noise",
        "dropout",
        "gyro",
        "motor",
        "start_offset",
    ):

        def active(name):
            return factor in ("combined", name) or (
                factor.startswith("without_") and factor != "without_" + name
            )

        if active("noise"):
            case = replace(case, sensor=replace(case.sensor, noise=6))
        if active("dropout"):
            case = replace(case, sensor=replace(case.sensor, dropout=0.08))
        if active("gyro"):
            case = replace(case, drift_dps=3)
        if active("motor"):
            case = replace(case, turn_motor_gain=0.75, turn_tau_s=0.08)
        if active("start_offset"):
            case = replace(case, off_x=12, off_deg=2)
        sim = AuditSim(case, seed=seed).run()
    elif factor == "nominal":
        sim = AuditSim(case, seed=seed).run()
    else:
        domain_factor = {"domain_combined": "combined", "domain_nominal": "nominal"}.get(factor, factor)
        d, _ = scenario(width, 10000 + index, domain_factor)
        case = replace(
            case,
            robot=replace(case.robot, side_angle=d["angle"]),
            sensor=replace(case.sensor, noise=d["noise"], dropout=d["drop"], mounting_error=0),
            off_x=d["off"],
            off_deg=d["rotation"],
        )
        sim = RandomizedSim(case, d, seed).run()
    row = dict(
        factor=factor,
        width=width,
        index=index,
        seed=seed,
        seed_pool=seed_pool,
        arrival=sim.reached is not None,
        contacts=sim.collisions,
        time=sim.t,
        failure=sim.failure,
        goal_s=sim.reached,
    )
    if isinstance(sim, AuditSim):
        row.update(metrics(sim))
    # Full manoeuvre trace, including the observation available to the controller.
    row["trace"] = sim.motions
    return row


def evaluate(executor, fw, domains):
    return list(executor.map(job, [(asdict(fw), *d) for d in domains], chunksize=1))


def summary(rows):
    return dict(
        n=len(rows),
        arrivals=sum(r["arrival"] for r in rows),
        contacts=sum(r["contacts"] for r in rows),
        mean_goal_penalized=sum(r["goal_s"] or 600 for r in rows) / len(rows),
    )


def objective(rows):
    nominal = [r for r in rows if r["factor"] == "nominal"]
    combined = [r for r in rows if r["factor"] == "combined"]
    n, c = summary(nominal), summary(combined)
    violation = 100 * (n["contacts"] + n["n"] - n["arrivals"])
    violation += 10 * max(0, 7 - c["arrivals"]) + max(0, c["contacts"] - 10)
    elapsed = (n["mean_goal_penalized"] + c["mean_goal_penalized"]) / 2
    # Ningun tiempo compensa una violacion de seguridad.
    return (1e6 + violation * 10000 + elapsed) if violation else elapsed


def write(out, name, value):
    out.mkdir(parents=True, exist_ok=True)
    path = out / name
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, ensure_ascii=False) + "\n")
    temporary.replace(path)


def train(a, executor):
    fw, text = config("new")
    write(a.out, "config_start.json", dict(text=text, firmware=asdict(fw)))
    initial = dict(
        explore_speed=180,
        comp_speed=300,
        kp_wall=0.3,
        kp_yaw=0.02,
        max_wall_deg=8,
        deadband=4,
        kd_yaw=0.0012,
        wall_rate_damp=0.035,
        adaptive_min_ratio=0.65,
        adaptive_unknown_ratio=0.95,
        adaptive_heading_deg=6,
        adaptive_caution_mm=80,
        adaptive_front_mm=100,
    )
    mean = np.array([(initial[k] - lo) / (hi - lo) for k, (lo, hi) in SPACE.items()])
    cma = CMA(mean, pop=a.population, sigma=0.14)
    training = domain("combined", "train") + domain("nominal")
    validation = domain("combined", "validation") + domain("nominal", "validation")
    start = time.monotonic()
    best, best_score, curve = initial, float("inf"), []
    initial_train = evaluate(executor, replace(fw, **initial), training)
    initial_val = evaluate(executor, replace(fw, **initial), validation)
    best_score = objective(initial_val)
    write(
        a.out,
        "initial.json",
        dict(
            params=initial,
            validation_score=best_score,
            training_score=objective(initial_train),
            generation=-1,
        ),
    )
    write(a.out, "initial_validation.json", initial_val)
    write(a.out, "initial_training.json", initial_train)
    write(
        a.out,
        "winner.json",
        dict(
            params=best,
            validation_score=best_score,
            feasible=best_score < 1e6,
            generation=-1,
            training_score=objective(initial_train),
        ),
    )
    write(a.out, "winner_validation.json", initial_val)
    write(a.out, "winner_training.json", initial_train)
    print("INITIAL", best_score, summary(initial_val[:10]), summary(initial_val[10:]), flush=True)
    for generation in range(a.generations):
        vectors = cma.ask()
        candidates = [
            {k: float(lo + np.clip(z, 0, 1) * (hi - lo)) for z, (k, (lo, hi)) in zip(x, SPACE.items())}
            for x in vectors
        ]
        if generation == 0:
            candidates[0] = initial
            vectors[0] = mean.copy()
        jobs = [(asdict(replace(fw, **p)), *d) for p in candidates for d in training]
        all_rows = list(executor.map(job, jobs, chunksize=1))
        size = len(training)
        scores = [objective(all_rows[i * size : (i + 1) * size]) for i in range(len(candidates))]
        cma.tell(vectors, scores)
        # Solo la validacion reservada elige el checkpoint; no realimenta CMA.
        eligible = [i for i in np.argsort(scores) if scores[i] < 1e6]
        if not eligible:
            eligible = [int(np.argmin(scores))]
        validations = []
        for i in eligible:
            rows = evaluate(executor, replace(fw, **candidates[i]), validation)
            value = objective(rows)
            validations.append(
                dict(candidate=int(i), score=value, combined=summary(rows[:10]), nominal=summary(rows[10:]))
            )
            if value < best_score and scores[i] < 1e6:
                best, best_score = candidates[i], value
                write(
                    a.out,
                    "winner.json",
                    dict(
                        params=best,
                        validation_score=value,
                        feasible=value < 1e6,
                        generation=generation,
                        training_score=scores[i],
                        elapsed_s=time.monotonic() - start,
                    ),
                )
                write(a.out, "winner_validation.json", rows)
                write(a.out, "winner_training.json", all_rows[i * size : (i + 1) * size])
        curve.append(
            dict(
                generation=generation,
                training_scores=scores,
                candidates=candidates,
                validation=validations,
                best_validation=best_score,
                elapsed_s=time.monotonic() - start,
            )
        )
        write(a.out, "curve.json", curve)
        print(json.dumps(curve[-1]), flush=True)
        if time.monotonic() - start >= a.minutes * 60:
            break
    write(
        a.out,
        "protocol.json",
        dict(
            space=SPACE,
            training=training,
            validation=validation,
            population=a.population,
            generations=len(curve),
            minutes=a.minutes,
            elapsed_s=time.monotonic() - start,
            selection="minimum held-out constrained objective",
            objective="hard nominal contacts=0 and all arrivals; combined arrivals>=7/10 contacts<=10; then time with 600 s per failure",
        ),
    )


def select_checkpoint(out):
    """Seleccion final auditable; terreno nuevo siempre debe reducir la consigna."""
    initial = json.loads((out / "initial.json").read_text())
    candidates = [initial]
    for row in json.loads((out / "curve.json").read_text()):
        for result in row["validation"]:
            i = result["candidate"]
            candidates.append(
                dict(
                    params=row["candidates"][i],
                    generation=row["generation"],
                    candidate=i,
                    validation_score=result["score"],
                    training_score=row["training_scores"][i],
                )
            )
    eligible = [
        p
        for p in candidates
        if p["training_score"] < 1e6
        and p["validation_score"] < 1e6
        and p["params"]["adaptive_unknown_ratio"] < 1
    ]
    if not eligible:
        raise RuntimeError("Ningun checkpoint cumple las restricciones; no portar automaticamente")
    selected = min(eligible, key=lambda p: p["validation_score"])
    write(
        out,
        "selected.json",
        dict(
            selected,
            feasible=True,
            eligible=len(eligible),
            selection="minimum reserved score with hard train/validation constraints and unknown ratio < 1",
        ),
    )
    print(json.dumps(selected), flush=True)
    return selected


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("action", choices=("ablate", "train", "select", "compare", "check"))
    ap.add_argument("--out", type=Path, default=OUT)
    ap.add_argument("--workers", type=int, default=6)
    ap.add_argument("--minutes", type=float, default=80)
    ap.add_argument("--generations", type=int, default=24)
    ap.add_argument("--population", type=int, default=8)
    a = ap.parse_args()
    if a.population < 2 or a.workers < 1 or a.generations < 1 or a.minutes <= 0:
        ap.error("poblacion >= 2, workers y generaciones >= 1, minutos > 0")
    if a.action == "select":
        select_checkpoint(a.out)
        return
    with ProcessPoolExecutor(max_workers=a.workers) as executor:
        if a.action == "train":
            train(a, executor)
            return
        results = {}
        versions = (
            ("ea39fae", "275b705")
            if a.action == "ablate"
            else (("new",) if a.action == "check" else ("ea39fae", "275b705", "new"))
        )
        for version in versions:
            fw, text = config(version)
            write(a.out, f"config-{version}.json", dict(text=text, firmware=asdict(fw)))
            if a.action in ("ablate", "check"):
                factors = [f for f in FACTORS if a.action == "ablate" or not f.startswith("without_")]
                domains = [d for f in factors for d in domain(f)]
            else:
                domains = (
                    domain("nominal")
                    + domain("combined")
                    + domain("combined", "validation")
                    + domain("nominal", "validation")
                )
                if version == "new":
                    domains += domain("combined", "train")
            rows = evaluate(executor, fw, domains)
            results[version] = rows
            write(a.out, a.action + ".json", results)
            for f in dict.fromkeys((r["factor"], r["seed_pool"]) for r in rows):
                print(
                    version, f, summary([r for r in rows if (r["factor"], r["seed_pool"]) == f]), flush=True
                )


if __name__ == "__main__":
    main()

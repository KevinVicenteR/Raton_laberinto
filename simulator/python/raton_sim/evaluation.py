"""Evaluaciones reproducibles sin modelos de redes."""

import argparse
import hashlib
import json
import math
import random
import time
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace
from pathlib import Path

from . import simulator as S
from . import tracks as P
from .native import ROOT, firmware_config
from .observed_map import ObservedMapSim

CONTROLLERS = ("baseline", "original", "corrected", "precision", "hybrid", "survey", "mapped", "mapped-raw")


def run_job(job):
    maze_seed, noise_seed, mode, controller, suite, trace, *options = job
    corridor = options[0] if options else 192
    if controller not in CONTROLLERS:
        raise ValueError("Controlador desconocido: " + controller)
    fw = firmware_config(S)
    if controller in ("precision", "hybrid", "survey", "mapped", "mapped-raw"):
        fw = replace(fw, precision_turns=True)
    if controller in ("hybrid", "survey", "mapped", "mapped-raw"):
        fw = replace(fw, goal_grid=True)
    if controller in ("survey", "mapped-raw"):
        fw = replace(fw, survey_samples=20, raw_survey=controller == "mapped-raw")
    known = None
    if suite in ("competition", "altered"):
        maze = S.comp_maze() if suite == "competition" else P.altered_comp()
        known = S.comp_maze() if suite == "altered" else None
        seconds = 120 if suite == "competition" else 300
    else:
        maze = S.random_maze(seed=maze_seed)
        seconds = 600
    # El modo 3 sin meta entrena la pista completa; no es una carrera directa.
    if mode == 3:
        seconds = 1200
    sensor = S.Sensor()
    if suite == "data":
        rng = random.Random(maze_seed * 10 + noise_seed)
        sensor = replace(
            sensor,
            noise=rng.choice((1.0, 3.0, 6.0)),
            dropout=rng.choice((0.0, 0.02, 0.08)),
            latency_ms=rng.choice((0, 20, 40)),
        )
    opts = dict(
        mode=mode,
        corridor=corridor,
        fw=fw,
        use_goal=mode == 5,
        known=known,
        seconds=seconds,
        sensor=sensor,
        controller="baseline" if controller in ("original", "baseline") else "corrected",
        trace=trace,
        sweep_guard=controller != "precision",
    )
    simulator = S.Sim
    if controller in ("mapped", "mapped-raw"):
        simulator = ObservedMapSim
        if mode == 4:
            opts["known"] = S.comp_maze()
    sim = simulator(S.Case(str(maze_seed), maze, P.START, P.GOAL, **opts), seed=noise_seed)
    sim.run()
    cx, cy = sim.x // sim.pitch, sim.y // sim.pitch
    center = ((cx + 0.5) * sim.pitch, (cy + 0.5) * sim.pitch)
    # Offline evaluation only; these truth coordinates are never a model input.
    axle = (sim.c.robot.length_front - sim.c.robot.length_back) / 2
    body_center = (sim.x + axle * math.cos(sim.th), sim.y + axle * math.sin(sim.th))
    position_error = math.hypot(body_center[0] - center[0], body_center[1] - center[1])
    out = dict(
        maze=maze_seed,
        noise_seed=noise_seed,
        mode=mode,
        controller=controller,
        suite=suite,
        corridor_mm=corridor,
        reached=sim.reached,
        contacts=sim.collisions,
        clean_arrival=sim.reached is not None and sim.collisions == 0,
        stopped=sim.reached is None,
        failure=sim.failure,
        elapsed=sim.t,
        simulation_limit_s=seconds,
        mapping_complete=sim.mapping_complete,
        cells_mapped=sim.cells_mapped,
        position_error_mm=position_error,
        turns=sim.turns,
        navigation_policy="cell-memory" if getattr(fw, "goal_grid", False) else "original-mode",
        precision_turns=fw.precision_turns if hasattr(fw, "precision_turns") else False,
        survey_samples=fw.survey_samples if hasattr(fw, "survey_samples") else 5,
        raw_survey=fw.raw_survey if hasattr(fw, "raw_survey") else False,
        mapped_guard=controller in ("mapped", "mapped-raw"),
        goal_position_error_mm=math.hypot(
            body_center[0] - (P.GOAL[0] + 0.5) * sim.pitch, body_center[1] - (P.GOAL[1] + 0.5) * sim.pitch
        ),
    )
    if suite == "data" or trace:
        out["motions"] = sim.motions
    if trace:
        if not sim.trace or sim.trace[-1]["t"] < sim.t:
            sim.trace.append(
                dict(
                    t=sim.t,
                    raw=sim.raw_readings.copy(),
                    filtered=sim.readings.copy(),
                    sensors=sim.observation(),
                    encoder_mm=sim.travelled,
                    heading_deg=math.degrees(sim.th),
                    truth=(sim.x, sim.y, sim.th),
                )
            )
        out.update(trace=sim.trace, trail=sim.trail, events=sim.events)
    return out


def jobs_for(controllers, suite="21"):
    jobs = []
    for controller in controllers:
        if suite in ("21", "all", "smoke"):
            for seed in (1,) if suite == "smoke" else (1, 2, 3):
                jobs.append((0, seed, 4, controller, "competition", False))
            if suite != "smoke":
                jobs.extend((0, seed, 4, controller, "altered", False) for seed in (1, 2))
                jobs.extend(
                    (maze, 1, mode, controller, "random", False)
                    for maze in range(100, 108)
                    for mode in (3, 5)
                )
        if suite in ("reserved", "fresh", "all"):
            seeds = range(400, 430) if suite == "fresh" else range(200, 230)
            jobs.extend(
                (maze, noise, mode, controller, suite, False)
                for maze in seeds
                for noise in (1, 2, 3)
                for mode in (3, 5)
            )
    return jobs


def evaluate(args):
    controllers = tuple(args.controllers.split(","))
    if not set(controllers) <= set(CONTROLLERS):
        raise ValueError("Controlador desconocido")
    jobs = jobs_for(controllers, args.suite)
    if not jobs:
        raise ValueError("Evaluación vacía")
    rows = []
    output = Path(args.out)
    output.parent.mkdir(parents=True, exist_ok=True)
    start = time.monotonic()
    hashes = {
        name: hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest()
        for name in ("simulator.py", "native.py", "evaluation.py", "observed_map.py")
    }
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        for row in pool.map(run_job, jobs, chunksize=1):
            rows.append(row)
            output.write_text(
                json.dumps(
                    dict(
                        complete=len(rows) == len(jobs),
                        expected=len(jobs),
                        elapsed_wall=time.monotonic() - start,
                        results=rows,
                        config=str(P.FW),
                        source_sha256=hashes,
                    )
                )
            )
            print(
                f"{len(rows)}/{len(jobs)} {row['controller']}: mapa {row['maze']}, modo {row['mode']}, contactos {row['contacts']}",
                flush=True,
            )
    return rows


def main(argv=None):
    parser = argparse.ArgumentParser(description="Evalúa los controladores deterministas.")
    parser.add_argument(
        "--out", type=Path, default=ROOT / "simulator/results/evaluacion.json", help="archivo de resultados"
    )
    parser.add_argument("--controllers", default="corrected,mapped", help="controladores separados por comas")
    parser.add_argument(
        "--suite", choices=("21", "reserved", "fresh", "all", "smoke"), default="21", help="banco de pruebas"
    )
    parser.add_argument("--workers", type=int, default=4, help="procesos de trabajo")
    evaluate(parser.parse_args(argv))

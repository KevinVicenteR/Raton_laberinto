"""Catálogo y sesiones de reproducción para la web."""

import argparse
import hashlib
import json
import math
from concurrent.futures import ProcessPoolExecutor
from datetime import datetime, timezone
from pathlib import Path

from . import simulator as S
from . import tracks as P
from .evaluation import CONTROLLERS, run_job
from .native import ROOT

LABELS = dict(
    baseline="Control original",
    original="Control original",
    corrected="Control corregido",
    precision="Giros precisos",
    hybrid="Memoria de celdas",
    survey="Lectura estacionaria",
    mapped="Memoria de paredes",
    **{"mapped-raw": "Memoria de paredes y lecturas crudas"},
)


def publication_jobs(controller):
    """Matriz fija: mapas desconocidos y cuatro anchos físicos de pasillo."""
    jobs = [(0, 1, 4, controller, "competition", True, width) for width in (140, 168, 192, 220)]
    jobs.extend(
        (0, 1, mode, controller, suite, True, 192) for mode, suite in ((3, "competition"), (4, "altered"))
    )
    jobs.extend(
        (seed, 1, 3, controller, "random", True, width)
        for seed in range(100, 106)
        for width in ((140, 168, 192, 220) if seed < 102 else (192,))
    )
    jobs.extend((seed, 1, 5, controller, "random", True, 192) for seed in (106, 107))
    return jobs


def subsample(samples, limit):
    """Conserva los extremos y limita el peso de las series visuales."""
    if len(samples) <= limit:
        return samples
    stride = math.ceil((len(samples) - 1) / (limit - 1))
    return samples[:-1:stride] + samples[-1:]


def replay_description(row, maze_seed):
    """Distingue la llegada física del resultado de la exploración posterior."""
    state = (
        "Llegada sin contacto"
        if row["clean_arrival"]
        else "Llegada con contacto"
        if row["reached"] is not None
        else "Sin llegada"
    )
    if row["failure"]:
        state += " (" + row["failure"] + ")"
    map_name = {"competition": "Competencia", "altered": "Competencia con pared cambiada"}.get(
        row["suite"], "Aleatorio " + str(maze_seed)
    )
    label = (
        f"{state} · modo {row['mode']} · {map_name} · {row['corridor_mm']} mm · {row['contacts']} contactos"
    )
    summary = row["failure"] or (
        "Meta alcanzada" if row["reached"] is not None else "Exploración terminada sin llegada a la meta"
    )
    return label, summary


def publish(out, controller="mapped", evaluation=None, workers=4):
    """Publica todas las ejecuciones, incluidas las que no llegan a la meta."""
    if controller not in CONTROLLERS:
        raise ValueError("Controlador desconocido")
    expected_rows = None
    if evaluation is None:
        jobs = publication_jobs(controller)
    else:
        data = json.loads(Path(evaluation).read_text())
        if not data.get("complete"):
            raise ValueError("La evaluación está incompleta")
        if any(row["controller"] not in CONTROLLERS for row in data["results"]):
            raise ValueError("La evaluación contiene controladores retirados")
        expected_rows = [r for r in data["results"] if r["controller"] == controller]
        jobs = [
            (r["maze"], r["noise_seed"], r["mode"], controller, r["suite"], True, r.get("corridor_mm", 192))
            for r in expected_rows
        ]
    if not jobs:
        raise ValueError("Falta el controlador solicitado")
    with ProcessPoolExecutor(max_workers=workers) as pool:
        executions = []
        for row in pool.map(run_job, jobs, chunksize=1):
            executions.append(row)
            print(
                f"{len(executions)}/{len(jobs)} {row['suite']} {row['maze']} · "
                f"modo {row['mode']} · {row['corridor_mm']} mm · "
                f"llegada {row['reached']} · {row['contacts']} contactos",
                flush=True,
            )
    rows = [
        {k: v for k, v in row.items() if k not in ("trace", "trail", "events", "motions")}
        for row in executions
    ]
    if expected_rows is not None:
        for row, expected in zip(rows, expected_rows, strict=True):
            for key in ("reached", "contacts", "failure", "elapsed"):
                if row[key] != expected[key]:
                    raise ValueError("La reproducción difiere de la evaluación: " + key)
    selected = rows
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    replays = []
    sessions = {}
    for expected, row in zip(rows, executions, strict=True):
        maze = (
            S.comp_maze()
            if row["suite"] == "competition"
            else P.altered_comp()
            if row["suite"] == "altered"
            else S.random_maze(seed=row["maze"])
        )
        walls, pitch, height, width = S.build_walls(
            S.Case("replay", maze, P.START, P.GOAL, corridor=row["corridor_mm"])
        )
        row.update(
            maze_seed=expected["maze"],
            maze=maze,
            walls=walls,
            pitch=pitch,
            height=height * pitch,
            width=width * pitch,
            start=P.START,
            goal=P.GOAL,
        )
        row["trace"] = subsample(row["trace"], 1800)
        row["trail"] = subsample(row["trail"], 1800)
        ident = f"{expected['suite']}-{expected['maze']}-n{expected['noise_seed']}-m{expected['mode']}-{controller}-w{row['corridor_mm']}"
        filename = ident + ".json"
        sessions[filename] = json.dumps(row, separators=(",", ":"))
        if len(sessions[filename].encode()) >= 1_500_000:
            raise ValueError("Sesión demasiado grande: " + filename)
        label, summary = replay_description(row, expected["maze"])
        replays.append(dict(id=ident, label=label, summary=summary, file="sim/" + filename))
    catalog = dict(
        generated_at=datetime.now(timezone.utc).isoformat(),
        run_id=hashlib.sha256(json.dumps(rows, sort_keys=True).encode()).hexdigest()[:12],
        objective_met=all(r["clean_arrival"] for r in selected),
        default_controller=controller,
        controller_labels=LABELS,
        results=rows,
        replays=replays,
    )
    if (
        sum(len(content.encode()) for content in sessions.values()) + len(json.dumps(catalog).encode())
        >= 30_000_000
    ):
        raise ValueError("Catálogo demasiado grande")
    # Sustituye solamente los artefactos de esta carpeta de publicación.
    for old in out.iterdir():
        if old.is_file() and old.suffix in (".json", ".html", ".svg"):
            old.unlink()
    for filename, content in sessions.items():
        (out / filename).write_text(content)
    (out / "dashboard.json").write_text(json.dumps(catalog, ensure_ascii=False, separators=(",", ":")))
    print(f"Publicados {len(rows)} resultados y {len(replays)} reproducciones en {out}")
    return catalog


def main(argv=None):
    parser = argparse.ArgumentParser(description="Publica el catálogo de simulación de la web.")
    parser.add_argument("--out", type=Path, default=ROOT / "web/public/sim", help="carpeta de publicación")
    parser.add_argument(
        "--evaluation", type=Path, help="evaluación completa; por defecto genera veinte casos"
    )
    parser.add_argument(
        "--controller", choices=CONTROLLERS, default="mapped", help="controlador de los ejemplos"
    )
    parser.add_argument("--workers", type=int, default=4, help="procesos de trabajo")
    args = parser.parse_args(argv)
    publish(args.out, args.controller, args.evaluation, args.workers)

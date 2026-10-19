"""CMA-ES con cinco etapas y evaluación final congelada; presupuesto configurable."""

import argparse
import json
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

from raton_sim.entrenamiento import cmaes as C
from raton_sim.entrenamiento import knowledge as B
from raton_sim.giros.native import ROOT

OUT = ROOT / "simulator/results/entrenamiento"
# Punto de partida del currículo histórico, conservado sin depender de archivos de entrenamiento.
INITIAL_PARAMS = {
    "side": 0.4185553963691786,
    "front": 0.2801731048451733,
    "margin": 19.90482543693946,
    "speed": 131.64079725701046,
    "wall": 0.18485380779142835,
    "yaw": 0.011868397590079939,
    "deadband": 8.52127407624824,
    "rub": 0.06,
    "skew": 10.051782390690978,
    "rubs": 2,
    "retry": 3,
    "progress": 3,
    "samples": 2,
}

SPACE = dict(
    C.SPACE,
    turn_cost=(0, 3),
    adachi_method=(0, 1),
    stop_on_proven_route=(0, 1),
    return_explore=(0, 1),
    prune=(0, 1),
)
INTEGER = C.INTEGER | {"adachi_method", "stop_on_proven_route", "return_explore", "prune"}


def decode(x):
    p = {k: float(lo + np.clip(z, 0, 1) * (hi - lo)) for z, (k, (lo, hi)) in zip(x, SPACE.items())}
    for k in INTEGER:
        p[k] = int(round(p[k]))
    p["samples"] = 2
    return p


def encode(p):
    return np.array([(p[k] - lo) / (hi - lo) for k, (lo, hi) in SPACE.items()])


def write(name, v):
    tmp = OUT / (name + ".tmp")
    tmp.write_text(json.dumps(v, indent=2, ensure_ascii=False))
    tmp.replace(OUT / name)


def cases(stage, validation=False):
    seed = 22000 if validation else 11000
    if stage == 1:
        return [(w, seed, "nominal", False) for w in (192, 220)]
    if stage == 2:
        return [(w, seed, "nominal", w == 168) for w in B.WIDTHS]
    if stage == 3:
        return [(B.WIDTHS[i % 7], seed + i, f, False) for i, f in enumerate(B.ISOLATED)]
    if stage == 4:
        return [(B.WIDTHS[i % 7], seed + i, f, False) for i, f in enumerate(B.EXTREMES)]
    return [(w, seed, "combined", True) for w in B.WIDTHS]


def evaluate(pool, p, domain):
    rows = list(pool.map(B.job, [(p, *c) for c in domain], chunksize=1))
    return B.error(rows), rows


def main(argv=None):
    global OUT
    ap = argparse.ArgumentParser()
    ap.add_argument("--minutes", type=float, default=80, help="presupuesto de tiempo en minutos")
    ap.add_argument("--workers", type=int, default=4, help="procesos de trabajo")
    ap.add_argument("--per-stage", type=int, default=5, help="máximo de generaciones por etapa")
    ap.add_argument("--population", type=int, default=8, help="candidatos por generación")
    ap.add_argument("--out", type=Path, default=OUT, help="directorio de resultados")
    ap.add_argument("--generations", type=int, default=None, help="límite total de generaciones")
    ap.add_argument("--smoke", action="store_true", help="dominio mínimo de humo")
    a = ap.parse_args(argv)
    if a.population < 2:
        ap.error("la población debe ser al menos 2")
    OUT = a.out
    OUT.mkdir(parents=True, exist_ok=True)
    begin = time.monotonic()
    base = dict(INITIAL_PARAMS)
    base.update(turn_cost=0, adachi_method=0, stop_on_proven_route=1, return_explore=0, prune=1)
    mean = encode(base)
    mean[-4:] = 0.5  # inicializa variantes discretas sin excluir ninguna
    cma = C.CMA(mean, pop=a.population, sigma=0.18)
    curve = []
    best = base
    best_val = float("inf")
    g = 0
    # Conjunto fijo comparable, todos los anchos y factores; semillas ajenas al train y test final.
    validation = [
        (
            w,
            22050 + i,
            ("nominal", "combined", "gyro", "post_echo", "offset", "start_offset", "dead_left")[i],
            w == 168,
        )
        for i, w in enumerate(B.WIDTHS)
    ]
    if a.smoke:
        validation = validation[:1]
    with ProcessPoolExecutor(max_workers=a.workers) as pool:
        initial, rows = evaluate(pool, base, validation)
        best_val = initial
        write("initial_validation.json", rows)
        write("best_validation.json", rows)
        write("mejor.json", dict(params=base, val_error=initial, initial_val_error=initial, gen=-1, stage=0))
        for stage in range(1, 6):
            if a.generations is not None and g >= a.generations:
                break
            plateau = 0
            train_min = float("inf")
            val_patience = 0
            # El tiempo sólo reduce generaciones; nunca elimina etapas.
            maxgens = a.per_stage if time.monotonic() - begin < a.minutes * 60 * 0.75 else 1
            for local in range(maxgens):
                if a.generations is not None and g >= a.generations:
                    break
                xs = cma.ask()
                params = [decode(x) for x in xs]
                domain = cases(stage)
                if a.smoke:
                    domain = domain[:1]
                allrows = list(pool.map(B.job, [(p, *c) for p in params for c in domain], chunksize=1))
                n = len(domain)
                errors = [B.error(allrows[i * n : (i + 1) * n]) for i in range(len(params))]
                win = int(np.argmin(errors))
                val, vrows = evaluate(pool, params[win], validation)
                cma.tell(xs, errors)
                if val < best_val - 1e-6:
                    best_val = val
                    best = params[win]
                    val_patience = 0
                    write("best_validation.json", vrows)
                    write(
                        "mejor.json",
                        dict(
                            params=best,
                            val_error=val,
                            initial_val_error=initial,
                            gen=g,
                            stage=stage,
                            elapsed_seconds=time.monotonic() - begin,
                        ),
                    )
                else:
                    val_patience += 1
                row = dict(
                    gen=g,
                    stage=stage,
                    train_error_best=float(min(errors)),
                    train_error_mean=float(np.mean(errors)),
                    val_error=val,
                    sigma=cma.sigma,
                    best_params=params[win],
                    elapsed_seconds=time.monotonic() - begin,
                    checkpoint_error=best_val,
                )
                curve.append(row)
                write("curva.json", curve)
                print(json.dumps({k: v for k, v in row.items() if k != "best_params"}), flush=True)
                g += 1
                if min(errors) < train_min - 1e-4:
                    train_min = min(errors)
                    plateau = 0
                else:
                    plateau += 1
                if stage < 5 and plateau >= 3:
                    break
                if stage == 5 and val_patience >= 4:
                    break
                if time.monotonic() - begin > a.minutes * 60 * 0.85:
                    break
        write(
            "protocol.json",
            dict(
                elapsed_training_seconds=time.monotonic() - begin,
                train_seed_base=11000,
                validation_seeds=[c[1] for c in validation],
                test_seeds=[33000, 33001, 34000, 34001],
                space=SPACE,
                population=a.population,
                max_generations_stage=a.per_stage,
                stages_completed=sorted({r["stage"] for r in curve}),
                initial_val_error=initial,
                final_val_error=best_val,
                selection="minimum fixed held-out validation, including initial baseline",
                early_stop="only stage 5: validation patience 4; stages 1–4: train plateau 3 or stage cap",
                train_domains={str(s): cases(s) for s in range(1, 6)},
                validation_domain=validation,
                minutes=a.minutes,
                smoke=a.smoke,
                generation_limit=a.generations,
            ),
        )
        write("frozen.json", dict(before=base, after=best))
        domain = [
            (w, 33000 + j, f, w == 168 or f == "competition")
            for w in B.WIDTHS
            for j in range(2)
            for f in ("nominal", "combined", "competition")
        ]
        domain += [
            (w, 34000 + j, f, False) for w in (168, 192) for j in range(2) for f in B.ISOLATED + B.EXTREMES
        ]
        if a.smoke:
            domain = domain[:1]
        final = {}
        for label, p in [("before", base), ("after", best)]:
            _, rows = evaluate(pool, p, domain)
            final[label] = rows
            write("evaluacion.json", final)
            print("FINAL", label, B.error(rows), flush=True)
    print("DONE", time.monotonic() - begin, flush=True)


if __name__ == "__main__":
    main()

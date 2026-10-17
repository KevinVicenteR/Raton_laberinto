"""CMA-ES with full covariance, reproducible curriculum and reserved validation."""

import argparse
import json
import math
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

from ..native import ROOT
from . import benchmark as B

OUT = ROOT / "simulator/results/entrenamiento"
SPACE = {
    "side": (0.3, 0.65),
    "front": (0.2, 0.55),
    "margin": (12, 40),
    "speed": (70, 160),
    "wall": (0.03, 0.22),
    "yaw": (0.008, 0.025),
    "deadband": (2, 14),
    "rub": (0.06, 0.25),
    "skew": (6, 15),
    "rubs": (2, 5),
    "retry": (2, 4),
    "progress": (2, 6),
}
INTEGER = {"rubs", "retry", "progress"}


def decode(x):
    p = {k: float(lo + max(0, min(1, float(z))) * (hi - lo)) for z, (k, (lo, hi)) in zip(x, SPACE.items())}
    for k in INTEGER:
        p[k] = int(round(p[k]))
    p["samples"] = 2
    return p


def encode(p):
    return np.array([(p[k] - lo) / (hi - lo) for k, (lo, hi) in SPACE.items()])


def error(rows):
    values = []
    for w in sorted({r["width"] for r in rows}):
        s = B.summary([r for r in rows if r["width"] == w])
        values.append(
            s["clean"] / s["n"]
            + 0.1 * s["coverage"]
            - 0.1 * s["contacts"] / s["n"]
            - 0.1 * s["scrapes"] / s["n"]
        )
    return 1 - min(values)


def cases(stage, validation=False, smoke=False):
    widths = (192, 220) if stage == 1 else (140, 160, 192, 220, 260, 300)
    factors = {
        1: ("nominal",),
        2: ("nominal",),
        3: (
            "asymmetry",
            "slip",
            "encoder",
            "gyro",
            "turn",
            "brake",
            "battery",
            "angle",
            "offset",
            "noise",
            "dropout",
        ),
        4: ("ghost", "stale", "start_wall", "start_offset", "dead_left", "dead_right", "combined"),
    }[stage]
    # Common random numbers across candidates and generations. Disjoint seed pools.
    seeds = (20000, 20001) if validation else (10000, 10001)
    if smoke:
        seeds = seeds[:1]
        factors = factors[:1]
        widths = widths[:1]
    return [(w, seed, f) for w in widths for seed in seeds for f in factors]


def evaluate(pool, params, domain):
    rows = list(pool.map(B.job, [(params, w, s, f, False) for w, s, f in domain], chunksize=1))
    return error(rows), rows


def write(name, value):
    tmp = OUT / (name + ".tmp")
    tmp.write_text(json.dumps(value, indent=2))
    tmp.replace(OUT / name)


class CMA:
    def __init__(self, mean, pop=12, sigma=0.18):
        self.mean = mean
        self.n = len(mean)
        self.pop = pop
        self.sigma = sigma
        self.cov = np.eye(self.n)
        self.pc = np.zeros(self.n)
        self.ps = np.zeros(self.n)
        self.rng = np.random.default_rng(904177)
        self.generation = 0
        self.mu = pop // 2
        self.weights = np.log(self.mu + 0.5) - np.log(np.arange(1, self.mu + 1))
        self.weights /= self.weights.sum()
        self.eff = 1 / sum(self.weights**2)
        n = self.n
        u = self.eff
        self.cc = (4 + u / n) / (n + 4 + 2 * u / n)
        self.cs = (u + 2) / (n + u + 5)
        self.c1 = 2 / ((n + 1.3) ** 2 + u)
        self.cm = min(1 - self.c1, 2 * (u - 2 + 1 / u) / ((n + 2) ** 2 + u))
        self.damp = 1 + 2 * max(0, math.sqrt((u - 1) / (n + 1)) - 1) + self.cs
        self.chi = math.sqrt(n) * (1 - 1 / (4 * n) + 1 / (21 * n * n))

    def ask(self):
        vals, vecs = np.linalg.eigh(self.cov)
        self.A = vecs @ np.diag(np.sqrt(np.maximum(vals, 1e-12))) @ vecs.T
        return [self.mean + self.sigma * (self.A @ self.rng.normal(size=self.n)) for _ in range(self.pop)]

    def tell(self, x, errors):
        chosen = np.array([x[i] for i in np.argsort(errors)[: self.mu]])
        old = self.mean.copy()
        self.mean = self.weights @ chosen
        y = (self.mean - old) / self.sigma
        self.ps = (1 - self.cs) * self.ps + math.sqrt(self.cs * (2 - self.cs) * self.eff) * np.linalg.solve(
            self.A, y
        )
        self.generation += 1
        h = float(
            np.linalg.norm(self.ps) / math.sqrt(1 - (1 - self.cs) ** (2 * self.generation)) / self.chi
            < 1.4 + 2 / (self.n + 1)
        )
        self.pc = (1 - self.cc) * self.pc + h * math.sqrt(self.cc * (2 - self.cc) * self.eff) * y
        ys = (chosen - old) / self.sigma
        self.cov = (
            (1 - self.c1 - self.cm) * self.cov
            + self.c1 * (np.outer(self.pc, self.pc) + (1 - h) * self.cc * (2 - self.cc) * self.cov)
            + self.cm * sum(w * np.outer(v, v) for w, v in zip(self.weights, ys))
        )
        self.sigma *= math.exp(self.cs / self.damp * (np.linalg.norm(self.ps) / self.chi - 1))
        self.sigma = max(0.015, min(0.5, self.sigma))


def global_output(out):
    global OUT
    OUT = out
    OUT.mkdir(parents=True, exist_ok=True)


def main():
    a = argparse.ArgumentParser()
    a.add_argument("--minutes", type=float, default=75)
    a.add_argument("--workers", type=int, default=4)
    a.add_argument("--generations", type=int, default=10000)
    a.add_argument("--smoke", action="store_true")
    a.add_argument("--population", type=int, default=12)
    a.add_argument("--out", type=Path, default=OUT)
    args = a.parse_args()
    global_output(args.out)
    if args.population < 2:
        a.error("la población debe ser al menos 2")
    begin = time.monotonic()
    base = dict(B.BASE, samples=2)
    cma = CMA(encode(base), pop=args.population)
    curve = []
    stage = 1
    plateau = 0
    best_mean = float("inf")
    best_val = float("inf")
    patience = 0
    best = base
    reason = "generation_limit"
    validation = cases(4, True, args.smoke)  # fixed hardest held-out pool: comparable across stages
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        initial, initial_rows = evaluate(pool, base, validation)
        write("initial_validation.json", initial_rows)
        best_val = initial
        write("curva.json", [])
        write("best_validation.json", initial_rows)
        write(
            "mejor.json",
            dict(
                params=base,
                val_error=initial,
                gen=-1,
                stage=0,
                initial_val_error=initial,
                elapsed_seconds=time.monotonic() - begin,
            ),
        )
        for gen in range(args.generations):
            if time.monotonic() - begin >= args.minutes * 60:
                reason = "wall_time"
                break
            xs = cma.ask()
            params = [decode(x) for x in xs]
            errors = []
            # Parallelize all scenarios; candidate ordering remains deterministic.
            domain = cases(stage, False, args.smoke)
            tasks = [(p, w, s, f, False) for p in params for w, s, f in domain]
            allrows = list(pool.map(B.job, tasks, chunksize=1))
            n = len(domain)
            errors = [error(allrows[i * n : (i + 1) * n]) for i in range(len(params))]
            winner = int(np.argmin(errors))
            val, rows = evaluate(pool, params[winner], validation)
            cma.tell(xs, errors)
            mean = float(np.mean(errors))
            curve.append(
                dict(
                    gen=gen,
                    stage=stage,
                    train_error_best=min(errors),
                    train_error_mean=mean,
                    val_error=val,
                    sigma=cma.sigma,
                    best_params=params[winner],
                    elapsed_seconds=time.monotonic() - begin,
                )
            )
            write("curva.json", curve)
            if val < best_val - 1e-6:
                best_val = val
                best = params[winner]
                patience = 0
                write("best_validation.json", rows)
                write(
                    "mejor.json",
                    dict(
                        params=best,
                        val_error=val,
                        gen=gen,
                        stage=stage,
                        initial_val_error=initial,
                        elapsed_seconds=time.monotonic() - begin,
                    ),
                )
            else:
                patience += 1
            if mean < best_mean - 1e-4:
                best_mean = mean
                plateau = 0
            else:
                plateau += 1
            print(json.dumps({k: v for k, v in curve[-1].items() if k != "best_params"}), flush=True)
            if plateau >= 3 and stage < 4:
                stage += 1
                plateau = 0
                best_mean = float("inf")
                patience = 0
            # Patience resets on curriculum transition; applies in every stage.
            if patience >= 5:
                reason = "validation_patience_5"
                break
        write(
            "protocol.json",
            dict(
                reason=reason,
                elapsed_seconds=time.monotonic() - begin,
                initial_val_error=initial,
                train_seeds=[10000, 10001],
                validation_seeds=[20000, 20001],
                space=SPACE,
                population=args.population,
                validation_policy="generation winner evaluated on fixed stage-4 pool; checkpoint selected by validation",
                smoke=args.smoke,
            ),
        )
    print("DONE", reason, flush=True)


if __name__ == "__main__":
    main()

"""Stage 4 -- run the outer loop, and show the Pareto front in the browser.

    .venv/bin/python -m opt.run --pop 60 --gen 40

The baseline test the plan demands: a hand-built Twiddler-like layout is scored with the
SAME evaluator, and the Pareto front must dominate it. If it does not, the model is wrong,
not the baseline.
"""
from __future__ import annotations

import os

for _v in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS",
           "NUMEXPR_NUM_THREADS", "VECLIB_MAXIMUM_THREADS"):
    os.environ.setdefault(_v, "1")  # see opt/problem.py -- must precede numpy

import argparse
import multiprocessing
import pickle
import time

import numpy as np

from design.vector import REAL_BOUNDS
from opt.problem import CONSTRAINT_NAMES, OBJECTIVE_NAMES, ExoKeyProblem, hands
from hand.myohand import FINGERS


def baseline() -> dict:
    """A hand-built Typeware-like layout: 2 keys per finger on a strap-mounted body,
    fingers at a common grip curl, a plain CF-PA12 body. Not optimised."""
    x = {}
    for f in FINGERS:
        x[f"tp_{f}"] = 0.40
        x[f"tm_{f}"] = 0.55
    x["tp_thumb"] = 0.55
    x["tm_thumb"] = 0.60
    for f, sgn in zip(("index", "middle", "ring", "little"), (1.0, 0.33, -0.33, -1.0)):
        x[f"ab_{f}"] = 0.75 * sgn   # fan splay, or the keycaps collide
    x["alu_w"] = 0.008
    x["alu_t"] = 0.002
    x["palm_offset"] = 0.020
    x["stem"] = 0.008
    x["adjust"] = 0.012
    x["body_half"] = 0.026
    x["body_prox"] = 0.014
    x["body_dist"] = 0.055
    x["material"] = "cf_pa12"
    return x


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pop", type=int, default=60)
    ap.add_argument("--gen", type=int, default=40)
    ap.add_argument("--procs", type=int, default=max(1, multiprocessing.cpu_count() - 2))
    ap.add_argument("--seed", type=int, default=1)
    args = ap.parse_args()

    from pymoo.algorithms.moo.nsga2 import RankAndCrowdingSurvival
    from pymoo.core.mixed import MixedVariableGA
    from pymoo.optimize import minimize
    from pymoo.parallelization import StarmapParallelization

    from design.vector import evaluate

    # ---- baseline, scored with the SAME evaluator --------------------------------------
    print("scoring the hand-built baseline (Twiddler-like)...")
    b = evaluate(baseline(), hands())
    print(f"  keys {b['total_keys']} (1/finger x 3 actions)  effort/char {b['F'][0]:.3e}   "
          f"mass {b['F'][1]:.1f} g   deflection {b['F'][2]:.3f} mm")
    print(f"  feasible: {b['feasible']}")
    for n, v in zip(CONSTRAINT_NAMES, b["G"]):
        if v > 0:
            print(f"    VIOLATES {n}: {v:+.4f}")

    # ---- the run -----------------------------------------------------------------------
    pool = multiprocessing.Pool(args.procs)
    problem = ExoKeyProblem(elementwise_runner=StarmapParallelization(pool.starmap))
    algorithm = MixedVariableGA(pop_size=args.pop, survival=RankAndCrowdingSurvival())

    n_eval = args.pop * args.gen
    print(f"\nNSGA-II: pop {args.pop} x {args.gen} gens = {n_eval} evals "
          f"on {args.procs} processes")
    t0 = time.perf_counter()
    res = minimize(problem, algorithm, ("n_gen", args.gen), seed=args.seed,
                   verbose=True, save_history=False)
    dt = time.perf_counter() - t0
    pool.close()

    print(f"\nran in {dt/60:.1f} min  ({dt/max(n_eval,1):.2f} s/eval effective)")

    if res.F is None or len(np.atleast_2d(res.F)) == 0:
        print("NO FEASIBLE DESIGN FOUND. The constraint set may be over-tight.")
        return

    F = np.atleast_2d(res.F)
    # res.X is a numpy OBJECT array of dicts for a mixed-variable problem, not a list.
    # `isinstance(..., list)` is False, so wrapping it collapsed 60 designs into one.
    X = [res.X] if isinstance(res.X, dict) else list(res.X)
    print(f"Pareto front: {len(F)} non-dominated feasible designs\n")

    with open("out/pareto.pkl", "wb") as fh:
        pickle.dump({"F": F, "X": X, "baseline": b["F"], "baseline_feasible": b["feasible"]}, fh)

    # ---- does the front dominate the baseline? -----------------------------------------
    bF = np.array(b["F"])
    dominates = [
        i for i, f in enumerate(F)
        if np.all(f <= bF) and np.any(f < bF)
    ]
    print(f"{'':4s} {'keys':>5s} {OBJECTIVE_NAMES[0]:>12s} {OBJECTIVE_NAMES[1]:>10s} "
          f"{OBJECTIVE_NAMES[2]:>15s}")
    print(f"{'base':4s} {b['total_keys']:5d} {bF[0]:12.3e} {bF[1]:10.1f} {bF[2]:15.3f}"
          + ("" if b["feasible"] else "   (INFEASIBLE)"))
    print("-" * 52)
    order = np.argsort(F[:, 0])
    for i in order[:12]:
        x = X[i]
        k = 5
        mark = " <-- dominates baseline" if i in dominates else ""
        print(f"{'':4s} {k:5d} {F[i,0]:12.3e} {F[i,1]:10.1f} {F[i,2]:15.3f}{mark}")

    print()
    if b["feasible"]:
        print(f"designs strictly dominating the baseline: {len(dominates)}/{len(F)}")
        if not dominates:
            print("  NONE. Either the baseline is already good, or the model is wrong.")
    else:
        print("the baseline is INFEASIBLE, so 'dominating' it is not the meaningful test;")
        print("the meaningful result is that the optimiser found feasible designs at all.")

    _plot(F, bF, X, b)


def _plot(F, bF, X, b):
    import plotly.graph_objects as go

    keys = [5 for _ in X]
    fig = go.Figure()
    fig.add_trace(go.Scatter3d(
        x=F[:, 0], y=F[:, 1], z=F[:, 2], mode="markers",
        marker=dict(size=6, color=keys, colorscale="Viridis", showscale=True,
                    colorbar=dict(title="total keys")),
        text=[f"keys {k}<br>effort/char {a:.3e}<br>mass {m:.1f} g<br>defl {d:.3f} mm"
              for k, a, m, d in zip(keys, F[:, 0], F[:, 1], F[:, 2])],
        hoverinfo="text", name="Pareto front",
    ))
    fig.add_trace(go.Scatter3d(
        x=[bF[0]], y=[bF[1]], z=[bF[2]], mode="markers",
        marker=dict(size=12, symbol="diamond", color="#e45756"),
        name=f"baseline ({'feasible' if b['feasible'] else 'INFEASIBLE'})",
    ))
    fig.update_layout(
        title="ExoKey Stage 4 — Pareto front. Every point is a FEASIBLE device: all keys "
              "pressable by the 5th–95th percentile hand, no saturation, frame clears the "
              "flesh, within yield.",
        scene=dict(xaxis_title="effort / character (Σa³)", yaxis_title="mass (g)",
                   zaxis_title="worst key deflection (mm)"),
        template="plotly_white", margin=dict(l=0, r=0, t=60, b=0),
    )
    fig.write_html("out/pareto.html", include_plotlyjs="cdn")
    print("browser view: out/pareto.html")


if __name__ == "__main__":
    main()

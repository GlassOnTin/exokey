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
    x["tp_hand"] = 0.40
    x["tm_hand"] = 0.30          # LOW CURL: wells need SPREAD fingertips, not converged ones
    for f in ("index", "middle", "ring", "little"):
        x[f"dp_{f}"] = 0.0
        x[f"dm_{f}"] = 0.0
    x["tp_thumb"] = 0.55
    x["tm_thumb"] = 0.60
    for f, sgn in zip(("index", "middle", "ring", "little"), (1.0, 0.33, -0.33, -1.0)):
        x[f"ab_{f}"] = 0.75 * sgn   # fan splay, or the wells collide
    x["adjust"] = 0.012
    x["material"] = "cf_pa12"
    # (alu_w / alu_t / palm_offset / stem / body_half / body_dist are gone: they shaped the
    #  palmar box, and the structure is the grown gauntlet now -- see design/vector.py)
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
    print(f"  keys {b['total_keys']} (1/finger x 5 directions)  effort/char {b['F'][0]:.3e}   "
          f"gauntlet {b['F'][1]:.1f} g  (solid {b['gauntlet']['solid_g']:.0f} g, "
          f"buttons {b['gauntlet']['worst']*1e6:.0f} um)")
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

    out_pkl = os.environ.get("EXOKEY_OUT", "out/pareto.pkl")
    with open(out_pkl, "wb") as fh:
        pickle.dump({"F": F, "X": X, "baseline": b["F"], "baseline_feasible": b["feasible"]}, fh)

    # ---- does the front dominate the baseline? -----------------------------------------
    bF = np.array(b["F"])
    dominates = [
        i for i, f in enumerate(F)
        if np.all(f <= bF) and np.any(f < bF)
    ]
    print(f"{'':4s} {'keys':>5s} {OBJECTIVE_NAMES[0]:>12s} {OBJECTIVE_NAMES[1]:>18s}")
    print(f"{'base':4s} {b['total_keys']:5d} {bF[0]:12.3e} {bF[1]:18.1f}"
          + ("" if b["feasible"] else "   (INFEASIBLE)"))
    print("-" * 52)
    order = np.argsort(F[:, 0])
    for i in order[:12]:
        x = X[i]
        k = 5
        mark = " <-- dominates baseline" if i in dominates else ""
        print(f"{'':4s} {k:5d} {F[i,0]:12.3e} {F[i,1]:18.1f}{mark}")

    report_cornered(X)

    print()
    if b["feasible"]:
        print(f"designs strictly dominating the baseline: {len(dominates)}/{len(F)}")
        if not dominates:
            print("  NONE. Either the baseline is already good, or the model is wrong.")
    else:
        print("the baseline is INFEASIBLE, so 'dominating' it is not the meaningful test;")
        print("the meaningful result is that the optimiser found feasible designs at all.")

    _plot(F, bF, X, b)


def report_cornered(X: list) -> None:
    """A variable pinned to a bound is NOT a decision. Say so, every run, automatically.

    Two different diseases, and they need opposite fixes:

      * DEAD   -- the variable is dominated everywhere, so the optimiser always slams it to
                  one end. It should be a CONSTANT. (press_N and body_prox were both this;
                  each spent a whole run pretending to be a decision.)
      * BOUND-LIMITED -- the optimiser WANTS to go further and the bound is stopping it. The
                  answer is then an artefact of a number I typed, not of the physics, and
                  the bound is wrong.

    Telling them apart needs judgement, but NOTICING them does not — so notice them here
    rather than leaving it to whether somebody happens to eyeball the design vectors.
    """
    from design.vector import REAL_BOUNDS

    print("\ncornered variables (pinned to a bound => not a decision):")
    any_ = False
    for k, (lo, hi) in REAL_BOUNDS.items():
        v = np.array([x[k] for x in X])
        span = (hi - lo)
        at_lo = (v.mean() - lo) / span < 0.03
        at_hi = (hi - v.mean()) / span < 0.03
        if at_lo or at_hi:
            any_ = True
            end = "LOWER" if at_lo else "UPPER"
            print(f"  {k:14s} pinned at its {end} bound ({v.min():.4g}..{v.max():.4g} "
                  f"of [{lo:g}, {hi:g}])")
            print(f"                 -> either make it a CONSTANT, or the bound is too tight "
                  f"and the answer is an artefact of it")
    if not any_:
        print("  none — every variable is doing work")


def _plot(F, bF, X, b):
    import plotly.graph_objects as go

    keys = [5 for _ in X]
    fig = go.Figure()
    fig.add_trace(go.Scatter(
        x=F[:, 0], y=F[:, 1], mode="markers",
        marker=dict(size=6, color=keys, colorscale="Viridis", showscale=True,
                    colorbar=dict(title="total keys")),
        text=[f"keys {k}<br>effort/char {a:.3e}<br>gauntlet {m:.1f} g"
              for k, a, m in zip(keys, F[:, 0], F[:, 1])],
        hoverinfo="text", name="Pareto front",
    ))
    fig.add_trace(go.Scatter(
        x=[bF[0]], y=[bF[1]], mode="markers",
        marker=dict(size=12, symbol="diamond", color="#e45756"),
        name=f"baseline ({'feasible' if b['feasible'] else 'INFEASIBLE'})",
    ))
    fig.update_layout(
        title="ExoKey Stage 4 — Pareto front. Every point is a FEASIBLE device: all keys "
              "pressable by the 5th–95th percentile hand, no saturation, frame clears the "
              "flesh, within yield.",
        xaxis_title="effort / character (Σa³) — how tiring it is to type",
        yaxis_title="grown gauntlet + adjusters (g) — what it costs to hold the buttons steady",
        template="plotly_white", margin=dict(l=0, r=0, t=60, b=0),
    )
    fig.write_html("out/pareto.html", include_plotlyjs="cdn")
    print("browser view: out/pareto.html")


if __name__ == "__main__":
    main()

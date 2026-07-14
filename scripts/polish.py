"""IS THE GA LEAVING MONEY ON THE TABLE? POLISH ITS FRONT WITH A LOCAL GRADIENT METHOD.

    PYTHONPATH=. .venv/bin/python scripts/polish.py [n_designs]

THE USER: "When I previously dabbled with GA, I normally end up switching to a full parameter
vector bounded optimisation instead, like LM."

The instinct is right and the measurement says WHY it has not been tried here -- and also why it
should be.

LM itself does not fit: it is a LEAST-SQUARES method and this is not a least-squares problem, and
it gives ONE point where we want a FRONT. The right local tool is bounded SLSQP, which takes the
nonlinear constraints too, and the right way to get a front out of a local method is the
epsilon-CONSTRAINT: fix mass <= M, minimise effort, sweep M. (NOT a weighted sum. A weighted sum
lets the optimiser BUY its way out of a constraint through the weights, which is the exact defect
that killed v1 of this project.)

THE REAL OBSTACLE IS THE CLIFFS, AND THEY ARE MEASURED. Perturb the design vector and watch
effort/character:

    0.1% of range  ->  +/-  1.4%      <-- smooth
    1.0% of range  ->  +/-102.4%      <-- a cliff, and as wide as the whole Pareto front

A discrete character-to-action assignment (a Hungarian solve) sits inside the objective. Nudge a
curl and a letter jumps to a different finger direction; the cost falls off a shelf. Finite
differences ACROSS such a shelf are meaningless.

But INSIDE an assignment basin the objective is smooth -- that is what the 0.1% row says. So the
honest architecture is neither GA nor gradient but BOTH: a global method to land in the right
basin (get the discrete assignment right), then a local method to polish where the landscape
actually is smooth. This script measures whether that second half is worth anything at all.

Finite differences are computed IN PARALLEL -- 17 evaluations per gradient, one per core -- which
is the only reason a 20 s objective is tractable for a gradient method.
"""
from __future__ import annotations

import os

for _v in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_v, "1")

import multiprocessing
import pickle
import sys
import time

import numpy as np
from scipy.optimize import minimize

from design.vector import REAL_BOUNDS, evaluate
from opt.problem import hands

KEYS = sorted(REAL_BOUNDS)
LO = np.array([REAL_BOUNDS[k][0] for k in KEYS])
HI = np.array([REAL_BOUNDS[k][1] for k in KEYS])


def _eval(args):
    v, mat = args
    x = {k: float(val) for k, val in zip(KEYS, v)}
    x["material"] = mat
    try:
        r = evaluate(x, hands())
        return r["F"][0], r["F"][1], max(r["G"]), r["feasible"]
    except Exception:
        return 1e6, 1e6, 1e3, False


def polish(x0, mass_cap, pool, steps=12, eps=8e-4):
    """SLSQP-in-spirit: projected gradient descent on effort, subject to mass <= cap and G <= 0.

    Hand-rolled rather than scipy's SLSQP because the objective costs ~20 s and the ONLY way to
    afford a gradient is to evaluate the whole finite-difference stencil IN PARALLEL. scipy's
    solvers call the objective serially, one point at a time, and would take hours.
    """
    mat = x0["material"]
    v = np.array([float(x0[k]) for k in KEYS])
    f0, m0, g0, ok0 = _eval((v, mat))
    best = (f0, m0, v.copy())
    span = HI - LO

    for _ in range(steps):
        stencil = [(v, mat)] + [(np.clip(v + eps * span * np.eye(len(v))[i], LO, HI), mat)
                                for i in range(len(v))]
        res = pool.map(_eval, stencil)
        f_c, m_c, g_c, _ = res[0]
        grad = np.array([(res[i + 1][0] - f_c) / (eps * span[i]) for i in range(len(v))])
        n = np.linalg.norm(grad)
        if n < 1e-18 or not np.isfinite(n):
            break
        d = -grad / n

        # backtracking line search, and every trial point checked for FEASIBILITY -- a step that
        # improves effort by breaking a constraint is not an improvement, it is v1's disease.
        trials = [np.clip(v + a * span * d, LO, HI) for a in (0.05, 0.02, 0.008, 0.003, 0.001)]
        out = pool.map(_eval, [(t, mat) for t in trials])
        moved = False
        for t, (f, m, g, ok) in zip(trials, out):
            if ok and m <= mass_cap and f < best[0]:
                v, best, moved = t, (f, m, t.copy()), True
                break
        if not moved:
            eps *= 0.5
            if eps < 1e-5:
                break
    return best


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 4
    d = pickle.load(open("out/pareto.pkl", "rb"))
    X, F = list(d["X"]), np.atleast_2d(d["F"])
    order = np.argsort(F[:, 0])
    picks = [int(i) for i in order[np.linspace(0, len(order) - 1, min(n, len(order))).astype(int)]]

    print("CAN A LOCAL METHOD IMPROVE ON THE GA'S OWN FRONT?\n")
    print("  epsilon-constraint: hold mass at the GA's value, push effort DOWNHILL from there.")
    print("  (a weighted sum would let it BUY effort by paying mass -- which is v1's disease)\n")
    print(f"  {'#':>3s} {'GA effort':>12s} {'polished':>12s} {'gain':>8s} {'mass':>8s} {'t':>6s}")
    tot = []
    with multiprocessing.Pool(max(1, multiprocessing.cpu_count() - 2)) as pool:
        for i in picks:
            t0 = time.time()
            f1, m1, _v = polish(X[i], mass_cap=float(F[i, 1]) * 1.001, pool=pool)
            gain = 100 * (1 - f1 / F[i, 0])
            tot.append(gain)
            print(f"  {i:3d} {F[i,0]:12.3e} {f1:12.3e} {gain:+7.1f}% {m1:7.1f}g "
                  f"{time.time()-t0:5.0f}s", flush=True)

    g = np.array(tot)
    print(f"\n  mean improvement over the GA's front: {g.mean():+.1f}%  "
          f"(worst {g.min():+.1f}%, best {g.max():+.1f}%)")
    if g.mean() > 5:
        print(f"\n  -> THE GA WAS LEAVING MONEY ON THE TABLE. It found the right BASINS and then")
        print(f"     failed to walk downhill inside them. The right architecture is a HYBRID:")
        print(f"     global search for the discrete assignment, local descent for the rest.")
    else:
        print(f"\n  -> the GA's points are already at the bottom of their basins. The cliffs, not")
        print(f"     the descent, are what this problem is made of -- and that is what a population")
        print(f"     method is actually for.")


if __name__ == "__main__":
    main()

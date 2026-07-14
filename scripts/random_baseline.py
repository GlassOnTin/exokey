"""MONTE CARLO WITH A BACKUP PLAN? THEN LET US CHECK THE BACKUP PLAN IS DOING ANYTHING.

    PYTHONPATH=. .venv/bin/python scripts/random_baseline.py [n_evals]

THE USER: "My experience of GA is its better thought of as monte-carlo with a backup plan."

That is a claim with a test attached, and the test is cheap: give PURE RANDOM SAMPLING the SAME
evaluation budget the GA had, and compare the fronts. If random search finds an equally good
front, then crossover and mutation bought nothing here and the GA is a slow way to roll dice.

This project has reasons to suspect exactly that:
  * the GA has NEVER converged -- three runs, seeds splitting ~50/50
  * every real gain came from fixing the MODEL, never from more search
  * `tp_thumb` turned out to move the objective by 0.3% across its whole range
  * and the landscape is savagely rugged: a 1% nudge to the design vector moves effort/character
    by as much as the ENTIRE WIDTH of the front, because a discrete character-to-action assignment
    sits inside the objective and falls off a cliff when a letter jumps digit

Ruggedness is the case FOR a population method (gradients are useless on a cliff face) and the
case AGAINST expecting its genetics to help (there is no gradient for crossover to exploit either).
Which effect wins is an empirical question, and it has never been asked.

Compared on HYPERVOLUME -- the area the front dominates, relative to a common reference point.
It is the standard multi-objective quality measure and it does not care how many points you have.
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

from design.vector import INT_BOUNDS, MATERIAL_CHOICES, REAL_BOUNDS, evaluate
from opt.problem import hands


def sample(seed):
    rng = np.random.default_rng(seed)
    x = {k: float(rng.uniform(lo, hi)) for k, (lo, hi) in REAL_BOUNDS.items()}
    x["material"] = str(rng.choice(MATERIAL_CHOICES))
    for k, (lo, hi) in (INT_BOUNDS or {}).items():
        x[k] = int(rng.integers(lo, hi + 1))
    try:
        r = evaluate(x, hands())
        return (x, r["F"], r["feasible"])
    except Exception:
        return None


def nondominated(F):
    F = np.asarray(F, float)
    keep = np.ones(len(F), bool)
    for i in range(len(F)):
        if not keep[i]:
            continue
        dom = np.all(F <= F[i], axis=1) & np.any(F < F[i], axis=1)
        if dom.any():
            keep[i] = False
    return keep


def hypervolume(F, ref):
    """2-D hypervolume by sweep. Points must be non-dominated and below the reference."""
    F = np.asarray(F, float)
    F = F[(F[:, 0] < ref[0]) & (F[:, 1] < ref[1])]
    if not len(F):
        return 0.0
    F = F[np.argsort(F[:, 0])]
    hv, prev_y = 0.0, ref[1]
    for x, y in F:
        if y < prev_y:
            hv += (ref[0] - x) * (prev_y - y)
            prev_y = y
    return float(hv)


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 2400
    print(f"RANDOM SEARCH, {n} evaluations -- the same budget the GA had (60 x 40)\n")
    t0 = time.time()
    with multiprocessing.Pool(max(1, multiprocessing.cpu_count() - 2)) as pool:
        res = [r for r in pool.map(sample, range(n)) if r is not None]
    feas = [r for r in res if r[2]]
    print(f"  {len(res)} evaluated, {len(feas)} FEASIBLE ({100*len(feas)/max(len(res),1):.1f}%)"
          f"   [{time.time()-t0:.0f}s]")
    if not feas:
        print("\n  RANDOM SEARCH FOUND NOTHING FEASIBLE. That is itself the answer: the constraints")
        print("  are tight enough that you cannot stumble into a valid design, and the GA's")
        print("  constrained tournament -- not its genetics -- is what earns its keep.")
        pickle.dump(dict(X=[], F=[]), open("out/random.pkl", "wb"))
        return

    F = np.array([r[1] for r in feas])
    k = nondominated(F)
    Fr = F[k]
    Xr = [feas[i][0] for i in np.flatnonzero(k)]
    print(f"  random front: {len(Fr)} non-dominated")

    d = pickle.load(open("out/pareto.pkl", "rb"))
    Fg = np.atleast_2d(d["F"])
    print(f"  GA front:     {len(Fg)} non-dominated\n")

    both = np.vstack([Fr, Fg])
    ref = both.max(axis=0) * 1.05
    hv_r, hv_g = hypervolume(Fr, ref), hypervolume(Fg, ref)
    print(f"  {'':10s}{'best effort':>14s}{'lightest':>11s}{'HYPERVOLUME':>14s}")
    print(f"  {'RANDOM':10s}{Fr[:,0].min():14.3e}{Fr[:,1].min():10.1f}g{hv_r:14.3e}")
    print(f"  {'GA':10s}{Fg[:,0].min():14.3e}{Fg[:,1].min():10.1f}g{hv_g:14.3e}")
    print(f"\n  the GA's front dominates {hv_g/max(hv_r,1e-30):.2f}x the area random search does")
    if hv_g < 1.15 * hv_r:
        print(f"\n  -> THE GENETICS BOUGHT ALMOST NOTHING. On this landscape the GA IS monte carlo")
        print(f"     with a backup plan, and the backup plan is the constrained tournament.")
    else:
        print(f"\n  -> the GA is genuinely ahead. Crossover and mutation are finding structure that")
        print(f"     random sampling does not.")
    pickle.dump(dict(X=Xr, F=Fr, hv_random=hv_r, hv_ga=hv_g, n=n), open("out/random.pkl", "wb"))


if __name__ == "__main__":
    main()

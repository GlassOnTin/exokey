"""Merge Pareto fronts from independent NSGA-II runs (different seeds, different boxes).

    .venv/bin/python -m opt.merge out/pareto_seed1.pkl out/pareto_seed2.pkl

Multi-start, not just more compute. NSGA-II on a 27-variable mixed integer/categorical
problem with 9 hard constraints is not going to find the same front twice, and the first
run's 7-key ceiling looked much more like under-exploration than a real limit. Independent
seeds explore different basins; the union's non-dominated set is a strictly better front
than either alone, and comparing them tells you whether the search has converged at all --
if two seeds disagree about what is on the front, neither has.
"""
from __future__ import annotations

import pickle
import sys

import numpy as np

from hand.myohand import FINGERS


def nondominated(F: np.ndarray) -> np.ndarray:
    """Indices of the non-dominated rows (minimisation)."""
    keep = np.ones(len(F), dtype=bool)
    for i in range(len(F)):
        if not keep[i]:
            continue
        # j dominates i  <=>  j <= i everywhere and j < i somewhere
        dom = np.all(F <= F[i], axis=1) & np.any(F < F[i], axis=1)
        if dom.any():
            keep[i] = False
    return np.flatnonzero(keep)


def main(paths: list[str]) -> None:
    Fs, Xs, base, base_feas = [], [], None, None
    for p in paths:
        with open(p, "rb") as fh:
            d = pickle.load(fh)
        X = d["X"]
        if len(X) == 1 and not isinstance(X[0], dict):
            X = list(X[0])  # pymoo hands back an object ARRAY, not a list
        F = np.atleast_2d(d["F"])
        Fs.append(F)
        Xs.extend(X)
        base, base_feas = d["baseline"], d["baseline_feasible"]
        keys = [5 for _ in X]
        print(f"{p}: {len(F)} designs, {min(keys)}-{max(keys)} keys, "
              f"effort/char {F[:,0].min():.2e}..{F[:,0].max():.2e}, "
              f"mass {F[:,1].min():.0f}-{F[:,1].max():.0f} g")

    F = np.vstack(Fs)
    idx = nondominated(F)
    Fm, Xm = F[idx], [Xs[i] for i in idx]
    keys = np.array([5 for _ in Xm])

    print()
    print(f"MERGED: {len(F)} designs in -> {len(Fm)} non-dominated")
    print(f"  keys        {keys.min()} .. {keys.max()}   (2/finger = 10 is the Typeware target)")
    print(f"  effort/char {Fm[:,0].min():.3e} .. {Fm[:,0].max():.3e}")
    print(f"  mass        {Fm[:,1].min():.1f} .. {Fm[:,1].max():.1f} g")
    print(f"  deflection  {Fm[:,2].min():.3f} .. {Fm[:,2].max():.3f} mm")

    # How much did each seed actually contribute? If one seed supplies almost the whole
    # merged front, the other was wasted -- and if they split it evenly, neither had
    # converged and BOTH were under-explored.
    n = [len(f) for f in Fs]
    owner = np.searchsorted(np.cumsum(n), idx, side="right")
    print("\n  contribution to the merged front:")
    for k, p in enumerate(paths):
        c = int((owner == k).sum())
        print(f"    {p}: {c}/{len(Fm)} ({100*c/len(Fm):.0f}%)")
    if min((owner == k).sum() for k in range(len(paths))) > 0.2 * len(Fm):
        print("  => both seeds contribute substantially: the search has NOT converged.")
        print("     More seeds (or more generations) will still buy front quality.")

    with open("out/pareto.pkl", "wb") as fh:
        pickle.dump({"F": Fm, "X": Xm, "baseline": base, "baseline_feasible": base_feas}, fh)
    print("\nwrote out/pareto.pkl  ->  scripts/stage4_view.py --pick knee")


if __name__ == "__main__":
    main(sys.argv[1:])

"""DOES THE CHEAP STRUCTURAL COST RANK DESIGNS THE WAY THE REAL ONE DOES?

    PYTHONPATH=. .venv/bin/python scripts/verify_surrogate.py [n]

The GA cannot grow a gauntlet 5000 times -- one growth is ~10 minutes. So `structure.lattice.cost`
estimates the mass a layout needs from ONE solve of the solid lattice, by scaling the bar radius:

    w ~ r^-ALPHA,  m ~ r^2   =>   m_gate = m_solid * (w_solid / gate)^(2/ALPHA)

Uniform scaling is the DUMBEST way to hit a stiffness target -- ESO does far better, by moving
material to where it works -- so the estimate is an upper bound and it is a PROXY.

WHETHER A PROXY IS USABLE IS NOT A MATTER OF OPINION. What the GA needs is the RANKING: if the
proxy says design A needs less bone than B, the grown structures must agree. So: sample designs,
score each BOTH ways, and report the rank correlation. A proxy nobody measured is just a faster
way to be wrong -- and this project has already shipped one of those (the effort field's Tier-1/
Tier-2 split exists for exactly this reason).
"""
from __future__ import annotations

import multiprocessing
import os
import pickle
import sys
import time

for _v in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_v, "1")

import numpy as np
from scipy.stats import spearmanr

from design.params import DEFLECTION_MAX
from design.qwerty import best_action_map, used_actions
from design.vector import evaluate, posture, tm_of, tp_of
from hand.myohand import FINGERS
from opt.problem import hands
from structure.lattice import cost, grow


def score(x):
    """One design, both ways. Returns (tier1_g, tier2_g, struts, button_um, seconds)."""
    H = hands()
    ref = H[50]
    r = evaluate(x, H)
    wired = used_actions(r["action_map"])
    q = ref.compose({f: posture(ref, f, tp_of(x, f), tm_of(x, f),
                                float(x.get(f"ab_{f}", 0.0))) for f in FINGERS})
    c = cost(ref, q, wired=wired, gate=float(DEFLECTION_MAX))
    t0 = time.time()
    _n, _b, live, _btn, _c, _ak, _an, hist, _pc = grow(
        ref, q, wired=wired, gate=float(DEFLECTION_MAX))
    return (c["mass_g"], hist[-1][2] * 1000.0, len(live), hist[-1][1] * 1e6, time.time() - t0)


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 8
    d = pickle.load(open("out/pareto.pkl", "rb"))
    X = list(d["X"])
    rng = np.random.default_rng(7)
    picks = [X[i] for i in rng.choice(len(X), size=min(n, len(X)), replace=False)]

    print("TIER-1 (one solve of the solid, ~3 s)  vs  TIER-2 (grow the real bone, ~20 min)\n")
    with multiprocessing.Pool(min(n, max(1, multiprocessing.cpu_count() - 2))) as pool:
        res = pool.map(score, picks)

    print(f"  {'#':>3s} {'tier-1 g':>9s} {'tier-2 g':>9s} {'ratio':>7s} {'struts':>7s} "
          f"{'button':>9s} {'t2 time':>8s}")
    t1, t2 = [], []
    for k, (a, b, nl, w, dt) in enumerate(res):
        t1.append(a)
        t2.append(b)
        print(f"  {k:3d} {a:8.1f}g {b:8.1f}g {a/b:7.2f} {nl:7d} {w:7.0f}um {dt:7.0f}s")

    rho, pv = spearmanr(t1, t2)
    print(f"\n  SPEARMAN rho = {rho:+.3f}  (p = {pv:.4f})   n = {len(t1)}")
    print(f"  the proxy over-estimates mass by {np.mean(np.array(t1)/np.array(t2)):.2f}x on "
          f"average -- it must, since uniform scaling is worse than ESO")
    print("\n  WHAT MATTERS IS THE RANK, NOT THE VALUE. A high rho means the GA, steering by the")
    print("  proxy, walks toward the layouts that really are cheaper to support.")
    if rho < 0.7:
        print("\n  ⚠ rho IS TOO LOW. The proxy does not rank designs the way the real structure")
        print("    does, and steering the GA by it would be steering by noise. DO NOT SHIP THIS.")
    np.savez("out/surrogate.npz", t1=np.array(t1), t2=np.array(t2), rho=rho)


if __name__ == "__main__":
    main()

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

import pickle
import sys
import time

import numpy as np
from scipy.stats import spearmanr

from design.params import DEFLECTION_MAX
from design.qwerty import best_action_map, used_actions
from design.vector import evaluate, posture, tm_of, tp_of
from hand.myohand import FINGERS
from opt.problem import hands
from structure.lattice import cost, grow


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 8
    H = hands()
    ref = H[50]
    d = pickle.load(open("out/pareto.pkl", "rb"))
    X = list(d["X"])
    rng = np.random.default_rng(7)
    picks = [X[i] for i in rng.choice(len(X), size=min(n, len(X)), replace=False)]

    print(f"TIER-1 (one solve of the solid, ~3 s)  vs  TIER-2 (grow the real bone, ~10 min)\n")
    print(f"  {'#':>3s} {'tier-1 g':>9s} {'tier-2 g':>9s} {'ratio':>7s} {'struts':>7s} "
          f"{'button':>8s} {'t2 time':>8s}")
    t1, t2 = [], []
    for k, x in enumerate(picks):
        r = evaluate(x, H)
        wired = used_actions(r["action_map"])
        q = ref.compose({f: posture(ref, f, tp_of(x, f), tm_of(x, f),
                                    float(x.get(f"ab_{f}", 0.0))) for f in FINGERS})
        c = cost(ref, q, wired=wired, gate=float(DEFLECTION_MAX))
        t0 = time.time()
        _n, _b, live, _btn, _c, _ak, _an, hist, _pc = grow(
            ref, q, wired=wired, gate=float(DEFLECTION_MAX))
        dt = time.time() - t0
        m2 = hist[-1][2] * 1000.0
        t1.append(c["mass_g"])
        t2.append(m2)
        print(f"  {k:3d} {c['mass_g']:8.1f}g {m2:8.1f}g {c['mass_g']/m2:7.2f} "
              f"{len(live):7d} {hist[-1][1]*1e6:7.0f}um {dt:7.0f}s", flush=True)

    rho, p = spearmanr(t1, t2)
    print(f"\n  SPEARMAN rho = {rho:+.3f}  (p = {p:.4f})   n = {len(t1)}")
    print(f"  the proxy over-estimates the mass by {np.mean(np.array(t1)/np.array(t2)):.2f}x "
          f"on average (it must: uniform scaling is worse than ESO)")
    print("\n  WHAT MATTERS IS THE RANK, NOT THE VALUE. A high rho means the GA, steering by the")
    print("  proxy, walks toward the layouts that really are cheaper to support.")
    if rho < 0.7:
        print("\n  ⚠ rho IS TOO LOW. The proxy does not rank designs the way the real structure")
        print("    does, and steering the GA by it would be steering by noise. DO NOT SHIP THIS.")
    np.savez("out/surrogate.npz", t1=np.array(t1), t2=np.array(t2), rho=rho)


if __name__ == "__main__":
    main()

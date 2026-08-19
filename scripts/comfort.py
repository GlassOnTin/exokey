"""COMFORT POLISH: pull the winner's posture toward a relaxed hand where the constraints don't care.

    PYTHONPATH=. .venv/bin/python scripts/comfort.py [in.pkl] [out.pkl]

THE USER, holding the printed part: "my hand intuition feels that the little finger and thumb seem
too far from a relaxed hand position." The obvious hypothesis was unpriced slack: the knee holds
ab_little at -0.735 where -0.2 packs the cups IDENTICALLY (+1.1 mm box clearance either way). This
pass was built to reclaim that slack -- and VALIDATION FALSIFIED THE PREMISE. Run on the seed-1
knee (72 evals): every relaxed ab_little is vetoed by `swept-path` (+13.6 mm at 0, +3.2 mm even at
-0.55) -- the cups clear at the SEATED posture, but a less-splayed little finger sweeps through its
neighbour's well ON THE WAY IN. The splay is DONNING clearance, not sloppiness; the only true slack
found was dp_index 0.067 -> 0.050. So this script's real job is the opposite of its conception: it
PROVES a winner's posture is constraint-tight (or reclaims what little is not), one honest
evaluate() per candidate. If the posture must genuinely relax, the lever is the physical cup
envelope (CUP_WALL / SEAT_CLEAR / rim height), not the optimiser.

What is NOT slack stays: the index/middle/ring spread is at minimum for real 20 mm cups (+1.1 mm
margin), and the thumb's 54 mm stand-off is inherent to an OPPOSED thumb well (curling it closer
breaks `thumb-opposed` while barely moving the tip gap -- measured 52.7-57 mm regardless). This
pass touches only the per-finger POSTURE variables (ab_*, dp_*, dm_*), pulling each toward 0 (a
relaxed, un-splayed, common-curl hand) as far as ALL twelve constraints and a small objective
tolerance allow -- verified by the real evaluate() at every step, never by proxy.

Coordinate descent, not SLSQP: polish.py already measured the effort landscape's cliffs (a 1%
nudge moves effort +/-102% when the Hungarian assignment jumps), so gradients lie here. Trying
[0, half, quarter-relaxed] per variable and keeping what passes is slow but honest -- ~2-3 evals
per variable, and every accepted point is a design the full constraint set has blessed.
"""
from __future__ import annotations

import pickle
import sys
import time

import numpy as np

from design.vector import evaluate
from opt.problem import CONSTRAINT_NAMES, hands

# posture variables polished toward 0, most-contorted first; thumb + common curls stay
POLISH = ["ab_index", "ab_middle", "ab_ring", "ab_little",
          "dp_index", "dp_middle", "dp_ring", "dp_little",
          "dm_index", "dm_middle", "dm_ring", "dm_little"]
EFF_TOL = 1.02          # accept up to +2% effort
MASS_TOL = 1.02         # accept up to +2% mass


def main(inp="out/pareto.pkl", outp="out/comfort.pkl"):
    d = pickle.load(open(inp, "rb"))
    F = np.atleast_2d(d["F"])
    Fn = (F - F.min(0)) / (np.ptp(F, 0) + 1e-12)
    k = int(np.argmin((Fn ** 2).sum(1)))
    x = dict(d["X"][k])
    H = hands()

    r0 = evaluate(x, H)
    assert r0["feasible"], "polish starts from a feasible design or not at all"
    F0 = list(map(float, r0["F"]))
    print(f"knee before: effort {F0[0]:.3e}, mass {F0[1]:.2f} g")

    def ok(r):
        return (r["feasible"] and r["F"][0] <= F0[0] * EFF_TOL
                and r["F"][1] <= F0[1] * MASS_TOL)

    n_eval, t0 = 0, time.time()
    for _pass in range(2):
        moved = False
        for k_ in sorted(POLISH, key=lambda kk: -abs(float(x.get(kk, 0.0)))):
            v = float(x.get(k_, 0.0))
            if abs(v) < 1e-3:
                continue
            for frac in (0.0, 0.5, 0.75):          # full relax first; keep the best that passes
                xt = dict(x)
                xt[k_] = v * frac
                r = evaluate(xt, H)
                n_eval += 1
                if ok(r):
                    x, moved = xt, True
                    print(f"  {k_:10s} {v:+.3f} -> {v * frac:+.3f}   "
                          f"(effort {r['F'][0]:.3e}, mass {r['F'][1]:.2f} g)  [{n_eval} evals]")
                    break
                else:                              # say WHY, or the refusals teach nothing
                    if not r["feasible"]:
                        n_, g_ = max(zip(CONSTRAINT_NAMES, r["G"]), key=lambda t: t[1])
                        why = f"{n_} {g_:+.4f}"
                    else:
                        why = (f"effort {(r['F'][0]/F0[0]-1)*100:+.1f}%" if r["F"][0] > F0[0]*EFF_TOL
                               else f"mass {r['F'][1]-F0[1]:+.2f} g")
                    print(f"  {k_:10s} {v:+.3f} x> {v * frac:+.3f}   rejected: {why}")
        if not moved:
            break

    r1 = evaluate(x, H)
    assert r1["feasible"]
    worst = max(zip(CONSTRAINT_NAMES, r1["G"]), key=lambda t: t[1])
    print(f"\nknee after:  effort {r1['F'][0]:.3e} ({(r1['F'][0]/F0[0]-1)*100:+.1f}%), "
          f"mass {r1['F'][1]:.2f} g ({r1['F'][1]-F0[1]:+.2f})")
    print(f"tightest constraint: {worst[0]} {worst[1]:+.4f}   "
          f"[{n_eval} evals, {(time.time()-t0)/60:.1f} min]")
    pickle.dump({"F": [list(map(float, r1["F"]))], "X": [x],
                 "baseline": d.get("baseline"), "baseline_feasible": d.get("baseline_feasible"),
                 "polished_from": inp}, open(outp, "wb"))
    print(f"wrote {outp}")


if __name__ == "__main__":
    main(*sys.argv[1:])

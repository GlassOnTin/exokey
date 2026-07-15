"""THE IMPACT, through the gauntlet-on-strap path.  PYTHONPATH=. .venv/bin/python scripts/impact.py

A knock lands on the gauntlet (§8.15j). Two spreaders take it, in series:
  1. the RIGID LATTICE distributes the point knock to its several anchor loops (a keypress-style
     moment solve gives how the load splits between them);
  2. the SOFT STRAP spreads each loop's share over its band footprint, not a point foot.
So the peak skin pressure is  (worst anchor's share) / (band footprint), and this asks whether that
stays under a painful/injurious level for a firm knock.

⚠ QUASI-STATIC, as in §8.15i: a real impact adds dynamic amplification (energy, contact time) this
does not model, so the pressures are a LOWER bound. And the strap footprint is an estimate.
"""
from __future__ import annotations

import pickle

import numpy as np

from design.qwerty import used_actions
from design.vector import evaluate, posture, tm_of, tp_of
from hand.myohand import FINGERS
from opt.problem import hands
from structure.frame import MATERIALS, hand_axes
from structure.lattice import STRAP_K, ground, load_cases
from structure.section import Ellipse
from manufacture.bearing import KNOCK_N

E_TPU = MATERIALS["tpu"]["E"]
A_ANCHOR = 1.0e-4       # strap load area per anchor (~1 cm^2), for the compliance layer
A_BAND = 2.0e-4         # strap FOOTPRINT that carries one loop's share to the skin (~2 cm^2). Estimate.
FOOT = np.pi * (1.5e-3) ** 2   # a bare 1.5 mm anchor foot, for the "no strap" comparison


def main():
    z = np.load("out/bone.npz", allow_pickle=True)
    ref = hands()[50]
    x = pickle.load(open("out/final_design.pkl", "rb"))["x"]
    q = ref.compose({f: posture(ref, f, tp_of(x, f), tm_of(x, f), float(x.get(f"ab_{f}", 0.0)))
                     for f in FINGERS})
    wired = used_actions(evaluate(x, hands())["action_map"])
    btn = {f: int(i) for f, i in zip(z["fingers"], z["buttons"])}
    _n, _b, _bt, _l, ak, an, _tris, sn = ground(ref, q, pitch=float(z["pitch"]), reach=2.2)

    nodes = z["nodes"]
    bars = [tuple(bb) for bb in z["bars"]]
    live = [int(e) for e in z["live"]]
    sb = [bars[e] for e in live]
    EL = Ellipse(nodes, sb)
    b = np.asarray(z["b"], float)
    roll = np.asarray(z["roll"], float)
    idx = EL.fr.idx
    anch = [i for i in ak if i in idx]
    band = set(sn) & set(anch)
    ktot = sum(ak[i] for i in band) or 1.0
    ks = {i: (float(STRAP_K) * ak[i] / ktot if i in band else 0.0) for i in anch}
    akc = {i: 1.0 / (1.0 / ak[i] + 1.0 / (E_TPU * A_ANCHOR / 0.002)) for i in anch}   # +2 mm strap
    o, e_d, e_r, e_o = hand_axes(ref, q)

    def knock(node):
        """A KNOCK_N force pushing the gauntlet INTO the hand at `node`; return the anchor shares."""
        case = [("k", "knock", {int(node): -KNOCK_N * e_o})]
        lift: set = set()
        U = None
        for _ in range(8):
            spring = {i: (ks[i] if i in lift else akc[i]) for i in anch}
            U, *_ = EL.solve(b, np.zeros_like(b), roll, spring, case)
            nxt = {i for i in anch if float(U[0][6 * idx[i]:6 * idx[i] + 3] @ an[i]) > 0}
            if nxt == lift:
                break
            lift = nxt
        forces = {}
        for i in anch:
            di = U[0][6 * idx[i]:6 * idx[i] + 3] @ an[i]
            if di < 0:                     # pressing IN -> a compressive share to the skin
                forces[i] = akc[i] * (-di)
        return forces

    print(f"a {KNOCK_N:.0f} N knock, pushed into the hand at various points on the gauntlet:")
    print(f"  {'knock at':16} {'# loops sharing':>15} {'worst loop':>11} {'skin pressure':>14}")
    worst_p = 0.0
    spots = {f"{f} well": btn[f] for f in FINGERS}
    # also the most dorsal-proud lattice node (a knock on the back of the structure)
    used = [i for i in idx]
    dproud = max(used, key=lambda i: (nodes[i] - o) @ e_o)
    spots["dorsal ridge"] = dproud
    for name, node in spots.items():
        F = knock(node)
        if not F:
            print(f"  {name:16} {'(all lifted)':>15}")
            continue
        pk = max(F.values())
        nshare = sum(1 for v in F.values() if v > 0.2 * pk)
        p = pk / A_BAND
        worst_p = max(worst_p, p)
        print(f"  {name:16} {nshare:>15} {pk:>9.1f} N {p/1e3:>11.0f} kPa")

    print(f"\n  WORST peak skin pressure: {worst_p/1e3:.0f} kPa")
    print(f"  the same {KNOCK_N:.0f} N knock through ONE bare 1.5 mm foot would be "
          f"{KNOCK_N/FOOT/1e6:.1f} MPa ({KNOCK_N/FOOT/worst_p:.0f}x worse).")
    print(f"  the bearing shell (§8.15i) spread a {KNOCK_N:.0f} N knock to 56-86 kPa; the strap path "
          f"lands in the same range.")
    print("  refs: ~200 kPa a painful knock. ⚠ quasi-static (lower bound); band footprint estimated.")


if __name__ == "__main__":
    main()

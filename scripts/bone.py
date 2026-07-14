"""BONE.  PYTHONPATH=. .venv/bin/python scripts/bone.py

The friendly structure (153 members, every surface >= 1.5 mm radius, 0 spikes) costs 14.90 g against
6.17 g for the wire-thin one nobody could bear to wear. This is what an ELLIPTICAL section buys back.

THE USER: "I think the thickness of struts should be a spline too, with a major and minor radius,
and principal orientation as a spline."

    a circle spends material providing stiffness in a direction nothing is pushing.

Sized on the SCALE by optimality criteria; the ASPECT and the ROLL by WOLFF'S LAW -- turn each
section to its own principal moment, and proportion it to the ratio of the two. No gradient needed
for either: the answer is read straight off the solved end-forces.

⚠ AND THE ERGONOMIC FLOOR BINDS ON THE ELLIPSE'S *TIP*, NOT ITS WAIST. The sharpest point of an
ellipse is the end of the major axis, radius b^2/a -- so a 2:1 ellipse is SHARPER than the circle of
the same area, and the friendly constraint is b^2/a >= SKIN_R, i.e. s >= SKIN_R * k^1.5. You cannot
simply flatten it. Enforced, and checked at the end against the printed surface.
"""
from __future__ import annotations

import pickle
import time

import numpy as np

from design.params import DEFLECTION_MAX
from design.qwerty import used_actions
from design.vector import evaluate, posture, tm_of, tp_of
from hand.myohand import FINGERS
from manufacture.friendly import SKIN_R
from opt.problem import hands
from structure.lattice import STRAP_K, ground, load_cases
from structure.section import K_MAX, size_stadium


def main():
    H = hands()
    ref = H[50]
    x = pickle.load(open("out/final_design.pkl", "rb"))["x"]
    wired = used_actions(evaluate(x, H)["action_map"])
    q = ref.compose({f: posture(ref, f, tp_of(x, f), tm_of(x, f), float(x.get(f"ab_{f}", 0.0)))
                     for f in FINGERS})

    z = np.load("out/friendly.npz", allow_pickle=True)
    nodes = z["nodes"]
    bars = [tuple(b) for b in z["bars"]]
    live = [int(e) for e in z["live"]]
    btn = {f: int(i) for f, i in zip(z["fingers"], z["buttons"])}
    pitch = float(z["pitch"])
    circ = float(z["mass"])                       # the ROUND answer, in grams

    _n, _b, _bt, _l, ak, an, _t, sn = ground(ref, q, pitch=pitch, reach=2.2)
    cases = load_cases(ref, q, btn, wired=wired)
    sb = [bars[e] for e in live]
    R = float(SKIN_R)

    print(f"THE FRIENDLY STRUCTURE: {len(sb)} members, {circ:.2f} g as ROUND rods, "
          f"every surface >= {R*1e3:.1f} mm")
    print(f"  gate {float(DEFLECTION_MAX)*1e6:.0f} um, {len(cases)} wired load cases, "
          f"aspect capped at {float(K_MAX):.0f}:1\n")

    t0 = time.time()
    b, t, roll, m, w, EL = size_stadium(
        nodes, sb, btn, cases, ak, an, sn, float(STRAP_K),
        gate=float(DEFLECTION_MAX), b_min=R)
    if not np.isfinite(w):
        raise SystemExit("no stadium structure met the gate")

    from structure.section import Ellipse
    W = Ellipse.WALL
    ri = np.maximum(b - W, 0.0)
    hollow = ri > 0
    print(f"  {len(sb)} members, {m*1000:.2f} g, worst button {w*1e6:.0f} um "
          f"(gate {float(DEFLECTION_MAX)*1e6:.0f})   [{time.time()-t0:.0f}s]")
    print(f"  outer radius {b.min()*1e3:.2f} - {b.max()*1e3:.2f} mm   "
          f"wall {W*1e3:.1f} mm (two perimeters of a 0.4 mm nozzle)")
    print(f"  bore: {int(hollow.sum())}/{len(b)} members are HOLLOW "
          f"(a member thinner than the wall stays solid)")
    print(f"  the sharpest point on the whole part: {b.min()*1e3:.2f} mm "
          f"(floor {R*1e3:.1f} mm) -- "
          f"{'FRIENDLY' if b.min() >= R - 1e-9 else '⚠ TOO SHARP'}")
    print("  A tube's outer radius IS its minimum surface radius. Hollowing it changes NOTHING")
    print("  about how it feels, and it is what a long bone does with its marrow cavity.")
    print(f"\n  MASS: {circ:.2f} g solid  ->  {m*1000:.2f} g hollow  "
          f"({100*(m*1000/circ - 1):+.0f}%)")
    aspect = np.ones_like(b)

    np.savez("out/bone.npz", nodes=nodes, bars=np.array(bars), live=np.array(live),
             b=b, t=t, roll=roll, radii=b, aspect=aspect,
             buttons=z["buttons"], fingers=z["fingers"], anchors=z["anchors"],
             mass=m * 1000, button_um=w * 1e6, bars0=len(bars), build_dir=z["build_dir"],
             pitch=pitch, skin_r=R, pillars=z["pillars"], sagging=z["sagging"])
    print("\n  wrote out/bone.npz")


if __name__ == "__main__":
    main()

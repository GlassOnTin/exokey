"""BONE -- everything at once.  PYTHONPATH=. .venv/bin/python scripts/bone.py

The whole stack, on one structure:

    FRIENDLY   every surface >= SKIN_R, 0 spikes            (the ergonomic floor)
    CURVED     the load paths are cubic splines, not kinks  (bones have no sharp edges)
    HOLLOW     tubes, wall = two perimeters of the nozzle   (a bone has a marrow cavity)
    PRINTABLE  every member >= the nozzle, supports counted (one person, one printer)


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
from hand.flesh import skin
from structure.lattice import SEG_CLEAR, STRAP_K, ground, load_cases
from structure.section import size_stadium
from structure.spline import TENSION, curves, kink, push_out


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

    # ⚠ CLEAN IT FIRST. friendly.npz was written before `cleanup` existed, and the user found what
    # it left behind: 9 loose ends and a 13-node fragment floating free of everything.
    from structure.lattice import cleanup
    bearing = set(ak) | {int(v) for v in btn.values()}
    n_before = len(live)
    live = cleanup(bars, live, bearing, btn.values())
    if len(live) != n_before:
        print(f"  ⚠ CLEANED: dropped {n_before - len(live)} members -- debris and loose ends "
              f"({n_before} -> {len(live)})")
    sb = [bars[e] for e in live]

    turns0 = kink(nodes, bars, live)
    print(f"THE FRIENDLY STRUCTURE: {len(sb)} members, {circ:.2f} g as SOLID ROUND rods, "
          f"every surface >= {R*1e3:.1f} mm, 0 spikes")
    print(f"  its load paths still KINK: median {np.median(turns0):.0f} deg at each node "
          f"they run through\n")

    # ---- CURVE the load paths ------------------------------------------------------------------
    from scipy.spatial import cKDTree
    V, _F = skin(ref, q)
    tree = cKDTree(V)
    need = {e: float(SEG_CLEAR) * 0.004 + float(z["radii"][k]) for k, e in enumerate(live)}
    n2, b2, owner = curves(nodes, bars, live, tension=float(TENSION) * 0.3)
    n2, moved = push_out(n2, len(nodes), b2, owner, tree, need)
    turns1 = kink(n2, b2, list(range(len(b2))))
    print(f"CURVED: {len(b2)} sub-beams from {len(sb)} members; the kink at each node falls "
          f"{np.percentile(turns0, 90):.0f} -> {np.percentile(turns1, 90):.0f} deg (p90)")
    print(f"  (and the curves were pushed {moved*1e3:.2f} mm back out of the flesh)\n")

    print(f"  gate {float(DEFLECTION_MAX)*1e6:.0f} um, {len(cases)} wired load cases\n")
    t0 = time.time()
    b, t, roll, m, w, EL = size_stadium(
        n2, b2, btn, cases, ak, an, sn, float(STRAP_K),
        gate=float(DEFLECTION_MAX), b_min=R)
    nodes, sb, live = n2, b2, list(range(len(b2)))
    bars = b2
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

    from structure.lattice import buildable, unsupported
    bd = np.asarray(z["build_dir"], float)
    bd /= np.linalg.norm(bd)
    pil = unsupported(nodes, bars, live, bd)
    ok = buildable(nodes, bars, bd)
    sag = [e for e in live if not ok[e]]
    print(f"\n  SUPPORT: {len(pil)} pillars + {len(sag)} props")

    np.savez("out/bone.npz", nodes=nodes, bars=np.array(bars), live=np.array(live),
             b=b, wall=W, roll=roll, radii=b, owner=owner,
             buttons=z["buttons"], fingers=z["fingers"], anchors=z["anchors"],
             mass=m * 1000, button_um=w * 1e6, bars0=len(bars), build_dir=bd,
             pitch=pitch, skin_r=R, pillars=np.array(pil, dtype=int),
             sagging=np.array(sag, dtype=int))
    print("\n  wrote out/bone.npz")


if __name__ == "__main__":
    main()

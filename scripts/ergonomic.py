"""THE PRICE OF NOT BEING A KNUCKLE-DUSTER.  PYTHONPATH=. .venv/bin/python scripts/ergonomic.py

THE USER: "Sharp corners are not ergonomic, meaning they hurt flesh when forced against a hand, or
scratch skin even by accident. From a product perspective this abstracts to a keyboard needing to be
'friendly' and not weaponry like a knuckle duster."  And then: "THE WHOLE EXERCISE IS A HUMAN FACTORS
DESIGN. That's part of the appeal."

WHICH IS THE ORGANISING PRINCIPLE OF THIS WHOLE PROJECT, AND I HAD LOST IT.

Nearly every constraint here is a fact about PEOPLE: the deflection gate (a key that moves feels
mushy), the press force, the switch travel, the standoff (it must not rub), the strap grip (it must
not dig in), the adjustment range (it must fit the 5th to the 95th percentile), the muscle effort,
the mass (it is worn all day). Only THREE are facts about a MACHINE -- the nozzle, the overhang and
the bridging span.

And I let one of the machine's numbers set a limit that a HAND has to touch. `NOZZLE_R` = 0.4 mm is
what the PRINTER can lay. It is not, and was never, what a PALM can bear. Measured on the shipped
structure: 669 of 669 members (100%) are thinner than a friendly surface allows, at a median radius
of 0.41 mm -- a 0.8 mm wire -- and 56 of them end in a free 0.4 mm POINT.

So this sweeps the ERGONOMIC floor -- the smallest radius any surface a hand can touch is allowed to
have -- and measures what enforcing it costs.

⚠ AND IT IS ALSO THE TEST OF A CLAIM I MADE AND RETRACTED. VISION.md 8.15(a2) said that the
minimum-feature bound is what gives a topology-optimised structure its HIERARCHY -- few, thick,
graded members instead of many equal thin ones -- and I withdrew it, because the 0.4 mm nozzle floor
was far too small to force anything. An ERGONOMIC floor is several times larger, and floor mass goes
as r^2. If the claim is right, THIS is the constraint that finally makes the structure a bone.
"""
from __future__ import annotations

import pickle
import time

import numpy as np

from design.params import DEFLECTION_MAX
from design.qwerty import used_actions
from design.vector import evaluate, posture, tm_of, tp_of
from hand.myohand import FINGERS
from manufacture.friendly import SKIN_R, spikes
from opt.problem import hands
from structure.lattice import NOZZLE_R, STRAP_K, buildable, ground, load_cases, unsupported
from structure.sizing import size_and_prune

FLOORS = (4.0e-4, 8.0e-4, 1.2e-3, 1.5e-3, 2.0e-3)      # m -- the nozzle, then upward


def main():
    H = hands()
    ref = H[50]
    x = pickle.load(open("out/final_design.pkl", "rb"))["x"]
    wired = used_actions(evaluate(x, H)["action_map"])
    q = ref.compose({f: posture(ref, f, tp_of(x, f), tm_of(x, f), float(x.get(f"ab_{f}", 0.0)))
                     for f in FINGERS})

    pitch, reach = 0.008, 2.2
    nodes, bars, btn, _l, ak, an, _t, sn = ground(ref, q, pitch=pitch, reach=reach)
    cases = load_cases(ref, q, btn, wired=wired)
    z = np.load("out/printable.npz", allow_pickle=True)
    build = np.asarray(z["build_dir"], float)
    build /= np.linalg.norm(build)

    print(f"THE PRICE OF NOT BEING A KNUCKLE-DUSTER   ({len(bars)} candidate members, "
          f"gate {float(DEFLECTION_MAX)*1e6:.0f} um)\n")
    print(f"  {'floor':>7s} {'members':>8s} {'MASS':>8s} {'button':>7s} "
          f"{'radii (mm)':>16s} {'spikes':>7s} {'support':>8s}   what it is")
    base = None
    keep_all = {}
    for rmin in FLOORS:
        t0 = time.time()
        stop = {}
        live, r, m, w = size_and_prune(
            nodes, bars, btn, cases, ak, an, sn, float(STRAP_K),
            gate=float(DEFLECTION_MAX), r_print=rmin, build_dir=build,
            on_stop=lambda why, n_, mm: stop.update(why=why))
        if not len(live):
            print(f"  {rmin*1e3:5.1f}mm   NO STRUCTURE MEETS THE GATE ({stop.get('why')})")
            continue
        sp = spikes(nodes, bars, live, buttons=[int(i) for i in z["buttons"]])
        pil = len(unsupported(nodes, bars, live, build))
        ok = buildable(nodes, bars, build)
        props = int(sum(1 for e in live if not ok[e]))
        if base is None:
            base = m
        what = ("the NOZZLE -- what the printer can lay" if abs(rmin - float(NOZZLE_R)) < 1e-9
                else "" if rmin != float(SKIN_R) else "<- the ERGONOMIC floor")
        print(f"  {rmin*1e3:5.1f}mm {len(live):8d} {m*1000:6.2f} g {w*1e6:5.0f}um "
              f"{r.min()*1e3:5.2f}-{r.max()*1e3:5.2f} (p50 {np.median(r)*1e3:4.2f}) "
              f"{len(sp):7d} {pil+props:8d}   {what}  [{time.time()-t0:3.0f}s]", flush=True)
        keep_all[rmin] = (live, r, m, w)

    if not keep_all or base is None:
        raise SystemExit("nothing survived")

    print("\n  WHAT THE ERGONOMIC FLOOR DOES TO THE STRUCTURE:")
    n0 = len(keep_all[FLOORS[0]][0])
    for rmin, (live, r, m, w) in keep_all.items():
        if rmin == FLOORS[0]:
            continue
        print(f"    {rmin*1e3:.1f} mm:  {len(live):4d} members ({100*(len(live)/n0-1):+.0f}%), "
              f"{m*1000:5.2f} g ({100*(m/base-1):+.0f}%), "
              f"median radius {np.median(r)*1e3:.2f} mm "
              f"({np.median(r)/np.median(keep_all[FLOORS[0]][1]):.1f}x thicker)")

    print("\n  ⚠ AND EVERY ONE OF THESE IS PRINTABLE. The nozzle floor is 0.4 mm; the ergonomic")
    print("    floor is ABOVE it, so it is the binding one and the printer never notices.")

    if float(SKIN_R) in keep_all:
        live, r, m, w = keep_all[float(SKIN_R)]
        np.savez("out/friendly.npz", nodes=nodes, bars=np.array(bars), live=np.array(live),
                 radii=r, buttons=z["buttons"], fingers=z["fingers"], anchors=np.array(sorted(ak)),
                 mass=m * 1000, button_um=w * 1e6, bars0=len(bars), build_dir=build, pitch=pitch,
                 skin_r=float(SKIN_R),
                 pillars=np.array(unsupported(nodes, bars, live, build), dtype=int),
                 sagging=np.array([e for e in live
                                   if not buildable(nodes, bars, build)[e]], dtype=int))
        print(f"\n  wrote out/friendly.npz  ({len(live)} members, {m*1000:.2f} g, "
              f"every surface >= {float(SKIN_R)*1e3:.1f} mm radius, 0 spikes)")


if __name__ == "__main__":
    main()

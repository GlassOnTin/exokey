"""THE DEFINITIVE STRUCTURE: gradient-sized, at full resolution.

    PYTHONPATH=. .venv/bin/python scripts/size_final.py

THE USER: "The current best skeleton still looks a bit unnatural (zig-zaggy and
not-natural-intuitive-entropy) which is my guide to say we can still further converge or better
optimise the result."

Their eye is right, and it is right about something specific. The skeleton they were looking at was
grown by ESO, which is BINARY: a strut is in or it is out, so EVERY SURVIVING STRUT IS THE SAME
0.90 mm THICKNESS. There is no hierarchy -- no thick trunk tapering into thin braces -- and that
hierarchy is exactly the "natural entropy" the eye is looking for in a bone. ESO cannot express it;
it has no radius to vary.

The gradient sizer can, and does: radii spread over roughly 6x, at ~42% less mass. This grows the
real thing at full resolution and saves the per-strut radii so the render can finally draw each
member at the thickness the physics chose for it.
"""
from __future__ import annotations

import pickle
import time

import numpy as np

from design.params import DEFLECTION_MAX
from design.qwerty import used_actions
from design.vector import evaluate, posture, tm_of, tp_of
from hand.myohand import FINGERS
from opt.problem import hands
from structure.lattice import STRAP_K, ground, load_cases
from structure.sizing import size_and_prune


def main():
    H = hands()
    ref = H[50]
    fd = pickle.load(open("out/final_design.pkl", "rb"))
    x = fd["x"]
    r0 = evaluate(x, H)
    wired = used_actions(r0["action_map"])
    q = ref.compose({f: posture(ref, f, tp_of(x, f), tm_of(x, f), float(x.get(f"ab_{f}", 0.0)))
                     for f in FINGERS})

    nodes, bars, btn, _l, ak, an, _t, strap_n = ground(ref, q)      # FULL resolution
    cases = load_cases(ref, q, btn, wired=wired)
    print(f"{len(bars)} candidate struts, {len(cases)} load cases, "
          f"gate {float(DEFLECTION_MAX)*1e6:.0f} um\n")

    t0 = time.time()
    live, r, m, w = size_and_prune(
        nodes, bars, btn, cases, ak, an, strap_n, float(STRAP_K),
        gate=float(DEFLECTION_MAX),
        on_step=lambda s, n, mm, ww: print(
            f"  prune {s:2d}: {n:5d} struts  {mm*1000:6.2f} g  {ww*1e6:4.0f} um  "
            f"[{time.time()-t0:5.0f}s]", flush=True))

    print(f"\n  {len(live)} struts   {m*1000:.2f} g   buttons {w*1e6:.0f} um")
    print(f"  radii {r.min()*1e3:.2f} - {r.max()*1e3:.2f} mm  "
          f"(ESO forces every strut to 0.90 mm -- it has no radius to vary)")
    q10, q50, q90 = np.percentile(r * 1e3, [10, 50, 90])
    print(f"  p10 {q10:.2f}   median {q50:.2f}   p90 {q90:.2f} mm   "
          f"-> a {q90/max(q10,1e-9):.1f}x hierarchy, which is what a bone looks like")

    np.savez("out/sized.npz", nodes=nodes, bars=np.array(bars), live=np.array(live),
             radii=r,                                    # POSITIONAL: radii[k] <-> bars[live[k]]
             buttons=np.array([btn[f] for f in FINGERS]), fingers=np.array(FINGERS),
             anchors=np.array(sorted(ak)), mass=m * 1000, button_um=w * 1e6,
             bars0=len(bars))
    print("\n  wrote out/sized.npz")


if __name__ == "__main__":
    main()

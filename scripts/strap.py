"""THE STRAP subsystem, measured.  PYTHONPATH=. .venv/bin/python scripts/strap.py

The band path (hull of skin ∪ gauntlet), the watch-lug anchor at the feet, the buckle's adjustment
range across the population, and where a pad is needed.
"""
from __future__ import annotations

import pickle

import numpy as np

from design.vector import posture, tm_of, tp_of
from hand.myohand import FINGERS
from opt.problem import hands
from structure.anchor import strap_bands
from structure.frame import hand_axes
from manufacture.strap import (adjust_range, band_loop, bridging_fraction, lug_sites, perimeter)


def main():
    H = hands((5, 50, 95))
    ref = H[50]
    x = pickle.load(open("out/final_design.pkl", "rb"))["x"]
    q = ref.compose({f: posture(ref, f, tp_of(x, f), tm_of(x, f), float(x.get(f"ab_{f}", 0.0)))
                     for f in FINGERS})
    z = np.load("out/bone.npz", allow_pickle=True)
    nodes = z["nodes"]
    bars = [tuple(b) for b in z["bars"]]
    live = [int(e) for e in z["live"]]
    device = (nodes, bars, live, z["radii"])
    anchors = [int(i) for i in z["anchors"]]
    o, e_d, _r, _oo = hand_axes(ref, q)
    stations = strap_bands(ref, q, np.array([nodes[i] for i in anchors]))

    print("THE BAND PATH -- convex hull of (skin ∪ gauntlet), so it goes OVER the structure:")
    for name, st in zip(("wrist", "metacarpal"), stations):
        skin_only = band_loop(ref, q, st, device=None)
        band = band_loop(ref, q, st, device=device)
        bridge = bridging_fraction(band, ref, q)
        print(f"  {name:11} station {st*1e3:+5.0f}mm  circumference {perimeter(band)*1e3:5.0f}mm "
              f"(skin-only {perimeter(skin_only)*1e3:.0f}mm -- the band bulges "
              f"{(perimeter(band)-perimeter(skin_only))*1e3:.0f}mm over the device)")
        print(f"              {bridge*100:.0f}% of the band bridges >2.5mm off the skin "
              f"-> bears on the high points beside it (pad those)")

    print("\nTHE WATCH-LUG ANCHOR -- a captured pin at each anchor foot, strap loops it in SHEAR:")
    sites = lug_sites(ref, q, nodes, anchors, device)
    per_band = {}
    for s in sites:
        per_band.setdefault(s["band"], []).append(s)
    for b in sorted(per_band):
        name = ("wrist", "metacarpal")[b] if b < 2 else f"band{b}"
        print(f"  {name:11} {len(per_band[b])} lug(s), pin axis along the hand (e_d), "
              f"hole r {per_band[b][0]['hole_r']*1e3:.1f}mm (>= 2 nozzle walls)")

    print("\nADJUSTMENT -- the buckle/holes must cover the population's wrist-circumference spread:")
    a = adjust_range(H, x)
    for pct in sorted(a["per_hand"]):
        print(f"  {pct:2d}th percentile: {a['per_hand'][pct]*1e3:.0f}mm")
    print(f"  -> RANGE {a['min']*1e3:.0f}-{a['max']*1e3:.0f}mm, spread {a['spread']*1e3:.0f}mm "
          f"({a['max']/a['min']:.2f}x). A watch strap's holes span ~30-40mm, so one strap covers it.")

    print("\nBOND (materials, not modelled): PU adhesive for the TPU strap onto the printed lug; a")
    print("vinyl-silane primer keys to the GLASS in glass-nylon before the structural epoxy/PU. The")
    print("captured pin carries the load; the bond only has to survive handling, not the tension.")


if __name__ == "__main__":
    main()

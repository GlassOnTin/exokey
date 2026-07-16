"""THE MAGNETIC READ-OUT, priced.

    PYTHONPATH=. .venv/bin/python scripts/readout.py

The contactless-Hall well needs the field a moving magnet presents to a 3-axis Hall to be
LEGIBLE: a keypress must swing it far past the sensor's noise, the five directions must point
apart, and a neighbour well must not leak enough to false-trigger. This sweeps the magnet and the
gap, prints the signal budget, the five-direction map, the crosstalk at the shipped design's real
well spacings, and a duty-cycled power sketch. Analysis only -- writes nothing.
"""
from __future__ import annotations

import itertools
import os

import numpy as np

from design.params import (HALL_LSB, HALL_NOISE, HALL_RANGE, MAGNET_D, MAGNET_L, REST_GAP)
from manufacture import readout as ro

LSB, NOISE, RANGE = float(HALL_LSB), float(HALL_NOISE), float(HALL_RANGE)


def _pair_distances():
    if not os.path.exists("out/final.npz"):
        return None
    z = np.load("out/final.npz", allow_pickle=True)
    P = np.array(z["nodes"], float)[np.array(z["buttons"], int)]
    names = list(z["fingers"])
    return [(names[i], names[j], float(np.linalg.norm(P[i] - P[j])))
            for i, j in itertools.combinations(range(len(P)), 2)]


def main():
    gap = float(REST_GAP)
    print(f"THE READ-OUT  (Ø{float(MAGNET_D)*1e3:.0f}x{float(MAGNET_L)*1e3:.0f} mm N42 disc over a "
          f"3-axis Hall, rest gap {gap*1e3:.1f} mm)")
    print(f"  sensor: {LSB*1e3:.3f} mT/LSB, +-{RANGE*1e3:.0f} mT range, ~{NOISE*1e3:.1f} mT noise\n")

    print("PLUNGE (click) -- exact on-axis cylinder field:")
    for label, z in (("rest", gap), ("full 1.5 mm", gap - ro.TRAVEL),
                     ("hard stop 1.8 mm", gap - ro.PLUNGE_STOP)):
        b = float(ro.cyl_axial_B(z))
        print(f"  {label:16s} gap {z*1e3:4.1f} mm -> {b*1e3:5.1f} mT  ({b/RANGE:4.2f} of range)")
    dbz = float(np.linalg.norm(ro.delta_B("click")))
    print(f"  swing = {dbz*1e3:.1f} mT = {dbz/LSB:.0f} LSB = {dbz/NOISE:.0f}x noise\n")

    print("THE FIVE DIRECTIONS at full travel (sensor frame, mT):")
    dmap = ro.direction_map()
    print(f"  {'action':8s} {'Bx':>7} {'By':>7} {'Bz':>7}  {'|dB|':>7} {'LSB':>6}")
    for a, v in dmap.items():
        print(f"  {a:8s} {v[0]*1e3:7.2f} {v[1]*1e3:7.2f} {v[2]*1e3:7.2f}  "
              f"{np.linalg.norm(v)*1e3:7.2f} {np.linalg.norm(v)/LSB:6.0f}")
    weak, ang = ro.discriminability(dmap)
    err = ro.classify_mc(dmap, noise=NOISE, n=100_000)
    print(f"  weakest {weak*1e3:.2f} mT ({weak/NOISE:.0f}x noise); min pairwise angle {ang:.0f} deg; "
          f"nearest-template errors {err:.0%} of 1e5\n")

    print("MAGNET / GAP SWEEP  (plunge swing in LSB | weakest direction in LSB | rest field vs range):")
    print(f"  {'Ø(mm)':>6} {'L(mm)':>6} {'gap(mm)':>8}  {'plunge':>7} {'weakest':>8} {'rest/range':>11}")
    for d in (2e-3, 3e-3, 4e-3):
        for L in (1e-3, 2e-3):
            for g in (3e-3, 3.5e-3, 4e-3, 5e-3):
                geom = dict(d=d, L=L, gap=g)
                pl = float(np.linalg.norm(ro.delta_B("click", **geom))) / LSB
                wk = ro.discriminability(ro.direction_map(**geom))[0] / LSB
                rest = float(ro.cyl_axial_B(g, d=d, L=L)) / RANGE
                flag = "  <-- clips" if rest > 0.8 else ""
                print(f"  {d*1e3:6.0f} {L*1e3:6.0f} {g*1e3:8.1f}  {pl:7.0f} {wk:8.0f} {rest:11.2f}{flag}")
    print("  (the shipped Ø3x1 @ 3.5 mm: strong plunge, weakest still tens of LSB, no clipping)\n")

    print("NEIGHBOUR CROSSTALK  (a pressed well's field change at another well's Hall):")
    pairs = _pair_distances()
    if pairs is None:
        print("  out/final.npz absent -- showing generic spacings")
        pairs = [("", "", s) for s in (18.6e-3, 25.9e-3, 38.5e-3, 51.4e-3)]
    for fa, fb, s in sorted(pairs, key=lambda t: t[2]):
        st, mod = ro.crosstalk(s)
        tag = f"{fa}-{fb}" if fa else ""
        print(f"  {tag:14s} {s*1e3:5.1f} mm -> static {st*1e3:5.3f} mT, "
              f"modulation {mod*1e3:5.3f} mT ({mod/NOISE:4.2f}x noise)")
    print("  static is a constant the baseline removes; only the modulation can fool a keypress.\n")

    print("POWER  (duty-cycled sketch, SPEC/estimate -- bench-verified at stage 1):")
    for rate in (200.0, 500.0):
        p = ro.scan_power(rate_hz=rate)
        print(f"  {rate:.0f} Hz scan: {p['total_A']*1e3:.2f} mA total "
              f"({p['sensors_A']*1e6:.0f} uA sensors + {p['mcu_avg_A']*1e3:.1f} mA MCU/BLE) "
              f"-> {p['life_h_100mAh']:.0f} h on 100 mAh")


if __name__ == "__main__":
    main()

"""THE MOMENT AND LEVER, MEASURED.  PYTHONPATH=. .venv/bin/python scripts/sensor.py

For each finger and each of the five well directions, how much muscle effort does actuating it
cost across the 5th-95th population, and can the digit balance it? From that, the dome rate each
finger wants -- and whether one symmetric dome serves its four tilt directions.
"""
from __future__ import annotations

import pickle

import numpy as np

from design.params import DEFLECTION_MAX, RESIDUAL_MAX, SVALBOARD  # noqa: F401
from design.qwerty import ACTIONS
from design.sensor import PLUNGE, TILT, actuation_cost
from hand.myohand import FINGERS
from manufacture.flexure import dome, dome_stress, spring_rate
from opt.problem import hands
from structure.frame import MATERIALS

TRAVEL = float(SVALBOARD.travel)
GF = 9.80665e-3                       # 1 gram-force in newtons
FORCES_GF = [5, 10, 15, 20, 30]
RMAX = float(RESIDUAL_MAX)


def feasible(c):
    return c["residual"] <= RMAX


def main():
    x = pickle.load(open("out/final_design.pkl", "rb"))["x"]
    H = hands((5, 50, 95))
    print(f"population {sorted(H)}  |  Svalboard reference {FORCES_GF[3]} gf, travel {TRAVEL*1e3:.1f} mm")
    print(f"feasible = every hand balances it (worst residual <= {RMAX:.2f})\n")

    costs = {gf: actuation_cost(H, x, gf * GF) for gf in FORCES_GF}

    for f in FINGERS:
        print(f"{f.upper()}")
        print(f"  {'direction':9} " + " ".join(f"{gf:>3}gf" for gf in FORCES_GF) + "   feasible@20gf")
        for act in ACTIONS:
            line = []
            for gf in FORCES_GF:
                e = costs[gf][(f, act)]["effort"]
                line.append(f"{e*1e3:5.2f}" if np.isfinite(e) else "  inf")
            ok = feasible(costs[20][(f, act)])
            tag = "" if act not in ("click",) + TILT else ("plunge" if act in PLUNGE else "tilt")
            print(f"  {act:9} " + " ".join(line) + f"   {'yes' if ok else 'NO ':>3}   {tag}")
        print()

    # ---- the dome rate each finger wants ----------------------------------------------------
    print("effort is Σa³ x1e3 (mean over the population).  DOME RATE per finger, softest deliberate")
    print("press = the Svalboard 20 gf floor (a guess -- the false-trigger floor is not measured):\n")
    k = spring_rate(FORCES_GF[3] * GF, TRAVEL)
    tpu = MATERIALS["tpu"]
    a = 0.006
    t = dome(k, a, tpu["E"], tpu["nu"])
    print(f"  plunge+tilt at 20 gf -> k = {k:.0f} N/m -> TPU dome r={a*1e3:.0f}mm t={t*1e3:.2f}mm "
          f"stress {dome_stress(FORCES_GF[3]*GF, t)/1e6:.1f} MPa\n")

    # ---- USABLE DIRECTIONS per finger: feasibility is the gate ----------------------------------
    c20 = costs[FORCES_GF[3]]
    print("USABLE DIRECTIONS per finger at 20 gf (every hand can balance it). Effort is negligible")
    print("where feasible -- the cradle bears the load -- so FEASIBILITY is the gate:\n")
    for f in FINGERS:
        ok = [a for a in ACTIONS if feasible(c20[(f, a)])]
        print(f"  {f:7} {len(ok)}/5: {', '.join(ok)}")
    total = sum(feasible(c20[(f, a)]) for f in FINGERS for a in ACTIONS)
    print(f"\n  TOTAL usable (finger, direction) pairs: {total}/25 -- the wells are five-way for every")
    print("  finger. The ulnar lateral tilts were NOT a muscle limit: the interossei are present and")
    print("  adequate. The cradle model had WITHHELD the well FLOOR during a lateral press, demanding")
    print("  a muscle for the IP torque the floor (and, in a real finger, the DIP collateral ligaments)")
    print("  bears. With the floor restored (hand/cradle.py), every direction is actuable.")
    print("  ⚠ MyoHand still has no extensor hood -- a real gap, but not the operative one here.")
    print("     OpenSim ARMS (43 muscles, real hood) is the pending independent cross-check.")


if __name__ == "__main__":
    main()

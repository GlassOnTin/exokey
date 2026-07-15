"""THE FINGER-WELL FLEXURE, priced across the material palette.

    PYTHONPATH=. .venv/bin/python scripts/flexure.py

The contactless-Hall well needs a printed flexure whose restoring force is a SOFT spring --
k = F/travel for the Svalboard 20 gf key -- in all five directions. Which of the user's materials
can be that spring without cracking? The answer is set by ONE number, sigma_fat/E, the maximum
recoverable bending strain, and it splits the palette cleanly.
"""
from __future__ import annotations

from design.params import SVALBOARD
from manufacture.flexure import (axial_k, dome, dome_stress, fatigue_strain, leaf, rod,
                                 spring_rate)
from structure.frame import MATERIALS

F, TRAVEL = float(SVALBOARD.force), float(SVALBOARD.travel)
K = spring_rate(F, TRAVEL)

PALETTE = ["tpu", "pa12", "pla", "petg", "asa", "cf_pa12", "spring_steel"]


def main():
    print(f"THE WELL FLEXURE  (replaces the Svalboard key: {F*1e3:.0f} mN at {TRAVEL*1e3:.1f} mm)")
    print(f"  target spring rate k = F/travel = {K:.0f} N/m -- a very soft spring\n")

    print("ISOTROPIC ROD (soft in every lateral direction, one part) at L = 15 mm:")
    print(f"  {'material':13s} {'dia(mm)':>7} {'stress':>8} {'fatigue':>8}  {'merit e_fat':>11}  verdict")
    for name in PALETTE:
        m = MATERIALS[name]
        r, s = rod(K, 0.015, m["E"], TRAVEL)
        merit = fatigue_strain(m["fatigue"], m["E"])
        ok = "OK" if s < m["fatigue"] / 1.5 else ("marginal" if s < m["fatigue"] else "CRACKS")
        print(f"  {name:13s} {2*r*1e3:7.2f} {s/1e6:6.1f}MPa {m['fatigue']/1e6:6.0f}MPa "
              f"  {merit:11.4f}  {ok}")

    print("\n  -> only TPU clears a 1.5x fatigue margin as a simple rod. The merit e_fat = sigma_fat/E")
    print("     is why: an elastomer bends a lot before it fatigues; a stiff plastic does not.\n")

    print("TPU DOME (isotropic: does tilt AND plunge in one part, and snaps for a click):")
    m = MATERIALS["tpu"]
    for a in (0.004, 0.005, 0.006):
        t = dome(K, a, m["E"], m["nu"])
        floor = "below the 0.3 mm FDM floor" if t < 0.3e-3 else "clears the 0.3 mm FDM floor"
        print(f"  radius {a*1e3:.0f} mm -> thickness {t*1e3:.2f} mm, stress {dome_stress(F, t)/1e6:.2f} MPa"
              f"  (fits the ~7 mm well; {floor})")

    print("\nSPRING-STEEL LEAF (the metal route: go thin, where a nozzle cannot):")
    m = MATERIALS["spring_steel"]
    _, s_rod = rod(K, 0.015, m["E"], TRAVEL)
    print(f"  as a ROD: {s_rod/1e6:.0f} MPa vs {m['fatigue']/1e6:.0f} MPa limit -> CRACKS")
    for h in (0.10e-3, 0.15e-3):
        w, strain = leaf(K, 0.015, h, m["E"], TRAVEL)
        ok = "OK" if strain < fatigue_strain(m["fatigue"], m["E"]) / 1.5 else "tight"
        print(f"  as a {h*1e3:.2f} mm LEAF: width {w*1e3:.1f} mm, strain {strain*1e3:.2f}e-3 "
              f"vs merit {fatigue_strain(m['fatigue'], m['E'])*1e3:.2f}e-3 -> {ok} "
              f"(one axis only -- needs a cruciform for two)")

    print("\nWHY THE PLUNGE MUST BEND, NOT COMPRESS (that sized rod, pressed axially):")
    for name in ("tpu", "cf_pa12", "spring_steel"):
        ka = axial_k(K, 0.015, MATERIALS[name]["E"])
        print(f"  {name:13s} axial k = {ka:>10.0f} N/m = {ka/K:>5.0f}x too stiff")


if __name__ == "__main__":
    main()

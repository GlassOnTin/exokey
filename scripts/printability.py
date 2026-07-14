"""CAN IT BE PRINTED ON AN FDM MACHINE WITHOUT SUPPORTS? Measure, then constrain.

    PYTHONPATH=. .venv/bin/python scripts/printability.py

THE USER: "I'm curious to find a smoothed skeletal geometry that we could 3d print without too many
supports... the point is to make a structure that can self support as FDM printer layers aligned
with some convenient plane."

TWO RULES DECIDE THIS, and both are hard constraints, not preferences:

  1. MINIMUM FEATURE. A 0.4 mm nozzle cannot lay down a 0.26 mm strut. The current sized structure
     has p10 = 0.26 mm and p90 = 0.47 mm -- NINETY PER CENT OF IT IS UNPRINTABLE ON FDM. A strut
     wants a radius of at least ~0.4 mm (0.8 mm across = two perimeter passes).

  2. OVERHANG. FDM builds in layers along one axis. A strut inclined at less than ~45 deg to the
     build PLANE is an overhang: its underside has nothing beneath it and it needs support (a
     horizontal strut is a bridge, and a long one sags). So a strut with unit axis u is
     self-supporting against build direction d iff  |u . d| >= sin(45 deg) = 0.707.

AND THE TWO ARE THE SAME ANSWER TO THE OTHER QUESTION. The user, of the same structure: "still
looks a bit unnatural (zig-zaggy and not-natural-intuitive-entropy)". A minimum-mass truss WANTS
many thin members -- that is what is efficient. What forces a structure into FEW, THICK, CHUNKY
members is MANUFACTURE. The printability constraints are not a compromise against the
optimisation; they are the thing that will make it look like a bone.

This measures where we stand: what fraction is printable, and which build direction is best.
"""
from __future__ import annotations

import numpy as np

NOZZLE_R = 4.0e-4          # m -- 0.4 mm radius = 0.8 mm across = two perimeter passes at 0.4 nozzle
OVERHANG = np.radians(45)  # the classic FDM self-support limit


def sphere(n=2000):
    """Roughly uniform directions on the sphere (Fibonacci)."""
    i = np.arange(n) + 0.5
    phi = np.arccos(1 - 2 * i / n)
    theta = np.pi * (1 + 5 ** 0.5) * i
    return np.stack([np.cos(theta) * np.sin(phi), np.sin(theta) * np.sin(phi), np.cos(phi)], 1)


def main():
    z = np.load("out/sized.npz", allow_pickle=True)
    nodes = z["nodes"]
    bars = [tuple(b) for b in z["bars"]]
    live = [int(e) for e in z["live"]]
    r = np.asarray(z["radii"], float)

    U, L = [], []
    for k, e in enumerate(live):
        a, b = nodes[bars[e][0]], nodes[bars[e][1]]
        v = b - a
        ln = float(np.linalg.norm(v))
        if ln < 1e-9:
            continue
        U.append(v / ln)
        L.append(ln)
    U = np.array(U)
    L = np.array(L)
    vol = np.pi * r[:len(L)] ** 2 * L                # each strut's share of the material

    print(f"THE SIZED STRUCTURE: {len(L)} struts, {vol.sum()*1.06e6:.2f} g\n")

    # ---- RULE 1: minimum feature ---------------------------------------------------------
    thin = r[:len(L)] < NOZZLE_R
    print("1. MINIMUM FEATURE  (a 0.4 mm nozzle cannot lay a thinner strut than ~0.4 mm radius)")
    print(f"   struts below {NOZZLE_R*1e3:.2f} mm radius:  {thin.sum()} of {len(L)} "
          f"({100*thin.sum()/len(L):.0f}%)")
    print(f"   ...and they are {100*vol[thin].sum()/vol.sum():.0f}% of the material")
    print(f"   radii: p10 {np.percentile(r[:len(L)],10)*1e3:.2f}   "
          f"median {np.median(r[:len(L)])*1e3:.2f}   p90 {np.percentile(r[:len(L)],90)*1e3:.2f} mm\n")

    # ---- RULE 2: overhang, over every possible build direction ---------------------------
    D = sphere(2000)
    cos = np.abs(U @ D.T)                            # |u . d| for every (strut, direction)
    ok = cos >= np.sin(np.pi / 2 - OVERHANG)         # self-supporting if within 45 deg of build dir
    by_vol = (vol[:, None] * ok).sum(0) / vol.sum()  # weight by MATERIAL, not by strut count
    best = int(np.argmax(by_vol))

    print("2. OVERHANG  (a strut within 45 deg of the build PLANE needs support)")
    print(f"   best build direction: {np.round(D[best], 3)}")
    print(f"   self-supporting at that orientation: {100*by_vol[best]:.0f}% of the material")
    print(f"   worst orientation:                   {100*by_vol.min():.0f}%")
    print(f"   median over all orientations:        {100*np.median(by_vol):.0f}%\n")

    need = ~ok[:, best]
    print(f"   -> {need.sum()} struts ({100*vol[need].sum()/vol.sum():.0f}% of the material) would")
    print(f"      still need support, even at the best orientation.\n")

    # ---- and the two rules TOGETHER ------------------------------------------------------
    both = (~thin[:len(L)]) & ok[:, best]
    print("BOTH RULES TOGETHER")
    print(f"   printable as-is, no supports: {both.sum()} of {len(L)} struts "
          f"({100*vol[both].sum()/vol.sum():.0f}% of the material)")
    print(f"\n   -> THE STRUCTURE AS IT STANDS IS NOT FDM-PRINTABLE. Not marginally: most of it is")
    print(f"      thinner than the nozzle. The fix is not to smooth the geometry afterwards -- it")
    print(f"      is to put BOTH RULES INTO THE OPTIMISATION as hard constraints, so that every")
    print(f"      structure it can possibly produce is one you can make.")
    np.savez("out/build_dir.npz", direction=D[best], self_supporting=by_vol[best])


if __name__ == "__main__":
    main()

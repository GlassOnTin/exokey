"""STAGE-1 TPU FLEXURE COUPON.  PYTHONPATH=. .venv/bin/python scripts/coupon.py

The physical test article for the read-out bench (see COUPON.md, tests T1-T3). A rigid RIM clamps a
flat MEMBRANE of thickness `t` and radius `a` -- which is exactly what `manufacture.flexure.dome`
sizes (a clamped circular diaphragm, small-deflection plate theory), so its measured force/travel is
a direct check on the k = 130 N/m prediction. A central boss seats the Ø3x1 mm magnet.

Watertight by construction (a solid of revolution), unlike the old marching-cubes coupon soup.
Exported in MILLIMETRES so a slicer sizes it correctly with no unit fiddling.

The thickness sweep is the printability question: 0.32 mm is at the FDM single-perimeter floor, so
print all three and record which lays down without gaps (COUPON.md T2).
"""
from __future__ import annotations

import numpy as np
import trimesh

MAGNET_D = 3.0            # mm, Ø of the N42 disc it seats
SEAT_CLEAR = 0.2         # mm, radial clearance so the magnet drops in


def coupon(a: float, t: float, *, r_out: float = None, rim_h: float = 3.0,
           boss_h: float = 1.0, boss_wall: float = 0.8) -> trimesh.Trimesh:
    """A flanged flat-membrane flexure coupon (all mm). `a` = clamped membrane radius, `t` = membrane
    thickness. Revolve a closed (r, z) profile about the z-axis."""
    r_out = (a + 3.0) if r_out is None else r_out
    r_bi = 0.5 * MAGNET_D + SEAT_CLEAR             # boss inner radius (magnet seat)
    r_bo = r_bi + boss_wall                        # boss outer radius
    # closed profile, counter-clockwise in (r, z); r=radius, z=height off the bed
    prof = np.array([
        [0.0,   0.0],                              # bottom centre
        [r_out, 0.0],                              # bottom outer
        [r_out, rim_h],                            # rim outer, up
        [a,     rim_h],                            # rim top, in
        [a,     t],                                # rim inner wall, down to membrane
        [r_bo,  t],                                # membrane top, in to boss
        [r_bo,  t + boss_h],                       # boss outer, up
        [r_bi,  t + boss_h],                       # boss top, in
        [r_bi,  t],                                # boss inner, down to membrane
        [0.0,   t],                                # membrane top centre
        [0.0,   0.0],                              # close
    ])
    m = trimesh.creation.revolve(prof, sections=160)
    if not m.is_watertight:
        raise RuntimeError(f"coupon a={a} t={t} not watertight")
    return m


def main():
    import os
    os.makedirs("out", exist_ok=True)
    print(f"{'coupon':28s} {'membrane':>12s} {'mass(TPU)':>10s}  watertight")
    rho_tpu = 1.21e-3                              # g/mm^3, ~1210 kg/m^3
    for t in (0.25, 0.32, 0.40):
        for a in (6.0, 7.0):
            m = coupon(a, t)
            name = f"out/coupon_t{t:.2f}_a{a:.0f}.stl"
            m.export(name)
            print(f"{os.path.basename(name):28s} {f'Ø{2*a:.0f}x{t:.2f}mm':>12s} "
                  f"{m.volume*rho_tpu:>9.2f}g  {m.is_watertight}")
    print("\nDESIGN POINT: coupon_t0.32_a6.stl (k=130 N/m predicted). Print the sweep to find which")
    print("thickness lays down cleanly on your nozzle. Seat the Ø3x1 mm N42 magnet in the centre boss.")


if __name__ == "__main__":
    main()

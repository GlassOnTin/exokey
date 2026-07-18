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


def dome(a: float, t: float, h: float, *, r_out: float = None, rim_h: float = 3.0,
         arc_n: int = 72) -> trimesh.Trimesh:
    """A shallow SPHERICAL-CAP flexure coupon (all mm). Base radius `a`, wall `t`, apex rise `h`.

    Unlike the flat membrane -- which at >1x its thickness STRETCHES and stiffens ~cubically, so a
    printed 0.32 mm/6 mm one measured 230 g at 1.5 mm vs a 20 g target -- a cap deflects by BENDING /
    roll-through, so it stays soft to full travel and can snap (the tactile click flexure.dome could
    not size). `h` sets that: shallower is softer but snaps less. Revolve a closed (r, z) shell."""
    r_out = (a + 3.0) if r_out is None else r_out
    R = (a * a + h * h) / (2.0 * h)                # sphere through base (r=a,z=rim_h) and apex (0,rim_h+h)
    zc = rim_h + h - R                             # sphere centre height
    ro = np.linspace(a, 0.0, arc_n)                # outer surface, base -> apex
    zo = zc + np.sqrt(np.maximum(R * R - ro * ro, 0.0))
    ri = np.linspace(0.0, a, arc_n)                # inner surface, apex -> base (vertical wall t)
    zi = zc + np.sqrt(np.maximum(R * R - ri * ri, 0.0)) - t
    prof = np.vstack([
        [[a, 0.0], [r_out, 0.0], [r_out, rim_h]],  # rim: inner-bottom, outer-bottom, outer-top
        np.column_stack([ro, zo]),                 # outer cap, springs from (a, rim_h) up to apex
        np.column_stack([ri, zi]),                 # inner cap, apex down to (a, rim_h - t)
        [[a, 0.0]],                                # close down the rim inner wall
    ])
    m = trimesh.creation.revolve(prof, sections=180)
    if not m.is_watertight:
        raise RuntimeError(f"dome a={a} t={t} h={h} not watertight")
    return m


def main():
    import os
    os.makedirs("out", exist_ok=True)
    rho_tpu = 1.21e-3                              # g/mm^3, ~1210 kg/m^3
    print(f"{'coupon':30s} {'geometry':>16s} {'mass(TPU)':>10s}  watertight")
    for t in (0.25, 0.32, 0.40):
        for a in (6.0, 7.0):
            m = coupon(a, t)
            name = f"out/coupon_t{t:.2f}_a{a:.0f}.stl"; m.export(name)
            print(f"{os.path.basename(name):30s} {f'flat Ø{2*a:.0f}x{t:.2f}':>16s} "
                  f"{m.volume*rho_tpu:>9.2f}g  {m.is_watertight}")
    for h in (1.0, 2.0, 3.0):                      # apex-rise sweep -- what makes it soft / snap
        m = dome(6.0, 0.32, h)
        name = f"out/coupon_dome_a6_t0.32_h{h:.0f}.stl"; m.export(name)
        print(f"{os.path.basename(name):30s} {f'dome Ø12 h{h:.0f}':>16s} "
              f"{m.volume*rho_tpu:>9.2f}g  {m.is_watertight}")
    print("\nFLAT membrane measured 230 g @ 1.5 mm (target 20 g) -- stretching, too stiff. The DOMES")
    print("(coupon_dome_a6_t0.32_h{1,2,3}.stl) roll instead of stretch. Print the h sweep and re-run T2;")
    print("softest-that-still-returns wins. ⚠ Dial TPU flow to hit t=0.32 mm first (yours came ~0.47).")


if __name__ == "__main__":
    main()

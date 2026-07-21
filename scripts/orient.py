"""BEST FDM PRINT ORIENTATION for the finished solid (out/gauntlet.stl), and the support burden
each orientation costs.  PYTHONPATH=. .venv/bin/python scripts/orient.py

`printability.py` orients the STRUT AXES at an early stage -- it answers "which build direction lets
the truss self-support" on line segments. This works on the FINISHED MESH: cups, wrist housing,
fillets, the lot. A slicer does not choose orientation for you; this does, by the measure that
actually sets support: DOWNWARD-FACING (overhang) area.

THE RULE. Build up along d. A triangle with outward normal n is an overhang needing support when it
faces down more steeply than the 45 deg self-support limit:  n . d  <  -sin(45 deg) = -0.707
(n = -d is a flat ceiling, the worst; n perpendicular to d is a vertical wall, free). Support scales
with that area (and its height off the bed). We sweep d over the sphere and minimise it.

We do NOT re-implement supports: PrusaSlicer's organic/tree supports weave through a lattice like this
far better than anything here would. This picks the orientation and quantifies what support it leaves,
so the slicer's job is small and the choice is informed. It writes the oriented STL ready to load.
"""
from __future__ import annotations

import numpy as np
import trimesh

SELF_SUPPORT = np.sin(np.radians(45))     # 0.707: overhang iff n . d < -SELF_SUPPORT
BED_FLAT = np.cos(np.radians(10))         # a face within 10 deg of the bed counts as adhesion contact


def sphere(n=1500):
    """Roughly uniform build directions on the sphere (Fibonacci)."""
    i = np.arange(n) + 0.5
    phi = np.arccos(1 - 2 * i / n)
    theta = np.pi * (1 + 5 ** 0.5) * i
    return np.stack([np.cos(theta) * np.sin(phi), np.sin(theta) * np.sin(phi), np.cos(phi)], 1)


def score(m, d):
    """For build direction d (unit, = up): overhang area (mm^2), a support-volume proxy (mm^3),
    build height (mm), and bed-contact area (mm^2)."""
    n = m.face_normals
    A = m.area_faces
    nd = n @ d
    over = nd < -SELF_SUPPORT                       # downward-facing beyond 45 deg -> needs support
    h = m.triangles_center @ d                       # height of each face along the build axis
    z0 = (m.vertices @ d).min()
    # support-volume proxy: each overhang face casts a column down to the bed ~ area*|nd| * height
    vproxy = float((A[over] * -nd[over] * (h[over] - z0)).sum())
    bed = float(A[(nd < -BED_FLAT) & (h - z0 < 0.5)].sum())   # near-bed flat-down faces = adhesion
    return dict(over_area=float(A[over].sum()), over_frac=float(A[over].sum() / A.sum()),
                vproxy=vproxy, height=float(np.ptp(np.asarray(m.vertices) @ d)), bed=bed,
                n_over=int(over.sum()))


def main(path="out/gauntlet.stl"):
    m = trimesh.load(path)
    print(f"{path}: {len(m.faces)} faces, bbox {np.round(m.extents,1)} mm, total area {m.area:.0f} mm^2\n")

    D = sphere(1500)
    rows = [(d, score(m, d)) for d in D]
    rows.sort(key=lambda r: r[1]["over_area"])       # minimise overhang area

    def show(tag, d, s):
        print(f"  {tag:16} d=[{d[0]:+.2f} {d[1]:+.2f} {d[2]:+.2f}]  "
              f"overhang {s['over_area']:6.0f} mm^2 ({100*s['over_frac']:4.1f}%)  "
              f"support~{s['vproxy']/1000:5.1f} cm^3  H {s['height']:4.0f}  bed {s['bed']:5.0f} mm^2")

    Hmax = max(s["height"] for _d, s in rows)
    low = min((r for r in rows if r[1]["height"] < 0.72 * Hmax), key=lambda r: r[1]["over_area"])

    print("MIN-SUPPORT ORIENTATION (least overhang area -- but see height/bed):")
    show("min-overhang", *rows[0])
    print("\nBEST LOW-PROFILE ORIENTATION (height < 0.72 x tallest -- more stable, shorter print):")
    show("low-profile", *low)
    print("\nfor reference:")
    show("flat +Z (as-is)", np.array([0, 0, 1.0]), score(m, np.array([0, 0, 1.0])))
    show("median", *rows[len(rows)//2])
    show("worst", *rows[-1])

    import os
    if os.path.exists("out/build_dir.npz"):
        bd = np.load("out/build_dir.npz")["direction"]
        show("strut-axis best", bd / np.linalg.norm(bd), score(m, bd / np.linalg.norm(bd)))

    print("\nTHE TRADE-OFF. This lattice has ~0 flat bed contact in ANY orientation (organic, no flat")
    print("face), so a RAFT is needed whatever you pick. Given that, the choice is overhang area")
    print("(support material + how much you pick out of the lattice) vs height (time, toppling risk).")
    print("Low-profile is the pick: near the minimum overhang AND shorter AND less support than the")
    print("tall min-overhang -- which only wins overhang AREA by ~1.6 pts, at +42 mm height. Writing")
    tall = os.environ.get("ORIENT") == "tall"
    print(f"the {'TALL MIN-OVERHANG' if tall else 'LOW-PROFILE'} one; set ORIENT=tall for the other.\n")

    # WRITE the chosen winner, rotated so its build axis is +Z, resting on the bed.
    best_d, s0 = (rows[0] if tall else low)
    zc = np.array([0, 0, 1.0])
    v = np.cross(best_d, zc)
    c = float(best_d @ zc)
    if np.linalg.norm(v) < 1e-9:
        Rm = np.eye(3) if c > 0 else np.diag([1, -1, -1.0])
    else:
        vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        Rm = np.eye(3) + vx + vx @ vx * (1 / (1 + c))
    mo = m.copy()
    T = np.eye(4); T[:3, :3] = Rm
    mo.apply_transform(T)
    mo.apply_translation([0, 0, -(mo.vertices[:, 2].min())])   # drop onto the bed
    out = "out/gauntlet_oriented.stl"
    mo.export(out)
    print(f"\n  wrote {out}: build axis is +Z, {100*s0['over_frac']:.1f}% overhang area "
          f"(~{s0['vproxy']/1000:.1f} cm^3 support), {mo.extents[2]:.0f} mm tall.")
    print("  -> load into PrusaSlicer with ORGANIC (tree) supports, overhang threshold 45 deg,")
    print("     'support on build plate only' OFF (the lattice needs interior columns).")


if __name__ == "__main__":
    main()

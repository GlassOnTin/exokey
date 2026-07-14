"""THE GAUNTLET AS A PRINTABLE SOLID.  PYTHONPATH=. .venv/bin/python scripts/export_stl.py

Skeleton -> signed distance field -> smooth minimum -> marching cubes -> STL. No booleans.
The fillets at every junction are not modelled; they FALL OUT of the smooth-min, which is also
why the result looks like bone rather than like plumbing.
"""
from __future__ import annotations

import pickle

import numpy as np

from design.vector import posture, tm_of, tp_of
from hand.flesh import skin
from hand.myohand import FINGERS
from manufacture.mesh import BLEND, VOXEL, field, to_mesh, well_boxes
from opt.problem import hands
from structure.frame import MATERIALS
from structure.lattice import BAR_R


def main():
    h = hands()[50]
    fd = pickle.load(open("out/final_design.pkl", "rb"))
    x = fd["x"]
    q = h.compose({f: posture(h, f, tp_of(x, f), tm_of(x, f), float(x.get(f"ab_{f}", 0.0)))
                   for f in FINGERS})

    import os

    src = next(p for p in ("out/smooth.npz", "out/printable.npz", "out/sized.npz",
                          "out/final.npz") if os.path.exists(p))
    z = np.load(src, allow_pickle=True)
    nodes, bars = z["nodes"], [tuple(b) for b in z["bars"]]
    live = [int(e) for e in z["live"]]
    # PER-STRUT RADII, not one radius for all of them. A gradient-sized structure has a thick trunk
    # tapering into thin braces; melting them to a single rod prints a DIFFERENT DEVICE from the one
    # that was analysed -- and throws away the very hierarchy that makes it look like bone.
    r = z["radii"] if "radii" in z.files else float(BAR_R)

    struts = [(nodes[bars[e][0]], nodes[bars[e][1]]) for e in live]
    boxes = well_boxes(h, q, FINGERS)
    rr = np.atleast_1d(np.asarray(r, float))
    print(f"  {src}: {len(struts)} struts + {len(boxes)} well plates")
    print(f"  rod r = {rr.min()*1000:.2f}-{rr.max()*1000:.2f} mm, fillet = {BLEND*1000:.1f} mm, "
          f"voxel = {VOXEL*1000:.1f} mm")

    f, o, v = field(struts, boxes, r)
    print(f"  field {f.shape} = {f.size/1e6:.1f} M voxels")
    m = to_mesh(f, o, v)

    rho = MATERIALS["cf_pa12"]["rho"]
    print(f"\nTHE SOLID")
    print(f"  {len(m.vertices)} vertices, {len(m.faces)} faces")
    print(f"  watertight       {m.is_watertight}")
    print(f"  winding correct  {m.is_winding_consistent}")
    print(f"  volume           {m.volume*1e6:.2f} cm^3")
    print(f"  MASS (CF-PA12)   {m.volume*rho*1000:.1f} g")
    print(f"  bbox             {' x '.join(f'{d*1000:.0f}' for d in m.extents)} mm")

    # ⚠ THE BEAM MODEL SAID 5.1 g. IT IS NOT THE SAME NUMBER AND IT SHOULD NOT BE:
    # the beam model is a wire diagram -- it counts A*L per strut and knows nothing about the
    # MATERIAL AT THE JOINTS, where three or four rods meet and their volumes overlap. It
    # double-counts the overlaps (making it heavy) and it misses the fillets (making it light).
    bone = float(z["bone_g"]) if "bone_g" in z.files else float(z["mass"])
    print(f"\n  the beam model said {bone:.1f} g. The solid is {m.volume*rho*1000:.1f} g "
          f"({100*(m.volume*rho*1000/bone - 1):+.0f}%).")
    print(f"  A wire diagram counts A*L per strut. It DOUBLE-COUNTS the volume where rods overlap")
    print(f"  at a joint and MISSES the fillets that make the joint printable. Neither error is")
    print(f"  small when a third of the struts meet at a node, and only the solid is the truth.")

    # DOES THE PRINTED PART CLEAR THE HAND? The beam model checked line segments. A solid has
    # THICKNESS and FILLETS, and it is the solid you would be wearing.
    #
    # ⚠ BUT THE WELLS ARE SUPPOSED TO TOUCH. A well is a CUP THE FINGERTIP SITS IN -- its floor is
    # what `click` presses against. Measuring the whole part against the skin and calling the
    # smallest number a "rub" flags the one component whose entire job is CONTACT. So the
    # structure and the cups are measured separately, which is the only way either number means
    # anything.
    from scipy.spatial import cKDTree
    V, _ = skin(h, q)
    tree = cKDTree(V)

    fs, os_, vs = field(struts, [], r)              # the STRUCTURE alone -- no cups
    ms = to_mesh(fs, os_, vs)
    ds = tree.query(ms.vertices)[0]
    print(f"\n  clearance of the STRUCTURE (no cups) from the skin: "
          f"min {ds.min()*1000:.2f} mm")
    d = tree.query(m.vertices)[0]
    print(f"  closest approach of the WHOLE part (cups included):  {d.min()*1000:.2f} mm  "
          f"-- that is a CUP, and a cup is meant to touch")
    if ds.min() < 0.002:
        print("  ⚠ THE STRUCTURE is under 2 mm off the skin. The fillets have eaten the standoff.")
    else:
        print(f"  the structure clears. `hug` is now a clearance of the PART "
              f"(centreline + rod + fillet), not of its centreline.")

    m.export("out/gauntlet.stl")
    print("\n  wrote out/gauntlet.stl")


if __name__ == "__main__":
    main()

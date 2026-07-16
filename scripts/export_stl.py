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
from manufacture import wellmod as wm
from manufacture.mesh import BLEND, VOXEL, carve, field, to_mesh
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

    src = next(p for p in ("out/bone.npz", "out/smooth.npz", "out/printable.npz", "out/sized.npz",
                          "out/final.npz") if os.path.exists(p))
    z = np.load(src, allow_pickle=True)
    nodes, bars = z["nodes"], [tuple(b) for b in z["bars"]]
    live = [int(e) for e in z["live"]]
    # PER-STRUT RADII, not one radius for all of them. A gradient-sized structure has a thick trunk
    # tapering into thin braces; melting them to a single rod prints a DIFFERENT DEVICE from the one
    # that was analysed -- and throws away the very hierarchy that makes it look like bone.
    r = z["radii"] if "radii" in z.files else float(BAR_R)

    struts = [(nodes[bars[e][0]], nodes[bars[e][1]]) for e in live]
    rr = np.atleast_1d(np.asarray(r, float))
    seg_r = {}                                          # node-pair -> its strut radius (for grooves)
    for k, e in enumerate(live):
        i, j = bars[e]
        seg_r[frozenset((i, j))] = float(rr[k]) if rr.size > 1 else float(rr[0])

    # THE SENSOR MODULES replace the bare well cups: a rigid PA frame per finger (Hall seat, collar,
    # base) that blends into the struts, plus the wire grooves back to a wrist MCU housing.
    anchors = [int(a) for a in z["anchors"]]
    btn = {f: int(i) for f, i in zip(z["fingers"], z["buttons"])}
    mods = [wm.module_frame(h, q, f, mount=nodes[btn[f]]) for f in FINGERS]
    mboxes = [b for md in mods for b in md["boxes"]]
    mcaps = [c[0] for md in mods for c in md["caps"]]
    mcap_r = [c[1] for md in mods for c in md["caps"]]
    mcyls = [c for md in mods for c in md["cyls"]]
    cv_cyls = [c for md in mods for c in md["carve_cyls"]]
    cv_boxes = [c for md in mods for c in md["carve_boxes"]]

    from scipy.spatial import cKDTree
    V, _ = skin(h, q)
    stree = cKDTree(V)

    # HOUSING for the XIAO nRF52840 + LiPo at the wrist anchor cluster -- lifted OUT along the skin
    # normal there so the box sits proud of the hand instead of cutting into the wrist.
    anchor_c = nodes[np.array(anchors)].mean(axis=0)
    outward = anchor_c - V[stree.query(anchor_c)[1]]    # skin -> cluster: points dorsally, outward
    hboxes, hcaps, hcav = wm.housing(nodes[np.array(anchors)], outward)
    # tie the box to the nearest LIVE-strut nodes (guaranteed structural; anchor nodes may carry only
    # a spring, not a bar) so it cannot detach.
    live_nodes = np.array(sorted({i for e in live for i in bars[e]}))
    box_c = np.asarray(hboxes[0][0], float)
    near = live_nodes[np.argsort(np.linalg.norm(nodes[live_nodes] - box_c, axis=1))[:3]]
    hcaps = hcaps + [((nodes[int(i)], box_c), wm.STALK_R) for i in near]

    # WIRE GROOVES: sink a re-entrant capsule channel into each strut the route follows, on the
    # dorsal (away-from-skin) surface so a wire never presses the hand.
    routes = wm.harness_grooves(nodes, bars, live, btn, anchors)
    gcyls = []
    for route in routes:
        for i, j in zip(route[:-1], route[1:]):
            rs = seg_r.get(frozenset((i, j)), float(BAR_R))
            off = max(rs - wm.GROOVE_BURY, 0.0)         # sink the channel to breach the surface
            pa, pb = [], []
            for nidx, store in ((i, pa), (j, pb)):
                p = nodes[nidx]
                away = p - V[stree.query(p)[1]]
                away = away / (np.linalg.norm(away) + 1e-12)
                store.append(p + off * away)
            gcyls.append((pa[0], pb[0], wm.GROOVE_R))

    allstruts = struts + mcaps + [c[0] for c in hcaps]
    allr = list(rr) if rr.size > 1 else [float(rr[0])] * len(struts)
    allr = allr + list(mcap_r) + [c[1] for c in hcaps]
    print(f"  {src}: {len(struts)} struts + {len(mods)} sensor modules + housing")
    print(f"  wire grooves: {len(routes)} routes, {len(gcyls)} channel segments")
    print(f"  rod r = {rr.min()*1000:.2f}-{rr.max()*1000:.2f} mm, fillet = {BLEND*1000:.1f} mm, "
          f"voxel = {VOXEL*1000:.1f} mm")

    f, o, v = field(allstruts, mboxes + hboxes, allr, cyls=mcyls)
    print(f"  field {f.shape} = {f.size/1e6:.1f} M voxels")
    carve(f, o, v, cyls=cv_cyls + gcyls, boxes=cv_boxes + hcav)
    m = to_mesh(f, o, v)

    # DROP DEBRIS: the smooth-min + carves leave a scatter of near-zero-volume shells (slivers where
    # a groove grazes a surface). Keep only bodies that carry real material -- a printable part is one
    # solid, not one solid plus confetti a slicer would choke on.
    import trimesh
    bodies = m.split(only_watertight=False)
    if len(bodies) > 1:
        keep = [b for b in bodies if b.volume > 1e-9]      # > 1 mm^3
        dropped = len(bodies) - len(keep)
        m = trimesh.util.concatenate(keep) if len(keep) > 1 else keep[0]
        print(f"  dropped {dropped} debris shells (<1 mm^3); kept {len(keep)} real bod[y/ies]")

    rho = MATERIALS["cf_pa12"]["rho"]
    print(f"\nTHE SOLID")
    print(f"  {len(m.vertices)} vertices, {len(m.faces)} faces")
    print(f"  watertight       {m.is_watertight}")
    print(f"  winding correct  {m.is_winding_consistent}")
    print(f"  components       {m.body_count}  (1 = the housing + modules all attach)")
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

    fs, os_, vs = field(struts, [], r)              # the STRUCTURE alone -- no cups, no modules
    ms = to_mesh(fs, os_, vs)
    dg = (m.volume - ms.volume) * rho * 1000
    print(f"\n  the SENSOR MODULES + housing add {dg:.1f} g over the bare struts "
          f"({ms.volume*rho*1000:.1f} g -> {m.volume*rho*1000:.1f} g solid) -- MEASURED, not estimated.")
    ds = tree.query(ms.vertices)[0]
    print(f"  clearance of the STRUCTURE (no cups) from the skin: "
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

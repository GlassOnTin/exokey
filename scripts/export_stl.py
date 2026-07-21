"""THE GAUNTLET AS A PRINTABLE SOLID.  PYTHONPATH=. .venv/bin/python scripts/export_stl.py

Skeleton -> signed distance field -> smooth minimum -> marching cubes -> STL. No booleans.
The fillets at every junction are not modelled; they FALL OUT of the smooth-min, which is also
why the result looks like bone rather than like plumbing.
"""
from __future__ import annotations

import argparse
import pickle

import numpy as np

from design.vector import posture, tm_of, tp_of
from hand.flesh import skin
from hand.myohand import FINGERS, MyoHand
from hand.scaling import ANSUR_HAND_LENGTH_MM, REFERENCE_PERCENTILE
from manufacture import mount as mnt
from manufacture.mesh import BLEND, VOXEL, carve, field, to_mesh
from structure.frame import MATERIALS
from structure.lattice import BAR_R

REF_MM = ANSUR_HAND_LENGTH_MM[REFERENCE_PERCENTILE]   # 185 mm = the median hand the model IS


def main(hand_mm=REF_MM, out_path="out/gauntlet.stl"):
    # PER-USER FIT (partial, first-order). The hand AND the frame nodes below are both scaled by
    # s, so cups, sensor seats and skeleton stay aligned -- a uniformly-scaled median device
    # (Buchholz to first order). What does NOT re-fit: the optimised TOPOLOGY (which struts exist
    # and how they connect), the rod radii (the 1.5 mm floor is a human tissue constant, not a hand
    # dimension), and the component pockets (real part sizes). A fully re-optimised skeleton for one
    # hand size means re-running the optimiser at that scale.
    s = hand_mm / REF_MM
    h = MyoHand(scale=s)
    if abs(s - 1.0) > 1e-9:
        print(f"HAND FIT: {hand_mm:.0f} mm hand -> scale {s:.3f} (median is {REF_MM:.0f} mm). "
              f"Cups/seats re-fitted; frame topology stays population-median.")
    fd = pickle.load(open("out/final_design.pkl", "rb"))
    x = fd["x"]
    q = h.compose({f: posture(h, f, tp_of(x, f), tm_of(x, f), float(x.get(f"ab_{f}", 0.0)))
                   for f in FINGERS})

    import os

    src = next(p for p in ("out/bone.npz", "out/smooth.npz", "out/printable.npz", "out/sized.npz",
                          "out/final.npz") if os.path.exists(p))
    z = np.load(src, allow_pickle=True)
    # Scale the FRAME with the hand. MuJoCo scales body positions about the model origin, so a
    # scaled hand translates ~s*|origin->hand| away; scaling the node coordinates by the same s
    # about the same origin keeps frame, buttons, anchors and cups all tracking together (a
    # uniformly-scaled median device -- Buchholz to first order). Rod RADII are NOT scaled: the
    # 1.50 mm floor is a human tissue constant, not a hand dimension. No-op at s == 1.
    nodes, bars = z["nodes"] * s, [tuple(b) for b in z["bars"]]
    live = [int(e) for e in z["live"]]
    # PER-STRUT RADII, not one radius for all of them. A gradient-sized structure has a thick trunk
    # tapering into thin braces; melting them to a single rod prints a DIFFERENT DEVICE from the one
    # that was analysed -- and throws away the very hierarchy that makes it look like bone.
    r = z["radii"] if "radii" in z.files else float(BAR_R)

    struts = [(nodes[bars[e][0]], nodes[bars[e][1]]) for e in live]
    rr = np.atleast_1d(np.asarray(r, float))
    btn = {f: int(i) for f, i in zip(z["fingers"], z["buttons"])}

    # THE SENSOR MOUNTS, rebuilt ENTRY-FIRST (manufacture.mount): the thumb an independent well, the
    # four long fingers a shared cluster. Every finger's slide-in route is kept open by construction
    # (manufacture.entry / tests/test_mount.py) -- the failure that withdrew the previous geometry.
    LONG = ["index", "middle", "ring", "little"]
    mods = [mnt.well_mount(h, q, "thumb", nodes[btn["thumb"]]),
            mnt.cluster_mount(h, q, LONG, {f: nodes[btn[f]] for f in LONG})]
    mboxes = [b for md in mods for b in md["boxes"]]
    mcaps = [c[0] for md in mods for c in md["caps"]]
    mcap_r = [c[1] for md in mods for c in md["caps"]]
    mcyls = [c for md in mods for c in md["cyls"]]
    cv_cyls = [c for md in mods for c in md["carve_cyls"]]
    cv_boxes = [c for md in mods for c in md["carve_boxes"]]

    # WRIST MCU HOUSING + WIRE ROUTING -- far from the fingertips (they do not touch the entry route),
    # but the housing must clear the wrist and the wires sink into the dorsal strut surfaces.
    from scipy.spatial import cKDTree
    Vsk, _ = skin(h, q)
    stree = cKDTree(Vsk)
    anchors = [int(a) for a in z["anchors"]]
    live_nodes = np.array(sorted({i for e in live for i in bars[e]}))
    anchor_c = nodes[np.array(anchors)].mean(axis=0)
    outward = anchor_c - Vsk[stree.query(anchor_c)[1]]
    hboxes, hcaps, hcav = mnt.housing(nodes[np.array(anchors)], outward, nodes[live_nodes])
    seg_r = {frozenset(bars[e]): (float(rr[k]) if rr.size > 1 else float(rr[0]))
             for k, e in enumerate(live)}
    gcyls = []                                       # MINIMAL-COPPER shared bus (§8.15l qqq-2), not
    bus = mnt.harness_bus(nodes, bars, live, btn, anchors)   # five point-to-point runs
    bus_len = sum(float(np.linalg.norm(nodes[i] - nodes[j])) for i, j, _ in bus)
    for i, j, nw in bus:
        rs = seg_r.get(frozenset((i, j)), float(BAR_R))
        off = max(rs - 0.0004, 0.0)                  # sink the channel to breach the surface
        pa = nodes[i] + off * ((nodes[i] - Vsk[stree.query(nodes[i])[1]]) /
                               (np.linalg.norm(nodes[i] - Vsk[stree.query(nodes[i])[1]]) + 1e-9))
        pb = nodes[j] + off * ((nodes[j] - Vsk[stree.query(nodes[j])[1]]) /
                               (np.linalg.norm(nodes[j] - Vsk[stree.query(nodes[j])[1]]) + 1e-9))
        gcyls.append((pa, pb, 0.0004 + 0.00007 * nw))   # groove widens with conductor count (2->0.54, 6->0.82 mm)
    mboxes += hboxes
    mcaps += [c[0] for c in hcaps]
    mcap_r += [c[1] for c in hcaps]
    cv_cyls += gcyls
    cv_boxes += hcav

    allstruts = struts + mcaps
    allr = (list(rr) if rr.size > 1 else [float(rr[0])] * len(struts)) + list(mcap_r)
    print(f"  {src}: {len(struts)} struts + thumb well + long-finger cluster + housing + "
          f"{len(gcyls)} bus-groove segs ({bus_len*1e3:.0f} mm shared harness)")
    print(f"  rod r = {rr.min()*1000:.2f}-{rr.max()*1000:.2f} mm, fillet = {BLEND*1000:.1f} mm, "
          f"voxel = {VOXEL*1000:.1f} mm")

    f, o, v = field(allstruts, mboxes, allr, cyls=mcyls)
    print(f"  field {f.shape} = {f.size/1e6:.1f} M voxels")
    carve(f, o, v, cyls=cv_cyls, boxes=cv_boxes)
    m = to_mesh(f, o, v)
    import trimesh                                    # drop sub-mm^3 marching-cubes debris shells
    bodies = m.split(only_watertight=False)
    if len(bodies) > 1:
        keep = [b for b in bodies if b.volume > 1e-9]
        m = trimesh.util.concatenate(keep) if len(keep) > 1 else keep[0]
        print(f"  dropped {len(bodies) - len(keep)} debris shells; kept {len(keep)} real")
    print(f"  components       {getattr(m, 'body_count', 1)}")

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

    # EXPORT IN MILLIMETRES. The mesh is built in SI (metres); slicers assume mm, so a raw export
    # imports as a sub-mm speck. Scale here -- AFTER every metre-based clearance check above.
    m.apply_scale(1000.0)
    m.export(out_path)
    print(f"\n  wrote {out_path}  ({' x '.join(f'{d:.0f}' for d in m.extents)} mm)")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(
        description="Mesh the gauntlet to a printable STL. Defaults to the median (185 mm) hand.",
        epilog="Per-user fit is PARTIAL: --hand-mm re-fits the finger cups and sensor seats to "
               "your hand, but the frame topology stays the population-optimised median (a full "
               "re-fit means re-running the optimiser at your scale). Component pockets (Hall, "
               "magnet, XIAO) stay at true size and are NOT scaled.")
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--hand-mm", type=float, metavar="MM",
                   help="Your measured hand length (wrist crease to middle-fingertip), in mm. "
                        f"Median is {REF_MM:.0f}; the model covers ~165 (5th) to 205 (95th).")
    g.add_argument("--percentile", type=int, choices=sorted(ANSUR_HAND_LENGTH_MM),
                   help="Convenience: pick a hand size by ANSUR II percentile instead of mm.")
    ap.add_argument("--out", metavar="PATH", default=None,
                    help="Output STL path (default out/gauntlet.stl, or out/gauntlet_<mm>mm.stl "
                         "for a non-median hand).")
    a = ap.parse_args()

    hand_mm = a.hand_mm if a.hand_mm is not None else \
        (ANSUR_HAND_LENGTH_MM[a.percentile] if a.percentile is not None else REF_MM)
    out_path = a.out or ("out/gauntlet.stl" if abs(hand_mm - REF_MM) < 1e-9
                         else f"out/gauntlet_{hand_mm:.0f}mm.stl")
    main(hand_mm=hand_mm, out_path=out_path)

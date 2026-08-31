"""THE CARRIER GAUNTLET AS A PRINTABLE SOLID.

    PYTHONPATH=. .venv/bin/python scripts/export_carrier_stl.py

export_stl.py meshes the RETIRED self-built device: grown struts plus five printed wells
(mount.well_mount / cluster_mount). The 2026-08-29 reframe retires those wells -- the kit
carries its own keywells, so the printed part is the grown shell plus the DECK PLATE the kit
bolts to. The keywells are LOADS on the structure, not geometry to print.

So this meshes: struts (from out/final_carrier.npz, which already includes the protected
deck-plate bars) blended with a solid deck slab -- the plate the kit's tower brackets bolt
down onto. Same skeleton -> signed-distance -> smooth-min -> marching-cubes pipeline as
export_stl.py; the fillets fall out of the smooth-min exactly as they do there.

The slab thickness is a bracket choice (our part, not a kit dimension): 2 mm is a stiff
plate for CF-PA12 at this span and leaves the bolt heads room. It is the same class of
disclosed GUESS as CARRIER_STANDOFF.
"""
from __future__ import annotations

import argparse

import numpy as np

from design.vector import posture, tm_of, tp_of
from hand.flesh import skin
from hand.myohand import FINGERS, MyoHand
from hand.scaling import ANSUR_HAND_LENGTH_MM, REFERENCE_PERCENTILE
from manufacture.mesh import BLEND, VOXEL, carve, field, to_mesh
from structure.frame import MATERIALS, hand_axes
from structure.lattice import BAR_R

REF_MM = ANSUR_HAND_LENGTH_MM[REFERENCE_PERCENTILE]


def deck_slab(deck, o, e_r, e_d, e_o, t=0.002):
    """The mount plate as a solid box: spans the deck node cloud in the (radial, distal)
    plane, `t` thick along the dorsal normal. Returns (center, R, half_extents) for mesh.field."""
    deck = np.asarray(deck, float)
    c = deck.mean(0)
    span_r = (deck - c) @ e_r
    span_d = (deck - c) @ e_d
    hr = 0.5 * (span_r.max() - span_r.min()) + 0.002      # 2 mm margin past the outer nodes
    hd = 0.5 * (span_d.max() - span_d.min()) + 0.002
    R = np.stack([e_r, e_d, e_o])                          # rows = box axes
    h = np.array([hr, hd, 0.5 * t])
    return (c, R, h)


def main(hand_mm=REF_MM, out_path="out/gauntlet_carrier.stl"):
    s = hand_mm / REF_MM
    h = MyoHand(scale=s)
    if abs(s - 1.0) > 1e-9:
        print(f"HAND FIT: {hand_mm:.0f} mm hand -> scale {s:.3f} (median {REF_MM:.0f} mm).")

    z = np.load("out/final_carrier.npz", allow_pickle=True)
    nodes, bars = z["nodes"] * s, [tuple(b) for b in z["bars"]]
    live = [int(e) for e in z["live"]]
    r = z["radii"] if "radii" in z.files else float(BAR_R)
    struts = [(nodes[bars[e][0]], nodes[bars[e][1]]) for e in live]
    rr = np.atleast_1d(np.asarray(r, float))

    # THE DECK PLATE. The kit bolts to this; it is the one printed surface whose job is the
    # kit's interface, so it is a solid slab, not optimisable struts.
    q = np.zeros(h.model.nq)
    o, e_d, e_r, e_o = hand_axes(h, q)
    slab = deck_slab(z["deck"] * s, o * s, e_r, e_d, e_o, t=0.002)

    allstruts = struts
    allr = list(rr) if rr.size > 1 else [float(rr[0])] * len(struts)
    print(f"  {len(struts)} struts (incl. deck-plate bars) + deck slab "
          f"{slab[2][0]*2e3:.0f} x {slab[2][1]*2e3:.0f} x {slab[2][2]*2e3:.0f} mm")

    f, os_, v = field(allstruts, [slab], allr)
    print(f"  field {f.shape} = {f.size/1e6:.1f} M voxels")
    m = to_mesh(f, os_, v)
    import trimesh
    bodies = m.split(only_watertight=False)
    if len(bodies) > 1:
        keep = [b for b in bodies if b.volume > 1e-9]
        m = trimesh.util.concatenate(keep) if len(keep) > 1 else keep[0]
        print(f"  dropped {len(bodies)-len(keep)} debris shells; kept {len(keep)} real")

    rho = MATERIALS["cf_pa12"]["rho"]
    print(f"\nTHE SOLID")
    print(f"  {len(m.vertices)} vertices, {len(m.faces)} faces")
    print(f"  watertight       {m.is_watertight}")
    print(f"  winding correct  {m.is_winding_consistent}")
    print(f"  MASS (CF-PA12)   {m.volume*rho*1000:.1f} g")
    print(f"  bbox             {' x '.join(f'{d*1000:.0f}' for d in m.extents)} mm")

    # DOES THE PRINTED PART CLEAR THE HAND? The deck is MEANT to stand off the dorsum (the kit
    # bolts to it); the struts must clear the skin. Measure the structure alone, as export_stl does.
    from scipy.spatial import cKDTree
    V, _ = skin(h, q)
    tree = cKDTree(V)
    fs, os2, vs = field(struts, [], r)
    ms = to_mesh(fs, os2, vs)
    ds = tree.query(ms.vertices)[0]
    print(f"\n  clearance of the STRUCTURE (no deck) from the skin: min {ds.min()*1000:.2f} mm")
    if ds.min() < 0.002:
        print("  ⚠ THE STRUCTURE is under 2 mm off the skin. The fillets have eaten the standoff.")

    m.apply_scale(1000.0)
    m.export(out_path)
    print(f"\n  wrote {out_path}  ({' x '.join(f'{d:.0f}' for d in m.extents)} mm)")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Mesh the carrier gauntlet to a printable STL.")
    ap.add_argument("--hand-mm", type=float, default=REF_MM)
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    out_path = a.out or ("out/gauntlet_carrier.stl" if abs(a.hand_mm - REF_MM) < 1e-9
                         else f"out/gauntlet_carrier_{a.hand_mm:.0f}mm.stl")
    main(hand_mm=a.hand_mm, out_path=out_path)

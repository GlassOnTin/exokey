"""FROM THE SKELETON TO A PRINTABLE SOLID.

THE USER: "What would be a good way to get from the skeletal structure to an STL we could 3d
print?"

The obvious route -- boolean-union 316 cylinders -- is the wrong one, and not only because CSG on
that many primitives is slow and fragile. It gives you A PILE OF PIPES: every joint is a knife
edge where three or four tubes intersect at an angle, and a knife edge is (a) a stress
concentration, which is exactly where a 1.8 mm CF-PA12 rod will snap, and (b) a nightmare for any
slicer. Real bone has no knife edges. Every junction is FILLETED.

So: build a SIGNED DISTANCE FIELD and take a SMOOTH minimum.

    f(x) = -k * log( SUM_i exp( -d_i(x) / k ) )

where d_i is the exact distance from x to strut i's surface (a capsule: distance-to-segment minus
the rod radius). As k -> 0 this IS the union. At k ~ 1 mm the struts BLEND, and the blend radius is
the fillet -- so the fillets are not modelled, they FALL OUT of the formulation. Marching cubes on
f = 0 gives a watertight manifold in one step, with no booleans anywhere.

That is also why it looks like bone: a metaball blend and a trabecular junction are the same shape
for the same reason -- material accumulating where stress flows round a corner.

THE WELLS ARE PART OF THE PART. A cup is a U-channel (floor + two walls, open proximally so the
phalanx slides in), so it goes into the SAME field as three boxes and blends into the struts that
hold it. A gauntlet that is printed in one piece has to be MODELLED in one piece.
"""
from __future__ import annotations

import numpy as np

BLEND = 0.0006      # m. The fillet radius at every junction. See INFLATION below.
VOXEL = 0.0005      # m. 0.5 mm: fine enough to resolve a 1.8 mm rod (3-4 voxels across).


def _seg_dist(P, a, b):
    """Exact distance from each point in P to the segment a-b. Vectorised."""
    ab = b - a
    L2 = float(ab @ ab)
    if L2 < 1e-18:
        return np.linalg.norm(P - a, axis=-1)
    t = np.clip(((P - a) @ ab) / L2, 0.0, 1.0)
    return np.linalg.norm(P - (a + t[..., None] * ab), axis=-1)


def _box_sdf(P, c, R, h):
    """SDF of a box: centre c, orthonormal rows R (its axes), half-extents h."""
    q = np.abs((P - c) @ R.T) - h
    outside = np.linalg.norm(np.maximum(q, 0.0), axis=-1)
    inside = np.minimum(np.max(q, axis=-1), 0.0)
    return outside + inside


def field(struts, boxes, r, voxel=VOXEL, blend=BLEND, pad=0.006):
    """The smooth-min SDF of the whole part, on a grid. Returns (f, origin, voxel).

    Accumulates exp(-d/k) PER PRIMITIVE, and only in that primitive's own neighbourhood -- a strut
    10 cm away contributes exp(-100) and is not worth 10 million distance evaluations to discover.

    ⚠ THE SMOOTH-MIN INFLATES, AND YOU MUST KNOW BY HOW MUCH. Where N primitives are equidistant,
    -k*log(N*exp(-d/k)) = d - k*log(N): the surface moves OUT by k*log(N). At a junction of four
    struts with k = 1.2 mm that is 1.7 mm of extra radius on a 0.9 mm rod -- and the printed part
    came out at 23.8 g against a 5.1 g structural model, i.e. NOT THE STRUCTURE THAT WAS ANALYSED.
    A mesher that quietly fattens the part is a mesher that prints a different device.

    Two consequences, both handled: k is small (0.6 mm, still a real fillet), and the caller
    MEASURES the solid's mass and clearance rather than trusting the wire diagram.
    """
    pts = np.array([p for s in struts for p in s]
                   + [c for c, _R, _h in boxes] or [[0, 0, 0]])
    lo = pts.min(axis=0) - (r + pad)
    hi = pts.max(axis=0) + (r + pad)
    n = np.ceil((hi - lo) / voxel).astype(int) + 1
    acc = np.zeros(tuple(n), np.float64)

    def window(pmin, pmax, reach):
        i0 = np.maximum(np.floor((pmin - reach - lo) / voxel).astype(int), 0)
        i1 = np.minimum(np.ceil((pmax + reach - lo) / voxel).astype(int) + 1, n)
        if np.any(i1 <= i0):
            return None
        g = np.meshgrid(*[np.arange(i0[d], i1[d]) for d in range(3)], indexing="ij")
        P = lo + np.stack(g, axis=-1) * voxel
        return (slice(i0[0], i1[0]), slice(i0[1], i1[1]), slice(i0[2], i1[2])), P

    reach = 8.0 * blend + r        # beyond this, exp(-d/k) is < 1e-3 of the peak: nothing
    for a, b in struts:
        w = window(np.minimum(a, b), np.maximum(a, b), reach)
        if w is None:
            continue
        sl, P = w
        acc[sl] += np.exp(-(_seg_dist(P, a, b) - r) / blend)

    for c, R, h in boxes:
        ext = np.abs(R).T @ h
        w = window(c - ext, c + ext, reach)
        if w is None:
            continue
        sl, P = w
        acc[sl] += np.exp(-_box_sdf(P, c, R, h) / blend)

    f = np.where(acc > 1e-300, -blend * np.log(np.maximum(acc, 1e-300)), 10.0 * blend)
    # ⚠ THE FIELD MUST BE POSITIVE ON EVERY FACE OF THE GRID, or the surface runs off the edge and
    # marching cubes returns an OPEN mesh -- not watertight, not printable, and it says so quietly.
    f = np.pad(f, 1, constant_values=10.0 * blend)
    return f, lo - voxel, voxel


def to_mesh(f, origin, voxel):
    """Marching cubes on f = 0. One step, no booleans, watertight by construction."""
    import trimesh
    from skimage import measure

    v, faces, _n, _val = measure.marching_cubes(f, level=0.0, spacing=(voxel,) * 3)
    return trimesh.Trimesh(vertices=v + origin, faces=faces, process=True)


def well_boxes(h, q, fingers, wall=0.0015):
    """Each well as the U-CHANNEL it is: a floor and two walls. Open proximally and dorsally --
    the distal phalanx SLIDES IN along its own axis; it is not lowered onto a disc like a piston.
    """
    boxes = []
    for f in fingers:
        wf = h.well_frame(q, f)
        p = np.asarray(wf["pos"], float)
        ax = np.asarray(wf["axis"], float)          # along the bone (distal)
        fl = np.asarray(wf["floor"], float)         # palmar: what `click` presses into
        lat = np.asarray(wf["lateral"], float)
        r = float(wf["radius"])
        half = float(wf["half"])
        R = np.vstack([ax, fl, lat])                # rows = the box's own axes

        c = p - 0.5 * half * ax                     # the channel is centred behind the pad
        boxes.append((c + (r + 0.5 * wall) * fl, R,
                      np.array([half, 0.5 * wall, r + wall])))          # FLOOR
        for s in (+1.0, -1.0):
            boxes.append((c + s * (r + 0.5 * wall) * lat, R,
                          np.array([half, 0.5 * r, 0.5 * wall])))       # WALLS
    return boxes

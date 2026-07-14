"""IS IT FRIENDLY, OR IS IT A KNUCKLE-DUSTER?

THE USER, giving the reason I had been failing to find:

    "Sharp corners are not ergonomic, meaning they hurt flesh when forced against a hand, or scratch
     skin even by accident. From a product perspective this abstracts to a keyboard needing to be
     'friendly' and not weaponry like a knuckle duster."

THAT IS THE REQUIREMENT, AND IT IS NOT A STRUCTURAL ONE. I tried twice to justify smoothing the
structure on structural grounds, and both attempts failed their own measurement:

    CLEARANCE -- "a straight chord dips toward the flesh"     -> it does, but it does not BIND
                                                                 (0 of 669 members at the floor)
    FATIGUE   -- "a kink is a stress riser, it will crack"    -> peak stress is 10% of yield. The
                                                                 structure is STIFFNESS-limited.

Both were me looking for a reason in the CENTRELINE. The requirement lives on the SURFACE -- the part
that touches a hand -- and it is a HUMAN-FACTORS constraint, which is why no amount of FEM was ever
going to produce it.

WHAT THAT MEANS FOR THE GEOMETRY. Two distinct offences, and the spline only fixes one of them:

  1. A CONVEX RIDGE at a kink. Where two rods meet at an angle, the OUTSIDE of the bend is a convex
     edge. The smooth-min SDF blends it, but only over the blend radius -- so the sharper the kink,
     the tighter the ridge. Straightening the load path is what opens that radius out.

  2. A SPIKE at a free end. A member with a loose end is a rod sticking into space, capped at its own
     radius -- and the nozzle floor is 0.4 mm, so that cap is a 0.4 mm point. It will scratch, and
     NO amount of centreline smoothing removes it. It has to be capped, or the member has to go.

So this measures the PART, not the model: the sharpest convex feature anywhere on the printed
surface, and every place the structure comes to a point.
"""
from __future__ import annotations

import numpy as np

from design.params import P, Source

SKIN_R = P("SKIN_R", 0.0015, "m", Source.GUESS,
           "The smallest convex radius a surface may have anywhere a hand can touch it. It is what "
           "separates a wearable from a knuckle-duster, and it is a GUESS: consumer-product edge "
           "rules put accessible edges in the region of 0.5 mm and edges that bear PRESSURE rather "
           "higher, but nothing here has been checked against a standard or against a hand. 1.5 mm "
           "is a rod ~3 mm across -- about the smallest thing that does not feel like a wire.")


def spikes(nodes, bars, live, buttons=()):
    """Every place the structure comes to a POINT: a member with a free end.

    A free end is a rod capped at its own radius. At the 0.4 mm nozzle floor that cap is a 0.4 mm
    point -- a scratch waiting to happen -- and it is invisible to every structural measure in this
    project, because a cantilever tip carries no load and so costs nothing to leave in.

    Button mounts are excluded: a well is SUPPOSED to end at the fingertip.
    """
    deg: dict[int, int] = {}
    for e in live:
        for i in bars[e]:
            deg[int(i)] = deg.get(int(i), 0) + 1
    keep = {int(b) for b in buttons}
    return sorted(i for i, d in deg.items() if d == 1 and i not in keep)


def sharpness(mesh):
    """The convex radius of curvature over the printed surface, in metres.

    Estimated from the mesh's own discrete curvature: for each vertex, the mean of the dihedral
    turns to its neighbours over the mean edge length gives a curvature, and 1/kappa is the radius.
    CONVEX only -- a concave fillet cannot scratch anybody, so a tight one is not an offence.
    """
    import trimesh
    from trimesh.curvature import discrete_mean_curvature_measure

    V = mesh.vertices
    # a radius that scales with the local mesh, so the estimate is not a function of the voxel size
    rad = float(np.mean(mesh.edges_unique_length)) * 2.0
    H = discrete_mean_curvature_measure(mesh, V, rad) / (np.pi * rad ** 2)
    # trimesh's sign convention: positive = convex (bulging outward)
    kappa = np.maximum(H, 0.0)
    r = np.where(kappa > 1e-9, 1.0 / np.maximum(kappa, 1e-9), np.inf)
    assert isinstance(mesh, trimesh.Trimesh)
    return r

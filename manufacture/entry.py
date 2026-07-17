"""THE FINGER-ENTRY ROUTE — the swept path a fingertip must traverse to enter its cup.

THE ERROR THIS EXISTS TO STOP. A mount can clear a finger in its FINAL SEATED position and still
block it from ever ENTERING. Checking only the static seated clearance let a strut land across the
entry and a rim sit over the cup — the fingertip had nowhere to come in from. That is not a detail;
it is the difference between a device you can put on and one you cannot.

THE ROUTE. A well is "open proximally so the phalanx slides in" (`hand.cradle`): the distal phalanx
enters by translating along its own axis, from withdrawn (proximal) to seated. So the entry route is
the distal-phalanx skin SWEPT along -axis over the slide-in length. The mount must leave that swept
volume open.

BLOCK vs GUIDE — the crucial distinction. The cup is SUPPOSED to sit close to the seated finger (it
cradles it) and its side walls GUIDE the phalanx in. Neither is a block. A block is mount material
the finger would have to pass THROUGH — i.e. material INSIDE the finger along the route. So the test
is signed: evaluate the mount's exact primitive SDF at the swept skin points; a point INSIDE the
mount (SDF < 0) means the mount penetrates the entering finger. Walls beside the finger read SDF > 0
(the finger is outside them) and do not trip it; a wall across the path reads SDF < 0 and does.

No boolean, no rtree: the mount is built from the same primitives `manufacture.mesh` meshes, so its
SDF is the analytic min over boxes / capsules / cylinders, evaluated on the swept point cloud.
"""
from __future__ import annotations

import numpy as np

from hand.flesh import skin
from manufacture.mesh import _box_sdf, _cyl_sdf, _seg_dist

# a touching cup/guide wall sits at SDF ~= 0; only material deeper than this into the finger blocks.
TOUCH_TOL = 3e-4        # m


def phalanx_skin(h, q, finger) -> np.ndarray:
    """The distal-phalanx skin points (the part of the finger that enters the cup), world coords."""
    V, _F, L = skin(h, q, labels=True)
    bid = h.pad[finger][0]                       # the distal-phalanx body id
    tip = np.asarray(V)[np.asarray(L) == bid]
    if len(tip) == 0:                            # fall back to the whole hand near the pad if unlabelled
        pos = np.asarray(h.well_frame(q, finger)["pos"], float)
        d = np.linalg.norm(np.asarray(V) - pos, axis=1)
        tip = np.asarray(V)[d < 0.02]
    return tip


def entry_sweep(h, q, finger, *, length=0.020, n=16) -> np.ndarray:
    """The distal-phalanx skin swept along -axis (the proximal slide-in), as one point cloud."""
    tip = phalanx_skin(h, q, finger)
    ax = np.asarray(h.well_frame(q, finger)["axis"], float)
    ax = ax / (np.linalg.norm(ax) + 1e-12)
    ts = np.linspace(0.0, length, n)
    return np.concatenate([tip - t * ax for t in ts])


def mount_sdf(P, boxes=(), caps=(), cyls=()) -> np.ndarray:
    """Signed distance to the mount (union of the primitives it is built from), per point in P.
    Negative = inside the mount. `caps` are (endpoints, radius); boxes (c, R, h); cyls (a, b, r)."""
    P = np.asarray(P, float)
    d = np.full(len(P), 1e9)
    for c, R, hh in boxes:
        d = np.minimum(d, _box_sdf(P, np.asarray(c, float), np.asarray(R, float), np.asarray(hh, float)))
    for (a, b), r in caps:
        d = np.minimum(d, _seg_dist(P, np.asarray(a, float), np.asarray(b, float)) - r)
    for a, b, r in cyls:
        d = np.minimum(d, _cyl_sdf(P, np.asarray(a, float), np.asarray(b, float), r))
    return d


def entry_clearance(h, q, finger, boxes=(), caps=(), cyls=(), *, length=0.020, n=16) -> float:
    """How deep the mount reaches INTO the entering finger, over the whole slide-in (metres).

    Returns the minimum mount SDF over the swept phalanx skin. **>= -TOUCH_TOL means the finger
    enters freely** (walls may guide it, but nothing crosses its path); a clearly negative value is a
    block, and names how deep. This is the constraint every mount geometry must pass.
    """
    P = entry_sweep(h, q, finger, length=length, n=n)
    return float(mount_sdf(P, boxes, caps, cyls).min())


def enters_freely(h, q, finger, boxes=(), caps=(), cyls=(), **kw) -> bool:
    """True iff the finger can slide into its cup without passing through mount material."""
    return entry_clearance(h, q, finger, boxes, caps, cyls, **kw) >= -TOUCH_TOL

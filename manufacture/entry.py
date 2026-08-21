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
from hand.myohand import PIP_BREADTH
from manufacture.mesh import _box_sdf, _cyl_sdf, _seg_dist

# a touching cup/guide wall sits at SDF ~= 0; only material deeper than this into the finger blocks.
TOUCH_TOL = 3e-4        # m

# THE FIRST PRINT'S LESSON (2026-08-21): the corridor was sized to the FINGERTIP, and the user's
# index PIP -- 25 mm against a 20 mm tip -- jammed in it, yawing the whole hand so no other finger
# entered straight. Two model fictions compounded: (1) only the DISTAL phalanx was swept, but the
# PIP traverses the same corridor on the way in; (2) the flesh model has no knuckle at all -- its
# middle-phalanx capsule (16 mm) is NARROWER than the distal (17.9 mm), where a real PIP is the
# widest point of the finger. So the entering cloud is now distal + middle phalanx skin PLUS an
# explicit PIP bulge ring at the measured breadth (hand/myohand.PIP_BREADTH, caliper values).
# The ring is circular at the LATERAL breadth, which overstates the dorsoventral depth a little --
# conservative on purpose: dorsal struts crossing above the knuckle are a jam the print also risks.
_MID_BODIES = {"thumb": "proximal_thumb", "index": "midph2", "middle": "midph3",
               "ring": "midph4", "little": "midph5"}
_JOINT_BODIES = {"thumb": "distal_thumb", "index": "midph2", "middle": "midph3",
                 "ring": "midph4", "little": "midph5"}   # body frame origin sits AT the PIP/IP


def phalanx_skin(h, q, finger) -> np.ndarray:
    """The distal-phalanx skin points (the part of the finger that seats in the cup), world coords."""
    V, _F, L = skin(h, q, labels=True)
    bid = h.pad[finger][0]                       # the distal-phalanx body id
    tip = np.asarray(V)[np.asarray(L) == bid]
    if len(tip) == 0:                            # fall back to the whole hand near the pad if unlabelled
        pos = np.asarray(h.well_frame(q, finger)["pos"], float)
        d = np.linalg.norm(np.asarray(V) - pos, axis=1)
        tip = np.asarray(V)[d < 0.02]
    return tip


def entering_skin(h, q, finger) -> np.ndarray:
    """Everything that must pass through the corridor: distal + middle phalanx skin, plus the
    PIP (thumb: IP) joint as two rings of the MEASURED breadth, world coords."""
    import mujoco

    V, _F, L = skin(h, q, labels=True)
    V, L = np.asarray(V), np.asarray(L)
    pts = [V[L == h.pad[finger][0]]]
    m = h.model
    mid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, _MID_BODIES[finger])
    pts.append(V[L == mid])
    h.fk(q)
    c = np.array(h.data.xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY,
                                               _JOINT_BODIES[finger])], float)
    ax = np.asarray(h.well_frame(q, finger)["axis"], float)
    ax = ax / (np.linalg.norm(ax) + 1e-12)
    n1 = np.cross(ax, [0.0, 0.0, 1.0])
    if np.linalg.norm(n1) < 1e-6:
        n1 = np.cross(ax, [0.0, 1.0, 0.0])
    n1 /= np.linalg.norm(n1)
    n2 = np.cross(ax, n1)
    r = 0.5 * float(PIP_BREADTH[finger]) * float(getattr(h, "scale", 1.0))
    th = np.linspace(0.0, 2.0 * np.pi, 24, endpoint=False)
    ring = np.cos(th)[:, None] * n1 + np.sin(th)[:, None] * n2
    pts += [c + off * ax + r * ring for off in (-0.0025, 0.0025)]
    return np.concatenate(pts)


def approach_axis(h, q) -> np.ndarray:
    """The rigid-hand donning direction: the mean of the four finger well axes.

    The old model slid each finger along its OWN well axis -- five non-parallel translations no
    rigid hand can perform at once. Fingers can absorb ALONG-axis differences by curling, but they
    cannot translate SIDEWAYS relative to the palm, so the lateral geometry of donning is one
    shared direction. The thumb is excluded from the mean AND keeps its own axis in entry_sweep:
    it genuinely is mobile enough to snake in separately (CMC + MCP + IP), and forcing it onto the
    fingers' approach would demand a sideways-open thumb cup no design can offer."""
    from hand.myohand import FINGERS
    a = np.sum([np.asarray(h.well_frame(q, f)["axis"], float)
                for f in FINGERS if f != "thumb"], axis=0)
    return a / (np.linalg.norm(a) + 1e-12)


def entry_sweep(h, q, finger, *, length=0.020, n=16) -> np.ndarray:
    """The entering finger (distal + middle + PIP bulge) swept along the donning direction --
    the shared rigid-hand approach for the four fingers, the thumb's own axis for the thumb."""
    pts = entering_skin(h, q, finger)
    if finger == "thumb":
        ax = np.asarray(h.well_frame(q, finger)["axis"], float)
        ax = ax / (np.linalg.norm(ax) + 1e-12)
    else:
        ax = approach_axis(h, q)
    ts = np.linspace(0.0, length, n)
    return np.concatenate([pts - t * ax for t in ts])


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


def smin_sdf(P, struts=(), radii=(), boxes=(), caps=(), cyls=(), *, blend=None) -> np.ndarray:
    """The EXPORT's smooth-min SDF (`manufacture.mesh.field`), evaluated at points P directly.

    `mount_sdf` above is the HARD union (plain min) of the mount's own primitives. But the STL is
    marched from a SMOOTH min over struts AND mount together, and the smooth-min INFLATES the
    surface outward by up to k*log(N) where N primitives meet -- exactly the fillet material a hard
    union never sees. So a filleted junction that bulges into a finger reads clear under `mount_sdf`
    and blocked here. This is the same field the mesh comes from; evaluating it at points needs no
    grid and no meshing (so no `m.contains` OOM).

    Numerically stable log-sum-exp; ignores carving (which only ADDS clearance, so this is a lower
    bound on the printed clearance).  `struts`/`caps` are capsules (segment minus radius); `boxes`
    (c,R,h); `cyls` (a,b,r) flat-capped.  radii = one per strut (or one scalar).
    """
    from manufacture.mesh import BLEND
    k = BLEND if blend is None else blend
    P = np.asarray(P, float)
    rr = np.broadcast_to(np.asarray(radii, float), (len(struts),)) if len(struts) else np.zeros(0)
    z = -k * 1e9 * np.ones(len(P))                       # running max of the exponents -k*d
    # two passes: first the max exponent per point (for stable LSE), then the shifted sum.
    exps = []
    for (a, b), re in zip(struts, rr):
        exps.append(-(_seg_dist(P, np.asarray(a, float), np.asarray(b, float)) - float(re)))
    for c, R, hh in boxes:
        exps.append(-_box_sdf(P, np.asarray(c, float), np.asarray(R, float), np.asarray(hh, float)))
    for (a, b), rc in caps:
        exps.append(-(_seg_dist(P, np.asarray(a, float), np.asarray(b, float)) - float(rc)))
    for a, b, rc in cyls:
        exps.append(-_cyl_sdf(P, np.asarray(a, float), np.asarray(b, float), rc))
    if not exps:
        return np.full(len(P), 1e9)
    E = np.stack(exps) / k                               # (n_prim, n_pts) = -d_i / k
    m = E.max(axis=0)
    return -k * (m + np.log(np.exp(E - m).sum(axis=0)))


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

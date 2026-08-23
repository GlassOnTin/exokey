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
from design.params import DON_CLEAR as _DON_CLEAR, DON_LEN as _DON_LEN
from hand.myohand import PIP_BREADTH
from manufacture.mesh import _box_sdf, _cyl_sdf, _seg_dist

# a touching cup/guide wall sits at SDF ~= 0; only material deeper than this into the finger blocks.
TOUCH_TOL = 3e-4        # m

# The corridor check runs over a long path, so the cloud is strided: we need the finger's OUTER
# ENVELOPE, not every skin vertex, and every 4th point preserves it to well under the tolerances
# that matter here while keeping the check affordable inside the GA loop.
CORRIDOR_STRIDE = 4
# ⚠ THE KEEP-OUT AND THE CONSTRAINT MUST SAMPLE THE SAME TRAJECTORY. They did not -- ground()
# swept 6 postures while the constraint checked 9 -- so a strut could sit in a corridor position
# the keep-out never looked at and the constraint duly failed it. Measured on the merged front:
# neighbour cups cleared by 14-43 mm and the ONLY binding obstacle was a strut at -0.51 mm, in a
# design space that was supposed to have excluded it. One constant, both users.
DON_STEPS = 11
SEAT_ZONE = 0.005       # m -- within this of seated, the pad is ON its cup floor by design
                        # (SEAT_CLEAR), so contact there is the SEAT, not an obstruction. Measuring
                        # the whole sweep with one threshold reported that seat as the binding
                        # clearance and hid the real jam 20-70 mm out.

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


def donning_gaps(h, curls, boxes=(), caps=(), cyls=(), *, own=None, n=DON_STEPS) -> dict:
    """{finger: (seated gap, approach gap)} along the donning motion a hand actually makes.

    ⚠ ONE POSTURE FOR THE WHOLE HAND PER STEP, and that is both faster AND more honest. Doing it
    per finger meant hand.flesh.skin() -- which meshes the WHOLE HAND -- ran once per finger per
    step: 95 calls, 2.0 s of a 7.1 s evaluate, to look at two phalanges each time. But the fingers
    do not enter one at a time. The hand comes in as one object, fingers EXTENDED (a straight
    finger is slim and slips past struts a curled one would foul), and CURLS as the tips seat --
    which is also what COMMON_DRIVE says the four fingers must do anyway. So compose the whole
    hand at each step and read all five fingers off one mesh: 9 calls, not 95.

    (Capsule-axis sampling was tried as the cheaper alternative and REJECTED: MyoHand's capsules
    run 1.5-6 mm fatter than the skin, and the cups are fitted to the SKIN -- mount._seat measures
    skin extents -- so capsules report the cup penetrating a finger that is in fact seated in it.
    Fast and wrong. The saving here is the same order and keeps the geometry the artifact uses.)

    Two zones: within SEAT_ZONE of seated the pad rests on its own cup floor at SEAT_CLEAR by
    design, so contact there is the seat working, not a jam."""
    from design.vector import FINGERS, posture

    q0 = h.compose({f: posture(h, f, *(float(v) for v in curls[(f, 0)])) for f in FINGERS})
    ax = {f: (np.asarray(h.well_frame(q0, f)["axis"], float) if f == "thumb"
              else approach_axis(h, q0)) for f in FINGERS}
    ax = {f: a / (np.linalg.norm(a) + 1e-12) for f, a in ax.items()}
    L = float(_DON_LEN)
    out = {f: [np.inf, np.inf, np.inf] for f in FINGERS}   # seat, approach, own-cup
    for s in np.linspace(0.0, 1.0, n):
        q_s = h.compose({f: posture(h, f,
                                    0.05 + s * (float(curls[(f, 0)][0]) - 0.05),
                                    0.05 + s * (float(curls[(f, 0)][1]) - 0.05),
                                    float(curls[(f, 0)][2])) for f in FINGERS})
        V, _F, Lb = skin(h, q_s, labels=True)
        V, Lb = np.asarray(V), np.asarray(Lb)
        import mujoco
        for f in FINGERS:
            mid = mujoco.mj_name2id(h.model, mujoco.mjtObj.mjOBJ_BODY, _MID_BODIES[f])
            pts = np.concatenate([V[Lb == h.pad[f][0]], V[Lb == mid]])[::CORRIDOR_STRIDE]
            pts = np.vstack([pts, _pip_ring(h, q_s, f)]) - (1.0 - s) * L * ax[f]
            # ⚠ A FINGER'S OWN CUP IS NOT AN OBSTACLE, IT IS THE DESTINATION. Held to the same
            # donning clearance as everything else, the constraint is UNSATISFIABLE: the cup is
            # built SEAT_CLEAR (0.4 mm) off the finger because it must cradle it, and the finger
            # is still inside it for the whole cup length (~16-20 mm) on the way out -- so asking
            # for 2 mm there asks for a cup that does not hold anything. Measured: the GA drove
            # the corridor to exactly touching and stalled at cv 0.0019772 (= DON_CLEAR - 0) from
            # generation 60 to 90, and reported NO FEASIBLE DESIGN after 5.7 hours. So the own cup
            # is held only to non-interference; the STRUTS and the OTHER fingers' cups -- the
            # things that have no business in this finger's path -- are what must leave room.
            if own is None:
                d_room = float(mount_sdf(pts, boxes, caps, cyls).min())
                d_own = d_room
            else:
                ob, oc, oy = own[f]
                d_room = float(mount_sdf(pts, boxes, caps, cyls).min())
                d_own = float(mount_sdf(pts, ob, oc, oy).min())
            z = 0 if (1.0 - s) * L <= SEAT_ZONE else 1
            out[f][z] = min(out[f][z], d_room)
            out[f][2] = min(out[f][2], d_own)
    return {f: (min(v[0], v[2]), v[1] if np.isfinite(v[1]) else v[0])
            for f, v in out.items()}   # (non-interference incl. own cup, room needed)


def _pip_ring(h, q, finger, k=16):
    """The knuckle at its MEASURED breadth, as a ring of points (the flesh model has no bulge)."""
    import mujoco

    h.fk(q)
    c = np.asarray(h.data.xpos[mujoco.mj_name2id(h.model, mujoco.mjtObj.mjOBJ_BODY,
                                                 _JOINT_BODIES[finger])], float)
    ax = np.asarray(h.well_frame(q, finger)["axis"], float)
    ax = ax / (np.linalg.norm(ax) + 1e-12)
    n1 = np.cross(ax, [0.0, 0.0, 1.0])
    if np.linalg.norm(n1) < 1e-6:
        n1 = np.cross(ax, [0.0, 1.0, 0.0])
    n1 /= np.linalg.norm(n1)
    n2 = np.cross(ax, n1)
    r = 0.5 * float(PIP_BREADTH[finger]) * float(getattr(h, "scale", 1.0))
    th = np.linspace(0.0, 2.0 * np.pi, k, endpoint=False)
    ring = c + r * (np.cos(th)[:, None] * n1 + np.sin(th)[:, None] * n2)
    return np.vstack([ring - 0.0025 * ax, ring + 0.0025 * ax])


def donning_frames(h, curls, *, n=24):
    """The donning motion as renderable frames: [(q, offset), ...] from fully withdrawn+extended
    to seated. `q` is a whole-hand joint vector; `offset` is the rigid translation to apply to the
    hand at that frame (metres, world). Feed to a viewer to animate the hand entering the gauntlet.

    Saved with every final result (scripts/final.py -> out/donning.npz) so the entry can be
    rendered without re-deriving the trajectory -- the same interpolation the entry-route
    CONSTRAINT is judged on, so the animation shows what the optimiser was actually promising."""
    from design.vector import FINGERS, posture

    q0 = h.compose({f: posture(h, f, *(float(v) for v in curls[(f, 0)])) for f in FINGERS})
    ax = approach_axis(h, q0)
    L = float(_DON_LEN)
    frames = []
    for s in np.linspace(0.0, 1.0, n):
        q_s = h.compose({f: posture(h, f,
                                    0.05 + s * (float(curls[(f, 0)][0]) - 0.05),
                                    0.05 + s * (float(curls[(f, 0)][1]) - 0.05),
                                    float(curls[(f, 0)][2])) for f in FINGERS})
        frames.append((np.asarray(q_s, float), -(1.0 - s) * L * ax))
    return frames


def donning_gaps_split(h, curls, *, room, own, struts=(), n=DON_STEPS) -> dict:
    """{finger: (seated gap, approach gap)} with the two obstacle sets kept apart.

    `room[f]`  -- everything that must leave DON_CLEAR beside this finger: the OTHER fingers'
                  cups. Plus `struts`, which apply to every finger.
    `own[f]`   -- this finger's own cup, held only to non-interference: it is built SEAT_CLEAR
                  off the finger BECAUSE it cradles it, and the finger is inside it for the whole
                  cup length on the way out. Demanding donning clearance there is unsatisfiable,
                  and measurably so: the GA stalled at cv 0.0019772 (DON_CLEAR minus nothing) for
                  30 generations and returned NO FEASIBLE DESIGN after 5.7 hours."""
    from design.vector import FINGERS, posture

    q0 = h.compose({f: posture(h, f, *(float(v) for v in curls[(f, 0)])) for f in FINGERS})
    ax = {f: (np.asarray(h.well_frame(q0, f)["axis"], float) if f == "thumb"
              else approach_axis(h, q0)) for f in FINGERS}
    ax = {f: a / (np.linalg.norm(a) + 1e-12) for f, a in ax.items()}
    L = float(_DON_LEN)
    seat = {f: np.inf for f in FINGERS}
    app = {f: np.inf for f in FINGERS}
    import mujoco
    for s in np.linspace(0.0, 1.0, n):
        q_s = h.compose({f: posture(h, f,
                                    0.05 + s * (float(curls[(f, 0)][0]) - 0.05),
                                    0.05 + s * (float(curls[(f, 0)][1]) - 0.05),
                                    float(curls[(f, 0)][2])) for f in FINGERS})
        V, _F, Lb = skin(h, q_s, labels=True)
        V, Lb = np.asarray(V), np.asarray(Lb)
        for f in FINGERS:
            mid = mujoco.mj_name2id(h.model, mujoco.mjtObj.mjOBJ_BODY, _MID_BODIES[f])
            pts = np.concatenate([V[Lb == h.pad[f][0]], V[Lb == mid]])[::CORRIDOR_STRIDE]
            pts = np.vstack([pts, _pip_ring(h, q_s, f)]) - (1.0 - s) * L * ax[f]
            rb, rc, ry = room[f]
            d_room = float(mount_sdf(pts, rb, list(rc) + list(struts), ry).min())
            ob, oc, oy = own[f]
            d_own = float(mount_sdf(pts, ob, oc, oy).min())
            seat[f] = min(seat[f], d_own, d_room if (1.0 - s) * L <= SEAT_ZONE else np.inf)
            if (1.0 - s) * L > SEAT_ZONE:
                app[f] = min(app[f], d_room)
    return {f: (seat[f], app[f] if np.isfinite(app[f]) else seat[f]) for f in FINGERS}


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

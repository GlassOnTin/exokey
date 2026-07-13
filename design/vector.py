"""Design vector theta, and what it costs. The outer problem's evaluation.

The one modelling decision that makes this tractable and interpretable:

    KEYS ARE PARAMETERISED BY THE REFERENCE HAND'S FLEXION FRACTIONS, then FROZEN into
    hand-frame coordinates as the physical device.

So every key is reachable by the median hand BY CONSTRUCTION -- the optimiser never wastes
evaluations on keys nobody can touch, which is what killed v1's search -- and the variables
mean something you can say out loud ("index row 0 sits at 40% curl"). The device is then
ONE rigid object, and the 5th- and 95th-percentile hands have to reach the keys where the
median hand's fingers happened to leave them. That is the actual population constraint, and
it is hard: it is not satisfied by construction and the optimiser has to work for it.

Because scaling is isotropic, the hand axes are the same directions for every hand and only
the origin moves -- so a key fixed in hand-frame coordinates is a pure translation between
hands. (Verified: test_key_transfer_is_a_translation.)
"""
from __future__ import annotations

import itertools
import os

import numpy as np

from design.params import (COMMON_DRIVE as _CD, RESIDUAL_MAX as _RESID,
                           SVALBOARD, WELL_WALL as _WELL_WALL, check_coherent)
from hand.cradle import solve as cradle_solve
from hand.myohand import FINGERS, FLEXION_JOINTS, MyoHand

check_coherent(SVALBOARD)  # force and travel must describe the SAME switch
from structure.frame import DIGIT_FLESH, build_body, clearance, solve

# Switch travel and cap geometry. These describe a LOW-PROFILE wearable switch, and they
# must be consistent with PRESS_N below -- they were not, and it cost us the 10-key designs.
#
# SWITCH_TRAVEL was 3 mm, quoting "a mechanical switch actuates at ~2 mm". That is a
# Cherry-MX-style full-travel key. But the actuation force is 0.30 N, which is BELOW Cherry
# MX (0.45-0.60 N) and squarely in dome/scissor territory -- and a scissor switch (a laptop
# key) travels ~1.5 mm. The two numbers described different switches. Reference point:
# Typeware's wearable keyboard fits 2 keys per finger, which this model called infeasible.
SWITCH_TRAVEL = float(SVALBOARD.travel)  # see design/params.py — provenance-tagged
REACH_TOL = 0.005  # m
SATURATION = 0.95

# A keycap is a PHYSICAL OBJECT and two of them cannot occupy the same space. Nothing in
# the model said so until this was added, and measured, EVERY design violated it: with one
# key per finger the middle and ring keys came out 8.5 mm apart (curled fingertips
# converge) against a ~14 mm cap, and thumb rows landed 3.5 mm apart. The whole Pareto
# front was geometrically unbuildable and nothing flagged it.
# WELL PITCH. Derived per finger-pair from the model's own anatomy -- NOT a constant.
#
# This was 12 mm, inherited from when these were KEYCAPS sitting on stems. A WELL is a
# CAVITY THE FINGERTIP SITS INSIDE, and MyoHand's own fingertips are 12-14 mm wide (distal
# phalanx flesh radius 6.0-7.0 mm). Two adjacent wells therefore need
#
#     r_f + r_g + 2 * wall
#
# between their centres -- about 16-18 mm, not 12. A front optimised at 12 mm is a device
# whose wells physically overlap, and it is worth nothing.
WELL_WALL = float(_WELL_WALL)  # see design/params.py

# TWIN KEY. Two switches on ONE stem, at two positions -- the trick the target device
# (typeware.tech) uses, and it "reduces spacing by ~30%" versus standard keys.
#
# Without it, this model required 12 mm between a finger's OWN two keys, as though they
# were two independent caps that must not collide. They are not: they share a stem. That
# single wrong assumption is a large part of why 2-keys-per-finger came out infeasible
# here while the real device ships it.
#
# It also helps INDIRECTLY, and that matters more: a twin key lets the two rows sit closer
# together, so the deep row is less deeply curled, so adjacent fingers' deep keys converge
# less -- and the cross-finger pair (middle/ring row 1) was the binding collision.
TWIN_KEY_PITCH = 0.008  # m; two positions on one stem

# ⚠ PATENT. Typeware state their keyboard uses "patented mechanical (twin) keys". A patent
# is not copyright: reimplementing a claimed invention independently still infringes, and an
# open-source licence grants nothing against it.
#
# So the twin key is an OPTION, not an assumption -- and the optimiser is used to MEASURE
# WHAT IT IS WORTH. Set EXOKEY_NO_TWIN_KEY=1 and a finger's own two keys must clear the full
# KEY_PITCH, like any two independent caps on separate stems: a clean-room, non-infringing
# design. Optimise both, compare the fronts, and the question "do we actually need the
# patented feature?" gets a number instead of an opinion.
#
# Nothing else here is Typeware's: the muscle-effort model, the population scaling and the
# co-design loop are ours and are freely publishable either way.
ALLOW_TWIN_KEY = os.environ.get("EXOKEY_NO_TWIN_KEY", "") != "1"

# Switch actuation force. FIXED, not optimised.
#
# It was a design variable over [0.30, 0.80] N, and every one of the 60 Pareto designs
# pinned it to the lower bound. That is a corner solution, not a trade-off: a lighter
# switch cuts BOTH the muscle effort AND the frame deflection, so nothing pulls the other
# way and there is nothing for an optimiser to resolve. Carrying it as a variable just
# added a dimension to a 27-dimensional mixed search for no information.
#
# If a lower bound is ever wrong it will be because a real switch needs enough force to
# avoid accidental actuation -- a constraint on the SWITCH, not something the layout trades
# against. Encode it here, not in the objective.
PRESS_N = float(SVALBOARD.force)  # 20 gf, Svalboard spec — see design/params.py
COMMON_DRIVE = float(_CD)
RESIDUAL_MAX = float(_RESID)

MATERIAL_CHOICES = ["al6061", "al7075", "cf_pa12"]

# Continuous variables: name -> (lo, hi). Per-finger curl, splay, switch force, structure.
REAL_BOUNDS: dict[str, tuple[float, float]] = {}
REAL_BOUNDS["tp_thumb"] = (0.10, 0.80)      # the thumb drives independently of the fingers
REAL_BOUNDS["tm_thumb"] = (0.10, 0.80)

# COMMON DRIVE, BUILT IN RATHER THAN CHECKED.
#
# The four fingers share ONE curl `tm_hand` plus a small per-finger deviation. So the
# spread can never exceed COMMON_DRIVE and the constraint is satisfied BY CONSTRUCTION --
# the same move that made `reach` identically zero once the wells became adjustable.
#
# It was four INDEPENDENT curls with `spread <= 0.15` bolted on as a constraint, and that
# is a bad trade: four independent U(0.10,0.80) draws have an expected spread of ~0.42, so
# 98% of random designs were born violating it. The GA then spends its budget rediscovering
# a constraint we could simply have made unrepresentable. Measured on 240 random designs:
# common-drive was the single most-violated of the nine.
#
# ⚠ And note WHAT is being built in: COMMON_DRIVE = 0.15 is a GUESS (design/params.py), a
# stand-in for the enslavement MyoHand does not model. Building a guess into the
# parameterisation makes it structural, so it must stay visible -- hence the one place it is
# written down is params.py, and it is listed in VISION.md section 6.
# BOTH joints, not just the PIP. Constraining `tm` alone left `tp` (the MCP) free, and the
# optimiser drove straight through the gap: MCP spread 0.37 of range against a 0.15 limit,
# with the ring nearly straight (0.12) while the index was half-curled (0.49). That is the
# SAME hand nobody can make that the common-drive constraint was introduced to forbid --
# it had simply moved to the joint that was not being watched.
# Curl bounds widened to 0.90. The 0.80 cap was arbitrary and it was BINDING: the only
# postures where all four fingers can perform three directions sit at tp >= 0.80, and the
# residual keeps improving out to 0.85-0.90 (4.0% at the old bound, 3.8% beyond it). An
# optimum sitting exactly ON a bound is an artefact of the bound, not a design.
#
# ⚠ AND IT SETS UP A REAL FIGHT, which is the interesting part:
#     PERFORMABILITY wants a HIGH MCP curl (0.8-0.9) -- that is where the muscles can
#                    actually balance three of the five directions;
#     WELL PACKING   wants the fingertips SPREAD -- and curling CONVERGES them.
# The two constraints pull opposite ways. That is what the constrained search is for.
REAL_BOUNDS["tp_hand"] = (0.10, 0.90)
REAL_BOUNDS["tm_hand"] = (0.10, 0.90)
for _f in ("index", "middle", "ring", "little"):
    REAL_BOUNDS[f"dp_{_f}"] = (-float(COMMON_DRIVE) / 2, float(COMMON_DRIVE) / 2)
    REAL_BOUNDS[f"dm_{_f}"] = (-float(COMMON_DRIVE) / 2, float(COMMON_DRIVE) / 2)
# NO `dc_`: there are no ROWS. One key per finger; the three QWERTY rows come from three
# finger ACTIONS on one 3-position Hall key, not from three physical keys.
# SPLAY. Without it the fingers cannot separate their keys and KEY_PITCH is unsatisfiable:
# adjacent curled fingertips sit ~8.5 mm apart, which is inside a keycap. Abduction is
# already in each digit's dof set, so the effort model charges for the splay automatically.
for _f in ("index", "middle", "ring", "little"):
    REAL_BOUNDS[f"ab_{_f}"] = (-0.9, 0.9)  # fraction of the MCP abduction range
REAL_BOUNDS["alu_w"] = (0.004, 0.012)
REAL_BOUNDS["alu_t"] = (0.0008, 0.0030)
REAL_BOUNDS["palm_offset"] = (0.014, 0.030)  # how far palmar the palm support bears
# STAND-OFF of the well from the body face. Was (0.004, 0.014) and the optimiser PINNED it
# to 0.014 in every design on the front -- `report_cornered` flagged it, which is what that
# check exists for. A cornered variable means one of two things, and here it is the second:
# either the variable is dead, or THE BOUND IS WRONG AND THE ANSWER IS AN ARTEFACT OF IT.
# The geometry wants the wells further off the body than 14 mm, because that is how the body
# stays clear of the finger BONES while the wells stay out at the fingertips. Widened.
REAL_BOUNDS["stem"] = (0.004, 0.035)
REAL_BOUNDS["body_half"] = (0.016, 0.030)  # body half-width: a ONE-SIZE body must fit the
REAL_BOUNDS["body_dist"] = (0.040, 0.070)  # 5th percentile too, so its SIZE has to be free

# BODY_PROX is FIXED, not optimised. It pinned to its lower bound in all 200 designs of the
# first front, because reaching further proximally is monotonically WORSE: it adds mass AND
# adds deflection (63 -> 103 g, 0.016 -> 0.047 mm across the range). A variable that is
# dominated everywhere is not a decision, it is dead search space.
#
# ⚠ CAVEAT, and it is a real one: "worse" here is partly an artefact of the BEAM model. A
# longer body means longer, floppier beam members. A real MOULDED SHELL would get STIFFER
# with more material, not softer. So "shortest body wins" is a property of this idealisation,
# not necessarily of a real device, and it should not be taken as a design recommendation
# until it is checked with shell elements.
BODY_PROX = 0.008  # m

# PER-FINGER ADJUSTMENT RANGE. The device is built once, but each finger well SLIDES to fit
# the user -- Svalboard ships "individual 5-axis finger adjustability", and it is how a real
# one-size device covers a population at all.
#
# This is the plan's Stage-5 joint sub-problem, and it is now well posed: how much
# adjustment range must be built in to cover the 5th-95th percentile? More range = more
# mechanism = more mass, so the optimiser has to pay for it, and the MINIMUM range falls
# out of the answer instead of being assumed.
REAL_BOUNDS["adjust"] = (0.000, 0.030)  # m, per-finger well travel built into the device

INT_BOUNDS: dict = {}  # nothing integer left: one key per finger, always


def posture(h: MyoHand, finger: str, t_p: float, t_m: float, ab: float = 0.0) -> np.ndarray:
    """Hand posture with `finger` at flexion fractions (proximal, middle); distal at 2/3.
    `ab` splays the digit: a fraction of its abduction range.

    Fractions, never signed angles -- the thumb flexes positive at the CMC and negative at
    the MP and IP, so anything that touches raw angles gets the thumb backwards.
    """
    import mujoco

    prox, mid, dist = FLEXION_JOINTS[finger]
    q = h.q_neutral.copy()
    a_p, _, lim_p = h.flexion_span(prox)
    a_m, _, lim_m = h.flexion_span(mid)
    a_d, _, lim_d = h.flexion_span(dist)
    q[a_p] = np.clip(t_p, 0.02, 0.95) * lim_p
    q[a_m] = np.clip(t_m, 0.02, 0.95) * lim_m
    q[a_d] = np.clip((2.0 / 3.0) * t_m, 0.02, 0.95) * lim_d

    if ab and finger != "thumb":
        d = {"index": "2", "middle": "3", "ring": "4", "little": "5"}[finger]
        jid = mujoco.mj_name2id(h.model, mujoco.mjtObj.mjOBJ_JOINT, f"mcp{d}_abduction")
        lo, hi = h.model.jnt_range[jid]
        half = 0.5 * (float(hi) - float(lo))
        q[h.model.jnt_qposadr[jid]] = np.clip(ab, -1.0, 1.0) * half
    return q


_WELL_R: dict = {}


def well_radius(h: MyoHand, finger: str) -> float:
    """Half-width of the WELL for this finger: its fingertip has to fit inside.

    Measured from the model's distal-phalanx flesh capsule, not assumed. 6.0 mm (little) to
    7.0 mm (index/middle).
    """
    import mujoco

    key = (id(h.model), finger)
    if key in _WELL_R:
        return _WELL_R[key]
    m = h.model
    bid = h.pad[finger][0]
    r = 0.0
    for g in range(m.body_geomadr[bid], m.body_geomadr[bid] + m.body_geomnum[bid]):
        if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_CAPSULE:
            r = max(r, float(m.geom_size[g][0]))
    _WELL_R[key] = r
    return r


def _seg_seg_dist(p0, p1, q0, q1) -> float:
    """Shortest distance between two 3-D segments. Clamped, so it handles parallel channels."""
    u, v, w = p1 - p0, q1 - q0, p0 - q0
    a, b, c = u @ u, u @ v, v @ v
    d, e = u @ w, v @ w
    den = a * c - b * b
    sc = 0.0 if den < 1e-12 else np.clip((b * e - c * d) / den, 0.0, 1.0)
    tc = np.clip((a * e - b * d) / c, 0.0, 1.0) if c > 1e-12 else 0.0
    # one refinement pass is enough for the near-parallel channels we actually have
    sc = np.clip((b * tc - d) / a, 0.0, 1.0) if a > 1e-12 else 0.0
    tc = np.clip((b * sc + e) / c, 0.0, 1.0) if c > 1e-12 else 0.0
    return float(np.linalg.norm(w + sc * u - tc * v))


def well_channel(h: MyoHand, q: np.ndarray, finger: str) -> tuple[np.ndarray, np.ndarray, float]:
    """The well as the CHANNEL it is: the segment the distal phalanx occupies, and its
    lateral half-width. Returns (distal end, proximal end, radius)."""
    wf = h.well_frame(q, finger)
    distal = wf["pos"]
    proximal = wf["pos"] - 2.0 * wf["half"] * wf["axis"]
    return distal, proximal, wf["radius"]


def key_separation(keys: dict, h: MyoHand, curls: dict | None = None) -> tuple[float, tuple | None]:
    """Worst well-pair spacing, as a VIOLATION (required - actual; <= 0 is fine).

    THE WELL IS A CHANNEL, SO THIS IS A SEGMENT-SEGMENT TEST, NOT A POINT-POINT ONE.

    It was point-to-point between the two pads, which silently assumes a well is a DISC at the
    fingertip. It is not: the distal phalanx SLIDES INTO the well along its own axis, so the
    well is elongated -- and two channels can be comfortably apart at the pads while crossing
    further back, exactly where two fingers converge toward the knuckles. A point test cannot
    see that collision at all.

    Required clearance is still derived per pair from the two fingertips that must fit inside:
    r_f + r_g + 2*wall. It is not a constant, and it is not 12 mm -- that was a KEYCAP pitch.
    """
    if curls is None:  # no posture given: fall back to the pads alone (used by older callers)
        items = list(keys.items())
        worst, pair = -np.inf, None
        for i in range(len(items)):
            for j in range(i + 1, len(items)):
                (fa, _), (fb, _) = items[i][0], items[j][0]
                need = well_radius(h, fa) + well_radius(h, fb) + 2.0 * WELL_WALL
                v = need - float(np.linalg.norm(items[i][1][0] - items[j][1][0]))
                if v > worst:
                    worst, pair = v, (items[i][0], items[j][0])
        return worst, pair

    chan = {}
    for f in FINGERS:
        t_p, t_m, ab = curls[(f, 0)]
        chan[f] = well_channel(h, posture(h, f, t_p, t_m, ab), f)

    worst, pair = -np.inf, None
    for i, fa in enumerate(FINGERS):
        for fb in FINGERS[i + 1:]:
            da, pa, ra = chan[fa]
            db, pb, rb = chan[fb]
            need = ra + rb + 2.0 * WELL_WALL
            v = need - _seg_seg_dist(da, pa, db, pb)
            if v > worst:
                worst, pair = v, (fa, fb)
    return worst, pair

def well_finger_clearance(h: MyoHand, x: dict) -> tuple[float, tuple | None]:
    """A WELL IS A SOLID OBJECT. No OTHER digit's bones may pass through it.

    The gap the user pointed at: "still need some collision avoidance for the buttons".
    `key-overlap` compares WELL against WELL, and `clearance` compares the BODY against bone
    -- but nothing compared a WELL against a NEIGHBOURING FINGER. And a well is a box roughly
    16 x 14 x 7 mm sitting out at a fingertip, right where the next finger's MIDDLE and
    PROXIMAL phalanges run past it. Two wells can clear each other perfectly while one of them
    sits inside the ring finger.

    Well-vs-well is not enough on its own either, because it only ever compared the two DISTAL
    channels -- the proximal bones were invisible to every check we had.

    All five digits are placed in ONE composed hand posture first (`compose`), because that is
    what the hand actually does: the fingers are all in their wells at the same time. Checking
    each finger in isolation would miss precisely the collisions that matter.

    Returns (worst signed gap, offending pair). Negative = the well is inside a finger.
    """
    import mujoco

    m = h.model
    per = {f: posture(h, f, tp_of(x, f), tm_of(x, f), float(x.get(f"ab_{f}", 0.0)))
           for f in FINGERS}
    q_all = h.compose(per)

    wells = {}
    for f in FINGERS:
        wf = h.well_frame(q_all, f)
        d0 = wf["pos"] + 0.004 * wf["axis"]                    # distal end stop
        d1 = wf["pos"] - 2.0 * wf["half"] * wf["axis"]         # open proximal end
        wells[f] = (d0, d1, wf["radius"] + WELL_WALL)

    h.fk(q_all)
    bones = {}
    for f in FINGERS:
        segs = []
        for bname in _DIGIT_BODIES[f]:
            bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, bname)
            for g in range(m.body_geomadr[bid], m.body_geomadr[bid] + m.body_geomnum[bid]):
                if m.geom_type[g] != mujoco.mjtGeom.mjGEOM_CAPSULE:
                    continue
                r = float(m.geom_size[g][0])
                half = float(m.geom_size[g][1])
                c = h.data.geom_xpos[g]
                ax = h.data.geom_xmat[g].reshape(3, 3)[:, 2]
                segs.append((c - half * ax, c + half * ax, r))
        bones[f] = segs

    worst, pair = np.inf, None
    for f in FINGERS:
        a0, a1, rw = wells[f]
        for g in FINGERS:
            if g == f:
                continue          # a finger's OWN well is supposed to hold it
            for b0, b1, rb in bones[g]:
                gap = _seg_seg_dist(a0, a1, b0, b1) - rw - rb
                if gap < worst:
                    worst, pair = gap, (f, g)
    return float(worst), pair


def swept_path_clearance(h: MyoHand, keys: dict, curls: dict, n_keys: dict, shift, n_samples: int = 7) -> float:
    """As the hand goes INTO the device, does a finger sweep through a NEIGHBOUR'S WELL?

    The static clearance check cannot see this: it looks at the final posture, and the
    collision happens on the way there. Each finger is swept from nearly straight to its
    resting posture and checked against every OTHER digit's well (its own well it is supposed
    to sit in -- that is the entire point of a well).

    This is what stops the optimiser crossing the fingers over one another: splay is free over
    +/-90% of the abduction range, and at the extremes a finger will lie across its neighbour.

    THE OBSTACLE IS THE WELL RIM, AND ITS HEIGHT IS DERIVED, NOT DECLARED. It was
    `CAP_HEIGHT = 3 mm`: how far a low-profile KEYCAP stands proud of its plate. But a well is
    not a cap, it is a CUP -- and a cup that holds a fingertip and can be NUDGED SIDEWAYS by
    it must wrap that fingertip to about its equator. So the rim stands proud by roughly the
    fingertip's own radius, which is exactly what `well_radius` already measures (6-7 mm, not
    3). Same species as KEY_PITCH: a keycap number outliving the keycaps, and here it made the
    device look half as obstructive as it really is.

    Returns the signed minimum gap; negative means a finger drives through a neighbour's well.
    """
    import mujoco

    m = h.model
    other_caps, other_rims = {}, {}
    for f in FINGERS:
        other_caps[f] = np.array(
            [keys[(g, k)][0] + shift for g in FINGERS if g != f for k in range(n_keys[g])]
        )
        # the rim of EACH neighbour's well, sized by THAT neighbour's own fingertip
        other_rims[f] = np.array(
            [well_radius(h, g) for g in FINGERS if g != f for _ in range(n_keys[g])]
        )

    worst = np.inf
    for f in FINGERS:
        caps, rims = other_caps[f], other_rims[f]
        if not len(caps):
            continue
        deep = max(n_keys[f] - 1, 0)
        tp_end, tm_end, ab = curls[(f, deep)]
        for s in np.linspace(0.05, 1.0, n_samples):
            t_p = 0.05 + s * (tp_end - 0.05)
            t_m = 0.05 + s * (tm_end - 0.05)
            q = posture(h, f, t_p, t_m, ab)
            h.fk(q)
            # the digit's flesh: its skin capsules
            for bname in _DIGIT_BODIES[f]:
                bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, bname)
                for g in range(m.body_geomadr[bid], m.body_geomadr[bid] + m.body_geomnum[bid]):
                    if m.geom_type[g] != mujoco.mjtGeom.mjGEOM_CAPSULE:
                        continue
                    r = float(m.geom_size[g][0])
                    half = float(m.geom_size[g][1])
                    c = h.data.geom_xpos[g]
                    axis = h.data.geom_xmat[g].reshape(3, 3)[:, 2]
                    a, b = c - half * axis, c + half * axis
                    ab_ = b - a
                    tt = np.clip(((caps - a) @ ab_) / (ab_ @ ab_ + 1e-12), 0.0, 1.0)
                    closest = a + tt[:, None] * ab_
                    d = np.linalg.norm(caps - closest, axis=1) - r - rims
                    worst = min(worst, float(d.min()))
    return worst


_DIGIT_BODIES = {
    "thumb": ("firstmc", "proximal_thumb", "distal_thumb"),
    "index": ("proxph2", "midph2", "distph2"),
    "middle": ("proxph3", "midph3", "distph3"),
    "ring": ("proxph4", "midph4", "distph4"),
    "little": ("proxph5", "midph5", "distph5"),
}



def _curl(x: dict, finger: str, which: str) -> float:
    if finger == "thumb":
        return float(x[f"{which}_thumb"])
    lo, hi = REAL_BOUNDS[f"{which}_hand"]
    return float(np.clip(x[f"{which}_hand"] + x[f"d{which[1]}_{finger}"], lo, hi))


def tp_of(x: dict, finger: str) -> float:
    """The finger's PROXIMAL (MCP) curl. Common-driven across the four fingers."""
    return _curl(x, finger, "tp")


def tm_of(x: dict, finger: str) -> float:
    """The finger's MIDDLE (PIP) curl. Common-driven across the four fingers; the thumb is
    not enslaved to them and drives itself."""
    return _curl(x, finger, "tm")


def keys_on_reference(ref: MyoHand, x: dict) -> tuple[dict, dict]:
    """(finger, 0) -> (world pos, outward normal) of that finger's WELL on the REFERENCE
    hand, plus the curls that generated it (needed by the swept-path check).

    ONE well per finger. There are no rows: the three QWERTY rows come from three of the
    well's five joystick DIRECTIONS, which is firmware, not geometry.

    The posture this returns has to be a RELAXED, SPLAYED hand, and that is not a matter of
    taste -- it is forced. A well is a cavity the fingertip sits INSIDE, so the wells need
    the fingertips SPREAD (~17 mm apart), while gripping a body CONVERGES them (~6 mm at
    high curl). Measured: even at maximum splay and maximum stagger, a gripping hand cannot
    fit five wells -- middle and ring overlap by 2.1 mm. The hand must stay open.
    """
    keys, curls = {}, {}
    for f in FINGERS:
        ab = float(x.get(f"ab_{f}", 0.0))
        for k in (0,):  # ONE key per finger. Three rows come from three ACTIONS on it.
            t_p, t_m = tp_of(x, f), tm_of(x, f)
            curls[(f, k)] = (t_p, t_m, ab)
            q = posture(ref, f, t_p, t_m, ab)
            pos, _ = ref.pad_pose(q, f)
            # THE WELL'S AXIS IS THE PAD NORMAL -- the cup must face the pulp it cups.
            #
            # This REVERSES an earlier decision, and the reversal is the point. In the KEYCAP
            # era the axis was set to the direction the digit can PUSH, because a linear
            # switch registers travel along its own axis and force off that axis is wasted as
            # shear. Fine for a flat cap. WRONG FOR A WELL: a well is a CUP the pulp sits
            # INSIDE, so an axis 63 deg off the pad normal does not seat the fingertip at all
            # -- it jams it against the rim. And `action_dirs` already defines `click` as the
            # pad normal, so the two disagreed about the same well.
            #
            # The evidence for the old rule was that a pad-normal thumb key gave the
            # 95th-percentile hand EXACTLY ZERO press travel. Re-measured on the current
            # model, the thumb gets +35.0 mm along its pad normal against +9.8 mm along the
            # flexor push -- 3.6x MORE, not zero. That "zero" was an artefact of the thumb
            # SIGN bug (mp_flexion and ip_flexion flex NEGATIVE), which was found and fixed
            # AFTERWARDS. The workaround outlived the bug it was compensating for.
            _, n_pad = ref.pad_pose(q, f)
            keys[(f, k)] = (pos, n_pad)  # well axis = pad normal, pointing back at the pulp
    return keys, curls


def action_dirs(h: MyoHand, q: np.ndarray, finger: str) -> dict[str, np.ndarray]:
    """The FIVE directions a fingertip-in-a-cavity joystick senses, in the PAD's own frame.

    The cavity sits on a miniature thumbstick, so the axes are the stick's:

        n       the pad normal          -> CLICK, pressing into the cavity floor
        t_long  tangential, along the finger -> tilt FORWARD / BACK
        t_lat   tangential, across it       -> tilt LEFT / RIGHT

    t_long is built from the flexion arc's TANGENTIAL component, not from the arc itself:
    the flexion direction sits only 12-20 deg off the pad normal, so `flex` and `click` are
    very nearly the same action and cannot be told apart by a sensor. Taking the part of it
    perpendicular to n gives a genuinely independent axis -- which is what a joystick tilt is.

    Each direction is a force the digit must exert at the pad, so solve_activations() prices
    all five with no new machinery.
    """
    _, n = h.pad_pose(q, finger)
    n = n / (np.linalg.norm(n) + 1e-12)
    flex = h.press_dir_flexor(q, finger)
    t_long = flex - (flex @ n) * n  # the part of the flexion arc that SLIDES the pad
    nrm = np.linalg.norm(t_long)
    if nrm < 1e-6:  # degenerate: flexion is exactly along the normal
        t_long = np.cross(n, np.array([0.0, 0.0, 1.0]))
        nrm = np.linalg.norm(t_long)
    t_long = t_long / (nrm + 1e-12)
    t_lat = np.cross(n, t_long)
    return {
        "click": n, "forward": t_long, "back": -t_long,
        "left": t_lat, "right": -t_lat,
    }


def evaluate(x: dict, hands: dict[int, MyoHand], ref_pct: int = 50) -> dict:
    """One design: a strap-mounted body with an ADJUSTABLE finger well per digit, each a
    5-direction joystick (Svalboard/DataHand geometry), typing QWERTY.

    THE WELLS ADJUST. That is how a one-size device fits a population, and it is what the
    real device does ("individual 5-axis finger adjustability" -- svalboard.com). Every hand
    slides its wells to ITS OWN fingertip, at the SAME flexion fractions, so:

      * reach is ZERO by construction -- no posture solve, no press(), the loop gets much
        faster and much more honest;
      * what the population COSTS is no longer "can you reach it" but "how much adjustment
        range must be built in", which is the plan's Stage-5 question and now falls out as a
        number instead of being assumed. Measured: ~12 mm.

    DataHand's patents (early 1990s) have expired and Svalboard is an open implementation, so
    the 5-direction finger well is free prior art. Typeware's twin-key patent is not in the
    way: this is a different mechanism and an older one.
    """
    from design.qwerty import (ACTIONS, ROWS, Infeasible, best_action_map, cost_of,
                               used_actions)
    from opt.problem import CONSTRAINT_NAMES
    from structure.frame import hand_axes

    ref = hands[ref_pct]
    n_keys = {f: 1 for f in FINGERS}
    keys_ref, curls = keys_on_reference(ref, x)
    press_N = PRESS_N
    o_ref, e_d, e_r, e_o = hand_axes(ref, ref.q_neutral)
    ref_local = {f: keys_ref[(f, 0)][0] - o_ref for f in FINGERS}

    params = dict(
        sec_alu=(float(x["alu_w"]), float(x["alu_t"])),
        palm_offset=float(x["palm_offset"]),
        body_half=float(x["body_half"]),
        body_prox=BODY_PROX,
        body_dist=float(x["body_dist"]),
        stem=float(x["stem"]),
        mat_frame=str(x["material"]),
    )
    exo = build_body(ref, ref.q_neutral, keys_ref, params)
    st = solve(exo, [(f, 0) for f in FINGERS], press_N=press_N)

    worst_travel_deficit = -np.inf
    worst_saturation = -np.inf
    worst_residual = -np.inf   # CAN THE DIGIT ACTUALLY BALANCE THE KEY? see below
    worst_clear = np.inf
    worst_swept = np.inf
    required_adjust = 0.0
    per_hand: list = []

    for pct, h in hands.items():
        o_h, *_ = hand_axes(h, h.q_neutral)
        shift = o_h - o_ref

        effort, sat, trav, resid, postures = {}, {}, {}, {}, {}
        for f in FINGERS:
            ab = float(x.get(f"ab_{f}", 0.0))
            q = posture(h, f, tp_of(x, f), tm_of(x, f), ab)
            pos, _ = h.pad_pose(q, f)
            postures[(f, 0)] = q

            # how far this hand must SLIDE its well from where the median hand's sits
            required_adjust = max(
                required_adjust, float(np.linalg.norm((pos - o_h) - ref_local[f]))
            )

            dirs = action_dirs(h, q, f)
            for act in ACTIONS:
                d = dirs[act]
                # THE RESIDUAL WAS BEING THROWN AWAY. It was literally `_`, right here.
                #
                # solve_activations does NOT enforce equilibrium: it least-squares-fits the
                # required torque, pins the demand to the ACHIEVABLE torque tau* = A @ a_ls,
                # and returns the shortfall as `residual`. Nothing ever read it. So every
                # effort number in this project was the cheapest way to produce the CLOSEST
                # ACHIEVABLE torque -- not the torque that actually presses the key.
                #
                # Measured on the shipped design: the shortfall ran to 62% of the required
                # torque. The index was at 0.0% (it really can press); the thumb at 48%, the
                # ring at 62%. We were pricing presses the digit cannot perform.
                #
                # This is v1's disease one level deeper. v1 let the optimiser BUY its way out
                # of a soft constraint. Here the constraint was never checked at all.
                #
                # Verified it is the KEY and not the finger's own passive tension: at
                # press_N = 0 the residual is 0.0% for thumb/index/middle/ring. The digit can
                # hold the posture; it cannot hold the posture WHILE PRESSING.
                # THE WELL IS A CRADLE, NOT A PIN. `hand/cradle.py`.
                #
                # This used to apply the key reaction as a POINT FORCE AT THE PAD and demand
                # the digit's own muscles balance the whole resulting joint torque. On that
                # model an open hand "cannot press" -- 32-35% irreducible residual -- which is
                # contradicted by billions of people typing on flat keyboards every day.
                #
                # A well CRADLES the distal phalanx, so the reaction bears on the whole palmar
                # surface and the CENTRE OF PRESSURE is free to sit anywhere along the bone. A
                # reaction near the DIP has a far smaller moment arm about it than the same
                # force at the fingertip. That is the finger-as-a-strut, and it is the only
                # thing the well contributes -- it lends no muscle. The THUMB is the control:
                # it still cannot press, because it still has no adductor.
                a, e, r, smax = cradle_solve(h, q, f, act, press_N)
                effort[(f, act)] = float(e)
                sat[(f, act)] = float(smax)
                trav[(f, act)] = h.travel_along(q, f, d)
                resid[(f, act)] = float(r)

        per_hand.append((effort, sat, trav, resid))

        q_on = h.compose({f: postures[(f, 0)] for f in FINGERS})
        for q_chk in (q_on, h.q_neutral):
            gaps = clearance(h, q_chk, exo, offset=shift, only=DIGIT_FLESH, bone=True)
            worst_clear = min(worst_clear, min(gaps.values()))
        worst_swept = min(
            worst_swept, swept_path_clearance(h, keys_ref, curls, n_keys, shift)
        )

    # ONE MAPPING for the whole population -- it is firmware burned into the device, and
    # choosing it per hand hands every user a different keyboard.
    #
    # AN ACTION IS AVAILABLE ONLY IF EVERY HAND CAN ACTUALLY PERFORM IT. Choosing by effort
    # alone picked the IMPOSSIBLE actions, because an action the muscles cannot balance
    # produces a small achievable torque, hence small activations, hence LOW effort. The
    # cheapest number in the entire model (middle/click, 4e-08) was an unbalanced press.
    mean_effort = {k: float(np.mean([ph[0][k] for ph in per_hand])) for k in per_hand[0][0]}
    availability = {
        k: max(ph[3][k] for ph in per_hand) <= RESIDUAL_MAX for k in per_hand[0][3]
    }
    # PERFORMABILITY, AS A CONTINUOUS MARGIN -- NOT A COUNT.
    #
    # It was "how many of the 3 needed directions is this finger short of?", an INTEGER. So
    # the constraint was a STAIRCASE, and NSGA-II's constraint violation flatlined at exactly
    # 1.0000 for 72 generations: the GA could not tell a finger that ALMOST has a third
    # working direction from one that is hopeless. A cliff, not a gradient -- the exact thing
    # this project keeps warning itself about, and I walked into it anyway.
    #
    # The fix: a finger needs its THIRD-BEST direction to be performable (three rows, three
    # directions). So sort its five directions by residual and constrain the third. That is
    # the SAME condition, but it is now a smooth, signed margin the optimiser can descend.
    #
    # The thumb is excluded on purpose: QWERTY's left half carries no letters on it (15
    # letters; 4 fingers x 3 rows + the index's 2nd column = exactly 15). Which is just as
    # well, because it cannot press -- 11.9% irreducible residual against <=0.6% for every
    # finger, even after adding adductor pollicis. See tests/test_physics.py.
    worst_resid = {
        (f, a): max(ph[3][(f, a)] for ph in per_hand)   # worst over the population
        for f in FINGERS for a in ACTIONS
    }
    margin = -np.inf
    for f in FINGERS:
        if f == "thumb":
            continue
        third = sorted(worst_resid[(f, a)] for a in ACTIONS)[len(ROWS) - 1]
        margin = max(margin, third - RESIDUAL_MAX)
    availability = {k: v <= RESIDUAL_MAX for k, v in worst_resid.items()}

    try:
        action_map, _ = best_action_map(mean_effort, availability)
    except Infeasible:
        # Still produce a mapping so every other objective stays computable and the design is
        # SCORED rather than special-cased -- `margin` is positive and the constrained
        # tournament treats it as infeasible.
        action_map, _ = best_action_map(mean_effort)
    used = used_actions(action_map)

    per_hand_char_effort = []
    for effort, sat, trav, resid in per_hand:
        per_hand_char_effort.append(cost_of(action_map, effort))
        # Constrain ONLY the wired directions. 25 exist, 15 are used; demanding all five work
        # would drag the design to the cost of the worst -- index/left is near saturation.
        for f, acts in used.items():
            for act in acts:
                worst_saturation = max(worst_saturation, sat[(f, act)] - SATURATION)
                worst_travel_deficit = max(worst_travel_deficit, SWITCH_TRAVEL - trav[(f, act)])
                worst_residual = max(worst_residual, resid[(f, act)] - RESIDUAL_MAX)
                # note: when `shortfall` is 0 every wired action is performable BY
                # CONSTRUCTION (best_action_map only chose from available ones), so this is
                # a live self-check rather than the binding constraint. `shortfall` is.

    adjust = float(x["adjust"])
    # ⚠ PLACEHOLDER: the adjuster mass model is a guess (a slide + lock per finger).
    adj_mass = 5.0 * (2.0 + 0.15 * adjust * 1000.0)  # g

    f1 = float(np.mean(per_hand_char_effort))  # effort/char typing English QWERTY
    f2 = exo.mass() * 1000.0 + adj_mass  # g
    f3 = st["max_deflection"] * 1000.0  # mm

    # Structurally <= COMMON_DRIVE now (see REAL_BOUNDS). Kept in G as a live
    # self-check: if anyone reintroduces independent curls, this fires.
    four = [f for f in FINGERS if f != "thumb"]
    tm4 = np.array([tm_of(x, f) for f in four])
    tp4 = np.array([tp_of(x, f) for f in four])
    spread = max(float(tm4.max() - tm4.min()), float(tp4.max() - tp4.min()))
    sep_violation, sep_pair = key_separation(keys_ref, ref, curls)
    well_finger_gap, wf_pair = well_finger_clearance(ref, x)

    g = [
        worst_travel_deficit,       # every WIRED direction usable, on every hand
        worst_saturation,           # no muscle maxed out in any wired direction
        required_adjust - adjust,   # the wells must actually reach: STAGE 5, as a constraint
        -worst_clear,               # body clears the finger BONES of every hand
        st["max_util"] - 1.0,
        f3 - 0.5,
        spread - float(COMMON_DRIVE),  # common drive: built in, and re-checked here
        sep_violation,              # the five wells must physically fit
        -worst_swept,
        -well_finger_gap,           # a well is SOLID: no other digit's bones inside it
        float(margin),              # THE DIGIT MUST ACTUALLY BE ABLE TO PERFORM THE ACTION
    ]

    return dict(
        F=[f1, f2, f3], G=g,
        n_keys=n_keys, total_keys=5,
        keys_ref=keys_ref, curls=curls, exo=exo, struct=st, press_N=press_N,
        key_sep=sep_violation, key_sep_pair=sep_pair, swept=worst_swept,
        action_map=action_map, required_adjust=required_adjust, adjust=adjust,
        char_effort_by_hand=per_hand_char_effort,
        feasible=all(v <= 0 for v in g),
    )

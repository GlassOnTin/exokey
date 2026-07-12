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

from hand.myohand import FINGERS, FLEXION_JOINTS, MyoHand
from structure.frame import DIGIT_FLESH, build_body, clearance, solve

# Switch travel and cap geometry. These describe a LOW-PROFILE wearable switch, and they
# must be consistent with PRESS_N below -- they were not, and it cost us the 10-key designs.
#
# SWITCH_TRAVEL was 3 mm, quoting "a mechanical switch actuates at ~2 mm". That is a
# Cherry-MX-style full-travel key. But the actuation force is 0.30 N, which is BELOW Cherry
# MX (0.45-0.60 N) and squarely in dome/scissor territory -- and a scissor switch (a laptop
# key) travels ~1.5 mm. The two numbers described different switches. Reference point:
# Typeware's wearable keyboard fits 2 keys per finger, which this model called infeasible.
SWITCH_TRAVEL = 0.0015  # m; "a few mm for any keypress" (Svalboard, magneto-optical)
CAP_HEIGHT = 0.003  # m; how far the cap stands proud of its plate
REACH_TOL = 0.005  # m
SATURATION = 0.95
N_CHARS = 30  # characters to encode: the 30 cheapest chords carry the alphabet

# A keycap is a PHYSICAL OBJECT and two of them cannot occupy the same space. Nothing in
# the model said so until this was added, and measured, EVERY design violated it: with one
# key per finger the middle and ring keys came out 8.5 mm apart (curled fingertips
# converge) against a ~14 mm cap, and thumb rows landed 3.5 mm apart. The whole Pareto
# front was geometrically unbuildable and nothing flagged it.
KEY_PITCH = 0.012  # m; minimum spacing between keys on DIFFERENT fingers (separate stems)

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
PRESS_N = 0.196  # N = 20 gf, the Svalboard's magneto-optical keys (svalboard.com).
# Not a guess: their spec. Light, front-loaded force profile, no spring, ~few mm of travel --
# which is also what makes SWITCH_TRAVEL = 1.5 mm the consistent figure.

MATERIAL_CHOICES = ["al6061", "al7075", "cf_pa12"]

# Continuous variables: name -> (lo, hi). Per-finger curl, splay, switch force, structure.
REAL_BOUNDS: dict[str, tuple[float, float]] = {}
for _f in FINGERS:
    REAL_BOUNDS[f"tp_{_f}"] = (0.10, 0.80)  # proximal-joint flexion fraction
    REAL_BOUNDS[f"tm_{_f}"] = (0.10, 0.80)  # middle-joint flexion fraction
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
REAL_BOUNDS["stem"] = (0.004, 0.014)      # key stand-off from the body face
REAL_BOUNDS["body_half"] = (0.016, 0.030)  # body half-width: a ONE-SIZE body must fit the
REAL_BOUNDS["body_prox"] = (0.008, 0.030)  # 5th percentile too, so its SIZE has to be free
REAL_BOUNDS["body_dist"] = (0.040, 0.070)

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


def key_separation(keys: dict) -> tuple[float, tuple | None]:
    """Worst key-pair spacing, as a VIOLATION (required pitch - actual gap; <= 0 is fine).

    Two pitches, because there are two kinds of pair:
      * different fingers  -> KEY_PITCH: separate stems, separate caps, they must not touch;
      * same finger        -> TWIN_KEY_PITCH: a TWIN KEY, two switches sharing one stem, so
                              they are allowed to sit much closer. Treating a finger's own
                              two keys as independent colliding caps is simply wrong, and it
                              is what made 2-keys-per-finger look impossible.
    """
    items = list(keys.items())
    worst, pair = -np.inf, None
    for i in range(len(items)):
        for j in range(i + 1, len(items)):
            (fa, _), (fb, _) = items[i][0], items[j][0]
            need = TWIN_KEY_PITCH if (fa == fb and ALLOW_TWIN_KEY) else KEY_PITCH
            v = need - float(np.linalg.norm(items[i][1][0] - items[j][1][0]))
            if v > worst:
                worst, pair = v, (items[i][0], items[j][0])
    return worst, pair


def swept_path_clearance(h: MyoHand, keys: dict, curls: dict, n_keys: dict, shift, n_samples: int = 7) -> float:
    """Does a curling finger sweep through ANOTHER finger's keycap on its way down?

    The static clearance check cannot see this: it looks at two postures, and the collision
    happens BETWEEN them. Here each finger is swept from nearly straight to its deepest key,
    and its phalanx flesh is checked against every OTHER digit's caps (its own caps it is
    supposed to touch -- that is what a key is for).

    The cap is an obstacle of height CAP_HEIGHT, NOT a sphere of radius KEY_PITCH/2. That
    was the bug: a low-profile keycap is WIDE AND FLAT (~13 mm across, ~3 mm tall), so for a
    finger passing OVER it the obstacle is its HEIGHT. Using half its width doubled the
    apparent obstacle and made every 2-key-per-finger layout collide. KEY_PITCH is for
    PACKING keys side by side; it is the wrong number for a clearance question.

    Returns the signed minimum gap; negative means a finger drives through a neighbour's key.
    """
    import mujoco

    m = h.model
    other_caps = {}
    for f in FINGERS:
        other_caps[f] = np.array(
            [keys[(g, k)][0] + shift for g in FINGERS if g != f for k in range(n_keys[g])]
        )

    worst = np.inf
    for f in FINGERS:
        caps = other_caps[f]
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
                    d = np.linalg.norm(caps - closest, axis=1) - r - CAP_HEIGHT
                    worst = min(worst, float(d.min()))
    return worst


_DIGIT_BODIES = {
    "thumb": ("firstmc", "proximal_thumb", "distal_thumb"),
    "index": ("proxph2", "midph2", "distph2"),
    "middle": ("proxph3", "midph3", "distph3"),
    "ring": ("proxph4", "midph4", "distph4"),
    "little": ("proxph5", "midph5", "distph5"),
}


def keys_on_reference(ref: MyoHand, x: dict) -> tuple[dict, dict]:
    """(finger, row) -> (world pos, outward normal) on the REFERENCE hand, plus the curls
    that generated them (needed by the swept-path check).

    Rows are CENTRED on the finger's base curl, not stacked ever-deeper from it: row k sits
    at `tm + (k - (n-1)/2)*dc`, so with three rows the finger EXTENDS to reach one, sits at
    rest for the middle one, and CURLS for the last. That is how a chorder actually works.

    Stacking rows only deeper (tm, tm+dc, tm+2dc) fails on two constraints at once: the far
    row runs out of press travel (it is jammed against the flexion limit) AND the rows stay
    too close together to fit a keycap between them (measured: 10.6 mm against a 12 mm
    pitch, even at maximum dc).
    """
    keys, curls = {}, {}
    for f in FINGERS:
        ab = float(x.get(f"ab_{f}", 0.0))
        for k in (0,):  # ONE key per finger. Three rows come from three ACTIONS on it.
            t_p, t_m = x[f"tp_{f}"], x[f"tm_{f}"]
            curls[(f, k)] = (t_p, t_m, ab)
            q = posture(ref, f, t_p, t_m, ab)
            pos, _ = ref.pad_pose(q, f)
            # THE KEY'S AXIS IS THE DIRECTION THE DIGIT CAN PUSH, not the way its pad faces.
            #
            # A switch registers travel along its own axis; force off that axis is wasted as
            # shear. For the four fingers the two agree to 12-20 deg and it makes no odds.
            # For the THUMB they are 82 deg apart -- it has no adductor pollicis in this
            # model, so FPL alone cannot push squarely out of the thumb pulp. Orienting the
            # thumb key to its pad normal asks it to push in a direction it cannot, and the
            # 95th-percentile hand came out with EXACTLY ZERO press travel on that key.
            #
            # Consequence, stated not hidden: the thumb's pad meets its key at ~80 deg of
            # obliquity. A real build would need a contoured or angled thumb cap. That is a
            # symptom of the missing thenar muscles, not of the layout.
            press = ref.press_dir_flexor(q, f)
            keys[(f, k)] = (pos, -press)  # key normal points back at the digit
    return keys, curls


def _chord_efforts(effort: dict, n_keys: dict) -> list[float]:
    """Cost of every physically valid chord: at most ONE key per finger.

    A finger cannot press two of its own keys at once -- that constraint is what makes more
    keys per finger worth having, and leaving it out would let the model invent chords no
    hand can make.

    Chord cost = sum of its keys' efforts. That is the plan's Tier-1 model and it IGNORES
    inter-finger coupling; MyoHand has none to give (its FDP2..FDP5 are independent), so a
    coupled chord cost needs a different hand model. Stated, not hidden.
    """
    per_finger = [
        [None] + [(f, k) for k in range(n_keys[f])]  # None = this finger presses nothing
        for f in FINGERS
    ]
    out = []
    for combo in itertools.product(*per_finger):
        pressed = [c for c in combo if c is not None]
        if pressed:
            out.append(sum(effort[c] for c in pressed))
    return out


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
    from design.qwerty import ACTIONS, best_action_map, cost_of, used_actions
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
        body_prox=float(x["body_prox"]),
        body_dist=float(x["body_dist"]),
        stem=float(x["stem"]),
        mat_frame=str(x["material"]),
    )
    exo = build_body(ref, ref.q_neutral, keys_ref, params)
    st = solve(exo, [(f, 0) for f in FINGERS], press_N=press_N)

    worst_travel_deficit = -np.inf
    worst_saturation = -np.inf
    worst_clear = np.inf
    worst_swept = np.inf
    required_adjust = 0.0
    per_hand: list = []

    for pct, h in hands.items():
        o_h, *_ = hand_axes(h, h.q_neutral)
        shift = o_h - o_ref

        effort, sat, trav, postures = {}, {}, {}, {}
        for f in FINGERS:
            ab = float(x.get(f"ab_{f}", 0.0))
            q = posture(h, f, x[f"tp_{f}"], x[f"tm_{f}"], ab)
            pos, _ = h.pad_pose(q, f)
            postures[(f, 0)] = q

            # how far this hand must SLIDE its well from where the median hand's sits
            required_adjust = max(
                required_adjust, float(np.linalg.norm((pos - o_h) - ref_local[f]))
            )

            dirs = action_dirs(h, q, f)
            for act in ACTIONS:
                d = dirs[act]
                a, e, _, _, _ = h.solve_activations(q, f, press_N, -d)
                effort[(f, act)] = float(e)
                sat[(f, act)] = float(a.max())
                trav[(f, act)] = h.travel_along(q, f, d)

        per_hand.append((effort, sat, trav))

        q_on = h.compose({f: postures[(f, 0)] for f in FINGERS})
        for q_chk in (q_on, h.q_neutral):
            gaps = clearance(h, q_chk, exo, offset=shift, only=DIGIT_FLESH, bone=True)
            worst_clear = min(worst_clear, min(gaps.values()))
        worst_swept = min(
            worst_swept, swept_path_clearance(h, keys_ref, curls, n_keys, shift)
        )

    # ONE MAPPING for the whole population -- it is firmware burned into the device, and
    # choosing it per hand hands every user a different keyboard.
    mean_effort = {k: float(np.mean([ph[0][k] for ph in per_hand])) for k in per_hand[0][0]}
    action_map, _ = best_action_map(mean_effort)
    used = used_actions(action_map)

    per_hand_char_effort = []
    for effort, sat, trav in per_hand:
        per_hand_char_effort.append(cost_of(action_map, effort))
        # Constrain ONLY the wired directions. 25 exist, 15 are used; demanding all five work
        # would drag the design to the cost of the worst -- index/left is near saturation.
        for f, acts in used.items():
            for act in acts:
                worst_saturation = max(worst_saturation, sat[(f, act)] - SATURATION)
                worst_travel_deficit = max(worst_travel_deficit, SWITCH_TRAVEL - trav[(f, act)])

    adjust = float(x["adjust"])
    # ⚠ PLACEHOLDER: the adjuster mass model is a guess (a slide + lock per finger).
    adj_mass = 5.0 * (2.0 + 0.15 * adjust * 1000.0)  # g

    f1 = float(np.mean(per_hand_char_effort))  # effort/char typing English QWERTY
    f2 = exo.mass() * 1000.0 + adj_mass  # g
    f3 = st["max_deflection"] * 1000.0  # mm

    tm4 = np.array([x[f"tm_{f}"] for f in FINGERS if f != "thumb"])
    spread = float(tm4.max() - tm4.min())
    sep_violation, sep_pair = key_separation(keys_ref)

    g = [
        worst_travel_deficit,       # every WIRED direction usable, on every hand
        worst_saturation,           # no muscle maxed out in any wired direction
        required_adjust - adjust,   # the wells must actually reach: STAGE 5, as a constraint
        -worst_clear,               # body clears the finger BONES of every hand
        st["max_util"] - 1.0,
        f3 - 0.5,
        spread - 0.15,              # common drive (MyoHand has no enslavement of its own)
        sep_violation,              # the five wells must physically fit
        -worst_swept,
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

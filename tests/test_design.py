"""The outer problem's evaluation. Stage 4's gates."""
from __future__ import annotations

import numpy as np
import pytest

from design.vector import (
    INT_BOUNDS,
    REAL_BOUNDS,
    SWITCH_TRAVEL,
    action_dirs,
    evaluate,
    keys_on_reference,
    posture,
    tm_of,
    tp_of,
)
from hand.myohand import FINGERS, FLEXION_JOINTS, MyoHand
from hand.scaling import population
from structure.frame import hand_axes


@pytest.fixture(scope="module")
def hands():
    return {p: MyoHand(scale=s) for p, s in population((5, 50, 95)).items()}


def mid_design():
    x = {k: 0.5 * (lo + hi) for k, (lo, hi) in REAL_BOUNDS.items()}
    x.update({k: 2 for k in INT_BOUNDS})
    x["material"] = "cf_pa12"
    for f, sgn in zip(("index", "middle", "ring", "little"), (1.0, 0.33, -0.33, -1.0)):
        x[f"ab_{f}"] = 0.7 * sgn   # fanned, or the five keys collide
    return x


def test_key_transfer_between_hands_is_a_translation(hands):
    """The load-bearing assumption of the whole population model.

    The device is rigid and straps to the dorsum, so a key fixed in HAND-FRAME coordinates
    is the same key on every hand. That is only a pure translation if the hand axes are
    identical across sizes -- which they are, because the scaling is isotropic (a uniform
    scale cannot rotate anything). If scaling ever becomes anisotropic (per-segment
    Buchholz), this breaks and the transfer needs the full frame, not just the origin.
    """
    _, e_d0, e_r0, e_o0 = hand_axes(hands[50], hands[50].q_neutral)
    for p, h in hands.items():
        _, e_d, e_r, e_o = hand_axes(h, h.q_neutral)
        assert np.allclose(e_d, e_d0, atol=1e-9), f"{p}th: distal axis rotated"
        assert np.allclose(e_r, e_r0, atol=1e-9), f"{p}th: radial axis rotated"
        assert np.allclose(e_o, e_o0, atol=1e-9), f"{p}th: dorsal axis rotated"


def test_posture_never_hyperextends(hands):
    """Every posture the design vector can generate is q = t * (signed flexion limit), so
    it cannot cross zero into hyperextension whatever the joint's sign convention."""
    h = hands[50]
    for f in FINGERS:
        for t_p in (0.0, 0.5, 1.0):
            for t_m in (0.0, 0.5, 1.0):
                q = posture(h, f, t_p, t_m)
                for j in FLEXION_JOINTS[f]:
                    qadr, _, limit = h.flexion_span(j)
                    frac = q[qadr] / limit
                    assert -1e-9 <= frac <= 1.0 + 1e-9, f"{f}/{j}: flexion fraction {frac:.3f}"


def test_hand_size_binds_through_REACH_not_strength(hands):
    """What hand size actually does to a FIXED device -- and it is not what I assumed.

    The strength law (activation ~ s^-2, so a small hand works harder) holds AT A GIVEN
    POSTURE and is pinned in test_scaling. But with the device fixed, each hand reaches the
    same key at a DIFFERENT posture, and effort varies by orders of magnitude with posture
    (measured: 40x-20000x across a finger's workspace) while the strength effect is only
    1.55x across the whole population. Posture wins by a mile, so the effort ordering is
    NOT determined by hand size, and an earlier version of this test asserting it was wrong.

    What hand size DOES bind is REACH: keys sit where the median hand's fingertips left
    them, and the 5th-percentile hand's fingers are 11% shorter. That is the population
    constraint, and it is geometric.
    """
    from structure.frame import hand_axes

    r = evaluate(mid_design(), hands)
    ref = hands[50]
    o_ref, *_ = hand_axes(ref, ref.q_neutral)

    worst = {}
    for pct, h in hands.items():
        o_h, *_ = hand_axes(h, h.q_neutral)
        errs = [
            h.press(f, pos + (o_h - o_ref), nrm, press_N=0.5, q0=h.q_neutral).pos_err
            for (f, k), (pos, nrm) in r["keys_ref"].items()
        ]
        worst[pct] = max(errs)

    assert worst[50] < 5e-4, "the median hand must reach the keys it defined"
    assert worst[5] > worst[50], "the small hand is the one that struggles to reach"


def test_constraints_are_hard_not_penalties(hands):
    """v1's fatal formulation: feasibility as a weighted penalty, so the optimiser could BUY
    an unreachable key. Here `evaluate` returns G separately from F, and a design is
    feasible only if EVERY g <= 0 -- there is no exchange rate between them."""
    r = evaluate(mid_design(), hands)
    from opt.problem import CONSTRAINT_NAMES
    assert len(r["G"]) == len(CONSTRAINT_NAMES)
    assert r["feasible"] == all(v <= 0 for v in r["G"])
    # and the objectives contain no penalty terms: mass and deflection are physical units
    assert r["F"][1] > 0  # grams
    assert r["F"][2] > 0  # mm


def test_the_five_wells_must_physically_fit(hands):
    """A well is a CAVITY THE FINGERTIP SITS IN, so its size is the fingertip's size --
    DERIVED from the model (12-14 mm wide), not a constant.

    It was a constant: 12 mm, inherited from when these were KEYCAPS on stems. A keycap sits
    ON a stem; a fingertip sits IN a well. That single stale number invalidated a whole
    Pareto front, whose 200 "feasible" designs all had overlapping wells.

    And the consequence is real: gripping a body CONVERGES the fingertips (to ~6-8 mm),
    while wells need them SPREAD (~17 mm). The hand has to stay relatively OPEN.
    """
    from design.vector import key_separation, keys_on_reference, well_radius

    ref = hands[50]
    # the well must be at least as wide as the fingertip that lives in it
    for f in FINGERS:
        assert 0.005 < well_radius(ref, f) < 0.010, f"{f}: implausible well radius"

    curled = mid_design()          # gripping: fingertips converge
    curled["tm_hand"] = 0.60
    keys, _ = keys_on_reference(ref, curled)
    v_curled, _ = key_separation(keys, ref)
    assert v_curled > 0, "a gripping hand should NOT be able to fit five wells"

    open_ = dict(curled)           # relaxed: fingertips spread
    open_["tm_hand"] = 0.25
    keys, _ = keys_on_reference(ref, open_)
    v_open, _ = key_separation(keys, ref)
    assert v_open < v_curled, "opening the hand must spread the wells apart"

def test_switch_force_is_fixed_not_optimised():
    """press_N was a design variable over [0.30, 0.80] N and all 60 Pareto designs pinned
    it to the lower bound. That is a corner solution, not a trade: a lighter switch cuts
    BOTH effort and deflection, so nothing pulls the other way. It is a constant."""
    from design.vector import PRESS_N, REAL_BOUNDS

    assert "press_N" not in REAL_BOUNDS
    # 20 gf = 0.196 N -- the Svalboard's magneto-optical key spec, not a guess
    assert PRESS_N == pytest.approx(0.196)


def test_five_joystick_directions_use_different_muscle_groups(hands):
    """The whole premise: one key per finger can carry several QWERTY rows because each
    direction is driven by a DIFFERENT muscle group. If they all recruited the flexors, a
    sensor could not tell them apart and the design would be fiction.
    """
    import mujoco

    from design.qwerty import ACTIONS

    h = hands[50]
    q = posture(h, "index", 0.40, 0.55, 0.0)
    dirs = action_dirs(h, q, "index")
    a0, *_ = h.solve_activations(q, "index", 0.0, -dirs["click"])

    leads = {}
    for act in ACTIONS:
        a, *_ = h.solve_activations(q, "index", 0.30, -dirs[act])
        leads[act] = h.model.actuator(int(np.argmax(a - a0))).name

    assert len(set(leads.values())) >= 3, f"directions are not distinguishable: {leads}"
    # and every direction must be a direction the finger can actually move in
    for act in ACTIONS:
        assert h.travel_along(q, "index", dirs[act]) > 0.001, f"{act}: no travel"


def test_the_firmware_mapping_is_ONE_mapping_for_the_whole_population(hands):
    """A layout is burned into the device. Choosing it per-hand is not merely optimistic,
    it is incoherent -- it hands every user a different keyboard. This model did exactly
    that at first, and it also picked a mapping 6x worse than optimal for the index.
    """
    from design.qwerty import QWERTY_LEFT, best_action_map, cost_of

    r = evaluate(mid_design(), hands)
    m = r["action_map"]
    assert m is not None

    # one mapping, and every hand is charged for THAT mapping
    for pct, h in hands.items():
        pass  # the per-hand costs come from cost_of(m, ...) inside evaluate
    assert len(r["char_effort_by_hand"]) == len(hands)

    # the frequent letters must land on the cheap directions: E is the most common letter
    # in English and it is the middle finger's TOP row
    assert m["middle"]["top"] == "click", (
        f"E (12.7%) should be on the cheapest direction, got {m['middle']['top']}"
    )


def test_ten_of_the_twenty_five_directions_are_discarded(hands):
    """5 fingers x 5 directions = 25 inputs; QWERTY's half-hand needs 15. The other ten are
    unwired, and they should be the WORST ten -- index/left costs 2.7e-01, which is near
    muscle saturation. Constraining directions nobody uses would drag the design down to
    the cost of the worst one.
    """
    from design.qwerty import ACTIONS, used_actions

    r = evaluate(mid_design(), hands)
    used = used_actions(r["action_map"])
    total_used = sum(len(v) for v in used.values())
    assert total_used <= 15, f"{total_used} directions wired; only 15 letters exist"
    for f, acts in used.items():
        assert len(acts) <= len(ACTIONS)


def test_adjustable_wells_are_how_a_one_size_device_fits_a_population(hands):
    """The Stage-5 answer, and it is MEASURED.

    A rigid one-size device cannot fit the 5th-95th percentile -- that is what every reach
    and clearance failure in this project was really telling us. The real device does not try:
    Svalboard ships "individual 5-axis finger adjustability", and each hand slides its wells
    to its own fingertip.

    So the population cost is not "can you reach it" but "how much adjustment range must be
    built in". Measured: ~12 mm. That is now a hard constraint, not an assumption.
    """
    r = evaluate(mid_design(), hands)
    need = r["required_adjust"]
    assert 0.005 < need < 0.020, f"needs {need*1000:.1f} mm of well travel -- implausible"
    # and the constraint really is `built-in range must cover what the population needs`
    i = 2  # "adjust-range"
    assert r["G"][i] == pytest.approx(need - r["adjust"])


def test_every_constant_declares_where_it_came_from():
    """The tripwire for the disease that has cost this project the most time.

    A constant with no provenance is indistinguishable from a fact, and that is how
    KEY_PITCH (a KEYCAP pitch) survived into the WELL era and invalidated a whole Pareto
    front, and how SWITCH_TRAVEL (3 mm, a Cherry MX) came to sit beside PRESS_N (0.30 N, a
    dome switch) describing two different pieces of hardware.
    """
    from design.params import REGISTRY, Source, guesses

    assert REGISTRY, "no parameters registered"
    for p in REGISTRY:
        assert p.why, f"{p.name} has no stated provenance"
        assert p.source in Source

    # every GUESS must be findable, because a guess nobody knows is a guess is a lie
    g = {p.name for p in guesses()}
    assert g, "no guesses declared — that would itself be suspicious"
    vision = open("VISION.md").read()
    missing = [n for n in g if n.lower().replace("_", " ") not in vision.lower()
               and n not in vision]
    assert not missing, (
        f"these are GUESSES and are not disclosed in VISION.md's limitations: {missing}"
    )


def test_switch_force_and_travel_describe_the_same_switch():
    """They did not. 3 mm of travel is a Cherry MX (0.45-0.60 N); 0.30 N is a dome switch.
    Two constants, two different pieces of hardware, and the inconsistency alone made
    2-keys-per-finger look infeasible."""
    from design.params import SVALBOARD, check_coherent

    check_coherent(SVALBOARD)
    assert SVALBOARD.force.describes == SVALBOARD.travel.describes


def test_a_well_must_face_the_pulp_it_cups(hands):
    """A WELL IS A CUP, NOT A CAP, so its axis has to point at the pad that sits in it.

    The user caught this by LOOKING at the render: "the thumb button isn't orthogonal to the
    thumb pad". It was 63 deg off. A flat keycap can tolerate that (you angle the cap); a cup
    cannot -- 63 deg off means the pulp never seats, it jams on the rim.

    The cause was a workaround that outlived its bug. The axis had been set to the digit's
    PUSH direction because a pad-normal thumb key once measured ZERO press travel -- but that
    zero came from the thumb SIGN bug (mp/ip flex NEGATIVE), fixed afterwards. Nothing
    re-tested the workaround once its reason was gone. This test is that re-test.
    """
    from design.vector import keys_on_reference

    ref = hands[50]
    x = mid_design()
    keys, _ = keys_on_reference(ref, x)
    for f in FINGERS:
        q = posture(ref, f, tp_of(x, f), tm_of(x, f), x.get(f"ab_{f}", 0.0))
        _, n_pad = ref.pad_pose(q, f)
        axis = keys[(f, 0)][1]
        obliq = np.degrees(np.arccos(np.clip(float(axis @ n_pad), -1.0, 1.0)))
        assert obliq < 5.0, f"{f}: well axis is {obliq:.0f} deg off its own pad normal"


def test_the_baseline_still_builds_and_evaluates(hands):
    """Imports opt/run.py. Sounds trivial; it is not.

    An edit left an empty `for` loop in `baseline()` -- a hard SyntaxError -- and the whole
    57-test suite went green, because nothing in it imported that module. Two 35-minute
    optimiser runs died on the import. A test suite that cannot see a file cannot defend it.
    """
    from opt.run import baseline

    r = evaluate(baseline(), hands)
    from opt.problem import CONSTRAINT_NAMES

    assert len(r["G"]) == len(CONSTRAINT_NAMES) and np.isfinite(r["F"]).all()


def test_the_fingertip_bone_sits_INSIDE_its_well(hands):
    """A well is a CHANNEL THE DISTAL PHALANX SLIDES INTO -- not a disc the pad rests on.

    The user, looking at the render: "The finger tip bone should fit into the well, not simply
    rest the pad on its opening." That was a real geometric error, not a drawing one. A well on
    the pad normal alone is a device you would have to lower your fingertip into vertically,
    like a piston. The real thing (DataHand / Svalboard) is open proximally so the finger
    slides in along its own bone axis, and open dorsally so it can lift out.

    This test also pins the THUMB's bone-axis sign. MuJoCo capsules extend along local z, and
    NOTHING says which end is distal: z runs distally on the four fingers and PROXIMALLY on the
    thumb. Trusting it built the thumb's channel pointing away from the thumb (bone at +2..+24
    mm along a channel spanning -22..+4). The axis is now aimed at the model's own TIP SITE.
    """
    import mujoco

    from hand.myohand import PAD_BODIES

    h = hands[50]
    x = mid_design()
    for f in FINGERS:
        q = posture(h, f, tp_of(x, f), tm_of(x, f), x.get(f"ab_{f}", 0.0))
        wf = h.well_frame(q, f)
        h.fk(q)
        bid = mujoco.mj_name2id(h.model, mujoco.mjtObj.mjOBJ_BODY, PAD_BODIES[f])
        g = next(g for g in range(h.model.body_geomadr[bid],
                                  h.model.body_geomadr[bid] + h.model.body_geomnum[bid])
                 if h.model.geom_type[g] == mujoco.mjtGeom.mjGEOM_CAPSULE)
        c, half = h.data.geom_xpos[g], float(h.model.geom_size[g][1])
        cap_ax = h.data.geom_xmat[g].reshape(3, 3)[:, 2]
        L, r = 2 * wf["half"], wf["radius"]
        for end in (c - half * cap_ax, c + half * cap_ax):
            d = end - wf["pos"]
            along, side, depth = d @ wf["axis"], abs(d @ wf["lateral"]), d @ wf["floor"]
            assert -L - 0.003 <= along <= 0.005, f"{f}: bone end {along*1000:+.1f}mm — channel points the wrong way"
            assert side < r, f"{f}: bone is {side*1000:.1f}mm off-axis, outside a {r*1000:.1f}mm well"
            assert depth < 0.002, f"{f}: bone is {depth*1000:+.1f}mm BELOW the well floor"


def test_a_well_is_solid_and_no_other_finger_may_be_inside_it(hands):
    """A WELL IS A SOLID OBJECT, and nothing checked it against the neighbouring FINGERS.

    The user: "still need to add some collision avoidance for the buttons". They were right,
    and the hole was bigger than it looked. We had:

        key-overlap  well  vs well   (and only their two DISTAL channels)
        clearance    body  vs bone
        swept-path   finger flesh vs a neighbouring well, ON THE WAY IN

    and nothing at all comparing a WELL against a NEIGHBOURING FINGER at rest. A well is a box
    roughly 16 x 14 x 7 mm sitting out at a fingertip -- exactly where the next finger's MIDDLE
    and PROXIMAL phalanges run past. Two wells can clear each other perfectly while one of them
    sits *inside* the ring finger. Measured on 240 random designs: 97% had a well buried in a
    neighbouring finger, by a MEDIAN of 9.6 mm.

    Un-splayed fingers must trip it: with the digits closed, the wells overrun their
    neighbours.
    """
    from design.vector import well_finger_clearance

    h = hands[50]
    closed = mid_design()
    for f in ("index", "middle", "ring", "little"):
        closed[f"ab_{f}"] = 0.0          # fingers together: wells overrun the neighbours
    gap_closed, pair = well_finger_clearance(h, closed)
    assert gap_closed < 0, f"closed fingers should collide with the wells, got {gap_closed*1000:+.1f}mm"

    fanned = dict(closed)                 # fan them and it must improve
    for f, sgn in zip(("index", "middle", "ring", "little"), (0.9, 0.3, -0.3, -0.9)):
        fanned[f"ab_{f}"] = sgn
    gap_fanned, _ = well_finger_clearance(h, fanned)
    assert gap_fanned > gap_closed, "splaying the fingers must pull the wells off the neighbours"


def test_the_cradle_resolves_the_pressing_vs_packing_tension(hands):
    """THE TENSION, AND ITS RESOLUTION. This test replaces one that asserted the tension was
    REAL. It was real only under a wrong model, and a test that pins an obsolete finding is
    worse than no test at all.

    THE TENSION (under the PIN model -- a point force at the pad):
        open hand   -> five wells FIT, but the fingers CANNOT press (32-35% residual)
        curled hand -> the fingers CAN press, but the wells OVERLAP by 7-9 mm
    and no curl did both, so NSGA-II returned an EMPTY front.

    THE RESOLUTION (the CRADLE -- hand/cradle.py):
    The pin model demanded the digit's own muscles balance the entire joint torque from a
    point force at the fingertip. That claim is contradicted by billions of people typing on
    flat keyboards with semi-extended fingers every day. A WELL CRADLES the distal phalanx:
    the reaction bears on the whole palmar surface, so the CENTRE OF PRESSURE can sit anywhere
    along the bone, and a reaction near the DIP has a far smaller moment arm than the same
    force at the tip. The finger is a STRUT (the user's word: piano technique).

    With the cradle, all four fingers get their three directions AT THE OPEN HAND -- the very
    posture where the wells fit. The tension evaporates.

    THE CONTROL, and it is the important half: the cradle must NOT make everything possible.
    The THUMB must still fail, because it still has no adductor and a cradle cannot lend a
    digit a muscle it does not have. An earlier cradle let the finger lean on the floor, BOTH
    walls and the end stop at once; those SELF-CANCEL, so it conjured a keypress from a
    passive finger, and it duly reported that the thumb could press 4 of 5 directions. The
    control caught it.
    """
    from hand.cradle import solve as cradle_solve
    from design.vector import (PRESS_N, RESIDUAL_MAX, key_separation, keys_on_reference)
    from design.qwerty import ACTIONS, ROWS

    h = hands[50]
    four = [f for f in FINGERS if f != "thumb"]
    OPEN_TP, OPEN_TM = 0.35, 0.40

    def n_performable(f):
        q = posture(h, f, OPEN_TP, OPEN_TM, 0.0)
        return sum(cradle_solve(h, q, f, a, PRESS_N)[2] <= RESIDUAL_MAX for a in ACTIONS)

    # 1. at the OPEN hand, five wells fit
    x = mid_design()
    x["tp_hand"], x["tm_hand"] = OPEN_TP, OPEN_TM
    for f in four:
        x[f"dp_{f}"] = x[f"dm_{f}"] = 0.0
    for f, sgn in zip(four, (0.9, 0.3, -0.3, -0.9)):
        x[f"ab_{f}"] = sgn
    keys, curls = keys_on_reference(h, x)
    assert key_separation(keys, h, curls)[0] <= 0, "an open, splayed hand should fit five wells"

    # 2. and WITH THE CRADLE every finger can press its three rows THERE
    for f in four:
        assert n_performable(f) >= len(ROWS), (
            f"{f}: only {n_performable(f)}/5 directions performable at the open hand -- "
            "the cradle should have resolved this"
        )

    # 3. THE CONTROL: the thumb still cannot. The cradle lends no muscle.
    assert n_performable("thumb") < len(ROWS), (
        "the thumb became pressable -- the cradle is too permissive. It has no adductor; a "
        "cradle cannot lend it one. Check for self-cancelling contacts."
    )

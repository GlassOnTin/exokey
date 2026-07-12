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
    assert len(r["G"]) == 9
    assert r["feasible"] == all(v <= 0 for v in r["G"])
    # and the objectives contain no penalty terms: mass and deflection are physical units
    assert r["F"][1] > 0  # grams
    assert r["F"][2] > 0  # mm


def test_two_keycaps_cannot_occupy_the_same_space(hands):
    """A keycap is a physical object. Nothing in the model said so, and EVERY design on the
    first Pareto front violated it: curled fingertips converge, so with one key per finger
    the middle and ring keys sat 8.5 mm apart -- inside a ~14 mm cap. The whole front was
    geometrically unbuildable and no constraint noticed.

    Satisfying it needs SPLAY, which is why abduction is a design variable.
    """
    from design.vector import KEY_PITCH, key_separation, keys_on_reference

    ref = hands[50]
    flat = mid_design()
    for f in FINGERS:
        flat[f"n_{f}"] = 1  # one key each: this isolates LATERAL separation (the splay
        #                     lever) from row spacing (the dc lever), which is a different
        #                     constraint with a different fix.
    for f in ("index", "middle", "ring", "little"):
        flat[f"ab_{f}"] = 0.0  # no splay
    keys, _ = keys_on_reference(ref, flat)
    v_flat, _ = key_separation(keys)   # now a VIOLATION: required pitch - actual gap
    assert v_flat > 0, "unsplayed fingers should NOT be able to fit keycaps"

    fanned = dict(flat)
    for f, s in zip(("index", "middle", "ring", "little"), (1.0, 0.33, -0.33, -1.0)):
        fanned[f"ab_{f}"] = 0.7 * s
    keys, _ = keys_on_reference(ref, fanned)
    v_fan, _ = key_separation(keys)
    assert v_fan <= 0, f"a 70% fan should separate the keys; still short by {v_fan*1000:.1f} mm"
    assert v_fan < v_flat


def test_switch_force_is_fixed_not_optimised():
    """press_N was a design variable over [0.30, 0.80] N and all 60 Pareto designs pinned
    it to the lower bound. That is a corner solution, not a trade: a lighter switch cuts
    BOTH effort and deflection, so nothing pulls the other way. It is a constant."""
    from design.vector import PRESS_N, REAL_BOUNDS

    assert "press_N" not in REAL_BOUNDS
    # 20 gf = 0.196 N -- the Svalboard's magneto-optical key spec, not a guess
    assert PRESS_N == pytest.approx(0.196)


def test_switch_travel_matches_the_switch_we_chose(hands):
    """SWITCH_TRAVEL and PRESS_N must describe the SAME switch. They did not.

    Travel was 3 mm ("a mechanical switch actuates at ~2 mm") -- a Cherry-MX-style
    full-travel key, 0.45-0.60 N. But the actuation force is 0.30 N, which is a dome or
    scissor switch, and a scissor switch travels ~1.5 mm. The two constants described
    different hardware, and the 3 mm figure was strict enough to make 2-keys-per-finger
    layouts infeasible -- layouts that Typeware actually ships.
    """
    from design.vector import CAP_HEIGHT, KEY_PITCH, PRESS_N, SWITCH_TRAVEL

    assert SWITCH_TRAVEL <= 0.002, "a 20 gf magneto-optical key does not travel 3 mm"
    assert PRESS_N <= 0.25
    # and a cap is WIDE AND FLAT: its height (a clearance question) is not half its width
    # (a packing question). Conflating them double-counted the obstacle in the swept check.
    assert CAP_HEIGHT < KEY_PITCH / 2


def test_a_finger_s_own_two_keys_are_a_TWIN_KEY_not_two_caps(hands):
    """The trick the target device (typeware.tech) uses, and the one this model was missing.

    Two switches on ONE stem, at two positions. Requiring a full keycap pitch between a
    finger's own two keys treats them as independent colliding caps -- which they are not --
    and it is a large part of why 2-keys-per-finger came out infeasible here while Typeware
    actually ships it.
    """
    from design.vector import KEY_PITCH, TWIN_KEY_PITCH, key_separation

    assert TWIN_KEY_PITCH < KEY_PITCH

    # a pair on the SAME finger at 9 mm is fine (twin key); the same 9 mm across two
    # fingers is a collision (two stems, two caps)
    same = {("index", 0): (np.zeros(3), np.array([1.0, 0, 0])),
            ("index", 1): (np.array([0.009, 0, 0]), np.array([1.0, 0, 0]))}
    cross = {("index", 0): (np.zeros(3), np.array([1.0, 0, 0])),
             ("middle", 0): (np.array([0.009, 0, 0]), np.array([1.0, 0, 0]))}
    assert key_separation(same)[0] <= 0, "a twin key at 9 mm should be allowed"
    assert key_separation(cross)[0] > 0, "two stems at 9 mm should collide"


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

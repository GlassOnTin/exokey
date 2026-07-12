"""Physics invariants. Each of these caught a real bug; they exist to keep it caught.

Run: .venv/bin/python -m pytest tests/ -q
"""
from __future__ import annotations

import numpy as np
import pytest

import mujoco

from hand.myohand import FINGERS, PAD_BODIES, TIP_SITES, MyoHand

LONG_EXTENSORS = ("EDC", "EIP", "EDM", "EPL", "EPB")  # the muscles that STRAIGHTEN a finger


@pytest.fixture(scope="module")
def h():
    return MyoHand()


def _act(h, name):
    return mujoco.mj_name2id(h.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)


def test_pad_is_palmar_not_dorsal(h):
    """The pad must be the PULP, not the fingernail.

    Each distal body carries a flat `class="skin"` ellipsoid that looks exactly like a
    finger pad and is not one -- it is the nail, on the extensor side of the bone. Taking
    it as the pad put every key behind the fingernail and turned a keypress into an
    extension press (it recruited EIP). The physics was self-consistent throughout, so
    nothing failed; it just answered the wrong question.

    Invariant: the nail geom lies on the OPPOSITE side of the bone from the pad normal.
    Derived from the tendon insertions, checked here against the nail geom -- two
    independent statements of the model's own anatomy.
    """
    m = h.model
    for f in FINGERS:
        bid, pad_l, palmar_l = h.pad[f]
        nail = [
            g
            for g in range(m.body_geomadr[bid], m.body_geomadr[bid] + m.body_geomnum[bid])
            if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_ELLIPSOID
        ]
        assert nail, f"{f}: no nail ellipsoid to check against"
        sid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, TIP_SITES[f])
        to_nail = m.geom_pos[nail[0]] - m.site_pos[sid]
        assert to_nail @ palmar_l < 0, (
            f"{f}: pad normal points at the nail geom -- the pad is on the DORSAL side"
        )
        # and the pad itself must sit palmar of the bone tip
        assert (pad_l - m.site_pos[sid]) @ palmar_l > 0


@pytest.mark.parametrize("finger,digit", [("index", 2), ("middle", 3), ("ring", 4), ("little", 5)])
def test_keypress_recruits_flexors(h, finger, digit):
    """A keypress is resisted by the FLEXORS. If it recruits extensors, the key is behind
    the fingernail. This is the test that catches a flipped pad normal -- and nothing
    else does, because the wrong-side physics stays perfectly self-consistent.
    """
    m = h.model
    q = h.q_neutral
    _, n = h.pad_pose(q, finger)
    a1, *_ = h.solve_activations(q, finger, 0.6, -n)  # key faces back at the pad
    a0, *_ = h.solve_activations(q, finger, 0.0, -n)
    da = a1 - a0  # muscles recruited by the press specifically

    fdp = _act(h, f"FDP{digit}")  # deep flexor: inserts on the distal phalanx
    assert da[fdp] > 0, f"{finger}: pressing the key does not recruit FDP{digit}"

    lead = m.actuator(int(np.argmax(da))).name
    assert not lead.startswith(LONG_EXTENSORS), (
        f"{finger}: press is led by {lead}, an extensor -- the key is behind the nail"
    )


def _key_at(hand, finger, gap=0.004):
    """A key sitting `gap` off the finger's rest pad, facing back at it."""
    p, n = hand.pad_pose(hand.q_neutral, finger)
    return p + gap * n, -n


def test_all_joints_limited(h):
    """v1's fatal bug: IK ran with pose_bounds = [(None,None)]*45, so 'reachable'
    could mean an anatomically impossible pose. Every dof must be bounded."""
    assert h.model.jnt_limited.all(), "some joint has no limit"
    assert np.all(h.lo < h.hi)


def test_rest_posture_is_physiological(h):
    """Midrange-of-limits hyperextends the thumb IP to -25 deg, which silently makes the
    thumb unable to press. Rest must be inside the flexion side for every digit."""
    import mujoco

    def ang(name):
        jid = mujoco.mj_name2id(h.model, mujoco.mjtObj.mjOBJ_JOINT, name)
        return np.rad2deg(h.q_neutral[h.model.jnt_qposadr[jid]])

    # Stated as FLEXION FRACTIONS, never as raw signed degrees. A joint called
    # "*_flexion" does not tell you which sign is flexion: the thumb flexes POSITIVE at
    # the CMC and NEGATIVE at the MP and IP. The old form of this test asserted
    # `ip_flexion > 0` "or it is hyperextended", which had it exactly backwards --
    # ip_flexion = -25 deg is 25 deg of ordinary FLEXION.
    from hand.myohand import FLEXION_JOINTS

    for f in FINGERS:
        for j in FLEXION_JOINTS[f]:
            qadr, straight, limit = h.flexion_span(j)
            t = h.q_neutral[qadr] / limit  # flexion fraction: 0 straight, 1 fully flexed
            assert 0.0 < t < 0.8, f"{f}/{j}: rest is at flexion fraction {t:.2f}"


def test_thumb_rest_is_opposed(h):
    """The thumb must OPPOSE the fingers at rest, and it must be checked, not asserted.

    This test replaces `assert cmc_abduction > 15, "thumb in palmar abduction"`, which was
    false: `cmc_abduction` in this model is RADIAL abduction. +30 deg swings the thumb tip
    from 19 mm to 96 mm radially -- straight out to the side, a hitchhiker's thumb, painful
    and unusable -- while barely changing its stand-off from the palm. The old test passed
    happily the whole time, because it tested the number I had typed in, not the hand.

    Opposition is the thing that actually defines the position of function, so test that:
    the thumb pulp faces the index/middle pulps, across a natural (non-pinching) gap.
    """
    q = h.q_neutral
    pt, nt = h.pad_pose(q, "thumb")
    pi, _ = h.pad_pose(q, "index")
    pm, _ = h.pad_pose(q, "middle")
    target = 0.5 * (pi + pm)

    v = target - pt
    gap = float(np.linalg.norm(v))
    facing = float(nt @ (v / gap))

    assert facing > 0.8, f"thumb pulp does not face the fingers (cos={facing:+.2f})"
    assert 0.030 < gap < 0.060, f"thumb sits {gap*1000:.0f} mm from the fingers, not opposed"


@pytest.mark.parametrize("finger", FINGERS)
def test_key_at_the_flexion_limit_has_no_travel(h, finger):
    """A finger at its flexion limit cannot press anything -- and that is exactly where
    unconstrained min-effort wants to put it, because a slack finger costs nothing.

    Pin both halves: at the flexion limit there is no travel, and at the rest posture
    there is. Without this check the "optimal" key for the ring finger came out fully
    curled into the palm while its neighbours stayed relaxed.
    """
    from hand.myohand import FLEXION_JOINTS

    _, n = h.pad_pose(h.q_neutral, finger)
    assert h.can_press(h.q_neutral, finger, -n), f"{finger}: no travel even at rest"

    q_max = h.q_neutral.copy()
    for j in FLEXION_JOINTS[finger]:
        adr, _, hi = h.flexion_span(j)
        q_max[adr] = hi
    _, n_max = h.pad_pose(q_max, finger)
    assert h.press_travel(q_max, finger, -n_max) < 1e-9, (
        f"{finger}: claims travel while fully flexed"
    )


def test_rest_within_limits(h):
    assert np.all(h.q_neutral >= h.lo - 1e-9)
    assert np.all(h.q_neutral <= h.hi + 1e-9)


@pytest.mark.parametrize("finger", FINGERS)
def test_effort_increases_with_press_force(h, finger):
    """The load-bearing physical check. Pressing harder MUST cost more muscle.

    This failed three separate ways before it passed:
      1. the press reaction was applied with the wrong sign (flexors/extensors swapped);
      2. the feasibility constraint had slack, so the solver bought effort reductions
         by paying torque error (v1's disease, in miniature);
      3. equilibrium included the forearm, so 'holding the arm up' swamped the keypress
         and effort actually DECREASED with press force.
    """
    q0 = h.q_neutral.copy()
    p, n = h.pad_pose(q0, finger)
    key_pos, key_n = p + 0.004 * n, -n
    efforts = [
        h.press(finger, key_pos, key_n, press_N=F, q0=q0).effort
        for F in (0.0, 0.3, 0.6, 0.8)
    ]
    assert all(
        efforts[i] <= efforts[i + 1] + 1e-6 for i in range(len(efforts) - 1)
    ), f"{finger}: effort not monotonic in press force: {efforts}"


def test_unloaded_rest_is_nearly_slack(h):
    """An unloaded hand at its rest posture must be nearly slack.

    Stated as an ABSOLUTE bound on activation, not as a ratio to a loaded press. The
    earlier ratio form (free < 5% of loaded) silently encoded how expensive a press
    happened to be -- so when the pad moved from the fingernail to the pulp and a press
    got 16x cheaper (the flexors are good at this; the extensors were not), the test
    failed even though the hand had not changed. A test that moves when the thing it
    is not testing moves is measuring the wrong quantity.
    """
    for f in FINGERS:
        _, n = h.pad_pose(h.q_neutral, f)
        a, _, _, _, _ = h.solve_activations(h.q_neutral, f, 0.0, -n)
        assert a.max() < 0.05, f"{f}: resting hand holds {a.max():.3f} activation, not slack"


def test_passive_tension_only_at_extreme_postures(h):
    """Where the posture cost in `effort` actually comes from, pinned in both directions.

    MyoHand's muscles are SLACK over most of the joint range: an unloaded finger near its
    rest pose needs literally zero activation. Passive force-length tension switches on
    only when a muscle is stretched near a joint limit, and then it is large.

    Both halves matter. The zero half means `effort` is almost purely the cost of the
    PRESS, so it is a clean design signal and not contaminated by a posture offset. The
    non-zero half means a key that drags a finger to the end of its range is penalised
    without needing any extra hand-tuned posture term.

    (An earlier version of this asserted a 6 mm displacement costs effort. It does not --
    6 mm is deep inside the slack region. The physics was right and the test was wrong.)
    """
    dofs = h.digit_dofs["index"]

    q_near = h.q_neutral.copy()  # a few degrees off rest: still slack
    q_near[dofs] += np.deg2rad(5.0)
    q_near = np.clip(q_near, h.lo, h.hi)
    _, n = h.pad_pose(q_near, "index")
    _, effort_near, _, _, _ = h.solve_activations(q_near, "index", 0.0, -n)
    assert effort_near < 1e-8, f"near rest, unloaded, should be slack; got {effort_near:.2e}"

    # MCP driven to full EXTENSION stretches the long flexors, whose passive tension
    # pulls the joint back into flexion; EDC2 has to fire to hold it. (Full *flexion* is
    # the one limit that costs nothing -- it shortens the flexors -- and picking it was
    # how an earlier version of this test managed to measure zero and look broken.)
    mcp = mujoco.mj_name2id(h.model, mujoco.mjtObj.mjOBJ_JOINT, "mcp2_flexion")
    q_ext = h.q_neutral.copy()
    q_ext[h.model.jnt_qposadr[mcp]] = h.model.jnt_range[mcp][0]  # 0 deg = straight
    _, n = h.pad_pose(q_ext, "index")
    a, effort_ext, _, _, _ = h.solve_activations(q_ext, "index", 0.0, -n)
    assert effort_ext > 1e-4, (
        f"MCP at full extension, unloaded: passive tension should be real; got {effort_ext:.2e}"
    )
    edc = _act(h, "EDC2")
    assert a[edc] > 0.05, "the extensor should be the one resisting stretched flexors"


def test_thumb_thenar_intrinsics_are_missing(h):
    """Pin a MODEL limitation so no thumb result is ever quietly trusted.

    MyoHand gives the thumb five actuators: APL, EPB, EPL, FPL, OP. The thenar intrinsics
    -- adductor pollicis (ADP), flexor pollicis brevis (FPB), abductor pollicis brevis
    (APB) -- are absent. ADP is *the* thumb press/pinch muscle. Without it the thumb must
    press using FPL and OP alone, against EPL/EPB/APL as antagonists, so MyoHand
    systematically OVERSTATES the cost of a thumb keypress (measured here: ~1000x the
    index for the same 0.5 N at the resting pad, which is not believable).

    A chording keyboard puts prime real estate under the thumb, so this is not a corner
    case. Thumb effort from this model ranks thumb keys against each other at best; it
    must NOT be compared against the fingers'. Cross-check needs a model that has the
    thenar group (the plan's OpenSim ARMS, 43 muscles).

    If this test ever fails, MyoHand gained the thenar muscles -- delete the caveat.
    """
    have = {h.model.actuator(i).name for i in range(h.nu)}
    assert not ({"ADP", "FPB", "APB"} & have), "MyoHand gained thenar intrinsics"
    assert {"FPL", "OP", "APL", "EPL", "EPB"} <= have


def test_no_saturation_at_rest_keys(h):
    """A key placed just off the pad at rest must be pressable by every digit at 0.5 N.
    If a digit saturates here, the posture or the model is wrong -- not the design."""
    q0 = h.q_neutral.copy()
    for f in FINGERS:
        p, n = h.pad_pose(q0, f)
        post = h.press(f, p + 0.004 * n, -n, press_N=0.5, q0=q0)
        assert not post.saturated, f"{f} saturates at 0.5 N (max_act={post.max_act:.3f})"
        assert post.ok, f"{f} infeasible: reach={post.pos_err*1000:.2f}mm"


def test_key_position_actually_matters(h):
    """If effort barely varies with key position there is nothing for the optimiser to
    do. We need a real spread -- that spread IS the design signal."""
    q0 = h.q_neutral.copy()
    efforts = []
    for f in FINGERS:
        p, n = h.pad_pose(q0, f)
        u = np.cross(n, [0.0, 0.0, 1.0])
        u /= np.linalg.norm(u) + 1e-12
        for k in (-1, 0, 1):
            post = h.press(f, p + 0.004 * n + 0.008 * k * u, -n, press_N=0.5, q0=q0)
            efforts.append(post.effort)
    spread = max(efforts) / max(min(efforts), 1e-12)
    assert spread > 2.0, f"effort barely depends on key position (spread {spread:.2f}x)"


def test_cantilever_matches_closed_form():
    """FEA gate from the plan: beam tip deflection within 1% of Euler-Bernoulli."""
    from verify.bench import verify_cantilever

    _, _, err_pct = verify_cantilever()
    assert err_pct < 1.0, f"beam FEA off by {err_pct:.2f}%"


def test_no_key_requires_hyperextension(h):
    """No key may demand a hyperextended digit. Pressing is a flexion task.

    The four fingers cannot hyperextend (every flexion joint is [0, 90] deg), but ALL FOUR
    of the thumb's joints go negative -- ip_flexion runs to -75 deg. A posture sweep that
    takes "5% of the joint range" therefore drives the thumb to -72 deg, and that garbage
    posture wins on effort. flexion_span() clamps the usable span to the flexion side.
    """
    from hand.myohand import FLEXION_JOINTS

    for f in FINGERS:
        for j in FLEXION_JOINTS[f]:
            qadr, straight, limit = h.flexion_span(j)
            assert straight == 0.0, "the flexion span must start from straight"
            # every posture we generate is q = t*limit for t in [0,1], so it can never
            # cross zero into hyperextension, whichever sign `limit` has
            assert abs(limit) > 0.0
            assert 0.0 <= h.q_neutral[qadr] / limit <= 1.0


@pytest.mark.parametrize("finger", FINGERS)
def test_a_hand_can_reach_a_key_at_its_own_fingertip(h, finger):
    """The most basic possible reach test, and it FAILED for a long time.

    Generate a key from a posture, then ask press() to find a posture reaching it. The
    answer is in press()'s own search space by construction, so pos_err must be ~0.

    It was 1.5-3.2 mm, because pose_cost weighed reach (in metres SQUARED -- 1 mm is 1e-6)
    against a normalised posture-comfort penalty of order 1e-5. The comfort term outweighed
    reach by ~4e10 and the solver bought comfort with reach error. Feasibility as a soft
    penalty the optimiser can pay its way out of: v1's exact disease, reproduced by a units
    mismatch, and it made the whole NSGA-II run infeasible.
    """
    from design.vector import posture

    for t_p, t_m in ((0.25, 0.35), (0.45, 0.55), (0.65, 0.70)):
        q = posture(h, finger, t_p, t_m)
        pos, n = h.pad_pose(q, finger)
        post = h.press(finger, pos, -n, press_N=0.5, q0=h.q_neutral)
        assert post.pos_err < 5e-4, (
            f"{finger} @ curl {t_p:.2f}/{t_m:.2f}: cannot reach a key at its own "
            f"fingertip -- off by {post.pos_err*1000:.2f} mm"
        )

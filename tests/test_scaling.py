"""Hand-size scaling. Verified against the scaling LAW, not just 'it ran'."""
from __future__ import annotations

import mujoco
import numpy as np
import pytest

from hand.myohand import MyoHand
from hand.scaling import ANSUR_HAND_LENGTH_MM, population


def hand_length(h: MyoHand) -> float:
    h.fk(np.zeros(h.model.nq))
    cap = h.data.xpos[mujoco.mj_name2id(h.model, mujoco.mjtObj.mjOBJ_BODY, "capitate")]
    tip = h.data.site_xpos[mujoco.mj_name2id(h.model, mujoco.mjtObj.mjOBJ_SITE, "MFtip")]
    return float(np.linalg.norm(tip - cap))


@pytest.fixture(scope="module")
def hands():
    return {p: MyoHand(scale=s) for p, s in population().items()}


def test_population_spans_5th_to_95th(hands):
    pop = population()
    assert set(pop) == {5, 25, 50, 75, 95}
    assert pop[50] == pytest.approx(1.0)
    ratio = ANSUR_HAND_LENGTH_MM[95] / ANSUR_HAND_LENGTH_MM[5]
    assert ratio == pytest.approx(hand_length(hands[95]) / hand_length(hands[5]), rel=1e-6)


def test_lengths_scale_linearly(hands):
    ref = hand_length(hands[50])
    for p, s in population().items():
        assert hand_length(hands[p]) == pytest.approx(s * ref, rel=1e-9)


def test_muscle_force_scales_as_area(hands):
    """Peak isometric force ~ physiological cross-sectional area ~ s^2. Scale it as s
    (or not at all) and a big hand comes out as a long WEAK one."""
    def fmax(h):
        i = mujoco.mj_name2id(h.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "FDP2")
        return float(h.model.actuator_gainprm[i, 2])

    ref = fmax(hands[50])
    for p, s in population().items():
        assert fmax(hands[p]) == pytest.approx(s**2 * ref, rel=1e-9)


def test_bigger_hands_are_relatively_stronger(hands):
    """The payoff, and the reason the population problem is interesting.

    torque capacity ~ moment arm (s) * force (s^2) = s^3, while the demand from a fixed key
    force is J^T F ~ s. So activation ~ s^-2. Big hands press the same switch with less
    muscle; the SMALL hand is the binding one for effort and saturation.
    """
    acts = {}
    for p, h in hands.items():
        _, n = h.pad_pose(h.q_neutral, "index")
        a, _, _, _, _ = h.solve_activations(h.q_neutral, "index", 0.5, -n)
        acts[p] = float(a.max())

    pop = population()
    for p in (5, 25, 75, 95):
        predicted = acts[50] * (pop[p] ** -2)
        assert acts[p] == pytest.approx(predicted, rel=0.05), (
            f"{p}th: activation {acts[p]:.4f} does not follow s^-2 (expected {predicted:.4f})"
        )
    assert acts[5] > acts[95], "the small hand must work harder"


def test_joint_limits_are_scale_invariant(hands):
    """Angles do not scale. If the ranges moved, something scaled that should not have."""
    ref = hands[50].model.jnt_range.copy()
    for p, h in hands.items():
        assert np.allclose(h.model.jnt_range, ref)


def test_rest_posture_is_scale_invariant(hands):
    ref = hands[50].q_neutral
    for p, h in hands.items():
        assert np.allclose(h.q_neutral, ref, atol=1e-9)

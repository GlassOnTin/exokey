"""Structural invariants. Stage 3's pass/fail gates, defined before trusting any number.

Run: .venv/bin/python -m pytest tests/ -q
"""
from __future__ import annotations

import numpy as np
import pytest

from hand.myohand import MyoHand
from structure.frame import (
    MATERIALS,
    build_exo,
    clearance,
    solve,
    torsion_constant,
    von_mises,
)


@pytest.fixture(scope="module")
def hand():
    return MyoHand()


@pytest.fixture(scope="module")
def keys(hand):
    """Keys just off each rest pad -- enough to build a frame around."""
    out = {}
    for f in hand.pad:
        p, n = hand.pad_pose(hand.q_neutral, f)
        out[f] = (p + 0.004 * n, -n)
    return out


@pytest.fixture(scope="module")
def exo(hand, keys):
    return build_exo(hand, hand.q_neutral, keys)


def test_cantilever_matches_closed_form():
    """FEA gate from the plan: tip deflection within 1% of Euler-Bernoulli."""
    from verify.bench import verify_cantilever

    _, _, err_pct = verify_cantilever()
    assert err_pct < 1.0, f"beam FEA off by {err_pct:.2f}%"


def test_torsion_constant_is_not_the_polar_moment():
    """J for a rectangle is St Venant's, NOT Ip = (b*d^3 + d*b^3)/12. Using the polar
    moment overstates torsional stiffness of a thin strip by a large factor, and every
    strut here is a thin strip."""
    b, d = 0.008, 0.002
    J = torsion_constant(b, d)
    Ip = (b * d**3 + d * b**3) / 12.0
    assert J < Ip
    # thin strip: J -> a*t^3/3
    assert J == pytest.approx(b * d**3 / 3.0, rel=0.25)


def test_von_mises_reduces_to_bending():
    """With only a bending moment, sigma_vm must equal the elementary M*c/I."""
    b, d = 0.008, 0.002
    My = 0.02
    I = b * d**3 / 12.0
    assert von_mises(0, My, 0, 0, b, d) == pytest.approx(My * (d / 2) / I, rel=1e-9)


def test_straps_are_tension_only():
    """Webbing cannot push. A strap loaded in compression must carry ~zero force, and the
    load must go somewhere else -- checked on a truss small enough to reason about.

    Two bars from a common apex down to two pinned feet, with a horizontal load at the
    apex. One diagonal goes into tension, the other into compression. As a tension-only
    strap the compressed one must drop out.
    """
    from Pynite import FEModel3D

    def truss(tension_only: bool) -> tuple[float, float]:
        fem = FEModel3D()
        m = MATERIALS["al6061"]
        fem.add_material("al", E=m["E"], G=m["G"], nu=m["nu"], rho=m["rho"])
        fem.add_section("s", A=1e-5, Iy=1e-11, Iz=1e-11, J=1e-11)
        fem.add_node("apex", 0.0, 0.0, 0.1)
        fem.add_node("L", -0.1, 0.0, 0.0)
        fem.add_node("R", 0.1, 0.0, 0.0)
        for nm in ("L", "R"):
            fem.def_support(nm, True, True, True, True, True, True)  # fully fixed feet
        # apex: hold it out of plane and against spin, leave x/z free to respond
        fem.def_support("apex", False, True, False, True, True, True)
        fem.add_member("dL", "apex", "L", "al", "s", tension_only=tension_only)
        fem.add_member("dR", "apex", "R", "al", "s", tension_only=tension_only)
        fem.add_node_load("apex", "FX", 50.0)
        fem.analyze(check_statics=False)
        return (
            fem.members["dL"].max_axial("Combo 1"),
            fem.members["dR"].max_axial("Combo 1"),
        )

    # SIGN CONVENTION, verified directly rather than assumed: PyNite reports axial force
    # NEGATIVE in tension and POSITIVE in compression. (A bar pulled +100 N, which visibly
    # stretches, comes back as axial = -100.) Get this backwards and the test "passes" on
    # the compressed member.
    nL, nR = truss(tension_only=False)
    tL, tR = truss(tension_only=True)

    assert max(nL, nR) > 1.0, f"expected a COMPRESSED diagonal, got {nL:.2f}/{nR:.2f}"
    assert min(nL, nR) < -1.0, f"expected a TENSIONED diagonal, got {nL:.2f}/{nR:.2f}"

    assert max(tL, tR) < 1e-6, f"a tension-only strap carried compression: {tL:.3f}/{tR:.3f}"
    assert min(tL, tR) < -1.0, "the tensioned diagonal should still carry load"


def test_frame_clears_the_hand(hand, exo):
    """The frame must not pass through the flesh. v1's collapse began with
    'Remove hard collision correction causing upward drift' -- here it is a hard check."""
    gaps = clearance(hand, hand.q_neutral, exo)
    worst = min(gaps.values())
    assert worst > 0.0, (
        f"frame intersects the hand: {min(gaps, key=gaps.get)} at {worst*1000:.1f} mm"
    )


def test_press_load_deflects_the_key_away_from_the_finger(hand, exo, keys):
    """Sanity of sign: the finger pushes the key along -normal, so the key must move that
    way. If it moves toward the finger the load is applied backwards."""
    from Pynite import FEModel3D  # noqa: F401

    r = solve(exo, [("index", 0)], press_N=0.5)
    assert r["deflection"][("index", 0)] > 0.0
    # and pressing harder must deflect it further, linearly
    r2 = solve(exo, [("index", 0)], press_N=1.0)
    assert r2["deflection"][("index", 0)] == pytest.approx(
        2.0 * r["deflection"][("index", 0)], rel=1e-6
    )


def test_softer_tissue_gives_a_softer_key(hand, exo):
    """The support stiffness is poorly known, so its DIRECTION of effect must at least be
    right: a frame bearing on softer tissue must let the key move more. If deflection did
    not respond to k, the supports would not be in the load path and the elastic-support
    modelling would be decorative."""
    soft = solve(exo, [("index", 0)], k_soft=10e3)["max_deflection"]
    stiff = solve(exo, [("index", 0)], k_soft=50e3)["max_deflection"]
    assert soft > stiff, f"deflection did not respond to soft-tissue k ({soft:.2e} vs {stiff:.2e})"


def test_mass_is_plausible_for_a_worn_device(exo):
    """A hand-worn frame of aluminium strip and nylon must land in tens of grams. If this
    reads kilograms the sections or the density are wrong by orders of magnitude."""
    g = exo.mass() * 1000
    assert 5.0 < g < 200.0, f"device mass {g:.1f} g is not a wearable"

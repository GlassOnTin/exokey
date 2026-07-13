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


def test_the_palm_is_a_cup_not_a_plate():
    """The user: "invert the support structure -- it doesn't follow the natural shape of the
    hand." They are right, and the hand says so: the palm is a CUP.

    Measured off the metacarpal meshes: the radial and ulnar edges PROTRUDE palmar and the
    middle is HOLLOW, by 6.4 mm. `build_body` bolts four corners of a FLAT RECTANGLE across
    it at one depth, so they either float off the eminences or dig into the hollow.
    """
    from opt.problem import hands
    from structure.frame import palmar_arch

    h = hands()[50]
    pts = palmar_arch(h, h.q_neutral, 0.045, n=5)
    z = np.array([p[1] for p in pts])
    assert z.max() - z.min() > 0.003, "the palm should be a cup, several mm deep"
    # the middle must be the SHALLOWEST (least palmar) -- that is what "hollow" means
    assert int(np.argmax(z)) not in (0, len(z) - 1), "the hollow should be in the MIDDLE"


def test_where_the_compliance_lives_DEPENDS_ON_THE_DESIGN():
    """I TRIED TO MAKE THIS A UNIVERSAL CLAIM, TWICE, IN BOTH DIRECTIONS. It is not one.

    The user asked for the support structure to follow the hand's natural shape -- and they
    are right: the palm is a CUP 6.4 mm deep and `build_body` bolts a FLAT PLATE across it
    (see test_the_palm_is_a_cup_not_a_plate). The obvious next claim is that an ARCH would
    also be stiffer, because a keypress pushes the body INTO the palm and an arch takes that
    in COMPRESSION where a plate takes it in BENDING.

    THE MEASUREMENT REFUSES TO SUPPORT THAT CLAIM, and it refuses in both directions. Stiffen
    each group 100x, see what the key feels:

        design                key face   floor legs   palm support
        optimiser's lightest      49%          38%            13%
        optimiser's knee          44%          27%            10%
        hand-built baseline        2%          12%            18%   <-- palm DOMINATES
        bounds midpoint            3%           --             9%   <-- palm dominates

    On the devices the OPTIMISER produces, the compliance is in the cantilever out to the
    wells and the palm is minor -- so an arch cannot help. On HAND-BUILT devices the palm
    dominates -- so it could. The optimiser has already chosen sections (a wide, thin strip)
    that move the softness elsewhere.

    ⚠ AND MY "3.8x STIFFER ARCH" WAS AN ARTIFACT. That version skipped the floor routing
    entirely and cut straight through the fingers; the gain came from DELETING THE CANTILEVER,
    not from the arch.

    SO: the arch is a FIT and PRESSURE-DISTRIBUTION change -- bear on the two eminences the
    hand actually presents, rather than a plate across the hollow -- and THIS MODEL CANNOT
    SCORE COMFORT. Its 3x mass penalty is also largely a BEAM-MODEL ARTIFACT (10 discrete
    beams where a plate is 4; a MOULDED SHELL following the palm costs nothing extra). Settling
    it needs shell elements, not beams.

    This test pins the MEASUREMENT, not a conclusion -- because the conclusion is
    design-dependent and I got it wrong twice by pretending otherwise.
    """
    from dataclasses import replace

    from design.vector import BODY_PROX, PRESS_N, keys_on_reference
    from hand.myohand import FINGERS
    from opt.problem import hands
    from opt.run import baseline
    from structure.frame import build_body, solve

    h = hands()[50]
    x = baseline()
    keys, _ = keys_on_reference(h, x)
    par = dict(sec_alu=(float(x["alu_w"]), float(x["alu_t"])), palm_offset=float(x["palm_offset"]),
               body_half=float(x["body_half"]), body_prox=BODY_PROX, body_dist=float(x["body_dist"]),
               stem=float(x["stem"]), mat_frame=str(x["material"]))
    chords = [(f, 0) for f in FINGERS]
    base = solve(build_body(h, h.q_neutral, keys, par), chords, press_N=PRESS_N)["max_deflection"]

    def stiffen(pred):
        exo = build_body(h, h.q_neutral, keys, par)
        exo.members = [replace(m, b=m.b * 10, d=m.d * 10) if pred(m.name) else m
                       for m in exo.members]
        return 1.0 - solve(exo, chords, press_N=PRESS_N)["max_deflection"] / base

    share = {g: stiffen(lambda n, g=g: n.startswith(g))
             for g in ("palm_", "face", "floorleg", "wall")}
    assert all(0.0 <= v < 1.0 for v in share.values()), f"implausible shares: {share}"
    assert sum(share.values()) > 0.15, (
        f"no group carries the compliance: {share}. The measurement is broken, not the design."
    )
    # and the thing that must NOT be assumed: that any one group always dominates.
    assert max(share.values()) < 0.9, (
        "one group carries almost everything -- if that is real, say WHICH, and note it is "
        "design-dependent. It inverts between the baseline and the optimiser's designs."
    )


def test_the_shell_model_reproduces_a_textbook_plate():
    """THE SHELL GATE, and it is the same discipline that validated the beam model against a
    closed-form cantilever: a simply-supported square plate under uniform load has a closed
    form, w = 0.00406 q a^4 / D (Timoshenko).

    A shell model that cannot reproduce a textbook plate has no business being asked about an
    arch. Measured, and it CONVERGES: 2.3% at 4x4, 1.9% at 8x8, 1.0% at 12x12, 0.7% at 16x16.
    """
    from structure.shell import simply_supported_plate

    errs = []
    for n in (4, 8, 12):
        fe, ct = simply_supported_plate(a=0.20, t=0.002, q=1000.0, n=n)
        errs.append(abs(fe - ct) / ct)
    assert errs[-1] < 0.02, f"shell is {errs[-1]:.1%} off the closed form -- do not trust it"
    assert errs[-1] < errs[0], f"it must CONVERGE with mesh refinement, got {errs}"


def test_the_beam_model_lied_about_the_arch_by_25x():
    """WHY A SHELL MODEL WAS NEEDED AT ALL.

    An arch carries load as MEMBRANE action -- compression in its own mid-surface -- and a
    stick figure of beams HAS NO MEMBRANE. Worse, the beam model CHARGES for the arch: it is
    10 discrete struts where a plate is 4, so PyNite billed +212% mass for a shape that,
    moulded, is the same shell merely curved.

        beam model:   arch is +212% mass, 1.00x stiffness   -> "not worth it"
        shell model:  arch is  +8.4% mass, 1.12x stiffness

    The mass penalty was wrong by a factor of 25. That is not a finding about arches; it is an
    artifact of idealising a shell as sticks.

    ⚠ AND THE ARCH IS STILL NOT CLEARLY WORTH IT. 1.12x on a component that carries only ~10%
    of the compliance is ~1% overall. The shell corrects the ARTIFACT without vindicating the
    ARCH -- those are different claims, and only the first is established.
    """
    import pickle

    from design.vector import BODY_PROX, PRESS_N
    from opt.problem import hands
    from opt.run import baseline
    from structure.shell import palm_shell

    h = hands()[50]
    x = baseline()
    par = dict(alu_t=float(x["alu_t"]), body_half=float(x["body_half"]), body_prox=BODY_PROX,
               body_dist=float(x["body_dist"]), palm_offset=float(x["palm_offset"]),
               mat_frame=str(x["material"]), press_N=PRESS_N)
    w_flat, m_flat = palm_shell(h, h.q_neutral, par, shape="flat")
    w_arch, m_arch = palm_shell(h, h.q_neutral, par, shape="follow")

    assert m_arch / m_flat < 1.3, (
        f"a curved shell should cost only its extra ARC LENGTH ({m_arch/m_flat:.2f}x). The "
        "beam model charged 3.1x, which is what made the arch look not worth having."
    )
    assert w_arch < w_flat, "an arch should be stiffer than a flat plate of the same thickness"


def test_a_hugging_frame_must_be_a_SHELL_not_sticks():
    """THE LAW, and it is why "hug the hand" and "use shells" are the SAME request.

    The user: "having the supporting structure far from the hand is a problem because it
    gets-in-the-way of me using my hands. If the supporting structure hugs the hand ... it
    becomes more a natural extension, rather than holding a big ball."

    Measured, and they are right: of `build_body`'s structural nodes, 15 of 16 are PALMAR of
    the hand, standing off it by a mean of 27 mm and a MAXIMUM OF 68 mm. That is the volume you
    use to hold a cup. The device is not on the hand -- it is a ball the hand is wrapped around.

        A BEAM FRAME BUYS ITS STIFFNESS WITH DEPTH.
        DEPTH IS EXACTLY WHAT GETS IN THE WAY.

    The palmar box is stiff because it is 57 mm deep. A dorsal frame that HUGS has ~5 mm of
    depth, and as a STICK FIGURE it is hopeless: triangulated, forked and cross-braced, it still
    deflected 2.58 mm against a 0.5 mm gate.

    A SHELL needs no depth. It gets stiffness from CURVATURE -- a curved section cannot bend
    without STRETCHING, and stretching is expensive. That is a tape measure, an eggshell, a
    fingernail. Same material, same thickness, same width, merely WRAPPED round the finger
    instead of laid flat across it:

        index  0.035 mm flat  ->  0.001 mm curved   (46x)
        middle 0.035 mm       ->  0.001 mm          (48x)

    So the hugging structure does not have to be floppy. It has to be a shell. And the STRUCTURE
    MODEL had to change before the ARCHITECTURE could even be seen -- which is the whole lesson
    of this project.
    """
    from design.vector import PRESS_N
    from opt.problem import hands
    from opt.run import baseline
    from structure.shell import dorsal_rail

    h = hands()[50]
    x = baseline()
    par = dict(alu_t=float(x["alu_t"]), mat_frame=str(x["material"]), press_N=PRESS_N)

    w_flat, m_flat, _ = dorsal_rail(h, h.q_neutral, "index", par, curved=False)
    w_curv, m_curv, _ = dorsal_rail(h, h.q_neutral, "index", par, curved=True)

    assert abs(m_curv / m_flat - 1.0) < 0.05, "same material -- curvature must be FREE"
    # ⚠ THE GAIN DEPENDS ON THICKNESS, and it depends the way it should. A FLAT strip's
    # bending stiffness goes as t^3, so thick stock is already stiff and curvature buys less;
    # a CURVED shell is stiff even when thin. Measured: 9x on the (thick) hand-built baseline,
    # 46x on the optimiser's own (0.85 mm) design. The thinner and lighter you want the device,
    # the MORE curvature is worth -- which is exactly the regime a wearable lives in.
    assert w_curv < w_flat / 5.0, (
        f"curvature should buy nearly an order of magnitude ({w_flat/w_curv:.0f}x measured). "
        "If it does not, a hugging frame cannot be made stiff and the palmar ball is "
        "unavoidable."
    )
    assert w_curv < 0.5e-3, "the curved rail must pass the 0.5 mm key-deflection gate"


def test_the_floor_legs_must_reach_DISTINCT_feet():
    """A DEFECT THE BEAM MODEL HID FOR THE WHOLE PROJECT.

    The rule was "connect each palm corner to its NEAREST key foot". Every palm corner is
    PROXIMAL of every well (corners at distal 8-45 mm, feet at 65-126 mm), so the nearest foot
    to all four is the SAME ONE -- the thumb's. ALL FOUR LEGS LANDED ON foot_thumb0: the entire
    keypress load from five wells funnelled through ONE NODE, and the whole key face
    cantilevered off it.

    A chain of struts carries load AXIALLY and axial stiffness is enormous, so the beam model
    still reported it as stiff (27 um). A SHELL cannot hide it -- a plate held at one node is a
    floppy cantilever (990 um). The idealisation was flattering a structure that is genuinely
    badly supported.

    Fixed with a one-to-one assignment (Hungarian), like the character layout, and for the same
    reason: "nearest" is a greedy rule, and greedy rules collide. Worth 1.2x stiffness for zero
    mass.
    """
    from design.vector import BODY_PROX, keys_on_reference
    from opt.problem import hands
    from opt.run import baseline
    from structure.frame import build_body

    h = hands()[50]
    x = baseline()
    keys, _ = keys_on_reference(h, x)
    par = dict(sec_alu=(float(x["alu_w"]), float(x["alu_t"])), palm_offset=float(x["palm_offset"]),
               body_half=float(x["body_half"]), body_prox=BODY_PROX, body_dist=float(x["body_dist"]),
               stem=float(x["stem"]), mat_frame=str(x["material"]))
    exo = build_body(h, h.q_neutral, keys, par)
    landed = {m.j for m in exo.members if m.name.startswith("floorleg")}
    assert len(landed) >= 3, (
        f"the floor legs land on only {len(landed)} foot(feet): {landed}. The whole device is "
        "hanging off one node."
    )

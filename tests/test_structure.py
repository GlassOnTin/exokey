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


# ⚠ THE PALMAR BOX IS NO LONGER A DESIGN VECTOR. Six variables (alu_w, alu_t, palm_offset, stem,
# body_half, body_dist) shaped a body bolted to the palm, and the structure is the grown gauntlet
# now, so they were deleted from REAL_BOUNDS. The tests below still exercise structure/frame.py --
# the shell-vs-beam lesson, the floor legs, where the compliance lives -- and those are still real
# physics about a real module. They just have to state the box's dimensions themselves instead of
# reading them out of a design vector that has stopped describing one.
BOX = dict(alu_w=0.008, alu_t=0.002, palm_offset=0.020, stem=0.008,
           body_half=0.026, body_dist=0.055, material="cf_pa12")


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
    par = dict(sec_alu=(float(BOX["alu_w"]), float(BOX["alu_t"])), palm_offset=float(BOX["palm_offset"]),
               body_half=float(BOX["body_half"]), body_prox=BODY_PROX, body_dist=float(BOX["body_dist"]),
               stem=float(BOX["stem"]), mat_frame=str(BOX["material"]))
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
    par = dict(alu_t=float(BOX["alu_t"]), body_half=float(BOX["body_half"]), body_prox=BODY_PROX,
               body_dist=float(BOX["body_dist"]), palm_offset=float(BOX["palm_offset"]),
               mat_frame=str(BOX["material"]), press_N=PRESS_N)
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
    par = dict(alu_t=float(BOX["alu_t"]), mat_frame=str(BOX["material"]), press_N=PRESS_N)

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
    par = dict(sec_alu=(float(BOX["alu_w"]), float(BOX["alu_t"])), palm_offset=float(BOX["palm_offset"]),
               body_half=float(BOX["body_half"]), body_prox=BODY_PROX, body_dist=float(BOX["body_dist"]),
               stem=float(BOX["stem"]), mat_frame=str(BOX["material"]))
    exo = build_body(h, h.q_neutral, keys, par)
    landed = {m.j for m in exo.members if m.name.startswith("floorleg")}
    assert len(landed) >= 3, (
        f"the floor legs land on only {len(landed)} foot(feet): {landed}. The whole device is "
        "hanging off one node."
    )


def test_the_anchor_must_be_a_PATCH_not_a_hinge():
    """THE BOUNDARY CONDITION THAT DECIDED EVERYTHING.

    The user: "Don't forget that the gauntlet still has to carry the force to the static wrist
    / palms" and then "What we need is the anchor points and boundary conditions first."

    Both were right, and both caught a mistake:

      1. The first gauntlet used RIGID supports -- a clamp. Rigid anchors absorb the keypress
         for free and flatter every number downstream. With honest soft-tissue anchors the
         deflection was 7x worse and it FAILED the gate at every thickness, up to 2 mm and 70 g.

      2. Worse, the anchors were the proximal RING of the metacarpal shells -- A LINE of nodes
         with ZERO extent along the lever. A keypress 121 mm away is a MOMENT, and A LINE
         CANNOT CARRY A MOMENT. 55% of the button's movement was the gauntlet ROCKING, and
         thickening the shell did nothing about it, because I was stiffening a beam that pivoted
         on a pin.

    The fix was not material. It was EXTENT: bear on the CARPUS as well as the metacarpals.

        anchor              extent   rocking at the button
        line at knuckles     ~0 mm            ~387 um
        patch + carpus       92 mm              0.2 um

    And the stiffness follows the MEASURED tissue: k = E*A/t, with t = 1.4-3.1 mm over the
    metacarpals (bone radius vs flesh capsule). Thin skin over bone is a STIFF anchor.
    SOFT_TISSUE_K = 25 N/mm was quoted for a PALM patch -- a muscle pad ten times thicker.
    """
    import numpy as np

    from design.vector import posture, tm_of, tp_of
    from hand.myohand import FINGERS
    from opt.problem import hands
    from opt.run import baseline
    from structure.anchor import bearing_surface
    from structure.frame import hand_axes

    h = hands()[50]
    x = baseline()
    q = h.compose({f: posture(h, f, tp_of(x, f), tm_of(x, f), x.get(f"ab_{f}", 0.0))
                   for f in FINGERS})
    P, N, K, T = bearing_surface(h, q)
    o, e_d, _, _ = hand_axes(h, q)
    d = (P - o) @ e_d

    assert (d.max() - d.min()) > 0.060, (
        f"the anchor has only {(d.max()-d.min())*1000:.0f} mm of extent along the lever. "
        "A hinge cannot carry a moment."
    )
    # the moment a keypress makes at the furthest button, against the patch's rotational stiffness
    r = d - d.mean()
    k_rot = float(np.sum(K * r**2))
    rock = (0.196 * 0.121) / k_rot * 0.121          # button movement from ROCKING alone
    assert rock < 50e-6, (
        f"the gauntlet rocks {rock*1e6:.0f} um on its anchor. That is most of the 500 um "
        "budget, and no amount of shell thickness will fix it."
    )
    assert T.min() > 0.0005 and T.max() < 0.010, f"implausible tissue thickness: {T.min()}..{T.max()}"


def test_the_flesh_model_is_measured_not_guessed():
    """WE FOUND A FLESH MODEL TO GO WITH THE BONES.

    The user: "I think we need to find a flesh model to go with the bones." It was the
    load-bearing gap. MyoHand ships bones and crude flesh CAPSULES -- and over the CARPUS,
    exactly where a gauntlet anchors, NO FLESH AT ALL. So `WRIST_TISSUE` was a GUESS (3 mm) and
    it set the stiffness of the whole structure's main anchor.

    Source: the PIANO hand-MRI dataset (Apache-2.0). We deliberately do NOT use NIMBLE, the
    parametric model built on it: its LICENSE.md is the unedited GitHub template, its weights
    sit on a Google Drive with no licence, and it emits MANO-topology vertices -- and MANO is
    Max Planck's non-commercial licence, which this project rejected on day one.

    MEASURED (ray-cast from each bone-surface voxel along its OWN outward normal):

        region                DORSAL    PALMAR      vs what we had
        wrist / carpus        6.8 mm    6.6 mm      3.0 mm  (a GUESS)
        metacarpals           4.8 mm    3.8 mm      1.4-3.1 (capsules)
        fingertips            2.8 mm    4.8 mm      -- pulp thicker than nail bed. Correct.

    THE TISSUE IS ~2x THICKER THAN ASSUMED, so the anchor was ~2x TOO STIFF -- the guess was
    flattering the design in the one place the structure hangs from.

    AND THE DESIGN TURNED OUT NOT TO CARE, which is the whole point of measuring: with the
    anchor 2.75x softer, the button moved 361 -> 376 um. 4%. The DISTRIBUTED PATCH had already
    made the structure insensitive to the number it used to hang on.
    """
    import numpy as np

    from design.vector import posture, tm_of, tp_of
    from hand.flesh import TISSUE, skin
    from hand.myohand import FINGERS
    from opt.problem import hands
    from opt.run import baseline

    # the anatomical sanity check that caught TWO wrong metrics on the way here
    assert TISSUE["carpus"][0] > TISSUE["metacarpal"][0], (
        "the wrist should carry more tissue than the back of the hand"
    )

    h = hands()[50]
    x = baseline()
    q = h.compose({f: posture(h, f, tp_of(x, f), tm_of(x, f), x.get(f"ab_{f}", 0.0))
                   for f in FINGERS})
    V, F = skin(h, q)
    assert len(V) > 10000 and len(F) > 10000, "the skin should be a real mesh"

    # the skin must ENCLOSE the bones it is wrapped around
    import mujoco

    m = h.model
    h.fk(q)
    b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "thirdmc")
    for g in range(m.body_geomadr[b], m.body_geomadr[b] + m.body_geomnum[b]):
        if m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        mid = m.geom_dataid[g]
        va, vn = m.mesh_vertadr[mid], m.mesh_vertnum[mid]
        B = (m.mesh_vert[va:va + vn] @ h.data.geom_xmat[g].reshape(3, 3).T
             + h.data.geom_xpos[g])[::11]
        d = np.array([float(np.min(np.linalg.norm(V - p, axis=1))) for p in B])
        assert d.max() < 0.012, f"a metacarpal pokes {d.max()*1000:.0f} mm out of the skin"
        break


def test_the_gauntlet_stands_off_the_SKIN_not_the_capsules():
    """The gauntlet is fitted to the hand's SKIN. It had been fitted to MyoHand's flesh CAPSULES.

    The user: "I think we need to find a flesh model to go with the bones."

    A capsule is a CIRCULAR TUBE. A hand is not: the metacarpals are FLAT and the fingers OVAL.
    A shell fitted to a tube either stands proud of the flats (bulk, and it gets in the way) or
    bites into the sides (it does not fit). And a capsule is not skin -- the measured dorsal
    tissue is 4.8-6.8 mm, roughly twice what MyoHand's capsules imply -- so the "4 mm standoff"
    was really the shell sitting INSIDE the hand.

    A skeleton is not what the device touches. This is the check that says so.
    """
    import numpy as np

    from design.vector import posture, tm_of, tp_of
    from hand.flesh import clearance_to_skin
    from hand.myohand import FINGERS
    from structure.gauntlet import domain

    h = MyoHand()
    hug = 0.004
    q = h.compose({f: posture(h, f, 0.45, 0.45, 0.0) for f in FINGERS})
    nodes, quads, _wells, _strap = domain(h, q, hug=hug)

    used = sorted({i for qd in quads for i in qd})
    gap = clearance_to_skin(h, q, np.asarray(nodes)[used])
    assert gap.min() > hug - 1e-4, (
        f"the gauntlet bites {(hug - gap.min())*1000:.1f} mm into the SKIN at its worst node. "
        f"A per-bone ring cannot see the NEIGHBOURING digit -- only the assembled skin can.")


def test_the_bone_axis_points_DISTALLY_even_on_the_thumb():
    """A capsule's local z is not a promise about which way is distal -- and on the thumb it lies.

    MuJoCo's capsule z runs distally on the fingers and PROXIMALLY on the whole thumb. Taken as
    given, the thumb's shell rings came out reversed, so `firstmc`'s LAST ring was stitched to
    `proximal_thumb`'s FAR end: a 43 mm leap across the hand, 66 shell elements with edges up to
    93 mm, and the fingertip WRAP -- the thing that carries the button -- capped on the wrong end
    of the bone. The solver integrated all of it without a murmur.

    The direction is now derived from the model's own kinematic tree (a body's frame sits at its
    PROXIMAL joint), so no one has to know which way z points.
    """
    import numpy as np

    from design.vector import posture, tm_of, tp_of
    from hand.flesh import skin
    from hand.myohand import FINGERS
    from structure.frame import hand_axes
    from structure.gauntlet import CHAIN, PALM, _bone_rings
    import mujoco

    h = MyoHand()
    q = h.compose({f: posture(h, f, 0.45, 0.45, 0.0) for f in FINGERS})
    m = h.model
    h.fk(np.zeros(m.nq))
    _, _, _, e_o0 = hand_axes(h, np.zeros(m.nq))
    dl = {}
    for bn in list(PALM) + [b for bs in CHAIN.values() for b in bs]:
        bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, bn)
        dl[bn] = h.data.xmat[bid].reshape(3, 3).T @ e_o0
    h.fk(q)
    Vs, _, Ls = skin(h, q, labels=True)

    for f, bones in CHAIN.items():
        for bn in bones:
            bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, bn)
            rings = _bone_rings(h, q, bn, dl, 0.004, 6, Vs, Ls, n_along=4)
            # the rings must march AWAY from the bone's own proximal joint, not toward it
            joint = h.data.xpos[bid]
            d0 = float(np.linalg.norm(rings[0][0] - joint))
            d1 = float(np.linalg.norm(rings[-1][0] - joint))
            assert d1 > d0, (
                f"{f}/{bn}: the shell rings run PROXIMALLY ({d0*1000:.0f} -> {d1*1000:.0f} mm "
                f"from the joint). The tip wrap, and the button on it, land on the wrong end.")


def test_no_shell_element_leaps_across_the_hand():
    """A 63 mm 'shell element' is not a mesh, it is a spike the solver integrates without complaint.

    Three separate defects each produced one, and NOT ONE of them changed a number enough to
    notice: the thumb's reversed rings (93 mm), an uncapped skin-clearance push that marched
    nodes 30 mm out into space (63 mm), and a per-direction radius taken as a percentile over a
    slab of BONE vertices, which lurched by 25 mm between ADJACENT arc directions whenever the
    slab clipped a condyle.

    They were caught by LOOKING at the render -- a black sheet stretched across the back of the
    hand. This test is what should have caught them.
    """
    import numpy as np

    from design.vector import posture
    from hand.myohand import FINGERS
    from structure.gauntlet import domain

    h = MyoHand()
    q = h.compose({f: posture(h, f, 0.45, 0.45, 0.0) for f in FINGERS})
    nodes, quads, _w, _s = domain(h, q)
    N = np.asarray(nodes)
    edge = np.array([max(np.linalg.norm(N[qd[(i + 1) % 4]] - N[qd[i]]) for i in range(4))
                     for qd in quads])
    assert edge.max() < 0.030, (
        f"a shell element spans {edge.max()*1000:.0f} mm. The domain's rings are ~7 mm apart, so "
        f"anything near 30 mm is a strip stitched to the wrong place.")


def test_a_singular_lattice_must_not_report_ZERO_deflection():
    """`max(0.0, nan)` is 0.0 in Python, and that is how a broken model reports a perfect score.

    A lattice sampled off a skin surface has floating islands -- knots of bars with no path back to
    an anchor. An island has six rigid-body modes and no restraint, so the stiffness matrix is
    singular; PyNite's sparse solve returns NaN rather than raising; and Python's `max` returns its
    FIRST argument whenever the comparison is False, which every comparison with NaN is. The model
    duly announced "buttons steady at 0 um". It survived HALVING THE BAR RADIUS unchanged, which is
    what gave it away: a real structure gets softer when you thin it.
    """
    import numpy as np

    from structure.lattice import solve

    # two bars, and a third that floats free of everything
    nodes = np.array([[0, 0, 0], [0.05, 0, 0], [0.10, 0, 0], [1.0, 1.0, 1.0], [1.0, 1.0, 1.05]],
                     dtype=float)
    bars = [(0, 1), (1, 2), (3, 4)]
    anchor_k = {0: 1e6}
    anchor_n = {0: np.array([0.0, 0.0, 1.0])}
    buttons = {"index": 3}                     # a button ON the island: no load path at all
    cases = [("index", "click", {3: np.array([0.0, 0.0, -0.2])})]

    w, *_ = solve(nodes, bars, [0, 1, 2], buttons, cases, anchor_k, anchor_n)
    assert not np.isfinite(w), (
        f"a button with NO load path to the anchor reported {w*1e6:.1f} um. A singular solve must "
        f"come back as inf, never as a number -- least of all as zero.")


def test_flesh_cannot_PULL_the_gauntlet_back_onto_the_hand():
    """The anchor is BILINEAR, and modelling it as a bidirectional spring is fiction.

    A keypress at a fingertip ~120 mm from the wrist is a MOMENT: it presses one end of the anchor
    patch INTO the hand and lifts the other end OFF it. Flesh can resist the first and not the
    second. Give the solver springs that pull and it will happily hang the whole structure from
    them -- the free-form lattice grew a 7.3 g design reporting 495 um at the buttons, of which
    40% of the anchor reaction was tension that nothing was supplying. Re-solved honestly, the
    same structure deflects 9178 um.

    So a node lifting OFF must feel the STRAP (soft webbing), not the tissue (stiff), and the
    difference has to show up in the answer.
    """
    import numpy as np

    from structure.lattice import STRAP_K, solve

    # a cantilever off a single anchored node, loaded so the anchor is pulled outward (+z)
    nodes = np.array([[0, 0, 0], [0.03, 0, 0], [0.06, 0, 0]], dtype=float)
    bars = [(0, 1), (1, 2)]
    anchor_k = {0: 1e6}                        # stiff tissue
    anchor_n = {0: np.array([0.0, 0.0, 1.0])}  # outward = +z
    buttons = {"index": 2}

    up, _s, _m, t_up, _p = solve(
        nodes, bars, [0, 1], buttons,
        [("index", "click", {2: np.array([0.0, 0.0, +0.2])})], anchor_k, anchor_n)
    dn, _s, _m, t_dn, _p = solve(
        nodes, bars, [0, 1], buttons,
        [("index", "click", {2: np.array([0.0, 0.0, -0.2])})], anchor_k, anchor_n)

    assert up > dn, (
        f"pulling the anchor OFF the hand ({up*1e6:.0f} um) is no softer than pressing it IN "
        f"({dn*1e6:.0f} um). The tissue spring is still bidirectional, so the structure is being "
        f"held on by a force nothing supplies.")
    assert t_up > 0 and t_dn == 0.0, (
        f"the strap carries {t_up:.3f} N when lifted and {t_dn:.3f} N when pressed. It must carry "
        f"the lift and nothing else -- webbing does not push.")

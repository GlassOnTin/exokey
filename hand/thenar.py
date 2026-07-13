"""The muscles MyoHand is missing. Chiefly ADDUCTOR POLLICIS -- the thumb's press muscle.

WHY. Measured, on the shipped model, two independent instruments agreeing:

    verify/strength.py   the thumb can exert  0.0 N  at its pad, at EVERY posture swept
    the equilibrium residual   the thumb can perform 0 of its 5 directions, ANYWHERE

Not "the thumb is weak". The thumb CANNOT PRESS. And the muscle list says why:

    FPL   flexes            EPL, EPB   extend            APL   abducts            OP   opposes

There is no ADDUCTOR. A thumb with no adductor cannot push *against* anything -- and pushing
against something is what pressing a key IS. Adductor pollicis is the muscle of key pinch;
without it the model is not an approximate thumb, it is a thumb with the relevant muscle
deleted. This is why thumb effort came out ~1000x the index: the optimiser was pricing an
action the digit cannot perform.

MyoSuite ships no alternative -- checked every XML in myo_sim, hand and arm: the only thumb
muscles anywhere are EPL, EPB, APL, FPL, OP. MUSIC (Xu et al., arXiv 2604.23886) augments
MyoHand with exactly FPB/APB/AdP (+FDM/ADM) for MuJoCo piano playing, but no model is
released. So we rebuild it, from the same sources MyoHand's own parameters come from.

WHAT IS DERIVED AND WHAT IS NOT -- read this before trusting any number downstream.

  DERIVED from the model's own geometry:
    * the frame of each bone: on `thirdmc` the model's OWN tendons say which way is which --
      FDS3/FDP3 (flexors) run at z<0, so -z is PALMAR; EDC3 (extensor) at z>0, so +z is
      DORSAL; RI3 (radial interosseous) inserts at x>0, so +x is RADIAL; sites run from
      y>0 proximal to y<0 distal. Nothing here is assumed.
    * the insertion point: placed on the surface of the thumb's proximal phalanx FACING the
      origin, computed in world coords and mapped back into the body frame. A muscle attaches
      to the side of the bone it pulls from; deriving it that way cannot put the tendon
      inside the bone.

  LITERATURE, and NOT read from a table by me:
    * ⚠ PCSA_RATIO. Adductor pollicis has a physiological cross-sectional area roughly 1.25x
      the flexor pollicis longus (ADP ~2.5 cm^2, FPL ~2.0 cm^2). This is RECALLED, not read.
      Peak force is anchored to FPL's own F0 by that ratio, which cancels the specific-tension
      constant and keeps ADP consistent with however MyoHand scaled everything else.
      The claim "the thumb can now press" MUST be shown to survive a sweep of this number --
      see `sweep_peak_force`. If it does not, the result is an artefact of a guess.

WHAT THIS IS NOT. It is not a validated musculoskeletal model of the thumb. It is one muscle,
placed from the model's own anatomy, whose PURPOSE is testable: the thumb should go from
"cannot press at all" to "can press", and the resulting force should be humanly plausible.
If it does not do that, it is wrong and must be thrown away, not tuned until it agrees.
"""
from __future__ import annotations

import mujoco
import numpy as np

HAND_XML = None  # set by caller; see build_spec

# ⚠ LITERATURE, RECALLED NOT READ. See module docstring. Swept in sweep_peak_force().
PCSA_RATIO_ADP_OVER_FPL = 1.25   # ADP ~2.5 cm^2 vs FPL ~2.0 cm^2
PCSA_RATIO_FPB_OVER_FPL = 0.5    # flexor pollicis brevis
PCSA_RATIO_APB_OVER_FPL = 0.5    # abductor pollicis brevis


def _body_frame_conventions(spec_model, data) -> None:
    """No-op placeholder: the conventions are asserted by tests, not by code."""


def _add_thenar_muscle(spec, name, origin_body, origin_local, f0, side):
    """Place one thenar muscle: origin on `origin_body`, insertion on the thumb's proximal
    phalanx, on the flank given by `side` (+1 = the side FACING the origin, i.e. ulnar for
    ADP; -1 = the opposite flank, radial, for FPB/APB).

    THE INSERTION IS DERIVED, NOT TYPED, and the derivation is the same move `_pad_frame`
    uses for `palmar`: take the direction to the origin, PROJECT OUT the along-bone part, and
    step off the bone axis by its radius.

    Getting that projection wrong is not cosmetic. The first attempt put the insertion on the
    straight line from the MP joint centre to the origin -- so the tendon passed THROUGH the
    joint and its mp_flexion moment arm came out EXACTLY 0.0000. The muscle was anatomically
    placed and mechanically inert at the joint it exists to move: a zero that reads like a
    rounding error and is actually a modelling one.
    """
    fpl = next(a for a in spec.actuators if a.name == "FPL")

    spec.body(origin_body).add_site(name=f"{name}-P1", pos=np.asarray(origin_local, float))

    tmp = spec.compile()
    d = mujoco.MjData(tmp)
    mujoco.mj_forward(tmp, d)
    o_w = d.site_xpos[mujoco.mj_name2id(tmp, mujoco.mjtObj.mjOBJ_SITE, f"{name}-P1")].copy()

    pid = mujoco.mj_name2id(tmp, mujoco.mjtObj.mjOBJ_BODY, "proximal_thumb")
    did = mujoco.mj_name2id(tmp, mujoco.mjtObj.mjOBJ_BODY, "distal_thumb")
    R = d.xmat[pid].reshape(3, 3)
    p_w = d.xpos[pid].copy()      # base of the proximal phalanx == the MP joint
    ip_w = d.xpos[did].copy()     # the IP joint: the bone's own axis is ip - mp

    axis = ip_w - p_w
    L = float(np.linalg.norm(axis))
    axis /= L + 1e-12

    u = o_w - p_w
    u = u - (u @ axis) * axis     # kill the along-bone part: we want the SIDE of the bone
    u /= np.linalg.norm(u) + 1e-12

    r_bone = 0.005                # m: on the surface of the phalanx, not inside it
    ins_w = p_w + 0.25 * L * axis + side * r_bone * u
    spec.body("proximal_thumb").add_site(name=f"{name}-P2", pos=R.T @ (ins_w - p_w))

    # A VIA POINT ON THE THUMB'S OWN METACARPAL. Without it the tendon is a STRAIGHT LINE
    # across open space, passing far from the CMC joint -- and the moment arm comes out
    # ENORMOUS: measured 22.7 mm about CMC abduction and 21.6 mm about CMC flexion, against
    # a published 10-15 mm for adduction and essentially nothing for flexion. ADP became a
    # wrecking bar: every time it supplied the MP flexion a press needs, it dumped a huge
    # unwanted CMC torque that no other thumb muscle could cancel. That is precisely the cone
    # geometry that left an irreducible residual and kept the thumb unable to press.
    #
    # Real ADP wraps around the first metacarpal through the web space, and MyoHand's own
    # muscles route this way (FPL has P7/P8 on `firstmc`). Derived, not typed: a point on the
    # surface of the first metacarpal, near its DISTAL end (so it hugs the bone all the way to
    # the insertion), on the flank that faces the origin.
    mc1 = mujoco.mj_name2id(tmp, mujoco.mjtObj.mjOBJ_BODY, "firstmc")
    R1 = d.xmat[mc1].reshape(3, 3)
    c1 = d.xpos[mc1].copy()                    # the CMC joint: base of the first metacarpal
    v = p_w - c1                               # along the metacarpal, toward the MP joint
    Lmc = float(np.linalg.norm(v))
    v /= Lmc + 1e-12
    w = o_w - c1
    w = w - (w @ v) * v                        # the flank of the metacarpal facing the origin
    w /= np.linalg.norm(w) + 1e-12
    via_w = c1 + 0.80 * Lmc * v + 0.006 * w    # distal on the bone, on its surface
    spec.body("firstmc").add_site(name=f"{name}-Pv", pos=R1.T @ (via_w - c1))

    t = spec.add_tendon(name=f"{name}_tendon")
    t.wrap_site(f"{name}-P1")
    t.wrap_site(f"{name}-Pv")
    t.wrap_site(f"{name}-P2")

    a = spec.add_actuator(name=name)
    a.target = f"{name}_tendon"
    a.trntype = mujoco.mjtTrn.mjTRN_TENDON
    a.gaintype = mujoco.mjtGain.mjGAIN_MUSCLE
    a.biastype = mujoco.mjtBias.mjBIAS_MUSCLE
    a.dyntype = mujoco.mjtDyn.mjDYN_MUSCLE
    a.ctrlrange = [0.0, 1.0]
    # inherit MyoHand's own Hill parameters off FPL, then set peak force. If MyoHand changes
    # its muscle conventions these follow, instead of silently disagreeing with the model
    # they live in.
    a.gainprm = list(fpl.gainprm)
    a.biasprm = list(fpl.biasprm)
    a.dynprm = list(fpl.dynprm)
    a.gainprm[2] = f0
    a.biasprm[2] = f0
    return spec


def add_thenar(spec, adp_force: float | None = None):
    """Add the three missing thenar muscles: ADP, FPB, APB.

    ⚠ PEAK FORCES are anchored to FPL's own F0 by PCSA ratio -- which cancels the specific
    tension constant, so ADP stays consistent with however MyoHand scaled everything else.
    The ratios (ADP ~1.25x FPL; FPB and APB ~0.5x) are LITERATURE, RECALLED NOT READ, and the
    conclusions must be shown to survive a sweep of them. See sweep_peak_force().
    """
    fpl = next(a for a in spec.actuators if a.name == "FPL")
    f0_fpl = float(fpl.gainprm[2]) or 201.0
    adp = adp_force if adp_force is not None else PCSA_RATIO_ADP_OVER_FPL * f0_fpl

    # ADP: transverse head off the PALMAR shaft of the 3rd metacarpal, inserting on the
    # ULNAR flank of the thumb's proximal phalanx (side=+1: the flank facing the origin).
    #   thirdmc frame, read off the model's OWN tendons: -y distal, -z palmar, +x radial
    _add_thenar_muscle(spec, "ADP", "thirdmc", [0.003, -0.012, -0.009], adp, side=+1)

    # FLEXOR POLLICIS BREVIS. It is not a refinement -- it is THE MISSING DEGREE OF FREEDOM.
    #
    # With ADP alone the thumb still could not press, and the shortfall was 77% on ONE joint:
    # mp_flexion. The reason is a trap in the muscle set:
    #
    #   FPL  flexes the MP, but ALSO crosses the IP (-1.70). The IP is already over-flexed,
    #        so FPL cannot fire any harder without over-flexing it further.
    #   EPL  could cancel that excess IP flexion -- but it EXTENDS the MP (+0.81), undoing the
    #        very thing we need.
    #   ADP  flexes the MP with a ZERO IP arm, which is right -- but its CMC arms are 3-4x its
    #        MP arm, so it saturates the CMC long before it supplies enough MP torque.
    #
    # NOTHING FLEXES THE MP WITHOUT ALSO FLEXING THE IP. In a real thumb exactly one muscle
    # does: FPB. It inserts on the PROXIMAL phalanx (so it cannot cross the IP) and arises from
    # the trapezium right beside the CMC (so its CMC arms are small). That combination is
    # unavailable in stock MyoHand, and it is the whole blockage.
    #
    # ⚠ A FIRST ATTEMPT AT FPB FAILED, AND THE FAILURE IS INSTRUCTIVE. Its insertion was offset
    # RADIALLY, and it came out EXTENDING the MP (+0.14) -- the joint FPB exists to flex. The
    # MP flexion moment arm comes from the tendon passing PALMAR to the joint, not radial to
    # it. So `palmar` is now DERIVED for the proximal phalanx exactly as `_pad_frame` derives it
    # for the distal one: from the model's own flexor and extensor tendons (FPL runs palmar,
    # EPL runs dorsal), with the along-bone component projected out.
    fpb_f0 = PCSA_RATIO_FPB_OVER_FPL * f0_fpl
    _add_flexor_brevis(spec, "FPB", "trapezium", [-0.006, -0.010, -0.008], fpb_f0)

    # ABDUCTOR POLLICIS BREVIS. Added, and the reason I first REFUSED to add it was wrong.
    #
    # I dismissed APB as "it abducts; it does not supply MP flexion, so it does not touch the
    # blockage". That is false. APB inserts on the PROXIMAL PHALANX, like FPB, so it FLEXES
    # THE MP -- and it does so while ABDUCTING the CMC. That combination is the one thing the
    # thumb was missing, and here is why it matters:
    #
    # With ADP + FPB, the MP-flexing muscles were firing at activations of 0.007-0.019 -- not
    # remotely saturated. They were not weak; they were CAPPED BY cmc_abduction, which was
    # already OVERSHOOTING (-0.0623 achieved against -0.0599 needed). EVERY muscle that flexes
    # the MP also ADDUCTS the CMC, so more MP flexion meant more unwanted adduction, and the
    # solver had to stop. APB pushes the CMC the OTHER WAY while still flexing the MP, which
    # releases the cap.
    #
    # ⚠ ITS MP MOMENT ARM IS UNDERSTATED. Fitted to published anatomy (APB is the thumb's main
    # abductor: CMC abduction 15-20 mm, MP flexion ~5 mm), a straight two-site tendon cannot
    # reproduce both -- as its abduction arm grows toward the published value, its MP arm
    # shrinks. Real APB routes through the RADIAL SESAMOID, which a straight line has no way to
    # represent. Taken: CMC abduction 12.7 mm (in the published band), MP flexion 1.9 mm
    # (published ~5, so LOW). The thumb is therefore modelled WEAKER at MP flexion than a real
    # one, which is the conservative direction.
    _add_flexor_brevis(spec, "APB", "trapezium", [0.020, 0.000, 0.002],
                       PCSA_RATIO_APB_OVER_FPL * f0_fpl)
    return spec


def _palmar_of_proximal_thumb(spec_compiled, data) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(base, bone axis, PALMAR unit vector) of the thumb's proximal phalanx, in world.

    Palmar is DERIVED from the model's own tendons -- FPL runs palmar of the bone, EPL runs
    dorsal of it -- with the along-bone component projected out. Exactly the recipe
    `MyoHand._pad_frame` uses for the distal phalanx, and for the same reason: a bone's
    palmar side is not something to guess at when the model states it.
    """
    m, d = spec_compiled, data

    def site(nm):
        return d.site_xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, nm)].copy()

    pid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "proximal_thumb")
    did = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "distal_thumb")
    base, tip = d.xpos[pid].copy(), d.xpos[did].copy()
    axis = tip - base
    axis /= np.linalg.norm(axis) + 1e-12

    v = site("FPL-P10") - site("EPL-P11")     # flexor side minus extensor side = dorsal->palmar
    v = v - (v @ axis) * axis
    return base, axis, v / (np.linalg.norm(v) + 1e-12)


def _add_flexor_brevis(spec, name, origin_body, origin_local, f0):
    """A short thumb flexor: origin on the carpus, insertion PALMAR on the proximal phalanx.

    Palmar is what gives it an MP FLEXION moment arm. Offset it radially instead and the
    muscle comes out EXTENDING the MP, which is what happened the first time.
    """
    fpl = next(a for a in spec.actuators if a.name == "FPL")
    spec.body(origin_body).add_site(name=f"{name}-P1", pos=np.asarray(origin_local, float))

    tmp = spec.compile()
    d = mujoco.MjData(tmp)
    mujoco.mj_forward(tmp, d)

    base, axis, palmar = _palmar_of_proximal_thumb(tmp, d)
    pid = mujoco.mj_name2id(tmp, mujoco.mjtObj.mjOBJ_BODY, "proximal_thumb")
    R = d.xmat[pid].reshape(3, 3)

    # on the PALMAR face of the base of the phalanx: that is the lever for MP flexion
    ins_w = base + 0.20 * float(np.linalg.norm(d.xpos[
        mujoco.mj_name2id(tmp, mujoco.mjtObj.mjOBJ_BODY, "distal_thumb")] - base)) * axis \
        + 0.005 * palmar
    spec.body("proximal_thumb").add_site(name=f"{name}-P2", pos=R.T @ (ins_w - base))

    t = spec.add_tendon(name=f"{name}_tendon")
    t.wrap_site(f"{name}-P1")
    t.wrap_site(f"{name}-P2")

    a = spec.add_actuator(name=name)
    a.target = f"{name}_tendon"
    a.trntype = mujoco.mjtTrn.mjTRN_TENDON
    a.gaintype = mujoco.mjtGain.mjGAIN_MUSCLE
    a.biastype = mujoco.mjtBias.mjBIAS_MUSCLE
    a.dyntype = mujoco.mjtDyn.mjDYN_MUSCLE
    a.ctrlrange = [0.0, 1.0]
    a.gainprm = list(fpl.gainprm)
    a.biasprm = list(fpl.biasprm)
    a.dynprm = list(fpl.dynprm)
    a.gainprm[2] = f0
    a.biasprm[2] = f0
    return spec


def build_spec(xml_path: str, peak_force: float | None = None):
    """Load MyoHand and add the missing thumb adductor. The submodule stays pristine."""
    spec = mujoco.MjSpec.from_file(xml_path)
    add_thenar(spec, adp_force=peak_force)
    return spec

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

    # FPB and APB: NOT ADDED. Attempted, measured, and thrown away.
    #
    # Placed from the trapezium onto the RADIAL flank of the proximal phalanx, they came out
    # with a POSITIVE mp_flexion moment arm (+0.14, +0.16) -- i.e. they EXTEND the joint that
    # flexor pollicis brevis exists to FLEX (FPL's arm is -1.42). And they moved the thumb's
    # irreducible residual by NOTHING: 11.9% with them, 11.9% without.
    #
    # The fix would be to slide the attachments until the moment arms come out right. That is
    # exactly what this module's docstring forbids: "it is wrong and must be thrown away, not
    # tuned until it agrees". Their real tendons pass PALMAR to the MP joint via the sesamoid,
    # and reproducing that needs attachment data I do not have -- not a plausible-looking
    # coordinate I chose because it gave me the answer I wanted.
    #
    # ADP alone is kept, because ADP alone is VERIFIED: its moment arms have the right signs
    # (adducts the CMC like OP, flexes the MP like FPL, zero at the IP because it does not
    # cross it), it cuts the thumb residual 45.6% -> 11.9%, and it leaves all four fingers
    # bit-for-bit unchanged. One muscle that is right beats three that flatter the result.
    return spec


def build_spec(xml_path: str, peak_force: float | None = None):
    """Load MyoHand and add the missing thumb adductor. The submodule stays pristine."""
    spec = mujoco.MjSpec.from_file(xml_path)
    add_thenar(spec, adp_force=peak_force)
    return spec

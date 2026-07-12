"""Hand-size population. Stage 1's missing half.

MyoHand is ONE generic hand. The device must fit the 5th-95th percentile, so we need a
population, and scaling a hand is not just scaling its bones -- it has to scale its
STRENGTH too, or a big hand comes out as a long weak one.

    lengths   (bones, tendon paths, moment arms)   ~ s
    muscle force (peak isometric ~ PCSA, an area)  ~ s^2
    => joint torque capacity                       ~ s^3

    press demand = J^T F, and J is a moment arm    ~ s

so activation for a given key force goes as s / s^3 = s^-2: bigger hands are relatively
STRONGER. That falls out of the scaling; it is not imposed. It is also the thing that
makes the population problem interesting -- the small hand is the binding one for effort
and saturation, while the large hand is the binding one for reach and collision.

Scaling law: Buchholz, Armstrong & Goldstein (1992) give phalanx lengths as regressions on
hand length, and to first order the proportions are constant -- so uniform isotropic
scaling by hand length IS Buchholz to first order. Anisotropic (per-segment) scaling is a
refinement we have NOT done.
"""
from __future__ import annotations

import numpy as np

# ---------------------------------------------------------------------------------------
# ⚠ NOT VERIFIED. These are approximate ANSUR II hand-length percentiles (combined sex,
# mm) recalled from summary statistics, NOT read from the dataset. The RATIOS are what the
# model uses, and the 95th/5th ratio (~1.24) is the load-bearing number; check it against
# the real ANSUR II release before any published claim. ANSUR II is public domain.
# ---------------------------------------------------------------------------------------
ANSUR_HAND_LENGTH_MM = {5: 165.0, 25: 176.0, 50: 185.0, 75: 194.0, 95: 205.0}

REFERENCE_PERCENTILE = 50  # the unscaled MyoHand is taken to BE the median hand


def population(percentiles=(5, 25, 50, 75, 95)) -> dict[int, float]:
    """Percentile -> scale factor relative to the reference (median) hand."""
    ref = ANSUR_HAND_LENGTH_MM[REFERENCE_PERCENTILE]
    return {p: ANSUR_HAND_LENGTH_MM[p] / ref for p in percentiles}


def scale_model(model, s: float) -> None:
    """Scale a MuJoCo hand model isotropically by `s`, IN PLACE.

    Lengths ~ s, muscle peak force ~ s^2, mass ~ s^3, inertia ~ s^5.
    """
    if abs(s - 1.0) < 1e-12:
        return

    # geometry
    model.body_pos[:] *= s
    model.body_ipos[:] *= s
    model.geom_pos[:] *= s
    model.geom_size[:] *= s
    model.site_pos[:] *= s
    model.mesh_vert[:] *= s  # visual + the flesh capsules we do clearance against

    # tendon paths and the muscle length normalisation that goes with them
    model.actuator_lengthrange[:] *= s
    if model.ntendon:
        model.tendon_lengthspring[:] *= s
        model.tendon_range[:] *= s

    # muscle peak isometric force ~ physiological cross-sectional area ~ s^2.
    # gainprm layout for mjGAIN_MUSCLE: [range0, range1, FORCE, scale, lmin, lmax, ...]
    # `force` is positive here, so MuJoCo uses it directly rather than deriving it.
    model.actuator_gainprm[:, 2] *= s**2
    model.actuator_biasprm[:, 2] *= s**2

    # inertial (statics with gravity off barely uses these, but keep them consistent)
    model.body_mass[:] *= s**3
    model.body_inertia[:] *= s**5

    # joint ranges are ANGLES -- scale-invariant. Deliberately untouched.

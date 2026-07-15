"""The finger-well SENSOR as a printed flexure.  [contactless Hall: magnet + 3-axis Hall]

Replaces the Svalboard 20 gf magneto-optical key (design.params.SVALBOARD) with a magnet on a
compliant flexure over a 3-axis Hall (TLV493D-class). The restoring force IS the flexure, and it
must be SOFT -- k = F/travel ~= 130 N/m -- in all five well directions (hand.cradle.SENSED).

Two things the arithmetic settles (scripts/flexure.py sweeps the material palette):

  1. STIFF THERMOPLASTICS CANNOT BE A SOFT ISOTROPIC FLEXURE. The bending strain to give 20 gf at
     1.5 mm exceeds their fatigue strain sigma_fat/E, so a rod or dome soft enough for the key runs
     past its own fatigue limit and cracks. PLA / PETG / ASA / glass-nylon all lose. The
     soft-flexure materials are TPU (a DOME -- high fatigue strain, and isotropic so it does all
     five senses in one part) and thin SPRING STEEL (a LEAF or cruciform -- go thin enough and its
     huge fatigue limit has headroom to spare, where an FDM plastic floors at ~0.3 mm).

  2. THE PLUNGE IS NOT AXIAL COMPRESSION. Even TPU is ~100x too stiff in compression, so the
     down/click cannot be the flexure shortening -- it must be a BENDING mode. That is why the
     one-part answer is a dome: soft in tilt AND in plunge, and it snaps for a tactile click.

sigma_fat / E -- the maximum recoverable bending strain -- is the whole story, and it is why an
elastomer or a thin metal leaf is a flexure and a stiff printed plastic is not.

⚠ NOT MODELLED HERE: the magnetic read-out (the field a moving magnet presents to the Hall), the
tactile snap of a shallow dome (that is buckling, not linear plate bending), and multi-axis
cross-talk. This file sizes the RESTORING SPRING only. The moment-and-lever optimisation the user
asked for -- tuning each finger's rate so its five directions land in that finger's comfortable
effort band (hand.cradle.solve) across the population -- builds on these, and is not done yet.
"""
from __future__ import annotations

import numpy as np


def spring_rate(force: float, travel: float) -> float:
    """How soft the flexure must be to actuate at `force` after `travel`: k = F/travel."""
    return force / travel


def rod(k: float, L: float, E: float, travel: float) -> tuple[float, float]:
    """Isotropic round cantilever flexure -- soft in EVERY lateral direction -- sized to spring
    rate `k` at length `L`. Returns (radius, root surface stress at full `travel`).

    sigma = 3 E travel r / L^2, with r fixed by k = 3EI/L^3 and I = pi r^4 / 4. The stress does
    not depend on the radius you solve for -- it is set by E, the travel, and the slenderness.
    """
    I = k * L ** 3 / (3.0 * E)
    r = (4.0 * I / np.pi) ** 0.25
    return r, 3.0 * E * travel * r / L ** 2


def axial_k(k: float, L: float, E: float) -> float:
    """Plunge stiffness of that same rod in PURE COMPRESSION. It comes out ~100-1000x too stiff,
    which is why the down-press has to bend something, not shorten a column."""
    r, _ = rod(k, L, E, 1.0)
    return E * np.pi * r ** 2 / L


def dome(k: float, a: float, E: float, nu: float) -> float:
    """Clamped circular diaphragm of radius `a` sized to plunge spring rate `k`. Returns thickness.

    k = 16 pi D / a^2,  D = E t^3 / (12 (1 - nu^2)).  Linear small-deflection plate theory; the
    real dome is shallow-conical and snaps (buckles), which this does not capture -- it sizes the
    membrane, not the click.
    """
    D = k * a ** 2 / (16.0 * np.pi)
    return (12.0 * (1.0 - nu ** 2) * D / E) ** (1.0 / 3.0)


def dome_stress(force: float, t: float) -> float:
    """Edge bending stress of the diaphragm under the central press (Roark, central patch load
    ~ 3F / (2 pi t^2)). Approximate -- a true point load is singular."""
    return 3.0 * force / (2.0 * np.pi * t ** 2)


def leaf(k: float, L: float, h: float, E: float, travel: float) -> tuple[float, float]:
    """A thin rectangular leaf, thickness `h` in the bending direction, sized to `k` at length `L`.
    Returns (width, surface strain at full `travel`) -- the spring-steel route.

    A leaf is soft in ONE bending axis and stiff in the other, so it cannot on its own do a
    two-axis-plus-plunge joystick; it takes a cruciform or a spider of leaves. But it is how a
    stiff, high-fatigue material (spring steel) becomes a soft flexure: drive the strain
    3 h travel / (2 L^2) below sigma_fat/E by making h small, which a shim can and a nozzle cannot.
    """
    w = 4.0 * k * L ** 3 / (E * h ** 3)          # from k = 3 E (w h^3 / 12) / L^3
    strain = 3.0 * h * travel / (2.0 * L ** 2)
    return w, strain


def fatigue_strain(fatigue: float, E: float) -> float:
    """The material's maximum recoverable bending strain, sigma_fat / E -- the flexure merit."""
    return fatigue / E

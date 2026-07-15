"""THE INNER BEARING SHELL -- a plate on the soft tissue, and an IMPACT distributor.

The gauntlet is worn all day and strapped to the hand, so every knock on it is a load path straight
INTO the flesh. The interface that carries that load does not exist in the structural model yet: the
bone bears through an ABSTRACT distributed spring hanging ~5.5 mm below the structure (nothing
touches the skin). Build that naively as feet on the nodes and it is a set of pressure points -- a
50 N knock through one 1.5 mm foot is on the order of a megapascal. A conformal SHELL spreads it.

The shell is a stiff PLATE on a soft ELASTIC FOUNDATION (the tissue, a Winkler spring
k = E_tissue / thickness). A concentrated load spreads over a characteristic length
lambda = (D/k)^{1/4}, and the peak skin pressure under it is P / (8 lambda^2). So the shell
THICKNESS sets how far a knock spreads -- and it is the IMPACT, not the steady strap preload, that
sets the thickness (the preload is nearly free; a 50 N knock is not).

⚠ THREE SIMPLIFICATIONS, all optimistic, all flagged:
  * QUASI-STATIC: a real impact adds a dynamic amplification (energy, contact time, masses) this
    does not model -- so the knock pressures are a LOWER bound on the peak.
  * INFINITE PLATE: a real shell has edges; a load near an edge spreads less than this predicts.
  * WINKLER: tissue is modelled as an array of independent linear springs, not a continuum.
These make it an order-of-magnitude sizing tool, not a final stress number.
"""
from __future__ import annotations

import numpy as np

# comfort/injury references (Pa). LITERATURE-ish, and the impact magnitude is a GUESS -- flagged.
CAPILLARY = 4.3e3      # capillary-occlusion pressure; the all-day pressure-sore threshold
COMFORT = 20e3         # a rough ceiling for a comfortable worn strap/pad
KNOCK_N = 50.0         # a firm accidental knock, as a quasi-static equivalent point force. GUESS.


def foundation_k(E_tissue: float, thickness: float) -> float:
    """Winkler foundation modulus of the soft tissue: pressure per unit deflection (N/m^3)."""
    return E_tissue / thickness


def plate_D(E: float, t: float, nu: float) -> float:
    """Flexural rigidity of a plate of thickness t (N·m)."""
    return E * t ** 3 / (12.0 * (1.0 - nu ** 2))


def char_length(D: float, k: float) -> float:
    """The length over which a point load spreads on an elastic foundation: lambda = (D/k)^{1/4}."""
    return (D / k) ** 0.25


def point_pressure(P: float, lam: float) -> float:
    """Peak skin pressure under a concentrated load P on an infinite Winkler plate: P / (8 lambda^2)."""
    return P / (8.0 * lam ** 2)


def shell_pressure(P: float, t: float, E: float, nu: float, k: float) -> tuple[float, float]:
    """(peak skin pressure, spreading lambda) for load P through a shell of thickness t."""
    lam = char_length(plate_D(E, t, nu), k)
    return point_pressure(P, lam), lam

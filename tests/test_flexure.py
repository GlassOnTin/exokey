"""The finger-well flexure findings, as executable claims.

If someone later tries to print the flexure in glass-nylon, or make the plunge an axial column,
these fail loudly. That is the whole point of writing them down.
"""
import numpy as np

from design.params import SVALBOARD
from manufacture.flexure import (axial_k, dome, dome_stress, fatigue_strain, leaf, rod,
                                 spring_rate)
from structure.frame import MATERIALS

F = float(SVALBOARD.force)          # 0.196 N  (20 gf)
TRAVEL = float(SVALBOARD.travel)    # 1.5 mm
K = spring_rate(F, TRAVEL)          # ~131 N/m -- a very soft spring


def test_the_target_is_a_soft_spring():
    assert 100 < K < 200             # 20 gf over 1.5 mm is order 100 N/m, not 1000s


def _best_rod_stress(name):
    """Lowest root stress a soft-enough rod of this material reaches, over well-length choices."""
    m = MATERIALS[name]
    return min(rod(K, L, m["E"], TRAVEL)[1] for L in (0.010, 0.015, 0.021))


def test_stiff_thermoplastics_are_not_a_durable_soft_isotropic_flexure():
    """Soft enough for the key, a stiff FDM plastic has no fatigue headroom as a rod or dome.

    Glass-nylon, PLA, PETG and ASA run OVER their fatigue limit outright. Plain PA12 is the one
    exception -- a long slender rod squeaks under (13 vs 16 MPa) -- but with no safety factor,
    which for a switch pressed millions of times is not a durable flexure. None clear a 1.5x
    fatigue margin, which is what separates them from TPU and a thin steel leaf.
    """
    for name in ("cf_pa12", "pla", "petg", "asa"):          # crack outright
        assert _best_rod_stress(name) > MATERIALS[name]["fatigue"], name
    m = MATERIALS["pa12"]                                    # the marginal exception
    assert m["fatigue"] / 1.5 < _best_rod_stress("pa12") < m["fatigue"]


def test_tpu_is_the_isotropic_flexure_material():
    """TPU as a rod OR a dome sits well under its fatigue limit and fits the ~7 mm well."""
    m = MATERIALS["tpu"]
    _, s_rod = rod(K, 0.015, m["E"], TRAVEL)
    assert s_rod < m["fatigue"]
    a = 0.006                                   # dome radius, inside the flesh-radius well
    t = dome(K, a, m["E"], m["nu"])
    assert dome_stress(F, t) < m["fatigue"]
    assert a < 0.007 and 0.15e-3 < t < 0.6e-3   # fits, and a near-printable membrane


def test_spring_steel_works_only_as_a_thin_leaf():
    """A steel ROD soft enough for the key over-stresses; a thin steel LEAF has headroom.

    This is why the flexure topology, not just the material, is the design: go thin (a shim) and
    steel's huge fatigue limit is fine; stay a rod and it cracks like the plastics.
    """
    m = MATERIALS["spring_steel"]
    _, s_rod = rod(K, 0.015, m["E"], TRAVEL)
    assert s_rod > m["fatigue"]                       # the rod fails
    _, strain = leaf(K, 0.015, 0.15e-3, m["E"], TRAVEL)   # a 0.15 mm shim leaf
    assert strain < fatigue_strain(m["fatigue"], m["E"])  # the leaf survives


def test_the_plunge_cannot_be_axial_compression():
    """For every candidate material the sized rod is far too stiff to press by compressing it."""
    for name in ("cf_pa12", "tpu", "spring_steel"):
        assert axial_k(K, 0.015, MATERIALS[name]["E"]) > 50 * K

"""The finger-well actuation findings, as executable claims.

The wells are not uniformly five-way: measured across the population, the lateral tilts are only
actuable on the radial digits. If that stops being true (a better hand model, a different design),
these fail and we look again.
"""
import pickle

import numpy as np

from design.params import RESIDUAL_MAX
from design.qwerty import ACTIONS
from design.sensor import actuation_cost
from hand.myohand import FINGERS
from opt.problem import hands

RMAX = float(RESIDUAL_MAX)
GF = 9.80665e-3


def _load():
    x = pickle.load(open("out/final_design.pkl", "rb"))["x"]
    return hands((5, 50, 95)), x


def _feasible(c):
    return c["residual"] <= RMAX


def test_effort_rises_with_actuation_force():
    """Gravity is off, so effort is press cost and must be monotonic in force -- a locked property
    of the whole model, re-checked in the sensor context."""
    H, x = _load()
    lo = actuation_cost(H, x, 10 * GF)
    hi = actuation_cost(H, x, 30 * GF)
    # over every (finger, action), heavier press costs at least as much effort
    assert all(hi[k]["effort"] >= lo[k]["effort"] - 1e-9 for k in lo)
    # and strictly so where there is real cost (the index's lateral tilt)
    assert hi[("index", "left")]["effort"] > lo[("index", "left")]["effort"]


def test_the_plunge_and_foreaft_are_universal():
    """click / forward / back can be actuated by every finger -- the cradle bears the load."""
    H, x = _load()
    c = actuation_cost(H, x, 20 * GF)
    for f in FINGERS:
        for act in ("click", "forward", "back"):
            assert _feasible(c[(f, act)]), f"{f}/{act} should be universally actuable"


def test_the_cradle_floor_makes_every_well_five_way():
    """The ulnar lateral tilts looked infeasible, but it was a CRADLE artefact, not a muscle-model
    gap: the model withheld the well FLOOR during a lateral press, demanding a muscle for the IP
    torque the floor (and, in a real finger, the DIP collateral ligaments) actually bears. With the
    floor available -- only the floor, never the opposing wall, so the "lend no muscle" control in
    test_design still holds -- STOCK MyoHand actuates all five directions on every finger. The
    interossei were adequate all along; the earlier "three-way ulnar well" was the withheld floor."""
    H, x = _load()
    c = actuation_cost(H, x, 20 * GF)
    for f in FINGERS:
        for act in ACTIONS:
            assert _feasible(c[(f, act)]), f"{f}/{act} should be actuable with the well floor"
    # and the two that used to only just miss (middle/right, ring/right) are now comfortably in,
    # not marginal -- the residual went to ~0 once the floor could bear the DIP torque
    for f in ("middle", "ring"):
        assert c[(f, "right")]["residual"] < 0.02

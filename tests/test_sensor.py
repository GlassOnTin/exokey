"""The finger-well actuation findings, as executable claims.

Five-way is a CAPABILITY of the muscle model + the well's cradle floor (§8.15g): every finger CAN
actuate all five directions at a suitable posture. It is NOT a property the optimiser is asked to
preserve on every finger -- abduction is a design variable (±0.9), and the layout is free to splay a
finger hard for key-packing, at which a digit loses its UNUSED lateral/fore-aft directions. So these
test the capability at a neutral splay (`_capability`), decoupled from the layout's packing choices;
that the design's own posture uses only each finger's WIRED directions is `performable`'s job, not
this file's. If the capability itself stops holding (a better hand model), these fail and we look.
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


def _capability(x):
    """The winning layout may SPLAY a finger for packing (the little at ab=-0.5 loses its unused
    back/left), so five-way is verified at NEUTRAL abduction -- the capability of the muscle model +
    cradle floor, not the optimiser's arbitrary splay. Curls are kept; only the splay is neutralised."""
    x = dict(x)
    for f in FINGERS:
        x[f"ab_{f}"] = 0.0
    return x


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
    """click / forward / back can be actuated by every finger at a suitable posture -- the cradle
    bears the load. Tested at neutral splay (`_capability`); the layout may splay a finger out of its
    unused fore-aft directions for packing, but the capability holds."""
    H, x = _load()
    c = actuation_cost(H, _capability(x), 20 * GF)
    for f in FINGERS:
        for act in ("click", "forward", "back"):
            assert _feasible(c[(f, act)]), f"{f}/{act} should be actuable at a suitable posture"


def test_the_cradle_floor_makes_every_well_five_way():
    """The ulnar lateral tilts looked infeasible, but it was a CRADLE artefact, not a muscle-model
    gap: the model withheld the well FLOOR during a lateral press, demanding a muscle for the IP
    torque the floor (and, in a real finger, the DIP collateral ligaments) actually bears. With the
    floor available -- only the floor, never the opposing wall, so the "lend no muscle" control in
    test_design still holds -- STOCK MyoHand actuates all five directions on every finger at a
    suitable posture. The interossei were adequate all along; the earlier "three-way ulnar well" was
    the withheld floor. (Tested at neutral splay via `_capability`: the optimiser may splay a finger
    hard for packing and lose its UNUSED directions -- a layout choice, not a muscle limit.)"""
    H, x = _load()
    c = actuation_cost(H, _capability(x), 20 * GF)
    for f in FINGERS:
        for act in ACTIONS:
            assert _feasible(c[(f, act)]), f"{f}/{act} should be actuable with the well floor"
    # and the two that used to only just miss (middle/right, ring/right) are now comfortably in,
    # not marginal -- the residual went to ~0 once the floor could bear the DIP torque
    for f in ("middle", "ring"):
        assert c[(f, "right")]["residual"] < 0.02

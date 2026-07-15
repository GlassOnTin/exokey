"""The finger-well SENSOR: what each direction COSTS to actuate, and the dome rate it implies.

The well is a five-direction joystick (hand.cradle). A TPU dome supplies the restoring force
(manufacture.flexure), and a symmetric dome gives just TWO rates: one PLUNGE rate for `click`, and
one TILT rate shared by the four tilt directions -- a round dome cannot tell `left` from `forward`.

So "optimise the moment and lever to the sensor actuation" is a measurement, not a guess: across
the 5th-95th population, at a given applied force, how much MUSCLE EFFORT does each direction cost
each finger, and can the digit even balance it? This reuses the exact cradle solve the gauntlet
optimisation already trusts (hand.cradle.solve) -- only the applied force, pinned to the Svalboard
20 gf everywhere else, is now the free variable the dome sets.

The effort rises monotonically with force (gravity is off, so it is almost pure press cost), so the
dome wants to be as SOFT as a deliberate press allows. What the sweep decides is not a clever
optimum but two real things: which directions each finger can serve at all, and whether one tilt
rate serves the four tilt directions or they diverge enough to want an asymmetric flexure.
"""
from __future__ import annotations

import numpy as np

from design.qwerty import ACTIONS
from design.vector import posture, tm_of, tp_of
from hand.cradle import solve as cradle_solve
from hand.myohand import FINGERS

# how the dome couples the directions: it presents ONE stiffness in plunge, ONE in tilt.
PLUNGE = ("click",)
TILT = ("forward", "back", "left", "right")


def actuation_cost(H: dict, x: dict, press_N: float) -> dict:
    """{(finger, action): dict(effort, residual, sat)} at applied force `press_N`.

    effort is the population MEAN sum(a^3); residual and sat are the population WORST (the binding
    hand), because a direction is only usable if EVERY hand can actuate it.
    """
    out = {}
    for f in FINGERS:
        ab = float(x.get(f"ab_{f}", 0.0))
        rows = {a: [] for a in ACTIONS}
        for _pct, h in H.items():
            q = posture(h, f, tp_of(x, f), tm_of(x, f), ab)
            for act in ACTIONS:
                _a, e, r, smax = cradle_solve(h, q, f, act, press_N)
                rows[act].append((e, r, smax))
        for act in ACTIONS:
            es = np.array(rows[act])
            out[(f, act)] = dict(effort=float(es[:, 0].mean()),
                                 residual=float(es[:, 1].max()),
                                 sat=float(es[:, 2].max()))
    return out

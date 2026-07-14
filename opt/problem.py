"""The outer problem. pymoo MixedVariableGA + NSGA-II survival.

THE WHOLE POINT, and the thing v1 got wrong: feasibility lives in the CONSTRAINT set, not
in the objective. NSGA-II's constrained tournament makes any feasible point dominate any
infeasible one, so the optimiser cannot buy an unreachable key by paying a penalty. v1 had
WEIGHT_UNREACHABLE=25.0 and could, and did.

Objectives (all minimised, genuinely conflicting):
    f1  effort per character   -- muscle activation, population-mean. More keys makes
                                 chords shorter and cheaper; cheaper keys make them cheaper.
    f2  device mass (g)        -- worn all day. More keys means more rows, bars, and mass.
    f3  worst key deflection   -- crispness. A stiffer frame is a heavier one.

Constraints (g <= 0), all HARD:
    every key pressable (>= 3 mm travel) by EVERY hand in the 5th-95th percentile
    no muscle saturated, on any hand
    every key reachable, on any hand
    the frame clears the flesh of every hand
    stress within yield / safety factor
    keys not mushy (<= 0.5 mm)
    the four fingers curl together (common drive -- MyoHand has no enslavement of its own)
"""
from __future__ import annotations

import os

# Pin BLAS to one thread BEFORE numpy loads. Each NSGA-II worker is its own process, and
# scipy's least-squares calls underneath press() are BLAS-heavy: 10 processes x 12 BLAS
# threads on 12 cores oversubscribes ~10x and the "parallel" run comes out SLOWER than
# serial. Measured: the pool was buying nothing at all until this was set.
for _v in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS",
           "NUMEXPR_NUM_THREADS", "VECLIB_MAXIMUM_THREADS"):
    os.environ.setdefault(_v, "1")

import numpy as np
from pymoo.core.problem import ElementwiseProblem
from pymoo.core.variable import Choice, Integer, Real

from design.vector import INT_BOUNDS, MATERIAL_CHOICES, REAL_BOUNDS, evaluate

CONSTRAINT_NAMES = [
    "travel", "saturation", "adjust-range", "yield", "strap-grip", "supportable", "common-drive",
    "key-overlap", "swept-path", "well-finger", "performable",
]
# "travel" and "saturation" now mean ALL THREE ACTIONS (push/lift/contort) on every hand:
# a key that can be pushed but not lifted is a one-row key.
OBJECTIVE_NAMES = ["effort/char", "gauntlet mass (g)"]
# ⚠ DEFLECTION IS NO LONGER AN OBJECTIVE. It was one while the structure was a fixed palmar box,
# where mass and crispness genuinely traded. The gauntlet is GROWN to the gate: ESO deletes struts
# until the buttons are exactly as crisp as they are allowed to be, so deflection is pinned by
# construction and the only question left is what that costs in GRAMS. Keeping it as an objective
# would have the GA hunting a number the structural model has already fixed.

# MuJoCo models are not picklable, so each worker process builds its own hands once and
# caches them here. Building them in the Problem would make the Problem unpicklable.
_HANDS: dict | None = None


def hands(percentiles=(5, 50, 95)) -> dict:
    """The population. 5th/50th/95th by default -- the EXTREMES are what bind (the small
    hand on effort and saturation, the large one on reach and collision), and the 25th/75th
    sit between them, so three hands buy nearly all the constraint and cost 40% less."""
    global _HANDS
    if _HANDS is None:
        from hand.myohand import MyoHand
        from hand.scaling import population

        _HANDS = {p: MyoHand(scale=s) for p, s in population(percentiles).items()}
    return _HANDS


class ExoKeyProblem(ElementwiseProblem):
    def __init__(self, **kw):
        vars = {}
        for name, (lo, hi) in REAL_BOUNDS.items():
            vars[name] = Real(bounds=(lo, hi))
        for name, (lo, hi) in INT_BOUNDS.items():
            vars[name] = Integer(bounds=(lo, hi))
        vars["material"] = Choice(options=MATERIAL_CHOICES)
        super().__init__(vars=vars, n_obj=2, n_ieq_constr=len(CONSTRAINT_NAMES), **kw)

    def _evaluate(self, x, out, *args, **kwargs):
        try:
            r = evaluate(x, hands())
            out["F"] = r["F"]
            out["G"] = r["G"]
        except Exception:
            # A frame can be geometrically degenerate (e.g. a singular FEA). Report it as
            # badly infeasible rather than crashing the run -- but NEVER as feasible.
            out["F"] = [1e6, 1e6]
            out["G"] = [1e3] * len(CONSTRAINT_NAMES)

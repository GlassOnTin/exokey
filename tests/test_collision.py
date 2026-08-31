"""REGRESSION TEST GATES FOR INTER-POD, STRUT-TO-STRUT, AND ANATOMICAL COLLISIONS."""
from __future__ import annotations

import numpy as np
import pytest

from design.vector import posture
from hand.myohand import FINGERS, MyoHand
from manufacture.carrier_gauntlet import build_organic_carrier_gauntlet
from manufacture.svalboard import build_all_svalboard_units
from opt.problem import hands
from structure.collision import audit_pod_intersections


@pytest.fixture(scope="module")
def h50():
    return hands()[50]


def test_svalboard_inter_pod_clearance_gate(h50):
    """All 5-way Svalboard key units must maintain >= 1.5 mm clearance without inter-pod collision."""
    # Natural typing posture with standard ergonomic abduction
    q = h50.compose({
        "index": posture(h50, "index", 0.45, 0.35, np.radians(2.5)),
        "middle": posture(h50, "middle", 0.45, 0.35, 0.0),
        "ring": posture(h50, "ring", 0.45, 0.35, np.radians(-2.5)),
        "little": posture(h50, "little", 0.45, 0.35, np.radians(-5.0)),
        "thumb": posture(h50, "thumb", 0.45, 0.35, 0.0)
    })
    
    sval_units = build_all_svalboard_units(h50, q)
    audit = audit_pod_intersections(sval_units, min_gap_mm=1.5)
    
    for (f1, f2), rep in audit["pairs"].items():
        assert rep["passes"] is True, (
            f"Inter-pod collision between {f1} and {f2}: clearance gap {rep['gap_mm']:.2f} mm < 1.50 mm gate"
        )
    assert audit["all_clear"] is True

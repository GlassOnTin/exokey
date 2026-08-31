"""REGRESSION TEST GATES FOR SVALBOARD CARRIER FEM & STRUCTURAL RIGIDITY."""
from __future__ import annotations

import numpy as np
import pytest

from design.params import DEFLECTION_MAX
from design.vector import posture
from hand.myohand import FINGERS, MyoHand
from opt.problem import hands
from structure.carrier_fem import build_carrier_fem_model, evaluate_carrier_load_cases


@pytest.fixture(scope="module")
def h50():
    return hands()[50]


def test_carrier_fem_model_assembles(h50):
    """The Svalboard carrier gauntlet FEM model must assemble with valid bars, shells, and springs."""
    q = np.zeros(h50.model.nq)
    model = build_carrier_fem_model(h50, q, mat="cf_pa12")
    assert len(model["nodes"]) >= 50
    assert len(model["bars"]) >= 80
    assert len(model["shells"]) >= 20
    assert len(model["pod_nodes"]) == 5
    assert model["frame"].lu is not None


def test_carrier_25_typing_deflection_cases_pass_gate(h50):
    """All 25 typing load cases (5 digits x 5 directions) must pass the <= 500 um deflection gate."""
    q = h50.compose({f: posture(h50, f, 0.45, 0.35, 0.0) for f in FINGERS})
    model = build_carrier_fem_model(h50, q, mat="cf_pa12")
    res = evaluate_carrier_load_cases(model, press_N=0.196, bash_N=3.0, knock_N=5.0)
    
    # 25 individual cases evaluated
    assert len(res["typing_deflections_um"]) == 25
    assert res["worst_typing_um"] <= float(DEFLECTION_MAX) * 1.0e6, (
        f"Worst-case typing deflection {res['worst_typing_um']:.1f} um exceeds {float(DEFLECTION_MAX)*1e6:.0f} um gate"
    )


def test_carrier_knock_and_bash_impact_safety_factor(h50):
    """Accidental knocks (5.0 N) and lateral bashes (3.0 N) must satisfy yield safety factor >= 2.0."""
    q = h50.compose({f: posture(h50, f, 0.45, 0.35, 0.0) for f in FINGERS})
    model = build_carrier_fem_model(h50, q, mat="cf_pa12")
    res = evaluate_carrier_load_cases(model, press_N=0.196, bash_N=3.0, knock_N=5.0)
    
    assert res["yield_safety_factor"] >= 2.0, (
        f"Knock yield safety factor {res['yield_safety_factor']:.2f}x < 2.0x required gate"
    )
    assert res["worst_bash_mm"] <= 8.0, f"Excessive lateral bash compliance: {res['worst_bash_mm']:.2f} mm"

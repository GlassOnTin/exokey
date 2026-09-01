"""REGRESSION TEST GATES FOR CONSOLIDATED EXOKEY 3D SPACE-FRAME FEM & SVALBOARD MAGNETIC ALIGNMENT."""
from __future__ import annotations

import numpy as np
import pytest

from design.vector import posture
from hand.myohand import FINGERS, MyoHand
from opt.problem import hands
from structure.carrier_fem import (
    build_carrier_fem_model,
    solve_carrier_typing_cases,
    solve_chord_typing_case,
    evaluate_impact_rigidity,
    svalboard_magnetic_force_profile
)


@pytest.fixture(scope="module")
def h50():
    return hands()[50]


def test_carrier_fem_model_assembles(h50):
    """The consolidated ExoKey FEM model must assemble with valid spine, arch, and phalanx elements."""
    q = np.zeros(h50.model.nq)
    model = build_carrier_fem_model(h50, q)
    fem = model["fem"]
    assert len(fem.nodes) >= 35
    assert len(fem.elements) >= 40
    assert len(model["pod_nodes"]) == 5


def test_svalboard_magnetic_alignment_dynamics():
    """Verify Svalboard magnetic dipole return force, breakaway snap, and self-centering alignment."""
    # 1. At resting neutral (z = 0.0 mm): peak tactile holding force 0.196 N (20 gf)
    p_rest = svalboard_magnetic_force_profile(0.0, f_peak_N=0.196, z0_mm=0.35)
    assert abs(p_rest["force_N"] - 0.196) < 1e-4
    assert p_rest["stiffness_N_per_m"] > 1000.0   # > 1000 N/m initial tactile holding gradient
    assert p_rest["k_align_N_per_m"] > 300.0       # > 300 N/m self-centering restoring stiffness

    # 2. At breakaway inflection (z = 0.35 mm): force drops to 25% of peak
    p_snap = svalboard_magnetic_force_profile(0.35, f_peak_N=0.196, z0_mm=0.35)
    assert abs(p_snap["tactile_drop_ratio"] - 0.25) < 1e-3

    # 3. At bottom-out (z = 1.2 mm): ultra-light holding force
    p_bottom = svalboard_magnetic_force_profile(1.2, f_peak_N=0.196, z0_mm=0.35)
    assert p_bottom["force_N"] < 0.02  # < 2 gf residual


def test_carrier_25_typing_deflection_and_crispness_ratio(h50):
    """All 25 typing load cases must pass <= 180 um deflection and achieve >= 10x crispness ratio."""
    q = h50.compose({f: posture(h50, f, 0.45, 0.35, 0.0) for f in FINGERS})
    model = build_carrier_fem_model(h50, q)
    res = solve_carrier_typing_cases(model)
    
    # 25 individual cases evaluated
    assert len(res["digit_cases"]) == 5
    for f, dirs in res["digit_cases"].items():
        assert len(dirs) == 5
        for d_name, case in dirs.items():
            assert case["disp_um"] <= 180.0, (
                f"Finger {f} ({d_name}) deflection {case['disp_um']:.1f} um exceeds 180 um gate"
            )
            assert case["safety_factor"] >= 200.0
            # Structural stiffness must dominate magnetic key gradient
            assert case["k_struct_N_per_m"] >= 1000.0
            
    assert res["passes_gate"] is True


def test_carrier_chord_typing_case(h50):
    """Simultaneous 5-finger chord typing (1.0 N total) must maintain high safety factors."""
    q = h50.compose({f: posture(h50, f, 0.45, 0.35, 0.0) for f in FINGERS})
    model = build_carrier_fem_model(h50, q)
    chord = solve_chord_typing_case(model)
    
    assert chord["max_stress_MPa"] <= 20.0
    assert chord["safety_factor"] >= 50.0
    assert chord["passes_gate"] is True


def test_carrier_knock_and_bash_impact_safety_factor(h50):
    """Accidental knocks (5.0 N), bashes (3.0 N), and snags (2.0 N) must satisfy SF >= 20.0x in CF."""
    q = h50.compose({f: posture(h50, f, 0.45, 0.35, 0.0) for f in FINGERS})
    model = build_carrier_fem_model(h50, q)
    impact = evaluate_impact_rigidity(model)
    
    assert impact["passes_gate"] is True
    for name, c in impact["cases"].items():
        assert c["safety_factor"] >= 20.0, f"Case {name} SF {c['safety_factor']:.1f}x < 20x gate"

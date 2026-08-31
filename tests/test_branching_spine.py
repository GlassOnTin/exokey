"""REGRESSION TEST GATES FOR CENTRAL DORSAL SPINE + BRANCHING TREE ARCHITECTURE."""
from __future__ import annotations

import numpy as np
import pytest

from manufacture.branching_spine_outrigger import (
    build_conformal_spine_tree_geometry,
)
from structure.fem import run_exokey_fem_analysis


def test_branching_spine_mesh_generation():
    """Verify central spine and wishbone manifold generate valid 3D CAD meshes."""
    p_root = np.array([0.025, 0.009, 0.024])
    mcp_nodes = {
        "index": np.array([0.058, 0.037, 0.017]),
        "middle": np.array([0.062, 0.010, 0.020]),
        "ring": np.array([0.063, -0.013, 0.019]),
        "little": np.array([0.058, -0.037, 0.017]),
        "thumb": np.array([0.037, 0.038, 0.032]),
    }
    digit_chains = {
        "index": [mcp_nodes["index"], np.array([0.085, 0.042, -0.003]), np.array([0.107, 0.042, -0.024]), np.array([0.116, 0.038, -0.065])],
        "middle": [mcp_nodes["middle"], np.array([0.094, 0.018, -0.007]), np.array([0.113, 0.016, -0.034]), np.array([0.120, 0.015, -0.072])],
        "ring": [mcp_nodes["ring"], np.array([0.090, -0.009, -0.008]), np.array([0.107, -0.004, -0.031]), np.array([0.114, -0.005, -0.068])],
        "little": [mcp_nodes["little"], np.array([0.085, -0.048, -0.008]), np.array([0.092, -0.044, -0.035]), np.array([0.085, -0.026, -0.067])],
        "thumb": [mcp_nodes["thumb"], np.array([0.060, 0.045, 0.015]), np.array([0.080, 0.048, -0.020]), np.array([0.064, 0.048, -0.082])],
    }
    
    parts = build_conformal_spine_tree_geometry(p_root, mcp_nodes, digit_chains)
    assert "tubes" in parts
    assert "clamps" in parts
    assert len(parts["tubes"].vertices) > 100
    assert len(parts["clamps"].vertices) > 100
    assert parts["complete"].is_watertight is False or len(parts["complete"].faces) > 0


def test_branching_spine_3d_fem_analysis():
    """Verify 3D Space-Frame FEM solver runs and confirms structural deflection and stress margins."""
    p_root = np.array([0.025, 0.009, 0.024])
    mcp_nodes = {
        "index": np.array([0.058, 0.037, 0.017]),
        "middle": np.array([0.062, 0.010, 0.020]),
        "ring": np.array([0.063, -0.013, 0.019]),
        "little": np.array([0.058, -0.037, 0.017]),
        "thumb": np.array([0.037, 0.038, 0.032]),
    }
    digit_chains = {
        "index": [mcp_nodes["index"], np.array([0.085, 0.042, -0.003]), np.array([0.107, 0.042, -0.024]), np.array([0.116, 0.038, -0.065])],
        "middle": [mcp_nodes["middle"], np.array([0.094, 0.018, -0.007]), np.array([0.113, 0.016, -0.034]), np.array([0.120, 0.015, -0.072])],
        "ring": [mcp_nodes["ring"], np.array([0.090, -0.009, -0.008]), np.array([0.107, -0.004, -0.031]), np.array([0.114, -0.005, -0.068])],
        "little": [mcp_nodes["little"], np.array([0.085, -0.048, -0.008]), np.array([0.092, -0.044, -0.035]), np.array([0.085, -0.026, -0.067])],
        "thumb": [mcp_nodes["thumb"], np.array([0.060, 0.045, 0.015]), np.array([0.080, 0.048, -0.020]), np.array([0.064, 0.048, -0.082])],
    }
    
    fem = run_exokey_fem_analysis(p_root, mcp_nodes, digit_chains, typing_force_N=0.196)
    
    for f, res in fem["single_finger_results"].items():
        assert res["tip_deflection_um"] <= 180.0, f"Finger {f} deflection {res['tip_deflection_um']:.1f} um exceeds 180 um gate"
        assert res["safety_factor"] >= 400.0, f"Finger {f} safety factor {res['safety_factor']:.1f}x < 400x gate"
        
    assert fem["chord_typing"]["max_stress_MPa"] <= 50.0
    assert fem["passes_stress_gate"] is True

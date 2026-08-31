"""REGRESSION TEST GATES FOR MODULAR PHALANX-FOLLOWING 4-NODE BALL-COLLET EXOSKELETON."""
from __future__ import annotations

import numpy as np
import pytest

from manufacture.ball_collet_outrigger import (
    build_phalanx_following_outrigger,
    calculate_phalanx_exoskeleton_mechanics,
)


def test_ball_collet_phalanx_exoskeleton_mesh_build():
    """Verify 4-node 3-link phalanx exoskeleton builds valid 3D CAD meshes."""
    nodes = [
        np.array([0.00, 0.00, 0.02]),  # Node 1: MCP
        np.array([0.04, 0.00, 0.01]),  # Node 2: PIP
        np.array([0.07, 0.00, -0.02]), # Node 3: DIP
        np.array([0.08, 0.00, -0.06]), # Node 4: Pod
    ]
    parts = build_phalanx_following_outrigger(nodes, r_tube=0.0022, r_ball=0.0030)
    
    assert "tubes" in parts
    assert "clamps" in parts
    assert len(parts["tubes"].vertices) > 50
    assert len(parts["clamps"].vertices) > 100
    assert parts["complete"].is_watertight is False or len(parts["complete"].faces) > 0


def test_ball_collet_phalanx_exoskeleton_rigidity_mechanics():
    """Verify 4-node phalanx exoskeleton satisfies typing deflection and holding safety factor gates."""
    mech = calculate_phalanx_exoskeleton_mechanics(
        link_lengths_m=[0.040, 0.035, 0.035], # 3 straight CF links
        r_tube_od=0.0022,
        r_tube_id=0.0013,
        E_cf=135.0e9,
        screw_size="M2.5",
        torque_Nm=0.40,
        friction_mu=0.35,
        typing_force_N=0.196
    )
    
    # 1. Deflection gate under 0.2 N typing load (must be <= 200 μm)
    assert mech["total_deflection_um"] <= 200.0, (
        f"3-link exoskeleton deflection {mech['total_deflection_um']:.1f} μm exceeds 200 μm gate"
    )
    
    # 2. Joint holding safety factor against typing slip (must be >= 25x)
    assert mech["safety_factor_typing"] >= 25.0, (
        f"Clamping slip safety factor {mech['safety_factor_typing']:.1f}x < 25x gate"
    )
    
    # 3. Clamping pre-tension
    assert mech["F_clamp_N"] >= 700.0

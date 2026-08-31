"""MODULAR ANATOMICALLY-ALIGNED 4-NODE / 3-LINK PHALANX-FOLLOWING EXOSKELETON ARCHITECTURE.

Kinematic Architecture per Finger (Index, Middle, Ring, Little):
1. NODE 1 — MCP BALL JOINT:
   - Positioned directly above the Metacarpophalangeal (MCP) knuckle.
   - Adjusts finger spread (yaw ±25°) and launch elevation (pitch ±30°).
2. LINK 1 — PROXIMAL PHALANX BOOM:
   - Straight pultruded CF tube (⌀ 4.4 mm OD / ⌀ 2.6 mm ID, L1 ≈ 35-55 mm).
   - Runs parallel to the proximal phalanx with 4.5 mm dorsal standoff.
3. NODE 2 — PIP BALL-COLLET:
   - Positioned directly above the Proximal Interphalangeal (PIP) knuckle.
   - Adjusts PIP curl angle (θ_PIP ∈ [15°, 75°]).
4. LINK 2 — MIDDLE PHALANX BOOM:
   - Straight pultruded CF tube (⌀ 4.4 mm OD / ⌀ 2.6 mm ID, L2 ≈ 30-38 mm).
   - Runs parallel to the middle phalanx with 4.5 mm dorsal standoff.
5. NODE 3 — DIP BALL-COLLET:
   - Positioned directly above the Distal Interphalangeal (DIP) knuckle.
   - Adjusts DIP flexure (θ_DIP ∈ [10°, 60°]) and fingertip approach angle.
6. LINK 3 — DISTAL POD BOOM:
   - Straight pultruded CF tube (⌀ 4.4 mm OD / ⌀ 2.6 mm ID, L3 ≈ 30-45 mm).
   - Docks into the keywell pod mounting flange in front of the nail.
7. NODE 4 — SENSOR POD FLANGE SWIVEL:
   - Ball swivel at the 5-way keywell pod (pitch ±30°, roll 360°).

Thumb Architecture:
- 3 Nodes, 2 Links (MCP -> IP -> Pod).
"""
from __future__ import annotations

import numpy as np
import trimesh

from hand.myohand import MyoHand
from manufacture.carrier_gauntlet import hand_axes


def _align_rot(u_vec: np.ndarray) -> np.ndarray:
    """Compute 3x3 rotation matrix aligning +Z axis with u_vec."""
    z_ref = np.array([0.0, 0.0, 1.0])
    rot_axis = np.cross(z_ref, u_vec)
    rot_norm = np.linalg.norm(rot_axis)
    if rot_norm < 1e-6:
        return np.eye(3) if u_vec[2] > 0 else np.diag([1.0, -1.0, -1.0])
    rot_axis /= rot_norm
    angle = np.arccos(np.clip(np.dot(z_ref, u_vec), -1.0, 1.0))
    K = np.array([[0.0, -rot_axis[2], rot_axis[1]],
                  [rot_axis[2], 0.0, -rot_axis[0]],
                  [-rot_axis[1], rot_axis[0], 0.0]])
    return np.eye(3) + np.sin(angle) * K + (1.0 - np.cos(angle)) * (K @ K)


def build_ball_clamp_mesh(pos: np.ndarray, dir_in: np.ndarray, dir_out: np.ndarray | None = None,
                          r_ball: float = 0.0030, is_terminal: bool = False,
                          sections: int = 16) -> trimesh.Trimesh:
    """Build a detailed M2.5 split-collar ball clamp joint mesh."""
    parts = []
    
    # 1. Central spherical ball stud
    ball = trimesh.creation.uv_sphere(radius=r_ball, count=[sections, sections])
    ball.apply_translation(pos)
    parts.append(ball)
    
    # 2. Clamping collar socket
    sock_len = 0.005
    sock_od = r_ball + 0.0014
    u_in = dir_in / (np.linalg.norm(dir_in) + 1e-12)
    
    cyl_sock = trimesh.creation.cylinder(radius=sock_od, height=sock_len, sections=sections)
    T_sock = np.eye(4)
    T_sock[:3, :3] = _align_rot(u_in)
    T_sock[:3, 3] = pos + 0.5 * sock_len * u_in
    cyl_sock.apply_transform(T_sock)
    parts.append(cyl_sock)
    
    # 3. M2.5 Screw boss
    boss_r = 0.0016
    boss_w = 0.0050
    v_side = np.cross(u_in, np.array([0.0, 0.0, 1.0]))
    if np.linalg.norm(v_side) < 1e-4:
        v_side = np.cross(u_in, np.array([0.0, 1.0, 0.0]))
    v_side /= np.linalg.norm(v_side)
    
    cyl_boss = trimesh.creation.cylinder(radius=boss_r, height=boss_w, sections=sections)
    T_boss = np.eye(4)
    T_boss[:3, :3] = _align_rot(v_side)
    T_boss[:3, 3] = pos + 0.0025 * u_in + (sock_od - 0.0004) * np.cross(u_in, v_side)
    cyl_boss.apply_transform(T_boss)
    parts.append(cyl_boss)
    
    if dir_out is not None and not is_terminal:
        u_out = dir_out / (np.linalg.norm(dir_out) + 1e-12)
        cyl_out = trimesh.creation.cylinder(radius=sock_od, height=sock_len, sections=sections)
        T_out = np.eye(4)
        T_out[:3, :3] = _align_rot(u_out)
        T_out[:3, 3] = pos + 0.5 * sock_len * u_out
        cyl_out.apply_transform(T_out)
        parts.append(cyl_out)
        
    return trimesh.util.concatenate(parts)


def build_straight_cf_tube(p_start: np.ndarray, p_end: np.ndarray,
                           r_od: float = 0.0022, sections: int = 16) -> trimesh.Trimesh:
    """Build a straight pultruded carbon-fiber tube mesh."""
    v = p_end - p_start
    L = float(np.linalg.norm(v))
    if L < 1e-5:
        return trimesh.creation.uv_sphere(radius=r_od)
    
    cyl = trimesh.creation.cylinder(radius=r_od, height=L, sections=sections)
    u = v / L
    T = np.eye(4)
    T[:3, :3] = _align_rot(u)
    T[:3, 3] = 0.5 * (p_start + p_end)
    cyl.apply_transform(T)
    return cyl


def build_phalanx_following_outrigger(nodes: list[np.ndarray],
                                      r_tube: float = 0.0022,
                                      r_ball: float = 0.0030) -> dict[str, trimesh.Trimesh]:
    """Build a multi-link serial exoskeleton outrigger tracking the phalanx chain."""
    n_nodes = len(nodes)
    tubes = []
    clamps = []
    
    # 1. Build straight CF tubes between consecutive joint nodes
    for i in range(n_nodes - 1):
        p0, p1 = nodes[i], nodes[i + 1]
        tubes.append(build_straight_cf_tube(p0, p1, r_od=r_tube))
        
    # 2. Build ball-collet clamp joints at each node
    for i in range(n_nodes):
        pos = nodes[i]
        if i == 0:
            dir_in = nodes[1] - nodes[0]
            clamps.append(build_ball_clamp_mesh(pos, dir_in=dir_in, r_ball=r_ball, is_terminal=True))
        elif i == n_nodes - 1:
            dir_in = nodes[i] - nodes[i - 1]
            clamps.append(build_ball_clamp_mesh(pos, dir_in=-dir_in, r_ball=r_ball * 0.9, is_terminal=True))
        else:
            dir_in = -(nodes[i] - nodes[i - 1])
            dir_out = nodes[i + 1] - nodes[i]
            clamps.append(build_ball_clamp_mesh(pos, dir_in=dir_in, dir_out=dir_out, r_ball=r_ball, is_terminal=False))
            
    cf_mesh = trimesh.util.concatenate(tubes)
    cl_mesh = trimesh.util.concatenate(clamps)
    
    return {
        "tubes": cf_mesh,
        "clamps": cl_mesh,
        "complete": trimesh.util.concatenate([cf_mesh, cl_mesh])
    }


def calculate_phalanx_exoskeleton_mechanics(link_lengths_m: list[float] | None = None,
                                            r_tube_od: float = 0.0022, r_tube_id: float = 0.0013,
                                            E_cf: float = 135.0e9,
                                            screw_size: str = "M2.5",
                                            torque_Nm: float = 0.40,
                                            friction_mu: float = 0.35,
                                            typing_force_N: float = 0.196) -> dict:
    """Calculate the structural rigidity and clamping safety factor of the multi-link chain."""
    if link_lengths_m is None:
        link_lengths_m = [0.040, 0.035, 0.035] # 3 links: Proximal, Middle, Distal
        
    I_tube = np.pi * (r_tube_od**4 - r_tube_id**4) / 4.0
    
    total_bending_deflection = 0.0
    for L_m in link_lengths_m:
        total_bending_deflection += (typing_force_N * (L_m**3)) / (3.0 * E_cf * I_tube)
        
    d_screw = 0.0025 if screw_size == "M2.5" else 0.0020
    F_clamp = torque_Nm / (0.20 * d_screw)
    r_ball = 0.0030
    M_slip = friction_mu * F_clamp * r_ball
    
    total_span_m = sum(link_lengths_m)
    M_typing = typing_force_N * total_span_m
    sf_typing = M_slip / max(M_typing, 1e-6)
    
    k_contact = E_cf * (np.pi * r_ball**2) / 0.001
    k_theta = k_contact * (r_ball**2)
    joint_deflection = len(link_lengths_m) * (M_typing / k_theta) * (total_span_m / len(link_lengths_m))
    
    total_deflection_um = (total_bending_deflection + joint_deflection) * 1.0e6
    
    return {
        "F_clamp_N": F_clamp,
        "M_slip_mNm": M_slip * 1.0e3,
        "M_typing_mNm": M_typing * 1.0e3,
        "safety_factor_typing": sf_typing,
        "total_deflection_um": total_deflection_um,
        "total_span_mm": total_span_m * 1.0e3,
        "n_links": len(link_lengths_m),
        "passes_gate": total_deflection_um <= 200.0
    }

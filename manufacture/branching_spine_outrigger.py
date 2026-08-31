"""CENTRAL DORSAL SPINE WITH BRANCHING TRANSVERSE TREE ARCHITECTURE (VERTEBRAL WISHBONE).

Kinematic & Structural Concept:
1. PRIMARY CENTRAL BACKBONE (Vertebral Column):
   - High-modulus pultruded carbon-fiber spine (⌀ 8.0 mm OD / ⌀ 6.0 mm ID, E = 180 GPa).
   - Anchors into a 4-Way Central Manifold Cross-Fitting Hub over the Middle Knuckle (MCP3).
2. 4-WAY CENTRAL MANIFOLD HUB (Middle Knuckle Joint):
   - Precision CNC Titanium / SLS CF-PA12 cross-fitting connecting:
     * Posterior Port: Primary Spine (⌀ 8.0 mm)
     * Lateral Ports: Transverse Knuckle Arch (⌀ 6.0 mm) to Ring & Index
     * Anterior Port: Middle Finger Phalanx Boom (⌀ 5.0 mm)
   - Clean, compact, flush-locking architecture with zero stray protruding artifacts.
3. UNIFIED CONTINUOUS KNUCKLE ARCH:
   - Little MCP <-> Ring MCP <-> Middle MCP <-> Index MCP <-> Thumb MCP.
4. DIRECT THUMB STRUT ATTACHED TO INDEX KNUCKLE JOINT:
   - Arches across the 1st webspace corridor directly from Index MCP to Thumb MCP.
5. CONFORMAL 3-LINK PHALANX BRANCHES:
   - ⌀ 5.0 mm OD / ⌀ 3.4 mm ID straight CF booms with M2.5 locking ball-collets.
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


def build_straight_cf_tube(p_start: np.ndarray, p_end: np.ndarray,
                           r_od: float = 0.0025, sections: int = 16) -> trimesh.Trimesh:
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


def build_compact_ball_joint_mesh(pos: np.ndarray, r_ball: float = 0.0035, sections: int = 16) -> trimesh.Trimesh:
    """Build a clean, compact spherical ball-collet clamp hub."""
    ball = trimesh.creation.uv_sphere(radius=r_ball, count=[sections, sections])
    ball.apply_translation(pos)
    return ball


def build_conformal_spine_tree_geometry(p_root: np.ndarray,
                                        mcp_nodes: dict[str, np.ndarray],
                                        digit_chains: dict[str, list[np.ndarray]],
                                        r_spine_od: float = 0.0040,
                                        r_arch_od: float = 0.0030,
                                        r_branch_od: float = 0.0025) -> dict[str, trimesh.Trimesh]:
    """Generate clean, physical 3D CAD meshes without stray protrusions or unaligned bosses."""
    cf_tubes = []
    clamps = []
    
    p_mcp_mid = mcp_nodes["middle"]
    p_mcp_idx = mcp_nodes["index"]
    p_mcp_th = mcp_nodes["thumb"]
    
    # 1. Primary Central Spine Tube (Saddle Hub -> Middle MCP Knuckle)
    spine_tube = build_straight_cf_tube(p_root, p_mcp_mid, r_od=r_spine_od, sections=18)
    cf_tubes.append(spine_tube)
    
    # Saddle root collar hub
    root_hub = build_compact_ball_joint_mesh(p_root, r_ball=0.0045)
    clamps.append(root_hub)
    
    # 2. Transverse Arch across Fingers (Little <-> Ring <-> Middle <-> Index)
    arch_order = ["little", "ring", "middle", "index"]
    for i in range(len(arch_order) - 1):
        pA = mcp_nodes[arch_order[i]]
        pB = mcp_nodes[arch_order[i+1]]
        t_arch = build_straight_cf_tube(pA, pB, r_od=r_arch_od, sections=14)
        cf_tubes.append(t_arch)
        
    for f in arch_order:
        # Clean spherical knuckle hub at each transverse arch junction
        r_hub = 0.0042 if f == "middle" else 0.0036
        c_node = build_compact_ball_joint_mesh(mcp_nodes[f], r_ball=r_hub)
        clamps.append(c_node)
        
    # 3. DIRECT THUMB STRUT ATTACHED TO INDEX KNUCKLE JOINT (Index MCP -> Web Arch -> Thumb MCP)
    p_web = 0.5 * (p_mcp_idx + p_mcp_th) + 0.008 * np.array([0.0, 0.0, 1.0]) + 0.006 * np.array([0.0, 1.0, 0.0])
    t_idx_web = build_straight_cf_tube(p_mcp_idx, p_web, r_od=r_arch_od, sections=14)
    t_web_th = build_straight_cf_tube(p_web, p_mcp_th, r_od=r_arch_od, sections=14)
    cf_tubes.extend([t_idx_web, t_web_th])
    
    c_web = build_compact_ball_joint_mesh(p_web, r_ball=0.0034)
    c_th_mcp = build_compact_ball_joint_mesh(p_mcp_th, r_ball=0.0036)
    clamps.extend([c_web, c_th_mcp])
    
    # 4. Phalanx Branches per Digit (PIP, DIP, and Pod joints)
    for f, nodes in digit_chains.items():
        for i in range(len(nodes) - 1):
            p0, p1 = nodes[i], nodes[i+1]
            t_link = build_straight_cf_tube(p0, p1, r_od=r_branch_od, sections=14)
            cf_tubes.append(t_link)
            
            # Inter-phalanx locking ball-collet hubs (PIP, DIP, Pod)
            c_joint = build_compact_ball_joint_mesh(p1, r_ball=0.0032 if i < len(nodes)-2 else 0.0028)
            clamps.append(c_joint)
            
    mesh_tubes = trimesh.util.concatenate(cf_tubes)
    mesh_clamps = trimesh.util.concatenate(clamps)
    
    return {
        "tubes": mesh_tubes,
        "clamps": mesh_clamps,
        "complete": trimesh.util.concatenate([mesh_tubes, mesh_clamps])
    }

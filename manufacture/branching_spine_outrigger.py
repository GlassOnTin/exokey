"""CENTRAL DORSAL SPINE WITH BRANCHING TRANSVERSE TREE & ORIENTED CNC CLAMPS.

Kinematic & Structural Concept:
1. PRIMARY CENTRAL BACKBONE (Vertebral Column):
   - High-modulus pultruded carbon-fiber spine (⌀ 8.0 mm OD / ⌀ 6.0 mm ID, E = 180 GPa).
   - Anchors into a 4-Way Central Manifold Cross-Fitting Hub over the Middle Knuckle (MCP3).
2. TANGENTIALLY ORIENTED 3-WAY & 4-WAY CNC KNUCKLE CLAMPS:
   - Symmetrical 2.0 mm 6061-T6 aluminum plates oriented parallel to the local dorsal hand surface.
   - 3-Way Tri-Lobe Clamps at Ring Knuckle (MCP4) and Index Knuckle (MCP2).
   - 4-Way Quad-Lobe Manifold Clamp at Middle Knuckle (MCP3).
3. 2-PIECE SYMMETRICAL DOGBONE CLAMPS:
   - Oriented along phalanx boom links (MCP -> PIP -> DIP -> Pod).
4. STANDOFF CARBON FIBER RODS:
   - Carbon tubes stop short at each end (4.5 - 5.0 mm standoff) to accommodate
     metal ball-stud threaded shanks and hex collar transitions.
"""
from __future__ import annotations

import numpy as np
import trimesh

from hand.myohand import MyoHand
from manufacture.dogbone_clamps import build_oriented_joint_clamp


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
                           r_od: float = 0.0025, sections: int = 14) -> trimesh.Trimesh:
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


def build_phalanx_carbon_strut_with_ball_standoff(p_start: np.ndarray, p_end: np.ndarray,
                                                  r_cf_od: float = 0.0025,
                                                  r_shank: float = 0.0017,
                                                  standoff: float = 0.0045) -> tuple[trimesh.Trimesh, trimesh.Trimesh]:
    """Build a carbon fiber tube that stops short for ball-studs, plus metallic shanks."""
    v = p_end - p_start
    L = float(np.linalg.norm(v))
    u = v / (L + 1e-12)
    
    actual_standoff = min(standoff, max(0.0015, 0.4 * L))
    p_tube_start = p_start + actual_standoff * u
    p_tube_end = p_end - actual_standoff * u
    
    cf_tube = build_straight_cf_tube(p_tube_start, p_tube_end, r_od=r_cf_od, sections=14)
    shank_start = build_straight_cf_tube(p_start, p_tube_start, r_od=r_shank, sections=10)
    shank_end = build_straight_cf_tube(p_tube_end, p_end, r_od=r_shank, sections=10)
    shanks = trimesh.util.concatenate([shank_start, shank_end])
    
    return cf_tube, shanks


def build_conformal_spine_tree_geometry(p_root: np.ndarray,
                                        mcp_nodes: dict[str, np.ndarray],
                                        digit_chains: dict[str, list[np.ndarray]],
                                        e_dorsal: np.ndarray | None = None,
                                        r_spine_od: float = 0.0040,
                                        r_arch_od: float = 0.0030,
                                        r_branch_od: float = 0.0025) -> dict[str, trimesh.Trimesh]:
    """Generate 3D CAD meshes featuring Metacarpal-Plane Aligned CNC Clamps & Standoff Carbon Rods."""
    from manufacture.dogbone_clamps import build_oriented_joint_clamp
    
    cf_tubes = []
    clamps = []
    
    p_mcp_mid = mcp_nodes["middle"]
    p_mcp_idx = mcp_nodes["index"]
    p_mcp_rng = mcp_nodes["ring"]
    p_mcp_lit = mcp_nodes["little"]
    p_mcp_th = mcp_nodes["thumb"]
    
    # True anatomical dorsal normal vector (flat across metacarpal bed)
    if e_dorsal is None:
        e_dorsal = np.array([-0.968, 0.076, -0.239])
    e_dorsal = e_dorsal / (np.linalg.norm(e_dorsal) + 1e-12)
    
    # -------------------------------------------------------------------------
    # 1. KNUCKLE CLAMP HUBS (Tangentially Oriented to Dorsal Surface)
    # -------------------------------------------------------------------------
    # 4-Way Middle Knuckle Hub (MCP3): Central Spine + Ring Arch + Index Arch + Middle Phalanx Boom
    clamp_mid, mid_balls = build_oriented_joint_clamp(
        p_center=p_mcp_mid,
        connected_pts=[p_root, p_mcp_rng, p_mcp_idx, digit_chains["middle"][1]],
        n_surface=e_dorsal,
        width=0.0070, arm_radius=0.0080
    )
    clamps.append(clamp_mid)
    p_ball_mid_spine, p_ball_mid_rng, p_ball_mid_idx, p_ball_mid_phx = mid_balls
    
    # 3-Way Ring Knuckle Hub (MCP4): Middle Arch + Little Arch + Ring Phalanx Boom
    clamp_rng, rng_balls = build_oriented_joint_clamp(
        p_center=p_mcp_rng,
        connected_pts=[p_mcp_mid, p_mcp_lit, digit_chains["ring"][1]],
        n_surface=e_dorsal,
        width=0.0070, arm_radius=0.0080
    )
    clamps.append(clamp_rng)
    p_ball_rng_mid, p_ball_rng_lit, p_ball_rng_phx = rng_balls
    
    # 3-Way Index Knuckle Hub (MCP2): Middle Arch + Thumb Web Arch + Index Phalanx Boom
    clamp_idx, idx_balls = build_oriented_joint_clamp(
        p_center=p_mcp_idx,
        connected_pts=[p_mcp_mid, p_mcp_th, digit_chains["index"][1]],
        n_surface=e_dorsal,
        width=0.0070, arm_radius=0.0080
    )
    clamps.append(clamp_idx)
    p_ball_idx_mid, p_ball_idx_th, p_ball_idx_phx = idx_balls
    
    # 2-Way Little Knuckle Hub (MCP5): Ring Arch + Little Phalanx Boom
    clamp_lit, lit_balls = build_oriented_joint_clamp(
        p_center=p_mcp_lit,
        connected_pts=[p_mcp_rng, digit_chains["little"][1]],
        n_surface=e_dorsal,
        width=0.0070, arm_radius=0.0075
    )
    clamps.append(clamp_lit)
    p_ball_lit_rng, p_ball_lit_phx = lit_balls
    
    # 2-Way Thumb Knuckle Hub (MCP1): Web Arch + Thumb Phalanx Boom
    clamp_th, th_balls = build_oriented_joint_clamp(
        p_center=p_mcp_th,
        connected_pts=[p_mcp_idx, digit_chains["thumb"][1]],
        n_surface=e_dorsal,
        width=0.0070, arm_radius=0.0075
    )
    clamps.append(clamp_th)
    p_ball_th_idx, p_ball_th_phx = th_balls
    
    # -------------------------------------------------------------------------
    # 2. MAIN CARBON FIBER SPINES & TRANSVERSE ARCHES (Connecting Ball-to-Ball)
    # -------------------------------------------------------------------------
    # Primary Central Spine (Root Saddle -> Middle Knuckle Spine Ball)
    t_spine, s_spine = build_phalanx_carbon_strut_with_ball_standoff(
        p_root, p_ball_mid_spine, r_cf_od=r_spine_od, r_shank=r_spine_od * 0.75, standoff=0.0045
    )
    cf_tubes.append(t_spine)
    clamps.append(s_spine)
    
    root_hub = trimesh.creation.uv_sphere(radius=0.0045, count=[14, 14])
    root_hub.apply_translation(p_root)
    clamps.append(root_hub)
    
    # Transverse Knuckle Arch: Middle <-> Ring
    t_mid_rng, s_mid_rng = build_phalanx_carbon_strut_with_ball_standoff(
        p_ball_mid_rng, p_ball_rng_mid, r_cf_od=r_arch_od, r_shank=r_arch_od * 0.75, standoff=0.0035
    )
    cf_tubes.append(t_mid_rng)
    clamps.append(s_mid_rng)
    
    # Transverse Knuckle Arch: Ring <-> Little
    t_rng_lit, s_rng_lit = build_phalanx_carbon_strut_with_ball_standoff(
        p_ball_rng_lit, p_ball_lit_rng, r_cf_od=r_arch_od, r_shank=r_arch_od * 0.75, standoff=0.0035
    )
    cf_tubes.append(t_rng_lit)
    clamps.append(s_rng_lit)
    
    # Transverse Knuckle Arch: Middle <-> Index
    t_mid_idx, s_mid_idx = build_phalanx_carbon_strut_with_ball_standoff(
        p_ball_mid_idx, p_ball_idx_mid, r_cf_od=r_arch_od, r_shank=r_arch_od * 0.75, standoff=0.0035
    )
    cf_tubes.append(t_mid_idx)
    clamps.append(s_mid_idx)
    
    # 1st Webspace Thumb Bridge: Index <-> Thumb
    t_idx_th, s_idx_th = build_phalanx_carbon_strut_with_ball_standoff(
        p_ball_idx_th, p_ball_th_idx, r_cf_od=r_arch_od, r_shank=r_arch_od * 0.75, standoff=0.0040
    )
    cf_tubes.append(t_idx_th)
    clamps.append(s_idx_th)
    
    # -------------------------------------------------------------------------
    # 3. PHALANX BOOM CHAINS (MCP -> PIP -> DIP -> Pod)
    # -------------------------------------------------------------------------
    finger_ball_starts = {
        "index": p_ball_idx_phx,
        "middle": p_ball_mid_phx,
        "ring": p_ball_rng_phx,
        "little": p_ball_lit_phx,
        "thumb": p_ball_th_phx
    }
    
    for f, nodes in digit_chains.items():
        chain = [finger_ball_starts[f]] + nodes[1:]
        
        for i in range(len(chain) - 1):
            p0, p1 = chain[i], chain[i+1]
            
            t_link, s_link = build_phalanx_carbon_strut_with_ball_standoff(
                p0, p1, r_cf_od=r_branch_od, r_shank=0.0017, standoff=0.0045
            )
            cf_tubes.append(t_link)
            clamps.append(s_link)
            
            # Form 2-way dogbone clamp at intermediate phalanx joints (PIP, DIP)
            if i < len(chain) - 2:
                p2 = chain[i+2]
                clamp_pip, _ = build_oriented_joint_clamp(
                    p_center=p1, connected_pts=[p0, p2], n_surface=e_dorsal,
                    width=0.0070, arm_radius=0.0065
                )
                clamps.append(clamp_pip)
            else:
                # Terminal pod bracket ball clamp
                clamp_pod, _ = build_oriented_joint_clamp(
                    p_center=p1, connected_pts=[p0, p1 + 0.006 * (p1 - p0)], n_surface=e_dorsal,
                    width=0.0065, arm_radius=0.0055
                )
                clamps.append(clamp_pod)
                
    mesh_tubes = trimesh.util.concatenate(cf_tubes)
    mesh_clamps = trimesh.util.concatenate(clamps)
    
    return {
        "tubes": mesh_tubes,
        "clamps": mesh_clamps,
        "complete": trimesh.util.concatenate([mesh_tubes, mesh_clamps])
    }

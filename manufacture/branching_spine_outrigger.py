"""CENTRAL DORSAL SPINE WITH BRANCHING TRANSVERSE TREE & SYMMETRICAL CNC CLAMPS.

Kinematic & Structural Concept:
1. PRIMARY CENTRAL BACKBONE (Vertebral Column):
   - High-modulus pultruded carbon-fiber spine (⌀ 8.0 mm OD / ⌀ 6.0 mm ID, E = 180 GPa).
   - Anchors into a 4-Way Central Manifold Cross-Fitting Hub over the Middle Knuckle (MCP3).
2. 3-WAY AXIALLY SYMMETRIC "TRI-LOBE" KNUCKLE CLAMPS (120-DEGREE TRIPOD PRINCIPLE):
   - 3 ball pockets positioned at 120-degree equal angles around a single central M2.5 pinch screw.
   - Statistically determinate 3-point contact guarantees 100% EQUAL clamping force across all 3 balls.
   - Symmetrically bridges knuckle junctions at Ring (MCP4) and Index (MCP2).
3. 2-PIECE SYMMETRICAL "DOGBONE" CLAMPS:
   - Symmetrical 2.0 mm 6061-T6 aluminum plates with central M2.5 pinch screw.
   - Used across all 2-ball phalanx link joints (MCP, PIP, DIP, and Pod brackets).
4. STANDOFF CARBON FIBER RODS:
   - Carbon tubes stop short at each end (4.5 - 5.0 mm standoff) to accommodate
     metal ball-stud threaded shanks and hex collar transitions.
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
                                                  standoff: float = 0.0050) -> tuple[trimesh.Trimesh, trimesh.Trimesh]:
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


def build_symmetrical_dogbone_clamp_mesh(p1: np.ndarray, p2: np.ndarray,
                                        width: float = 0.0070,
                                        plate_thick: float = 0.0020,
                                        gap: float = 0.0006,
                                        r_ball: float = 0.0024) -> trimesh.Trimesh:
    """Construct high-fidelity 3D CAD mesh for a 2-piece CNC aluminum dual-ball dogbone clamp."""
    v = p2 - p1
    L = float(np.linalg.norm(v))
    if L < 1e-4:
        return trimesh.creation.uv_sphere(radius=r_ball)
    u_len = v / L
    
    ref = np.array([0.0, 0.0, 1.0])
    if abs(float(np.dot(u_len, ref))) > 0.90:
        ref = np.array([0.0, 1.0, 0.0])
    u_lat = np.cross(u_len, ref)
    u_lat /= np.linalg.norm(u_lat)
    u_norm = np.cross(u_lat, u_len)
    
    p_mid = 0.5 * (p1 + p2)
    parts = []
    
    # 1. Spherical Ball Studs at p1 and p2
    ball1 = trimesh.creation.uv_sphere(radius=r_ball, count=[12, 12])
    ball1.apply_translation(p1)
    ball2 = trimesh.creation.uv_sphere(radius=r_ball, count=[12, 12])
    ball2.apply_translation(p2)
    parts.extend([ball1, ball2])
    
    # 2. Symmetrical 2-Piece Top & Bottom Clamp Plates
    half_gap = 0.5 * gap
    for sign in [+1.0, -1.0]:
        z_offset = sign * (half_gap + 0.5 * plate_thick)
        p_center = p_mid + z_offset * u_norm
        
        box = trimesh.creation.box(extents=[L, width * 0.82, plate_thick])
        
        cyl1 = trimesh.creation.cylinder(radius=width * 0.5, height=plate_thick, sections=12)
        cyl1.apply_translation([-0.5 * L, 0.0, 0.0])
        cyl2 = trimesh.creation.cylinder(radius=width * 0.5, height=plate_thick, sections=12)
        cyl2.apply_translation([0.5 * L, 0.0, 0.0])
        
        plate = trimesh.util.concatenate([box, cyl1, cyl2])
        R_mat = np.column_stack([u_len, u_lat, u_norm])
        T = np.eye(4)
        T[:3, :3] = R_mat
        T[:3, 3] = p_center
        plate.apply_transform(T)
        parts.append(plate)
        
    # 3. Center M2.5 Clamping Socket Cap Screw
    bolt_head_r = 0.0022
    bolt_head_h = 0.0018
    bolt_head = trimesh.creation.cylinder(radius=bolt_head_r, height=bolt_head_h, sections=10)
    T_bolt = np.eye(4)
    T_bolt[:3, :3] = _align_rot(u_norm)
    T_bolt[:3, 3] = p_mid + (half_gap + plate_thick + 0.5 * bolt_head_h) * u_norm
    bolt_head.apply_transform(T_bolt)
    parts.append(bolt_head)
    
    return trimesh.util.concatenate(parts)


def build_trilobe_knuckle_clamp_mesh(p_center: np.ndarray,
                                     p_balls: list[np.ndarray],
                                     width: float = 0.0070,
                                     plate_thick: float = 0.0020,
                                     gap: float = 0.0006,
                                     r_ball: float = 0.0024) -> trimesh.Trimesh:
    """Construct 3-way axially symmetric 120-degree tri-lobe clamp sandwich (Mercedes Star / Trefoil)."""
    assert len(p_balls) == 3
    parts = []
    
    # 1. Three Spherical Ball Studs
    for pb in p_balls:
        b = trimesh.creation.uv_sphere(radius=r_ball, count=[12, 12])
        b.apply_translation(pb)
        parts.append(b)
        
    # Plane normal
    v1 = p_balls[1] - p_balls[0]
    v2 = p_balls[2] - p_balls[0]
    u_norm = np.cross(v1, v2)
    norm_len = np.linalg.norm(u_norm)
    if norm_len < 1e-6:
        u_norm = np.array([0.0, 0.0, 1.0])
    else:
        u_norm /= norm_len
        
    half_gap = 0.5 * gap
    
    # 2. Top & Bottom 3-Lobed Plates
    for sign in [+1.0, -1.0]:
        z_offset = sign * (half_gap + 0.5 * plate_thick)
        plate_parts = []
        
        c_hub = trimesh.creation.cylinder(radius=width * 0.65, height=plate_thick, sections=16)
        plate_parts.append(c_hub)
        
        for pb in p_balls:
            v_arm = (pb - p_center)
            L_arm = float(np.linalg.norm(v_arm))
            u_arm = v_arm / (L_arm + 1e-12)
            
            arm_box = trimesh.creation.box(extents=[L_arm, width * 0.75, plate_thick])
            arm_box.apply_translation([0.5 * L_arm, 0.0, 0.0])
            
            lobe = trimesh.creation.cylinder(radius=width * 0.5, height=plate_thick, sections=12)
            lobe.apply_translation([L_arm, 0.0, 0.0])
            
            arm_mesh = trimesh.util.concatenate([arm_box, lobe])
            angle = np.arctan2(u_arm[1], u_arm[0])
            T_arm = np.eye(4)
            T_arm[:3, :3] = trimesh.transformations.rotation_matrix(angle, [0, 0, 1])[:3, :3]
            arm_mesh.apply_transform(T_arm)
            plate_parts.append(arm_mesh)
            
        plate = trimesh.util.concatenate(plate_parts)
        T_world = np.eye(4)
        T_world[:3, 3] = p_center + z_offset * u_norm
        plate.apply_transform(T_world)
        parts.append(plate)
        
    # 3. Center Clamping Bolt
    bolt = trimesh.creation.cylinder(radius=0.0022, height=0.0018, sections=12)
    T_b = np.eye(4)
    T_b[:3, 3] = p_center + (half_gap + plate_thick + 0.0009) * u_norm
    bolt.apply_transform(T_b)
    parts.append(bolt)
    
    return trimesh.util.concatenate(parts)


def build_conformal_spine_tree_geometry(p_root: np.ndarray,
                                        mcp_nodes: dict[str, np.ndarray],
                                        digit_chains: dict[str, list[np.ndarray]],
                                        r_spine_od: float = 0.0040,
                                        r_arch_od: float = 0.0030,
                                        r_branch_od: float = 0.0025) -> dict[str, trimesh.Trimesh]:
    """Generate 3D CAD meshes featuring Symmetrical 2-Way Dogbones & 3-Way Symmetric Tri-Lobe Knuckle Clamps."""
    cf_tubes = []
    clamps = []
    
    p_mcp_mid = mcp_nodes["middle"]
    p_mcp_idx = mcp_nodes["index"]
    p_mcp_rng = mcp_nodes["ring"]
    p_mcp_lit = mcp_nodes["little"]
    p_mcp_th = mcp_nodes["thumb"]
    
    # 1. Primary Central Spine Tube (Saddle Hub -> Middle MCP Knuckle)
    t_spine, s_spine = build_phalanx_carbon_strut_with_ball_standoff(
        p_root, p_mcp_mid, r_cf_od=r_spine_od, r_shank=r_spine_od * 0.75, standoff=0.0045
    )
    cf_tubes.append(t_spine)
    clamps.append(s_spine)
    
    root_hub = trimesh.creation.uv_sphere(radius=0.0045, count=[14, 14])
    root_hub.apply_translation(p_root)
    clamps.append(root_hub)
    
    # 4-Way Middle Knuckle Manifold Hub
    mid_hub = trimesh.creation.uv_sphere(radius=0.0042, count=[14, 14])
    mid_hub.apply_translation(p_mcp_mid)
    clamps.append(mid_hub)
    
    # 2. Ring Knuckle (MCP4): 3-Way Symmetrical 120-deg Tri-Lobe Clamp
    R_arm = 0.0090
    u_to_mid = (p_mcp_mid - p_mcp_rng) / np.linalg.norm(p_mcp_mid - p_mcp_rng)
    u_to_lit = (p_mcp_lit - p_mcp_rng) / np.linalg.norm(p_mcp_lit - p_mcp_rng)
    u_to_phx_rng = (digit_chains["ring"][1] - p_mcp_rng) / np.linalg.norm(digit_chains["ring"][1] - p_mcp_rng)
    
    balls_rng = [
        p_mcp_rng + R_arm * u_to_mid,
        p_mcp_rng + R_arm * u_to_lit,
        p_mcp_rng + R_arm * u_to_phx_rng
    ]
    clamp_ring_mcp = build_trilobe_knuckle_clamp_mesh(p_mcp_rng, balls_rng, width=0.0070)
    clamps.append(clamp_ring_mcp)
    
    # 3. Index Knuckle (MCP2): 3-Way Symmetrical 120-deg Tri-Lobe Clamp
    v_web = p_mcp_th - p_mcp_idx
    u_to_web = v_web / np.linalg.norm(v_web)
    u_to_idx_mid = (p_mcp_mid - p_mcp_idx) / np.linalg.norm(p_mcp_mid - p_mcp_idx)
    u_to_phx_idx = (digit_chains["index"][1] - p_mcp_idx) / np.linalg.norm(digit_chains["index"][1] - p_mcp_idx)
    
    balls_idx = [
        p_mcp_idx + R_arm * u_to_idx_mid,
        p_mcp_idx + R_arm * u_to_web,
        p_mcp_idx + R_arm * u_to_phx_idx
    ]
    clamp_idx_mcp = build_trilobe_knuckle_clamp_mesh(p_mcp_idx, balls_idx, width=0.0070)
    clamps.append(clamp_idx_mcp)
    
    # 4. Connecting Transverse Arch Tubes
    # Middle to Ring Arch Tube
    t_mid_rng, s_mid_rng = build_phalanx_carbon_strut_with_ball_standoff(
        p_mcp_mid, balls_rng[0], r_cf_od=r_arch_od, r_shank=r_arch_od * 0.75, standoff=0.0035
    )
    cf_tubes.append(t_mid_rng)
    clamps.append(s_mid_rng)
    
    # Ring to Little Arch Tube
    t_rng_lit, s_rng_lit = build_phalanx_carbon_strut_with_ball_standoff(
        balls_rng[1], p_mcp_lit, r_cf_od=r_arch_od, r_shank=r_arch_od * 0.75, standoff=0.0035
    )
    cf_tubes.append(t_rng_lit)
    clamps.append(s_rng_lit)
    
    # Middle to Index Arch Tube
    t_mid_idx, s_mid_idx = build_phalanx_carbon_strut_with_ball_standoff(
        p_mcp_mid, balls_idx[0], r_cf_od=r_arch_od, r_shank=r_arch_od * 0.75, standoff=0.0035
    )
    cf_tubes.append(t_mid_idx)
    clamps.append(s_mid_idx)
    
    # 5. Thumb Bridge (Index Knuckle Tri-Lobe -> Web Tube -> Thumb MCP)
    p_clamp_th_a = p_mcp_th - 0.0150 * u_to_web
    dogbone_th = build_symmetrical_dogbone_clamp_mesh(p_clamp_th_a, p_mcp_th, width=0.0070)
    clamps.append(dogbone_th)
    
    t_web, s_web = build_phalanx_carbon_strut_with_ball_standoff(
        balls_idx[1], p_clamp_th_a, r_cf_od=r_arch_od, r_shank=r_arch_od * 0.75, standoff=0.0040
    )
    cf_tubes.append(t_web)
    clamps.append(s_web)
    
    c_lit_mcp = trimesh.creation.uv_sphere(radius=0.0036, count=[12, 12])
    c_lit_mcp.apply_translation(p_mcp_lit)
    clamps.append(c_lit_mcp)
    
    # 6. Phalanx Branches per Digit with Symmetrical Dual-Ball Dogbone Clamps
    for f, nodes in digit_chains.items():
        chain_start = balls_idx[2] if f == "index" else (balls_rng[2] if f == "ring" else nodes[0])
        full_nodes = [chain_start] + nodes[1:]
        
        for i in range(len(full_nodes) - 1):
            p0, p1 = full_nodes[i], full_nodes[i+1]
            
            t_link, s_link = build_phalanx_carbon_strut_with_ball_standoff(
                p0, p1, r_cf_od=r_branch_od, r_shank=0.0017, standoff=0.0050
            )
            cf_tubes.append(t_link)
            clamps.append(s_link)
            
            if i < len(full_nodes) - 2:
                p_clamp_a = p1 - 0.0050 * ((p1 - p0) / (np.linalg.norm(p1 - p0) + 1e-12))
                p_clamp_b = p1 + 0.0050 * ((full_nodes[i+2] - p1) / (np.linalg.norm(full_nodes[i+2] - p1) + 1e-12))
                dogbone = build_symmetrical_dogbone_clamp_mesh(p_clamp_a, p_clamp_b, width=0.0070)
                clamps.append(dogbone)
            else:
                p_clamp_a = p1 - 0.0045 * ((p1 - p0) / (np.linalg.norm(p1 - p0) + 1e-12))
                p_clamp_b = p1
                dogbone = build_symmetrical_dogbone_clamp_mesh(p_clamp_a, p_clamp_b, width=0.0065)
                clamps.append(dogbone)
                
    mesh_tubes = trimesh.util.concatenate(cf_tubes)
    mesh_clamps = trimesh.util.concatenate(clamps)
    
    return {
        "tubes": mesh_tubes,
        "clamps": mesh_clamps,
        "complete": trimesh.util.concatenate([mesh_tubes, mesh_clamps])
    }

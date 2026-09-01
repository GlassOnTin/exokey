"""CENTRAL DORSAL SPINE WITH BRANCHING TRANSVERSE TREE & SYMMETRICAL CNC DOGBONE CLAMPS.

Kinematic & Structural Concept:
1. PRIMARY CENTRAL BACKBONE (Vertebral Column):
   - High-modulus pultruded carbon-fiber spine (⌀ 8.0 mm OD / ⌀ 6.0 mm ID, E = 180 GPa).
   - Anchors into a 4-Way Central Manifold Cross-Fitting Hub over the Middle Knuckle (MCP3).
2. 4-WAY CENTRAL MANIFOLD HUB (Middle Knuckle Joint):
   - Precision CNC Titanium / 6061 Aluminum cross-fitting connecting:
     * Posterior Port: Primary Spine (⌀ 8.0 mm)
     * Lateral Ports: Transverse Knuckle Arch (⌀ 6.0 mm) to Ring & Index
     * Anterior Port: Middle Finger Phalanx Boom via Symmetrical Dogbone Clamp.
3. UNIFIED CONTINUOUS KNUCKLE ARCH:
   - Little MCP <-> Ring MCP <-> Middle MCP <-> Index MCP <-> Thumb MCP.
4. DIRECT THUMB STRUT ATTACHED TO INDEX KNUCKLE JOINT:
   - Arches across the 1st webspace corridor directly from Index MCP to Thumb MCP.
5. SYMMETRICAL CNC ALUMINUM DUAL-BALL DOGBONE CLAMPS & STANDOFF RODS:
   - Carbon fiber rods stop short at each end (4.5 - 5.5 mm standoff) to accommodate
     metal ball-stud threaded shanks and hex collar transitions.
   - 2-piece symmetrical clamping sandwich plates (2.5 mm 6061-T6 aluminum).
   - Equidistant center M2.5 bolt pinches both ⌀ 6.0 mm ball studs simultaneously.
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
                                        width: float = 0.0075,
                                        plate_thick: float = 0.0022,
                                        gap: float = 0.0008,
                                        r_ball: float = 0.0030) -> trimesh.Trimesh:
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


def build_conformal_spine_tree_geometry(p_root: np.ndarray,
                                        mcp_nodes: dict[str, np.ndarray],
                                        digit_chains: dict[str, list[np.ndarray]],
                                        r_spine_od: float = 0.0040,
                                        r_arch_od: float = 0.0030,
                                        r_branch_od: float = 0.0025) -> dict[str, trimesh.Trimesh]:
    """Generate 3D CAD meshes featuring Symmetrical CNC Dual-Ball Dogbone Clamps & Standoff Carbon Rods."""
    cf_tubes = []
    clamps = []
    
    p_mcp_mid = mcp_nodes["middle"]
    p_mcp_idx = mcp_nodes["index"]
    p_mcp_th = mcp_nodes["thumb"]
    
    # 1. Primary Central Spine Tube (Saddle Hub -> Middle MCP Knuckle)
    # Tube stops short by 4.5mm at each hub collar
    t_spine, s_spine = build_phalanx_carbon_strut_with_ball_standoff(
        p_root, p_mcp_mid, r_cf_od=r_spine_od, r_shank=r_spine_od * 0.75, standoff=0.0045
    )
    cf_tubes.append(t_spine)
    clamps.append(s_spine)
    
    # Saddle root collar hub
    root_hub = trimesh.creation.uv_sphere(radius=0.0045, count=[14, 14])
    root_hub.apply_translation(p_root)
    clamps.append(root_hub)
    
    # 4-Way Middle Knuckle Manifold Cross-Fitting Hub
    mid_hub = trimesh.creation.uv_sphere(radius=0.0042, count=[14, 14])
    mid_hub.apply_translation(p_mcp_mid)
    clamps.append(mid_hub)
    
    # 2. Transverse Arch across Fingers (Little <-> Ring <-> Middle <-> Index)
    arch_order = ["little", "ring", "middle", "index"]
    for i in range(len(arch_order) - 1):
        pA = mcp_nodes[arch_order[i]]
        pB = mcp_nodes[arch_order[i+1]]
        t_arch, s_arch = build_phalanx_carbon_strut_with_ball_standoff(
            pA, pB, r_cf_od=r_arch_od, r_shank=r_arch_od * 0.75, standoff=0.0035
        )
        cf_tubes.append(t_arch)
        clamps.append(s_arch)
        
    for f in ["little", "ring", "index"]:
        c_node = trimesh.creation.uv_sphere(radius=0.0036, count=[12, 12])
        c_node.apply_translation(mcp_nodes[f])
        clamps.append(c_node)
        
    # 3. DIRECT THUMB STRUT ATTACHED TO INDEX KNUCKLE JOINT (Index MCP -> Web Tube -> Thumb MCP)
    # Uses standard modular 16mm dogbone clamps at both joint hubs connected by a ⌀ 6.0mm CF tube
    v_web = p_mcp_th - p_mcp_idx
    L_web = float(np.linalg.norm(v_web))
    u_web = v_web / (L_web + 1e-12)
    
    # Standard 16mm clamp span at Index MCP
    p_clamp_idx_a = p_mcp_idx
    p_clamp_idx_b = p_mcp_idx + 0.0160 * u_web
    dogbone_idx = build_symmetrical_dogbone_clamp_mesh(p_clamp_idx_a, p_clamp_idx_b, width=0.0075)
    clamps.append(dogbone_idx)
    
    # Standard 16mm clamp span at Thumb MCP
    p_clamp_th_a = p_mcp_th - 0.0160 * u_web
    p_clamp_th_b = p_mcp_th
    dogbone_th = build_symmetrical_dogbone_clamp_mesh(p_clamp_th_a, p_clamp_th_b, width=0.0075)
    clamps.append(dogbone_th)
    
    # Straight Carbon Fiber Bridge Tube across 1st Webspace (stopping short for ball studs)
    t_web, s_web = build_phalanx_carbon_strut_with_ball_standoff(
        p_clamp_idx_b, p_clamp_th_a, r_cf_od=r_arch_od, r_shank=r_arch_od * 0.75, standoff=0.0040
    )
    cf_tubes.append(t_web)
    clamps.append(s_web)
    
    # 4. Phalanx Branches per Digit with Symmetrical Dual-Ball Dogbone Clamps & Standoff Tubes
    for f, nodes in digit_chains.items():
        for i in range(len(nodes) - 1):
            p0, p1 = nodes[i], nodes[i+1]
            
            # Carbon tube stops short by 5.0 mm at each end for ball-stud shank/collar
            t_link, s_link = build_phalanx_carbon_strut_with_ball_standoff(
                p0, p1, r_cf_od=r_branch_od, r_shank=0.0017, standoff=0.0050
            )
            cf_tubes.append(t_link)
            clamps.append(s_link)
            
            # Symmetrical CNC 2-Piece Dogbone Clamp bridging between successive joint nodes
            if i < len(nodes) - 2:
                # Symmetrical dogbone clamp centered around joint p1
                p_prev_mid = 0.5 * (p0 + p1)
                p_next_mid = 0.5 * (p1 + nodes[i+2])
                p_clamp_a = p1 - 0.0055 * ((p1 - p0) / (np.linalg.norm(p1 - p0) + 1e-12))
                p_clamp_b = p1 + 0.0055 * ((nodes[i+2] - p1) / (np.linalg.norm(nodes[i+2] - p1) + 1e-12))
                dogbone = build_symmetrical_dogbone_clamp_mesh(p_clamp_a, p_clamp_b, width=0.0070)
                clamps.append(dogbone)
            else:
                # Terminal pod bracket ball clamp at DIP -> Pod
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

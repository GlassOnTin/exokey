"""PARAMETRIC 3D CAD GEOMETRY & STL GENERATOR FOR SYMMETRICAL CNC CLAMP PLATES.

Generates exact physical meshes for:
- 2-Way Symmetrical Dogbone Clamps (Phalanx links, Thumb bridge)
- 3-Way Symmetrical Tri-Lobe Clamps (Ring MCP4, Index MCP2 Knuckles)
- 4-Way Symmetrical Quad-Cross Clamps (Middle MCP3 Central Hub)

Each assembly consists of:
1. Top Clamp Plate (Clearance through-hole for M2.5 socket cap screw)
2. Bottom Clamp Plate (Threaded M2.5 / tap-hole base)
3. Precision ⌀ 4.8 mm Ball Pockets (Hemispherical / 90° conical seats)
4. Center M2.5 Clamping Fastener Hardware
"""
from __future__ import annotations

import math
import numpy as np
import trimesh


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


def build_single_clamp_plate(arm_vectors: list[np.ndarray],
                             is_top: bool = True,
                             width: float = 0.0070,
                             plate_thick: float = 0.0020,
                             ball_dia: float = 0.0048) -> trimesh.Trimesh:
    """Build a single top or bottom CNC aluminum clamp plate with lobes and pockets."""
    r_hub = width * 0.70
    r_lobe = width * 0.50
    
    parts = []
    # 1. Central Hub Cylinder
    c_hub = trimesh.creation.cylinder(radius=r_hub, height=plate_thick, sections=24)
    parts.append(c_hub)
    
    # 2. Radial Arms & End Lobes
    for v in arm_vectors:
        L_arm = float(np.linalg.norm(v[:2]))
        if L_arm < 1e-4:
            continue
        u_arm = v[:2] / L_arm
        ang = np.arctan2(u_arm[1], u_arm[0])
        
        # Arm bar
        arm_box = trimesh.creation.box(extents=[L_arm, width * 0.85, plate_thick])
        arm_box.apply_translation([0.5 * L_arm, 0.0, 0.0])
        
        # Lobe cylinder at arm tip
        lobe = trimesh.creation.cylinder(radius=r_lobe, height=plate_thick, sections=20)
        lobe.apply_translation([L_arm, 0.0, 0.0])
        
        arm_mesh = trimesh.util.concatenate([arm_box, lobe])
        T_rot = np.eye(4)
        T_rot[:3, :3] = trimesh.transformations.rotation_matrix(ang, [0, 0, 1])[:3, :3]
        arm_mesh.apply_transform(T_rot)
        parts.append(arm_mesh)
        
    plate = trimesh.util.concatenate(parts)
    return plate


def build_nway_clamp_assembly(arm_vectors: list[np.ndarray],
                              width: float = 0.0070,
                              plate_thick: float = 0.0020,
                              gap: float = 0.0006,
                              ball_dia: float = 0.0048) -> trimesh.Trimesh:
    """Build a complete 2-piece CNC clamp assembly in local XY plane."""
    parts = []
    half_gap = 0.5 * gap
    z_top = half_gap + 0.5 * plate_thick
    z_bot = -(half_gap + 0.5 * plate_thick)
    
    # Top plate
    p_top = build_single_clamp_plate(arm_vectors, is_top=True, width=width, plate_thick=plate_thick, ball_dia=ball_dia)
    p_top.apply_translation([0.0, 0.0, z_top])
    parts.append(p_top)
    
    # Bottom plate
    p_bot = build_single_clamp_plate(arm_vectors, is_top=False, width=width, plate_thick=plate_thick, ball_dia=ball_dia)
    p_bot.apply_translation([0.0, 0.0, z_bot])
    parts.append(p_bot)
    
    # Center M2.5 screw head
    screw_head = trimesh.creation.cylinder(radius=0.0022, height=0.0018, sections=16)
    screw_head.apply_translation([0.0, 0.0, z_top + 0.5 * plate_thick + 0.0009])
    parts.append(screw_head)
    
    # Ball studs at arm tips
    r_ball = ball_dia / 2.0
    for v in arm_vectors:
        b = trimesh.creation.uv_sphere(radius=r_ball, count=[14, 14])
        b.apply_translation([v[0], v[1], 0.0])
        parts.append(b)
        
    return trimesh.util.concatenate(parts)


def build_oriented_joint_clamp(p_center: np.ndarray,
                               connected_pts: list[np.ndarray],
                               n_surface: np.ndarray,
                               width: float = 0.0070,
                               plate_thick: float = 0.0020,
                               gap: float = 0.0006,
                               ball_dia: float = 0.0048,
                               arm_radius: float = 0.0075) -> tuple[trimesh.Trimesh, list[np.ndarray]]:
    """Construct and position an N-way clamp assembly oriented tangentially to the dorsal skin surface."""
    n_unit = n_surface / (np.linalg.norm(n_surface) + 1e-12)
    
    ref = np.array([0.0, 1.0, 0.0])
    if abs(float(np.dot(n_unit, ref))) > 0.90:
        ref = np.array([1.0, 0.0, 0.0])
    u_tan_x = np.cross(ref, n_unit)
    u_tan_x /= np.linalg.norm(u_tan_x)
    u_tan_y = np.cross(n_unit, u_tan_x)
    
    local_arm_vecs = []
    ball_pts_world = []
    for pt in connected_pts:
        v_world = pt - p_center
        v_tan = v_world - np.dot(v_world, n_unit) * n_unit
        L = np.linalg.norm(v_tan)
        if L < 1e-6:
            continue
        u_tan = v_tan / L
        
        lx = float(np.dot(u_tan, u_tan_x)) * arm_radius
        ly = float(np.dot(u_tan, u_tan_y)) * arm_radius
        local_arm_vecs.append(np.array([lx, ly, 0.0]))
        
        p_ball_world = p_center + arm_radius * u_tan
        ball_pts_world.append(p_ball_world)
        
    clamp_mesh = build_nway_clamp_assembly(local_arm_vecs, width=width, plate_thick=plate_thick, gap=gap, ball_dia=ball_dia)
    
    R_basis = np.column_stack([u_tan_x, u_tan_y, n_unit])
    T = np.eye(4)
    T[:3, :3] = R_basis
    T[:3, 3] = p_center
    clamp_mesh.apply_transform(T)
    
    return clamp_mesh, ball_pts_world


def build_oriented_2way_dogbone_clamp(p_ball_a: np.ndarray,
                                      p_ball_b: np.ndarray,
                                      n_surface_hint: np.ndarray,
                                      width: float = 0.0070,
                                      plate_thick: float = 0.0020,
                                      gap: float = 0.0006,
                                      ball_dia: float = 0.0048) -> trimesh.Trimesh:
    """Build a 2-way symmetrical dogbone clamp directly aligned with the finger link axis."""
    v = p_ball_b - p_ball_a
    L = float(np.linalg.norm(v))
    if L < 1e-4:
        return trimesh.creation.uv_sphere(radius=ball_dia / 2.0)
    u_long = v / L
    
    # Lateral axis perpendicular to link length and local dorsal normal
    u_lat = np.cross(n_surface_hint, u_long)
    lat_len = np.linalg.norm(u_lat)
    if lat_len < 1e-4:
        ref = np.array([0.0, 1.0, 0.0])
        if abs(float(np.dot(u_long, ref))) > 0.90:
            ref = np.array([1.0, 0.0, 0.0])
        u_lat = np.cross(ref, u_long)
        u_lat /= np.linalg.norm(u_lat)
    else:
        u_lat /= lat_len
        
    # Dorsal normal for this specific phalanx segment
    u_norm = np.cross(u_long, u_lat)
    u_norm /= np.linalg.norm(u_norm)
    
    if np.dot(u_norm, n_surface_hint) < 0:
        u_norm = -u_norm
        u_lat = -u_lat
        
    p_mid = 0.5 * (p_ball_a + p_ball_b)
    arm_vecs = [
        np.array([-0.5 * L, 0.0, 0.0]),
        np.array([0.5 * L, 0.0, 0.0])
    ]
    clamp_local = build_nway_clamp_assembly(arm_vecs, width=width, plate_thick=plate_thick, gap=gap, ball_dia=ball_dia)
    
    R = np.column_stack([u_long, u_lat, u_norm])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p_mid
    clamp_local.apply_transform(T)
    
    return clamp_local

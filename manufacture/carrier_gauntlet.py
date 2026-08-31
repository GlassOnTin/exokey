"""BIOMORPHIC CARRIER GAUNTLET & SVALBOARD CO-DESIGN

Generates an integrated, anatomical wearable exoskeleton structure:
  1. Conformal Double-Curved Dorsal Saddle Vault (spanning MC2-MC5)
  2. Four Cantilevered Hollow-Section Anatomical Rib Arches (over MCP2-MCP5)
  3. C-shaped Sweeping Radial Outrigger Boom (for Opposed Thumb Cluster)
  4. Proximal Carpal Anchor Hub with Wrist Strap Retention Lugs
  5. Direct Integration into Svalboard 5-Way Key Modules
"""
from __future__ import annotations

import numpy as np
import trimesh
from scipy.interpolate import make_interp_spline

from hand.myohand import FINGERS, MyoHand
from structure.frame import hand_axes


def _spline_tube(points: np.ndarray, radius: float = 0.0025, sections: int = 10, num_steps: int = 8) -> trimesh.Trimesh:
    """Generate a smooth swept tube along a 3D path using robust pure-trimesh cylinder/sphere chains."""
    points = np.asarray(points, float)
    if len(points) < 2:
        return trimesh.Trimesh()
    
    t = np.linspace(0, 1, len(points))
    t_fine = np.linspace(0, 1, max(len(points) * 3, num_steps))
    spline = make_interp_spline(t, points, k=min(3, len(points) - 1))
    path = spline(t_fine)
    
    cyls = []
    z_axis = np.array([0.0, 0.0, 1.0])
    
    for i in range(len(path) - 1):
        p0, p1 = path[i], path[i + 1]
        v = p1 - p0
        L = float(np.linalg.norm(v))
        if L < 1e-5:
            continue
            
        cyl = trimesh.creation.cylinder(radius=radius, height=L, sections=sections)
        dir_norm = v / L
        rot_axis = np.cross(z_axis, dir_norm)
        rot_norm = np.linalg.norm(rot_axis)
        
        if rot_norm < 1e-6:
            R = np.eye(3) if dir_norm[2] > 0 else np.diag([1, -1, -1])
        else:
            rot_axis /= rot_norm
            angle = np.arccos(np.clip(np.dot(z_axis, dir_norm), -1.0, 1.0))
            K = np.array([[0, -rot_axis[2], rot_axis[1]],
                          [rot_axis[2], 0, -rot_axis[0]],
                          [-rot_axis[1], rot_axis[0], 0]])
            R = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)
            
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = 0.5 * (p0 + p1)
        cyl.apply_transform(T)
        cyls.append(cyl)
        
        sph = trimesh.creation.uv_sphere(radius=radius, count=[sections, sections])
        sph.apply_translation(p0)
        cyls.append(sph)
        
    sph_end = trimesh.creation.uv_sphere(radius=radius, count=[sections, sections])
    sph_end.apply_translation(path[-1])
    cyls.append(sph_end)
    
    return trimesh.util.concatenate(cyls)


def _tapered_spline_tube(waypoints: np.ndarray, r_start: float, r_end: float,
                         sections: int = 10, num_steps: int = 18) -> trimesh.Trimesh:
    """Generate a smooth swept tube with continuous nonlinear cross-sectional taper along a 3D spline."""
    pts = np.asarray(waypoints, float)
    k = min(3, len(pts) - 1)
    if k < 1:
        return trimesh.Trimesh()
        
    t = np.linspace(0, 1, len(pts))
    t_fine = np.linspace(0, 1, max(len(pts) * 3, num_steps))
    spline = make_interp_spline(t, pts, k=k)
    path = spline(t_fine)
    
    cyls = []
    z_axis = np.array([0.0, 0.0, 1.0])
    
    # Nonlinear taper profile: gradual taper mid-span, slender distal tip
    r_profile = r_start + (r_end - r_start) * (t_fine ** 0.85)
    
    for i in range(len(path) - 1):
        p0, p1 = path[i], path[i + 1]
        v = p1 - p0
        L = float(np.linalg.norm(v))
        if L < 1e-5:
            continue
            
        r_seg = 0.5 * (r_profile[i] + r_profile[i + 1])
        cyl = trimesh.creation.cylinder(radius=r_seg, height=L, sections=sections)
        dir_norm = v / L
        rot_axis = np.cross(z_axis, dir_norm)
        rot_norm = np.linalg.norm(rot_axis)
        
        if rot_norm < 1e-6:
            R = np.eye(3) if dir_norm[2] > 0 else np.diag([1, -1, -1])
        else:
            rot_axis /= rot_norm
            angle = np.arccos(np.clip(np.dot(z_axis, dir_norm), -1.0, 1.0))
            K = np.array([[0, -rot_axis[2], rot_axis[1]],
                          [rot_axis[2], 0, -rot_axis[0]],
                          [-rot_axis[1], rot_axis[0], 0]])
            R = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)
            
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = 0.5 * (p0 + p1)
        cyl.apply_transform(T)
        cyls.append(cyl)
        
        sph = trimesh.creation.uv_sphere(radius=r_profile[i], count=[sections, sections])
        sph.apply_translation(p0)
        cyls.append(sph)
        
    sph_end = trimesh.creation.uv_sphere(radius=r_profile[-1], count=[sections, sections])
    sph_end.apply_translation(path[-1])
    cyls.append(sph_end)
    
    return trimesh.util.concatenate(cyls)


def _anatomical_outrigger_path(start_pt: np.ndarray, pod_base: np.ndarray, o: np.ndarray,
                               e_o: np.ndarray, e_d: np.ndarray, lateral_vec: np.ndarray,
                               is_brace: bool = False) -> np.ndarray:
    """Generate a sleek, strictly monotonic downward strut path from dorsal saddle to distal pod.
    
    In side view (e_o elevation), the path slopes continuously downward from the dorsal
    metacarpal saddle (+24 mm) down to the distal keywell pod (-75 mm), eliminating any upward
    camel humps while maintaining clean anatomical skin standoff along the lateral flank corridor.
    """
    p0 = np.asarray(start_pt, float)
    p_end = np.asarray(pod_base, float)
    
    h0 = float((p0 - o) @ e_o)
    h_end = float((p_end - o) @ e_o)
    total_drop = h0 - h_end
    
    t1, t2, t3, t4 = 0.18, 0.45, 0.72, 0.88
    
    lat_mag1 = 0.008 if is_brace else 0.006
    lat_mag2 = 0.026 if is_brace else 0.020
    lat_mag3 = 0.024 if is_brace else 0.018
    lat_mag4 = 0.012 if is_brace else 0.008
    
    p1_base = (1 - t1) * p0 + t1 * p_end + lat_mag1 * lateral_vec
    p2_base = (1 - t2) * p0 + t2 * p_end + lat_mag2 * lateral_vec
    p3_base = (1 - t3) * p0 + t3 * p_end + lat_mag3 * lateral_vec
    p4_base = (1 - t4) * p0 + t4 * p_end + lat_mag4 * lateral_vec
    
    if total_drop > 0.040:
        # Curled / relaxed posture: smooth steady downward descent dorsal to knuckles
        h1 = h0 - 0.03 * total_drop
        h2 = h0 - 0.12 * total_drop
        h3 = h0 - 0.38 * total_drop
        h4 = h0 - 0.75 * total_drop
    else:
        # Flat posture: gentle continuous downward slope
        h1 = h0 - 0.0005
        h2 = h0 - 0.0020
        h3 = h0 - 0.0045
        h4 = h0 - 0.0070
        
    p1 = p1_base - ((p1_base - o) @ e_o) * e_o + h1 * e_o
    p2 = p2_base - ((p2_base - o) @ e_o) * e_o + h2 * e_o
    p3 = p3_base - ((p3_base - o) @ e_o) * e_o + h3 * e_o
    p4 = p4_base - ((p4_base - o) @ e_o) * e_o + h4 * e_o
    
    return np.array([p0, p1, p2, p3, p4, p_end])


def build_organic_carrier_gauntlet(h: MyoHand, q: np.ndarray, sval_units: dict) -> dict[str, trimesh.Trimesh]:
    """Build the anatomical exoskeleton gauntlet with functionally graded saddle and tapered trusses."""
    o, e_d, e_r, e_o = hand_axes(h, q)
    
    from structure.anchor import bearing_surface
    P, N, K, T = bearing_surface(h, q)
    P = np.asarray(P)
    
    mc = P[(P - o) @ e_d > 0.002]
    if len(mc) < 4:
        mc = P
        
    r_lo, r_hi = np.min((mc - o) @ e_r), np.max((mc - o) @ e_r)
    d_lo, d_hi = np.min((mc - o) @ e_d), np.max((mc - o) @ e_d)
    
    hgt = float(np.percentile((P - o) @ e_o, 90)) + 0.0055   # 5.5 mm comfortable dorsal saddle standoff
    
    meshes = {}
    
    # -------------------------------------------------------------------------
    # 1. FUNCTIONALLY GRADED DORSAL SADDLE VAULT
    # -------------------------------------------------------------------------
    n_across, n_along = 6, 4
    rs = np.linspace(r_lo - 0.001, r_hi + 0.001, n_across)
    ds = np.linspace(d_lo - 0.004, d_hi + 0.004, n_along)
    
    r_mid = 0.5 * (r_lo + r_hi)
    r_span = max(r_hi - r_lo, 0.02)
    
    grid_nodes = []
    for d_val in ds:
        row = []
        for r_val in rs:
            arch_sag = 0.003 * (1.0 - ((r_val - r_mid) / (0.5 * r_span))**2)
            pt = o + r_val * e_r + d_val * e_d + (hgt + arch_sag) * e_o
            row.append(pt)
        grid_nodes.append(row)
    grid_nodes = np.array(grid_nodes)
    
    saddle_tubes = []
    # Functionally graded longitudinal ribs:
    # Thicker over MC2/MC3 (cols 3, 4) and strap borders (cols 0, 5); compliant over MC4/MC5 (cols 1, 2)
    for col_idx in range(n_across):
        pts = grid_nodes[:, col_idx, :]
        rad = 0.0022 if col_idx in [0, 3, 4, 5] else 0.0016
        saddle_tubes.append(_spline_tube(pts, radius=rad, sections=8, num_steps=8))
        
    # Functionally graded transverse arches:
    # Distal MCP torque bridge (row 3) = 2.4 mm; proximal strap beam (row 0) = 2.0 mm; mid = 1.5 mm
    for row_idx in range(n_along):
        pts = grid_nodes[row_idx, :, :]
        rad = 0.0024 if row_idx == n_along - 1 else (0.0020 if row_idx == 0 else 0.0015)
        saddle_tubes.append(_spline_tube(pts, radius=rad, sections=8, num_steps=10))
        
    # Diagonal shear braces
    for row_idx in range(n_along - 1):
        for col_idx in range(n_across - 1):
            p1 = grid_nodes[row_idx, col_idx]
            p2 = grid_nodes[row_idx + 1, col_idx + 1]
            p3 = grid_nodes[row_idx + 1, col_idx]
            p4 = grid_nodes[row_idx, col_idx + 1]
            saddle_tubes.append(_spline_tube(np.array([p1, p2]), radius=0.0012, sections=6, num_steps=4))
            saddle_tubes.append(_spline_tube(np.array([p3, p4]), radius=0.0012, sections=6, num_steps=4))
            
    # -------------------------------------------------------------------------
    # 2. PROXIMAL CARPAL ANCHOR & WRIST STRAP LUGS
    # -------------------------------------------------------------------------
    prox_row = grid_nodes[0, :, :]
    wrist_hub_left = prox_row[0] - 0.004 * e_d
    wrist_hub_right = prox_row[-1] - 0.004 * e_d
    
    lug_left = trimesh.creation.box(
        extents=[0.020, 0.005, 0.007],
        transform=trimesh.transformations.translation_matrix(wrist_hub_left + [0, 0, -0.002])
    )
    lug_right = trimesh.creation.box(
        extents=[0.020, 0.005, 0.007],
        transform=trimesh.transformations.translation_matrix(wrist_hub_right + [0, 0, -0.002])
    )
    saddle_tubes.extend([lug_left, lug_right])
    
    # -------------------------------------------------------------------------
    # 3. FOUR CONTINUOUSLY TAPERED OUTRIGGER RIBS (Index, Middle, Ring, Little)
    # -------------------------------------------------------------------------
    outrigger_tubes = []
    distal_row = grid_nodes[-1, :, :]
    
    finger_anchors = {
        "index": distal_row[int(n_across * 0.85)],
        "middle": distal_row[int(n_across * 0.60)],
        "ring": distal_row[int(n_across * 0.35)],
        "little": distal_row[int(n_across * 0.10)],
    }
    
    for f in ["index", "middle", "ring", "little"]:
        if f not in sval_units:
            continue
        start_pt = finger_anchors[f]
        u = sval_units[f]
        wf = h.well_frame(q, f)
        half = float(wf.get("half", 0.007))
        cavity_depth = 1.1 * half
        pod_base = u["center"] + (0.5 * cavity_depth + 0.008) * u["dirs"]["click"]
        
        lat = np.asarray(wf["lateral"])
        lat_out = -lat if f in ["index", "middle"] else lat
        
        path_main = _anatomical_outrigger_path(start_pt, pod_base, o, e_o, e_d, lat_out, is_brace=False)
        start_brace = start_pt - 0.005 * e_d + (0.003 if f in ["index", "middle"] else -0.003) * e_r
        pod_brace = pod_base + 0.002 * u["dirs"]["forward"]
        path_brace = _anatomical_outrigger_path(start_brace, pod_brace, o, e_o, e_d, lat_out, is_brace=True)
        
        # Continuously tapered booms: maximum depth at root, slender lightweight tip
        rib_tube1 = _tapered_spline_tube(path_main, r_start=0.0034, r_end=0.0022, sections=10, num_steps=18)
        rib_tube2 = _tapered_spline_tube(path_brace, r_start=0.0024, r_end=0.0016, sections=10, num_steps=18)
        
        mid1 = path_main[2]
        mid2 = path_brace[2]
        web = _spline_tube(np.array([mid1, mid2]), radius=0.0014, sections=8, num_steps=6)
        
        flange = trimesh.creation.cylinder(
            radius=0.0035, height=0.002,
            transform=trimesh.transformations.translation_matrix(pod_base)
        )
        
        outrigger_tubes.extend([rib_tube1, rib_tube2, web, flange])
        
    # -------------------------------------------------------------------------
    # 4. SWEEPING TAPERED RADIAL THUMB TRUSS
    # -------------------------------------------------------------------------
    if "thumb" in sval_units:
        u_thumb = sval_units["thumb"]
        wf_thumb = h.well_frame(q, "thumb")
        half_thumb = float(wf_thumb.get("half", 0.007))
        thumb_depth = 1.1 * half_thumb
        thumb_pod_base = u_thumb["center"] + (0.5 * thumb_depth + 0.008) * u_thumb["dirs"]["click"]
        
        thumb_root_main = grid_nodes[-1, -1]
        thumb_root_brace = grid_nodes[0, -1]
        
        def _natural_thumb_path(root_pt: np.ndarray, target_pt: np.ndarray, is_brace: bool = False) -> np.ndarray:
            p0 = np.asarray(root_pt, float)
            p_end = np.asarray(target_pt, float)
            
            h0 = float((p0 - o) @ e_o)
            h_end = float((p_end - o) @ e_o)
            total_drop = h0 - h_end
            
            t1, t2, t3 = 0.25, 0.58, 0.85
            
            p1_base = (1 - t1) * p0 + t1 * p_end + (0.012 if is_brace else 0.010) * e_r
            p2_base = (1 - t2) * p0 + t2 * p_end + (0.018 if is_brace else 0.015) * e_r
            p3_base = (1 - t3) * p0 + t3 * p_end + (0.012 if is_brace else 0.009) * e_r
            
            if total_drop > 0.040:
                h1 = h0 - 0.15 * total_drop
                h2 = h0 - 0.50 * total_drop
                h3 = h0 - 0.82 * total_drop
            else:
                h1 = h0 - 0.002
                h2 = h0 - 0.006
                h3 = h0 - 0.012
                
            p1 = p1_base - ((p1_base - o) @ e_o) * e_o + h1 * e_o
            p2 = p2_base - ((p2_base - o) @ e_o) * e_o + h2 * e_o
            p3 = p3_base - ((p3_base - o) @ e_o) * e_o + h3 * e_o
            
            return np.array([p0, p1, p2, p3, p_end])

        path_thumb_main = _natural_thumb_path(thumb_root_main, thumb_pod_base, is_brace=False)
        pod_brace_pt = thumb_pod_base + 0.002 * u_thumb["dirs"]["forward"]
        path_thumb_brace = _natural_thumb_path(thumb_root_brace, pod_brace_pt, is_brace=True)
        
        thumb_boom1 = _tapered_spline_tube(path_thumb_main, r_start=0.0030, r_end=0.0015, sections=10, num_steps=18)
        thumb_boom2 = _tapered_spline_tube(path_thumb_brace, r_start=0.0022, r_end=0.0012, sections=10, num_steps=18)
        thumb_web = _spline_tube(np.array([path_thumb_main[2], path_thumb_brace[2]]), radius=0.0015, sections=8, num_steps=6)
        
        thumb_flange = trimesh.creation.cylinder(
            radius=0.0045, height=0.003,
            transform=trimesh.transformations.translation_matrix(thumb_pod_base)
        )
        outrigger_tubes.extend([thumb_boom1, thumb_boom2, thumb_web, thumb_flange])
        
    saddle_mesh = trimesh.util.concatenate(saddle_tubes)
    outriggers_mesh = trimesh.util.concatenate(outrigger_tubes)
    full_chassis = trimesh.util.concatenate([saddle_mesh, outriggers_mesh])
    
    meshes["saddle"] = saddle_mesh
    meshes["outriggers"] = outriggers_mesh
    meshes["chassis"] = full_chassis
    return meshes

"""SVALBOARD KEY UNITS — 3D Geometric Modeling of the Svalboard 5-Way Finger Units and Towers.

Each Svalboard finger unit presents a 5-way cluster at the fingertip:
  1. Center Plunge Key (Down / Click into floor)
  2. North Paddle (Forward / Distal push)
  3. South Paddle (Back / Proximal pull)
  4. West Paddle (Left / Radial push)
  5. East Paddle (Right / Ulnar push)

Supported by:
  - Sensor Pod / PCBA Base under the well floor
  - U-channel Finger Well Cradle
  - Mounting Tower / Stalk connecting the unit to the carrier gauntlet deck bolt
"""
from __future__ import annotations

import numpy as np
import trimesh
import plotly.graph_objects as go

from hand.myohand import FINGERS, MyoHand
from structure.frame import hand_axes

FINGER_COLORS = {
    "thumb": "#e45756",
    "index": "#4c78a8",
    "middle": "#54a24b",
    "ring": "#f58518",
    "little": "#b279a2",
}

DIRECTION_COLORS = {
    "click": "#f28e2b",     # amber / gold for center plunge
    "forward": "#4e79a7",   # blue for forward/north
    "back": "#e15759",      # red/coral for back/south
    "left": "#76b7b2",      # teal for left/west
    "right": "#59a14f",     # green for right/east
}


def _rot_matrix_from_vectors(ax, fl, lat):
    """3x3 rotation matrix whose columns are the well basis vectors."""
    R = np.column_stack([ax, fl, lat])
    return R


def _transform_4x4(R, origin):
    """4x4 homogeneous transformation matrix."""
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = origin
    return T


def svalboard_key_unit(h: MyoHand, q: np.ndarray, finger: str, bolt_pos: np.ndarray | None = None):
    """Build high-fidelity 3D meshes for one Svalboard 5-way key unit.
    
    True Svalboard / DataHand Anatomical Orientation:
      1. Base Paddle (Center Plunge / Click): at the fingertip / nail tip
      2. Back Paddle (South / Flexor Pull): at the palmar finger pad
      3. Front Paddle (North / Extensor Push): over the dorsal fingernail top
      4. Left Paddle (West / Radial Push): on the radial flank
      5. Right Paddle (East / Ulnar Push): on the ulnar flank
      6. U-Cradle & Sensor Pod: cradling the distal fingertip apex
    """
    wf = h.well_frame(q, finger)
    pos = np.asarray(wf["pos"], float) # pad center on palmar skin
    ax = np.asarray(wf["axis"], float)
    ax /= (np.linalg.norm(ax) + 1e-12)
    fl = np.asarray(wf["floor"], float)
    fl /= (np.linalg.norm(fl) + 1e-12)
    lat = np.asarray(wf["lateral"], float)
    lat /= (np.linalg.norm(lat) + 1e-12)
    
    FINGER_RADII = {
        "thumb": 0.0062,
        "index": 0.0055,
        "middle": 0.0058,
        "ring": 0.0053,
        "little": 0.0046,
    }
    r = FINGER_RADII.get(finger, float(wf.get("radius", 0.0055)))
    half = float(wf.get("half", 0.007))
    
    # Unified keywell cavity center (middle of the distal fingertip enclosure)
    phalanx_center = pos - r * fl
    cavity_center = phalanx_center + 0.25 * half * ax
    
    cavity_depth = 1.1 * half      # longitudinal depth along plunge axis
    cavity_w = 2.0 * r + 0.0016     # compact lateral width (10.8 - 14.0 mm)
    cavity_h = 2.0 * r + 0.0016     # height between front & back walls
    
    # R_well basis:
    # X = lateral (radial -> ulnar)
    # Y = floor (dorsal -> palmar / pad normal)
    # Z = axis (proximal -> distal / nail tip plunge)
    R_well = np.column_stack([lat, fl, ax])
    
    def T_local(local_offset):
        world_pos = cavity_center + R_well @ np.asarray(local_offset, float)
        T = np.eye(4)
        T[:3, :3] = R_well
        T[:3, 3] = world_pos
        return T

    # -------------------------------------------------------------------------
    # 1. SENSOR POD / PCBA ENCLOSURE (mounted to the exterior floor of the well)
    # -------------------------------------------------------------------------
    pod_depth = 0.0045
    pod_len = cavity_h + 0.001
    pod_width = cavity_w + 0.001
    pod_box = trimesh.creation.box(
        extents=[pod_width, pod_len, pod_depth],
        transform=T_local([0.0, 0.0, 0.5 * cavity_depth + 0.001 + 0.5 * pod_depth])
    )
    pod_ear = trimesh.creation.box(
        extents=[pod_width * 0.75, 0.003, 0.0035],
        transform=T_local([0.0, 0.5 * pod_len, 0.5 * cavity_depth + 0.002])
    )
    pod_mesh = trimesh.util.concatenate([pod_box, pod_ear])

    # -------------------------------------------------------------------------
    # 2. U-CHANNEL FINGER CRADLE (Housing & Guide Frame for the 5-Way Well)
    # -------------------------------------------------------------------------
    cradle_base = trimesh.creation.box(
        extents=[cavity_w + 0.0016, cavity_h + 0.0016, 0.0010],
        transform=T_local([0.0, 0.0, 0.5 * cavity_depth + 0.0005])
    )
    cradle_dorsal = trimesh.creation.box(
        extents=[cavity_w + 0.0016, 0.0010, cavity_depth],
        transform=T_local([0.0, -0.5 * cavity_h - 0.0005, 0.0])
    )
    cradle_flank_l = trimesh.creation.box(
        extents=[0.0010, cavity_h + 0.0016, cavity_depth],
        transform=T_local([-0.5 * cavity_w - 0.0005, 0.0, 0.0])
    )
    cradle_flank_r = trimesh.creation.box(
        extents=[0.0010, cavity_h + 0.0016, cavity_depth],
        transform=T_local([0.5 * cavity_w + 0.0005, 0.0, 0.0])
    )
    cradle_mesh = trimesh.util.concatenate([cradle_base, cradle_dorsal, cradle_flank_l, cradle_flank_r])

    # -------------------------------------------------------------------------
    # 3. THE 5 SVALBOARD DIRECTIONAL ACTUATION PADDLES (Unified Enclosure)
    # -------------------------------------------------------------------------
    paddles = {}
    
    # A. CENTER PLUNGE KEY (Base Floor Paddle - bottom of the 4 walls)
    click_cap = trimesh.creation.box(
        extents=[cavity_w * 0.88, cavity_h * 0.88, 0.0012],
        transform=T_local([0.0, 0.0, 0.5 * cavity_depth - 0.0008])
    )
    paddles["click"] = click_cap
    
    # B. SOUTH / BACK PADDLE (Palmar wall - facing finger pad)
    back_paddle = trimesh.creation.box(
        extents=[cavity_w * 0.85, 0.0012, cavity_depth * 0.85],
        transform=T_local([0.0, 0.5 * cavity_h - 0.0008, 0.0])
    )
    paddles["back"] = back_paddle
    
    # C. NORTH / FORWARD PADDLE (Dorsal wall - facing nail top)
    fwd_paddle = trimesh.creation.box(
        extents=[cavity_w * 0.85, 0.0012, cavity_depth * 0.85],
        transform=T_local([0.0, -0.5 * cavity_h + 0.0008, 0.0])
    )
    paddles["forward"] = fwd_paddle
    
    # D. WEST / LEFT PADDLE (Radial flank wall)
    left_paddle = trimesh.creation.box(
        extents=[0.0012, cavity_h * 0.85, cavity_depth * 0.85],
        transform=T_local([-0.5 * cavity_w + 0.0008, 0.0, 0.0])
    )
    paddles["left"] = left_paddle
    
    # E. EAST / RIGHT PADDLE (Ulnar flank wall)
    right_paddle = trimesh.creation.box(
        extents=[0.0012, cavity_h * 0.85, cavity_depth * 0.85],
        transform=T_local([0.5 * cavity_w - 0.0008, 0.0, 0.0])
    )
    paddles["right"] = right_paddle
    
    # -------------------------------------------------------------------------
    # 4. MOUNTING TOWER / STALK (Connecting Deck Bolt to Sensor Pod)
    # -------------------------------------------------------------------------
    tower_mesh = None
    if bolt_pos is not None:
        bolt_pos = np.asarray(bolt_pos, float)
        pod_pos = cavity_center + (0.5 * cavity_depth + 0.002 + 0.5 * pod_depth) * ax
        tower_top = pod_pos + 0.5 * pod_len * fl - 0.2 * pod_depth * ax
        
        base_cyl = trimesh.creation.cylinder(
            radius=0.0045, height=0.003,
            transform=_transform_4x4(np.eye(3), bolt_pos + [0, 0, 0.0015])
        )
        
        diff = tower_top - bolt_pos
        dist = float(np.linalg.norm(diff))
        if dist > 1e-4:
            strut_dir = diff / dist
            z_axis = np.array([0.0, 0.0, 1.0])
            rot_axis = np.cross(z_axis, strut_dir)
            rot_norm = np.linalg.norm(rot_axis)
            if rot_norm < 1e-6:
                R_strut = np.eye(3) if strut_dir[2] > 0 else np.diag([1, -1, -1])
            else:
                rot_axis /= rot_norm
                angle = np.arccos(np.clip(np.dot(z_axis, strut_dir), -1.0, 1.0))
                K = np.array([[0, -rot_axis[2], rot_axis[1]],
                              [rot_axis[2], 0, -rot_axis[0]],
                              [-rot_axis[1], rot_axis[0], 0]])
                R_strut = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)
            
            strut_mid = 0.5 * (bolt_pos + tower_top)
            stalk_cyl = trimesh.creation.cylinder(
                radius=0.0032, height=dist, sections=16,
                transform=_transform_4x4(R_strut, strut_mid)
            )
            gusset = trimesh.creation.box(
                extents=[0.002, 0.006, dist * 0.7],
                transform=_transform_4x4(R_strut, strut_mid)
            )
            tower_mesh = trimesh.util.concatenate([base_cyl, stalk_cyl, gusset])
        else:
            tower_mesh = base_cyl
            
    dirs = {
        "click": ax,       # plunge into base paddle along distal nail tip axis
        "forward": -fl,    # dorsal push over fingernail top
        "back": fl,        # palmar flexor pull near finger pad
        "left": -lat,      # radial flank push
        "right": lat,      # ulnar flank push
    }
    
    return {
        "finger": finger,
        "tower": tower_mesh,
        "pod": pod_mesh,
        "cradle": cradle_mesh,
        "paddles": paddles,
        "center": cavity_center,
        "dirs": dirs,
    }


def build_all_svalboard_units(h: MyoHand, q: np.ndarray, deck_bolts: dict | None = None):
    """Generate all 5 Svalboard 5-way units (thumb, index, middle, ring, little)."""
    units = {}
    for f in FINGERS:
        bolt_pos = deck_bolts.get(f) if deck_bolts is not None else None
        units[f] = svalboard_key_unit(h, q, f, bolt_pos=bolt_pos)
    return units


def svalboard_plotly_traces(units: dict) -> list[go.Mesh3d | go.Scatter3d]:
    """Convert Svalboard key units into structured, beautiful Plotly traces."""
    traces = []
    
    LIGHT_KEY = dict(ambient=0.55, diffuse=0.85, specular=0.45, roughness=0.3, fresnel=0.25)
    LIGHT_METALLIC = dict(ambient=0.45, diffuse=0.8, specular=0.65, roughness=0.2, fresnel=0.3)
    
    for f, u in units.items():
        fc = FINGER_COLORS[f]
        
        # 1. Mounting Tower (Stainless / CF-Bracket)
        if u["tower"] is not None:
            V, F = np.asarray(u["tower"].vertices), np.asarray(u["tower"].faces)
            traces.append(go.Mesh3d(
                x=V[:, 0], y=V[:, 1], z=V[:, 2],
                i=F[:, 0], j=F[:, 1], k=F[:, 2],
                color="#78909c", opacity=1.0, flatshading=False,
                lighting=LIGHT_METALLIC,
                name=f"Svalboard Towers",
                legendgroup="svalboard_towers",
                showlegend=(f == "thumb")
            ))
            
        # 2. Sensor Pod / Electronics Housing
        if u["pod"] is not None:
            V, F = np.asarray(u["pod"].vertices), np.asarray(u["pod"].faces)
            traces.append(go.Mesh3d(
                x=V[:, 0], y=V[:, 1], z=V[:, 2],
                i=F[:, 0], j=F[:, 1], k=F[:, 2],
                color="#263238", opacity=1.0, flatshading=False,
                lighting=LIGHT_KEY,
                name=f"Sensor PCBA Enclosures",
                legendgroup="svalboard_pods",
                showlegend=(f == "thumb")
            ))
            
        # 3. Finger Cradle / Nest
        if u["cradle"] is not None:
            V, F = np.asarray(u["cradle"].vertices), np.asarray(u["cradle"].faces)
            traces.append(go.Mesh3d(
                x=V[:, 0], y=V[:, 1], z=V[:, 2],
                i=F[:, 0], j=F[:, 1], k=F[:, 2],
                color="#455a64", opacity=1.0, flatshading=False,
                lighting=LIGHT_KEY,
                name=f"Finger Well Cradles",
                legendgroup="svalboard_cradles",
                showlegend=(f == "thumb")
            ))
            
        # 4. The 5 Directional Switch Paddles
        for dname, pmesh in u["paddles"].items():
            dcol = DIRECTION_COLORS.get(dname, fc)
            V, F = np.asarray(pmesh.vertices), np.asarray(pmesh.faces)
            traces.append(go.Mesh3d(
                x=V[:, 0], y=V[:, 1], z=V[:, 2],
                i=F[:, 0], j=F[:, 1], k=F[:, 2],
                color=dcol, opacity=1.0, flatshading=False,
                lighting=LIGHT_KEY,
                name=f"5-Way Key: {dname.capitalize()}",
                legendgroup=f"svalboard_paddles_{dname}",
                showlegend=(f == "thumb")
            ))
            
        # 5. Visual 5-Way Stroke Vector Arrows
        center = u["center"]
        for dname, dvec in u["dirs"].items():
            dcol = DIRECTION_COLORS.get(dname, "#fff")
            tip = center + 0.010 * dvec
            traces.append(go.Scatter3d(
                x=[center[0], tip[0]],
                y=[center[1], tip[1]],
                z=[center[2], tip[2]],
                mode="lines+markers",
                line=dict(color=dcol, width=4),
                marker=dict(size=[2, 4], color=dcol),
                name=f"Keypress DOF ({dname})",
                legendgroup="motion_vectors",
                showlegend=(f == "thumb" and dname == "click"),
                hoverinfo="name"
            ))
            
    return traces

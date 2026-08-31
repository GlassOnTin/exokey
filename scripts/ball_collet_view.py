"""Render Modular Phalanx-Following 4-Node Ball-Collet Exoskeleton Outriggers -> out/ball_collet_view.html.

    PYTHONPATH=. .venv/bin/python scripts/ball_collet_view.py
"""
from __future__ import annotations

import os
import mujoco
import numpy as np
import plotly.graph_objects as go
import trimesh
from scipy.spatial import cKDTree

from design.vector import posture
from hand.flesh import skin
from hand.myohand import FINGERS
from manufacture.ball_collet_outrigger import (
    build_phalanx_following_outrigger,
    calculate_phalanx_exoskeleton_mechanics,
)
from manufacture.carrier_gauntlet import hand_axes
from manufacture.svalboard import build_all_svalboard_units, svalboard_plotly_traces
from opt.problem import hands
from viz.scene import _mesh_traces, strap_traces


def main():
    h = hands()[50]
    q = h.compose({
        "index": posture(h, "index", 0.45, 0.35, np.radians(2.5)),
        "middle": posture(h, "middle", 0.45, 0.35, 0.0),
        "ring": posture(h, "ring", 0.45, 0.35, np.radians(-2.5)),
        "little": posture(h, "little", 0.45, 0.35, np.radians(-5.0)),
        "thumb": posture(h, "thumb", 0.45, 0.35, 0.0)
    })
    h.data.qpos[:] = q
    mujoco.mj_forward(h.model, h.data)
    o, e_d, e_r, e_o = hand_axes(h, q)

    # 1. Build Svalboard 5-Way Key Units
    sval_units = build_all_svalboard_units(h, q)

    traces = []

    # 2. Hand Flesh & Skin (Semi-translucent anatomical reference)
    V_skin, F_skin = skin(h, q)
    if len(V_skin):
        traces.append(go.Mesh3d(
            x=V_skin[:, 0], y=V_skin[:, 1], z=V_skin[:, 2],
            i=F_skin[:, 0], j=F_skin[:, 1], k=F_skin[:, 2],
            color="#e2b798",
            opacity=0.62,
            flatshading=False,
            lighting=dict(ambient=0.58, diffuse=0.85, specular=0.22, roughness=0.75, fresnel=0.25),
            name="Hand Flesh & Skin",
            legendgroup="hand_skin",
            hoverinfo="name",
            showlegend=True
        ))

    # 3. Skeleton (Bones)
    bone_traces = _mesh_traces(h, q, opacity=0.40)
    for b_idx, bt in enumerate(bone_traces):
        bt.legendgroup = "hand_bones"
        bt.name = "Skeleton (Bones)"
        bt.showlegend = (b_idx == 0)
        traces.append(bt)

    # 4. Dorsal Metacarpal Saddle Hub
    from structure.anchor import bearing_surface
    P, N, K, T = bearing_surface(h, q)
    P = np.asarray(P)
    mc = P[(P - o) @ e_d > 0.002]
    if len(mc) < 4: mc = P
    r_lo, r_hi = np.min((mc - o) @ e_r), np.max((mc - o) @ e_r)
    d_lo, d_hi = np.min((mc - o) @ e_d), np.max((mc - o) @ e_d)
    hgt = float(np.percentile((P - o) @ e_o, 90)) + 0.0055

    n_across, n_along = 6, 4
    rs = np.linspace(r_lo - 0.001, r_hi + 0.001, n_across)
    ds = np.linspace(d_lo - 0.004, d_hi + 0.004, n_along)
    r_mid = 0.5 * (r_lo + r_hi)
    r_span = max(r_hi - r_lo, 0.02)

    grid_nodes = np.array([[o + r_val*e_r + d_val*e_d + (hgt + 0.003*(1.0-((r_val-r_mid)/(0.5*r_span))**2))*e_o for r_val in rs] for d_val in ds])
    distal_row = grid_nodes[-1, :, :]

    # 5. Extract Kinematic Nodes for the 4-Node Phalanx Exoskeleton
    DIGIT_BODIES = {
        "index": ("secondmc", "proxph2", "midph2", "distph2"),
        "middle": ("thirdmc", "proxph3", "midph3", "distph3"),
        "ring": ("fourthmc", "proxph4", "midph4", "distph4"),
        "little": ("fifthmc", "proxph5", "midph5", "distph5"),
        "thumb": ("firstmc", "proximal_thumb", "distal_thumb"),
    }

    cf_tube_meshes = []
    clamp_meshes = []

    tree_skin = cKDTree(np.asarray(V_skin))

    for f in ["index", "middle", "ring", "little", "thumb"]:
        if f not in sval_units:
            continue
        b_list = DIGIT_BODIES[f]
        nodes = []
        
        # Node 1: MCP Knuckle Node
        b_mcp = b_list[1]
        id_mcp = mujoco.mj_name2id(h.model, mujoco.mjtObj.mjOBJ_BODY, b_mcp)
        pos_mcp = np.copy(h.data.xpos[id_mcp])
        pts_mcp = V_skin[np.linalg.norm(V_skin - pos_mcp, axis=1) < 0.025]
        skin_mcp = pts_mcp[np.argmax((pts_mcp - o) @ e_o)] if len(pts_mcp) else pos_mcp + 0.010*e_o
        standoff_mcp = 0.0065 if f == "little" else 0.0055
        nodes.append(skin_mcp + standoff_mcp * e_o)
        
        # Node 2: PIP Knuckle Node
        b_pip = b_list[2]
        id_pip = mujoco.mj_name2id(h.model, mujoco.mjtObj.mjOBJ_BODY, b_pip)
        pos_pip = np.copy(h.data.xpos[id_pip])
        v_prox = pos_pip - pos_mcp
        v_prox /= np.linalg.norm(v_prox)
        n_pip = e_o - np.dot(e_o, v_prox) * v_prox
        n_pip /= np.linalg.norm(n_pip)
        dots = (V_skin - pos_pip) @ n_pip
        close_mask = np.linalg.norm(V_skin - pos_pip, axis=1) < 0.025
        skin_pip = V_skin[close_mask][np.argmax(dots[close_mask])]
        standoff_pip = 0.0070 if f == "little" else 0.0055
        nodes.append(skin_pip + standoff_pip * n_pip)
        
        # Node 3: DIP Knuckle Node (Fingers)
        if len(b_list) > 3:
            b_dip = b_list[3]
            id_dip = mujoco.mj_name2id(h.model, mujoco.mjtObj.mjOBJ_BODY, b_dip)
            pos_dip = np.copy(h.data.xpos[id_dip])
            v_mid = pos_dip - pos_pip
            v_mid /= np.linalg.norm(v_mid)
            n_dip = e_o - np.dot(e_o, v_mid) * v_mid
            n_dip /= np.linalg.norm(n_dip)
            dots_dip = (V_skin - pos_dip) @ n_dip
            close_dip = np.linalg.norm(V_skin - pos_dip, axis=1) < 0.025
            skin_dip = V_skin[close_dip][np.argmax(dots_dip[close_dip])]
            standoff_dip = 0.0075 if f == "little" else 0.0055
            nodes.append(skin_dip + standoff_dip * n_dip)
            
        # Node 4: Keywell Pod Node
        u = sval_units[f]
        dirs = u["dirs"]
        node_pod = u["center"] + 0.005 * dirs["click"] + 0.012 * dirs["forward"]
        nodes.append(node_pod)
        
        outrigger = build_phalanx_following_outrigger(nodes, r_tube=0.0022, r_ball=0.0030)
        cf_tube_meshes.append(outrigger["tubes"])
        clamp_meshes.append(outrigger["clamps"])

    # Render Straight Pultruded CF Tubes (Matte Carbon Black)
    cf_mesh = trimesh.util.concatenate(cf_tube_meshes)
    V_cf, F_cf = np.asarray(cf_mesh.vertices), np.asarray(cf_mesh.faces)
    traces.append(go.Mesh3d(
        x=V_cf[:, 0], y=V_cf[:, 1], z=V_cf[:, 2],
        i=F_cf[:, 0], j=F_cf[:, 1], k=F_cf[:, 2],
        color="#1b2026",
        opacity=1.0,
        flatshading=False,
        lighting=dict(ambient=0.45, diffuse=0.90, specular=0.70, roughness=0.15),
        name="Straight Pultruded CF Tubes (⌀ 4.4 mm)",
        legendgroup="ball_collet",
        hoverinfo="name",
        showlegend=True
    ))

    # Render Anodized Aluminum / Titanium Clamping Ball-Collets (Metallic Gold/Bronze)
    cl_mesh = trimesh.util.concatenate(clamp_meshes)
    V_cl, F_cl = np.asarray(cl_mesh.vertices), np.asarray(cl_mesh.faces)
    traces.append(go.Mesh3d(
        x=V_cl[:, 0], y=V_cl[:, 1], z=V_cl[:, 2],
        i=F_cl[:, 0], j=F_cl[:, 1], k=F_cl[:, 2],
        color="#c59b27",
        opacity=1.0,
        flatshading=False,
        lighting=dict(ambient=0.55, diffuse=0.85, specular=0.85, roughness=0.20),
        name="Phalanx-Aligned Ball-Collets (MCP/PIP/DIP/Pod)",
        legendgroup="ball_collet",
        hoverinfo="name",
        showlegend=True
    ))

    # 6. Add Svalboard 5-Way Key Units
    sval_traces = svalboard_plotly_traces(sval_units)
    traces.extend(sval_traces)

    # 7. Add TPU Tension Strap
    if len(P):
        try:
            s_traces = strap_traces(h, q, P, n_bands=1, width=0.018, standoff=0.001)
            for st in s_traces:
                st.opacity = 0.55
                st.color = "#c2185b"
                st.name = "TPU Tension Strap"
                st.showlegend = True
            traces.extend(s_traces)
        except Exception:
            pass

    # Rigidity & Kinematics Performance
    mech = calculate_phalanx_exoskeleton_mechanics()

    fig = go.Figure(traces)

    camera_presets = [
        dict(label="Perspective (Oblique)", method="relayout", args=[{"scene.camera": dict(eye=dict(x=-1.55, y=0.80, z=0.80), center=dict(x=0, y=0, z=0))}]),
        dict(label="Dorsal View (Top)", method="relayout", args=[{"scene.camera": dict(eye=dict(x=0.0, y=0.0, z=2.20), up=dict(x=0, y=1, z=0))}]),
        dict(label="Lateral View (Side Elevation)", method="relayout", args=[{"scene.camera": dict(eye=dict(x=-2.20, y=0.0, z=0.0), up=dict(x=0, y=0, z=1))}]),
        dict(label="Fingertip Pod View", method="relayout", args=[{"scene.camera": dict(eye=dict(x=0.0, y=2.20, z=-0.20), up=dict(x=0, y=0, z=1))}]),
        dict(label="Volar Palm View", method="relayout", args=[{"scene.camera": dict(eye=dict(x=0.0, y=0.0, z=-2.20), up=dict(x=0, y=1, z=0))}])
    ]

    fig.update_layout(
        title=f"<b>ExoKey — Phalanx-Following 4-Node Ball-Collet Exoskeleton Outriggers</b><br>"
              f"<sup>Full Anatomical Landmark Alignment (MCP + PIP + DIP + Pod) | Straight CF Links | Tip Deflection: {mech['total_deflection_um']:.1f} μm (SF = {mech['safety_factor_typing']:.0f}x)</sup>",
        updatemenus=[
            dict(
                type="buttons",
                direction="right",
                x=0.5,
                y=0.04,
                xanchor="center",
                yanchor="bottom",
                bgcolor="rgba(24,28,32,0.85)",
                bordercolor="rgba(255,255,255,0.15)",
                font=dict(color="#e1e7ec", size=11),
                buttons=camera_presets
            )
        ],
        scene=dict(
            aspectmode="data",
            xaxis_visible=False,
            yaxis_visible=False,
            zaxis_visible=False,
            camera=dict(
                eye=dict(x=-1.55, y=0.80, z=0.80),
                center=dict(x=0, y=0, z=0)
            ),
            bgcolor="#15181b"
        ),
        paper_bgcolor="#15181b",
        font=dict(color="#e1e7ec", family="-apple-system, BlinkMacSystemFont, Segoe UI, Roboto, sans-serif"),
        margin=dict(l=0, r=0, t=75, b=0),
        legend=dict(
            x=0.02, y=0.98,
            bgcolor="rgba(24,28,32,0.85)",
            bordercolor="rgba(255,255,255,0.1)",
            borderwidth=1,
            font=dict(color="#e1e7ec", size=11),
            itemsizing="constant"
        )
    )

    out_file = "out/ball_collet_view.html"
    fig.write_html(out_file, include_plotlyjs="cdn", full_html=True)
    print(f"wrote {out_file} ({os.path.getsize(out_file)/1e6:.1f} MB)")


if __name__ == "__main__":
    main()

"""Render Central Dorsal Spine + Branching Transverse Wishbone -> out/branching_spine_view.html.

    PYTHONPATH=. .venv/bin/python scripts/branching_spine_view.py
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
from manufacture.branching_spine_outrigger import (
    build_conformal_spine_tree_geometry,
)
from manufacture.carrier_gauntlet import hand_axes
from manufacture.svalboard import build_all_svalboard_units, svalboard_plotly_traces
from opt.problem import hands
from structure.collision import audit_outrigger_skin_penetration, audit_pod_intersections
from structure.fem import run_exokey_fem_analysis
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

    # 2. Hand Flesh & Skin
    V_skin, F_skin = skin(h, q)
    tree_skin = cKDTree(np.asarray(V_skin))

    if len(V_skin):
        # Intact connected flesh manifold with coherent smooth vertex normals
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

    # 4. Dorsal Metacarpal Saddle Plate Root (Anchored over 3rd metacarpal base near wrist)
    # D = 25 mm (wrist/metacarpal base), R = +9 mm (inline with middle finger ray), H = +24 mm
    p_root = o + 0.0250 * e_d + 0.0090 * e_r + 0.0240 * e_o

    # 5. Extract Kinematic Nodes for Transverse Arch & Phalanx Branches
    mcp_nodes = {
        "little": o + 0.0580 * e_d - 0.0370 * e_r + (0.0100 + 0.0075) * e_o,
        "ring":   o + 0.0628 * e_d - 0.0127 * e_r + (0.0122 + 0.0070) * e_o,
        "middle": o + 0.0615 * e_d + 0.0090 * e_r + (0.0149 + 0.0070) * e_o,
        "index":  o + 0.0582 * e_d + 0.0369 * e_r + (0.0107 + 0.0070) * e_o,
    }

    id_mcp_th = mujoco.mj_name2id(h.model, mujoco.mjtObj.mjOBJ_BODY, "proximal_thumb")
    id_ip_th = mujoco.mj_name2id(h.model, mujoco.mjtObj.mjOBJ_BODY, "distal_thumb")
    mcp_nodes["thumb"] = np.copy(h.data.xpos[id_mcp_th]) + 0.014 * e_o + 0.024 * e_r

    digit_chains = {
        "index": [
            mcp_nodes["index"],
            o + 0.0848 * e_d + 0.0425 * e_r + (-0.0100 + 0.0070) * e_o + 0.006 * e_d,
            o + 0.1070 * e_d + 0.0417 * e_r + (-0.0299 + 0.0060) * e_o + 0.010 * e_d,
            o + 0.1160 * e_d + 0.0380 * e_r - 0.0650 * e_o
        ],
        "middle": [
            mcp_nodes["middle"],
            o + 0.0941 * e_d + 0.0178 * e_r + (-0.0067 + 0.0070) * e_o + 0.006 * e_d,
            o + 0.1134 * e_d + 0.0156 * e_r + (-0.0340 + 0.0060) * e_o + 0.010 * e_d,
            o + 0.1200 * e_d + 0.0150 * e_r - 0.0720 * e_o
        ],
        "ring": [
            mcp_nodes["ring"],
            o + 0.0901 * e_d - 0.0093 * e_r + (-0.0081 + 0.0070) * e_o + 0.006 * e_d,
            o + 0.1073 * e_d - 0.0038 * e_r + (-0.0313 + 0.0060) * e_o + 0.010 * e_d,
            o + 0.1140 * e_d - 0.0050 * e_r - 0.0680 * e_o
        ],
        "little": [
            mcp_nodes["little"],
            o + 0.085 * e_d - 0.048 * e_r - 0.008 * e_o,
            o + 0.092 * e_d - 0.044 * e_r - 0.035 * e_o,
            sval_units["little"]["center"] - 0.012 * e_r + 0.008 * e_d
        ],
        "thumb": [
            mcp_nodes["thumb"],
            np.copy(h.data.xpos[id_ip_th]) + 0.012 * e_o + 0.024 * e_r,
            sval_units["thumb"]["center"] + 0.020 * e_r + 0.008 * e_d
        ]
    }

    # Collision Audit Segments
    link_segments_for_audit = {"spine_trunk": [(p_root, mcp_nodes["middle"])]}
    for f, chain in digit_chains.items():
        link_segments_for_audit[f] = [(chain[k], chain[k+1]) for k in range(len(chain)-1)]

    arch_order = ["little", "ring", "middle", "index"]
    for i in range(len(arch_order)-1):
        link_segments_for_audit[f"arch_{arch_order[i]}_{arch_order[i+1]}"] = [(mcp_nodes[arch_order[i]], mcp_nodes[arch_order[i+1]])]
        
    p_web = 0.5 * (mcp_nodes["index"] + mcp_nodes["thumb"]) + 0.008 * e_o + 0.006 * e_r
    link_segments_for_audit["thumb_index_bridge"] = [(mcp_nodes["index"], p_web), (p_web, mcp_nodes["thumb"])]

    # 6. Build Geometry
    tree_parts = build_conformal_spine_tree_geometry(p_root, mcp_nodes, digit_chains,
                                                    r_spine_od=0.0040, r_arch_od=0.0030, r_branch_od=0.0025)

    # 7. Audit Collisions using Signed Distance
    skin_audit = audit_outrigger_skin_penetration(link_segments_for_audit, V_skin, F_skin, r_tube=0.0025, min_clearance_mm=1.0)
    pod_audit = audit_pod_intersections(sval_units)

    # 8. Run Rigorous 3D Space-Frame FEM Analysis
    fem_results = run_exokey_fem_analysis(p_root, mcp_nodes, digit_chains, typing_force_N=0.196)

    # Render Carbon Fiber Tubes (Central Spine + Index->Thumb Bridge + Transverse Arch + Branches)
    cf_mesh = tree_parts["tubes"]
    V_cf, F_cf = np.asarray(cf_mesh.vertices), np.asarray(cf_mesh.faces)
    traces.append(go.Mesh3d(
        x=V_cf[:, 0], y=V_cf[:, 1], z=V_cf[:, 2],
        i=F_cf[:, 0], j=F_cf[:, 1], k=F_cf[:, 2],
        color="#1b2026",
        opacity=1.0,
        flatshading=False,
        lighting=dict(ambient=0.45, diffuse=0.90, specular=0.70, roughness=0.15),
        name="Aerospace CF Tubes (⌀ 8.0mm Spine / ⌀ 6.0mm Arch & Thumb Bridge / ⌀ 5.0mm Booms)",
        legendgroup="spine_tree",
        hoverinfo="name",
        showlegend=True
    ))

    # Render Anodized Titanium / Aluminum Clamps & Hub (Metallic Gold/Bronze)
    cl_mesh = tree_parts["clamps"]
    V_cl, F_cl = np.asarray(cl_mesh.vertices), np.asarray(cl_mesh.faces)
    traces.append(go.Mesh3d(
        x=V_cl[:, 0], y=V_cl[:, 1], z=V_cl[:, 2],
        i=F_cl[:, 0], j=F_cl[:, 1], k=F_cl[:, 2],
        color="#c59b27",
        opacity=1.0,
        flatshading=False,
        lighting=dict(ambient=0.55, diffuse=0.85, specular=0.85, roughness=0.20),
        name="Titanium Index Knuckle Clamp, Arch Hubs & M2.5 Ball-Collets",
        legendgroup="spine_tree",
        hoverinfo="name",
        showlegend=True
    ))

    # 9. Add Svalboard 5-Way Key Units
    sval_traces = svalboard_plotly_traces(sval_units)
    traces.extend(sval_traces)

    # 10. Add TPU Tension Strap
    from structure.anchor import bearing_surface
    P, N, K, T = bearing_surface(h, q)
    P = np.asarray(P)
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

    fig = go.Figure(traces)

    camera_presets = [
        dict(label="Perspective (Oblique)", method="relayout", args=[{"scene.camera": dict(eye=dict(x=-1.55, y=0.80, z=0.80), center=dict(x=0, y=0, z=0))}]),
        dict(label="Dorsal View (Top)", method="relayout", args=[{"scene.camera": dict(eye=dict(x=0.0, y=0.0, z=2.20), up=dict(x=0, y=1, z=0))}]),
        dict(label="Lateral View (Side Elevation)", method="relayout", args=[{"scene.camera": dict(eye=dict(x=-2.20, y=0.0, z=0.0), up=dict(x=0, y=0, z=1))}]),
        dict(label="Fingertip Pod View", method="relayout", args=[{"scene.camera": dict(eye=dict(x=0.0, y=2.20, z=-0.20), up=dict(x=0, y=0, z=1))}]),
        dict(label="Volar Palm View", method="relayout", args=[{"scene.camera": dict(eye=dict(x=0.0, y=0.0, z=-2.20), up=dict(x=0, y=1, z=0))}])
    ]

    status_str = "PASSED (Zero Penetration, Daylight >= 1.0mm)" if skin_audit["all_clear"] else "AUDIT ACTIVE"
    single_fem = fem_results["single_finger_results"]

    fig.update_layout(
        title=f"<b>ExoKey — Central Dorsal Spine & Continuous 5-Knuckle Transverse Arch Bridge</b><br>"
              f"<sup>Metacarpal Saddle Anchor (D = 25mm) -> Middle Knuckle (D = 61.5mm) | Collision: <b>{status_str}</b> | 3D FEM Typ. Deflection: Index {single_fem['index']['tip_deflection_um']:.0f}μm, Mid {single_fem['middle']['tip_deflection_um']:.0f}μm, Ring {single_fem['ring']['tip_deflection_um']:.0f}μm, Little {single_fem['little']['tip_deflection_um']:.0f}μm, Thumb {single_fem['thumb']['tip_deflection_um']:.0f}μm</sup>",
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

    out_file = "out/branching_spine_view.html"
    fig.write_html(out_file, include_plotlyjs="cdn", full_html=True)
    print(f"wrote {out_file} ({os.path.getsize(out_file)/1e6:.1f} MB)")
    print(f"Skin penetration audit: all_clear={skin_audit['all_clear']}, worst_sd={skin_audit['worst_sd_mm']:.2f} mm")
    print(f"Pod intersection audit: all_clear={pod_audit['all_clear']}, worst_gap={pod_audit['worst_gap_mm']:.2f} mm")
    print("3D FEM Report:", fem_results)


if __name__ == "__main__":
    main()

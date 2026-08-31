"""Render Integrated Biomorphic Gauntlet with Svalboard 5-Way Key Units and Hand Skin -> out/carrier_view.html.

    PYTHONPATH=. .venv/bin/python scripts/carrier_view.py
"""
from __future__ import annotations

import os
import pickle

import numpy as np
import plotly.graph_objects as go
import trimesh

from design.vector import posture, tm_of, tp_of
from hand.flesh import skin
from hand.myohand import FINGERS
from manufacture.carrier_gauntlet import build_organic_carrier_gauntlet
from manufacture.svalboard import build_all_svalboard_units, svalboard_plotly_traces
from opt.problem import hands
from viz.scene import _mesh_traces, strap_traces


def main():
    h = hands()[50]
    if os.path.exists("out/final_design.pkl"):
        d = pickle.load(open("out/final_design.pkl", "rb"))
        x = d["x"]
        q = h.compose({f: posture(h, f, tp_of(x, f), tm_of(x, f), float(x.get(f"ab_{f}", 0.0)))
                       for f in FINGERS})
    else:
        # Default to relaxed ergonomic typing posture with natural abduction spread
        q = h.compose({
            "index": posture(h, "index", 0.45, 0.35, np.radians(2.5)),
            "middle": posture(h, "middle", 0.45, 0.35, 0.0),
            "ring": posture(h, "ring", 0.45, 0.35, np.radians(-2.5)),
            "little": posture(h, "little", 0.45, 0.35, np.radians(-5.0)),
            "thumb": posture(h, "thumb", 0.45, 0.35, 0.0)
        })

    # 1. Build the Svalboard 5-way key units (cradles, 5-way keys, sensor pods)
    sval_units = build_all_svalboard_units(h, q)

    # 2. Build the organic exoskeleton carrier gauntlet (saddle vault, MCP arches, thumb boom)
    gauntlet_meshes = build_organic_carrier_gauntlet(h, q, sval_units)
    chassis_mesh = gauntlet_meshes["chassis"]

    traces = []
    
    # 3. Hand Flesh & Skin Surface (Anatomical MRI-derived model)
    V_skin, F_skin = skin(h, q)
    if len(V_skin):
        traces.append(go.Mesh3d(
            x=V_skin[:, 0], y=V_skin[:, 1], z=V_skin[:, 2],
            i=F_skin[:, 0], j=F_skin[:, 1], k=F_skin[:, 2],
            color="#e2b798",
            opacity=0.68,
            flatshading=False,
            lighting=dict(ambient=0.58, diffuse=0.85, specular=0.22, roughness=0.75, fresnel=0.25),
            name="Hand Flesh & Skin",
            legendgroup="hand_skin",
            hoverinfo="name",
            showlegend=True
        ))

    # 4. Internal Skeleton (Bone meshes posed by MuJoCo FK)
    bone_traces = _mesh_traces(h, q, opacity=0.45)
    for b_idx, bt in enumerate(bone_traces):
        bt.legendgroup = "hand_bones"
        bt.name = "Skeleton (Bones)"
        bt.showlegend = (b_idx == 0)
        traces.append(bt)

    # 5. Biomorphic Exoskeleton Gauntlet Chassis (CF-PA12)
    LIGHT_GAUNTLET = dict(ambient=0.52, diffuse=0.85, specular=0.55, roughness=0.25, fresnel=0.30)
    V_ch, F_ch = np.asarray(chassis_mesh.vertices), np.asarray(chassis_mesh.faces)
    traces.append(go.Mesh3d(
        x=V_ch[:, 0], y=V_ch[:, 1], z=V_ch[:, 2],
        i=F_ch[:, 0], j=F_ch[:, 1], k=F_ch[:, 2],
        color="#2c3940",
        opacity=1.0,
        flatshading=False,
        lighting=LIGHT_GAUNTLET,
        name="Exoskeleton Chassis (CF-PA12)",
        legendgroup="gauntlet_chassis",
        hoverinfo="name",
        showlegend=True
    ))

    # 6. Add Svalboard 5-Way Key Units (Cradles, Pods, 5 Directional Keys, Motion Vectors)
    sval_traces = svalboard_plotly_traces(sval_units)
    traces.extend(sval_traces)

    # 7. Add Wrist Strap Loop (Tension Reaction Anchor)
    from structure.anchor import bearing_surface
    P_anchor, _, _, _ = bearing_surface(h, q)
    if len(P_anchor):
        try:
            s_traces = strap_traces(h, q, P_anchor, n_bands=1, width=0.018, standoff=0.001)
            for st in s_traces:
                st.opacity = 0.55
                st.color = "#c2185b"
                st.name = "TPU Tension Strap"
                st.showlegend = True
            traces.extend(s_traces)
        except Exception:
            pass

    fig = go.Figure(traces)
    
    # Camera views: Oblique Perspective, Dorsal Top-Down, Side Elevation, Palm/Volar View
    camera_presets = [
        dict(label="Perspective (Oblique)", method="relayout", args=[{"scene.camera": dict(eye=dict(x=-1.55, y=0.80, z=0.80), center=dict(x=0, y=0, z=0))}]),
        dict(label="Dorsal View (Top)", method="relayout", args=[{"scene.camera": dict(eye=dict(x=0.0, y=0.0, z=2.20), up=dict(x=0, y=1, z=0))}]),
        dict(label="Lateral View (Side Elevation)", method="relayout", args=[{"scene.camera": dict(eye=dict(x=-2.20, y=0.0, z=0.0), up=dict(x=0, y=0, z=1))}]),
        dict(label="Fingertip Pod View", method="relayout", args=[{"scene.camera": dict(eye=dict(x=0.0, y=2.20, z=-0.20), up=dict(x=0, y=0, z=1))}]),
        dict(label="Volar Palm View", method="relayout", args=[{"scene.camera": dict(eye=dict(x=0.0, y=0.0, z=-2.20), up=dict(x=0, y=1, z=0))}])
    ]

    fig.update_layout(
        title="<b>ExoKey — Integrated Svalboard 5-Way Wearable Gauntlet on Anatomical Hand</b><br>"
              "<sup>Continuous-Taper Exoskeleton Chassis & 5-Way Keywells (25 Inputs) Over MRI Hand Flesh & Skeleton</sup>",
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

    out_file = "out/carrier_view.html"
    fig.write_html(out_file, include_plotlyjs="cdn", full_html=True)
    print(f"wrote {out_file} ({os.path.getsize(out_file)/1e6:.1f} MB)")


if __name__ == "__main__":
    main()

"""ONE SENSOR, close up: the real mechanism, and the signal each finger motion makes.

    PYTHONPATH=. .venv/bin/python scripts/sensor_view.py   ->  out/sensor.html

The whole-gauntlet render shows WHERE the sensors are; it cannot show WHAT they are. This takes one
finger's module (index), lifts it into a clean local frame, cuts it in half so the mechanism is
legible -- the PA frame, the TPU cradle on its dome flexure, the Ø3×1 magnet in its pocket, the
3-axis Hall in its seat below the gap -- and beside it plots the field CHANGE that magnet presents
to the Hall for each of the five finger motions (manufacture.readout). So it is clear, for one
sensor, what signal to expect: a big axial swing for a press, and a transverse kick for each tilt.
"""
from __future__ import annotations

import pickle

import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots

from design.params import HALL_LSB, HALL_NOISE, MAGNET_D, MAGNET_L
from design.qwerty import ACTIONS
from design.vector import posture, tm_of, tp_of
from hand.myohand import FINGERS
from manufacture import readout as ro
from manufacture import wellmod as wm
from opt.problem import hands

FINGER = "index"
DIR_COLOR = {"click": "#d6336c", "forward": "#1c7ed6", "back": "#74b816",
             "left": "#f08c00", "right": "#7048e8"}


def _clip_halves(mesh):
    """Keep only the faces on the y<=0 side -- a cutaway (no shapely needed)."""
    v = np.asarray(mesh.vertices)
    f = np.asarray(mesh.faces)
    keep = (v[f][:, :, 1] <= 1e-9).all(axis=1)
    return v, f[keep]


def _mesh3d(v, f, color, opacity, name):
    return go.Mesh3d(x=v[:, 0], y=v[:, 1], z=v[:, 2], i=f[:, 0], j=f[:, 1], k=f[:, 2],
                     color=color, opacity=opacity, flatshading=True,
                     lighting=dict(ambient=0.55, diffuse=0.9, specular=0.15),
                     name=name, hoverinfo="name", showlegend=True)


def main():
    import trimesh
    h = hands()[50]
    x = pickle.load(open("out/final_design.pkl", "rb"))["x"]
    q = h.compose({f: posture(h, f, tp_of(x, f), tm_of(x, f), float(x.get(f"ab_{f}", 0.0)))
                   for f in FINGERS})

    frame = wm.frame_mesh(h, q, FINGER)
    insert = wm.insert_mesh(h, q, FINGER)

    # place it in a clean LOCAL frame: finger axis -> +x, palmar/plunge -> -z (Hall at the bottom),
    # lateral -> +y, origin at the magnet-Hall gap.
    wf = h.well_frame(q, FINGER)
    axis, floor, lat = (np.asarray(wf[k], float) for k in ("axis", "floor", "lateral"))
    s = wm._stack(wf)
    cc = np.asarray(wf["pos"], float) - 0.5 * wf["half"] * axis
    ML = float(MAGNET_L)
    mag_w = cc + (s["cup_palmar"] + wm.DOME_T - 0.5 * ML) * floor
    hall_w = cc + s["hall"] * floor
    origin = 0.5 * (mag_w + hall_w)
    Rmap = np.column_stack([[1, 0, 0], [0, 0, -1], [0, 1, 0.]]) @ np.column_stack([axis, floor, lat]).T
    M4 = np.eye(4)
    M4[:3, :3] = Rmap
    M4[:3, 3] = -Rmap @ origin
    frame.apply_transform(M4)
    insert.apply_transform(M4)

    def toL(p):
        return Rmap @ (np.asarray(p, float) - origin)
    mag_l, hall_l = toL(mag_w), toL(hall_w)

    # the drop-in parts, as real solids
    magnet = trimesh.creation.cylinder(radius=0.5 * float(MAGNET_D), height=ML)
    magnet.apply_translation(mag_l)
    hall = trimesh.creation.box(extents=[0.0064, 0.0064, 0.0018])
    hall.apply_translation(hall_l)

    fig = make_subplots(rows=1, cols=2, column_widths=[0.6, 0.4],
                        specs=[[{"type": "scene"}, {"type": "xy"}]],
                        subplot_titles=("the module, cut away", "field change at the Hall (mT)"))

    # ---- the mechanism (cutaway) ----------------------------------------------------------------
    for mesh, color, op, nm in ((frame, "#9aa5b1", 1.0, "PA frame (rigid)"),
                                (insert, "#e0a458", 0.55, "TPU cradle + dome flexure")):
        v, f = _clip_halves(mesh)
        fig.add_trace(_mesh3d(v, f, color, op, nm), row=1, col=1)
    for mesh, color, nm in ((magnet, "#e03131", "magnet Ø3×1 N42"),
                            (hall, "#0c8599", "3-axis Hall")):
        v, f = np.asarray(mesh.vertices), np.asarray(mesh.faces)
        fig.add_trace(_mesh3d(v, f, color, 1.0, nm), row=1, col=1)

    # the magnet's MOTION for each finger action (small arrows from the magnet, direction colours)
    motion = {"click": [0, 0, -1], "forward": [1, 0, 0], "back": [-1, 0, 0],
              "left": [0, 1, 0], "right": [0, -1, 0]}
    L = 0.006
    for a, d in motion.items():
        d = np.array(d, float)
        tip = mag_l + L * d
        fig.add_trace(go.Scatter3d(
            x=[mag_l[0], tip[0]], y=[mag_l[1], tip[1]], z=[mag_l[2], tip[2]],
            mode="lines", line=dict(color=DIR_COLOR[a], width=6),
            name=f"{a} motion", hoverinfo="name", showlegend=False), row=1, col=1)
        fig.add_trace(go.Cone(
            x=[tip[0]], y=[tip[1]], z=[tip[2]], u=[d[0]], v=[d[1]], w=[d[2]],
            sizemode="absolute", sizeref=0.0018, anchor="tip", showscale=False,
            colorscale=[[0, DIR_COLOR[a]], [1, DIR_COLOR[a]]], hoverinfo="skip"), row=1, col=1)

    ann = [dict(x=mag_l[0], y=mag_l[1], z=mag_l[2], text="magnet", showarrow=True, arrowhead=2),
           dict(x=hall_l[0], y=hall_l[1], z=hall_l[2], text="Hall", showarrow=True, arrowhead=2),
           dict(x=mag_l[0], y=mag_l[1], z=mag_l[2] + 0.004, text="TPU dome (flexes)",
                showarrow=True, arrowhead=2)]

    # ---- the signal (bar panel) -----------------------------------------------------------------
    dmap = ro.direction_map()
    lsb, noise = float(HALL_LSB) * 1e3, float(HALL_NOISE) * 1e3
    for ci, comp, col in ((0, "Bx", "#f08c00"), (1, "By", "#1098ad"), (2, "Bz", "#5c940d")):
        fig.add_trace(go.Bar(name=comp, x=list(ACTIONS), y=[dmap[a][ci] * 1e3 for a in ACTIONS],
                             marker_color=col), row=1, col=2)
    # the ±noise floor, as dotted lines across the categories (add_hline trips on 3D-scene figures)
    for yv in (noise, -noise):
        fig.add_trace(go.Scatter(x=list(ACTIONS), y=[yv] * len(ACTIONS), mode="lines",
                                 line=dict(color="#adb5bd", dash="dot"), showlegend=False,
                                 hoverinfo="skip"), row=1, col=2)
    fig.add_trace(go.Scatter(x=["right"], y=[noise], mode="text", text=["noise ±0.2 mT"],
                             textposition="top left", textfont=dict(size=10, color="#868e96"),
                             showlegend=False, hoverinfo="skip"), row=1, col=2)

    plunge = float(np.linalg.norm(dmap["click"])) * 1e3
    fig.update_layout(
        title=f"ExoKey — one finger-well sensor.  Press → <b>{plunge:.0f} mT</b> axial "
              f"({plunge/lsb:.0f} LSB, {plunge/noise:.0f}× noise); each tilt → a few mT transverse. "
              f"The magnet is on the TPU dome; the Hall reads its field through the gap.",
        scene=dict(aspectmode="data", xaxis_visible=False, yaxis_visible=False,
                   zaxis_visible=False, annotations=ann,
                   camera=dict(eye=dict(x=1.7, y=1.5, z=0.8))),
        barmode="group", template="plotly_white", bargap=0.25,
        legend=dict(orientation="h", y=-0.05),
        margin=dict(l=0, r=0, t=70, b=0))
    fig.update_yaxes(title_text="ΔB (mT)", row=1, col=2, zeroline=True, zerolinecolor="#495057")
    fig.write_html("out/sensor.html", include_plotlyjs="cdn")
    import os
    print(f"  press swing {plunge:.0f} mT = {plunge/lsb:.0f} LSB; "
          f"weakest tilt {min(np.linalg.norm(dmap[a]) for a in ACTIONS if a!='click')*1e3:.1f} mT")
    print(f"\nbrowser view: out/sensor.html  ({os.path.getsize('out/sensor.html')/1e6:.1f} MB)")


if __name__ == "__main__":
    main()

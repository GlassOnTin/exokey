"""THE STRAP, rendered over the gauntlet.  PYTHONPATH=. .venv/bin/python scripts/strap_view.py

Shows the band routed as the convex hull of (skin ∪ device) -- it bulges OVER the structure it holds
down (§8.15h), not under it. The gauntlet is drawn as context; the two pink bands are the straps.
"""
from __future__ import annotations

import pickle

import numpy as np
import plotly.graph_objects as go

from design.vector import posture, tm_of, tp_of
from hand.myohand import FINGERS
from opt.problem import hands
from structure.frame import hand_axes
from viz.scene import skin_trace, strap_traces


def tube_mesh(nodes, bars, live, radii, n=8, color="#9a8f77"):
    """Every live strut as a solid round tube, at its own radius, one bone colour."""
    V, I, J, K = [], [], [], []
    for kk, e in enumerate(live):
        a = np.asarray(nodes[bars[e][0]], float)
        b = np.asarray(nodes[bars[e][1]], float)
        ax = b - a
        L = float(np.linalg.norm(ax))
        if L < 1e-9:
            continue
        ax /= L
        u = np.cross(ax, [0.0, 0.0, 1.0])
        if np.linalg.norm(u) < 1e-6:
            u = np.cross(ax, [0.0, 1.0, 0.0])
        u /= np.linalg.norm(u)
        w = np.cross(ax, u)
        r = float(radii[kk])
        base = len(V)
        for end in (a, b):
            for t in range(n):
                th = 2 * np.pi * t / n
                V.append(end + r * (np.cos(th) * u + np.sin(th) * w))
        for t in range(n):
            t2 = (t + 1) % n
            I += [base + t, base + t]
            J += [base + t2, base + n + t2]
            K += [base + n + t2, base + n + t]
    V = np.array(V)
    return go.Mesh3d(x=V[:, 0], y=V[:, 1], z=V[:, 2], i=I, j=J, k=K, color=color,
                     opacity=1.0, flatshading=True,
                     lighting=dict(ambient=0.55, diffuse=0.6, specular=0.15),
                     name="gauntlet", hoverinfo="skip", showlegend=False)


def main():
    ref = hands((50,))[50]
    x = pickle.load(open("out/final_design.pkl", "rb"))["x"]
    q = ref.compose({f: posture(ref, f, tp_of(x, f), tm_of(x, f), float(x.get(f"ab_{f}", 0.0)))
                     for f in FINGERS})
    z = np.load("out/bone.npz", allow_pickle=True)
    nodes = z["nodes"]
    bars = [tuple(b) for b in z["bars"]]
    live = [int(e) for e in z["live"]]
    device = (nodes, bars, live, z["radii"])
    A = nodes[[int(i) for i in z["anchors"]]]

    traces = [skin_trace(ref, q, opacity=0.22)]
    traces.append(tube_mesh(nodes, bars, live, z["radii"]))     # the gauntlet, as SOLID tubes
    # the strap PULL nodes, and the anchor feet the lugs sit on
    from structure.anchor import under_strap
    pulled = sorted(set(under_strap(ref, q, nodes, [int(i) for i in z["anchors"]])))
    if pulled:
        Pp = nodes[pulled]
        traces.append(go.Scatter3d(x=Pp[:, 0], y=Pp[:, 1], z=Pp[:, 2], mode="markers",
                                   marker=dict(size=7, color="#b03060", symbol="diamond",
                                               line=dict(width=1, color="#fff")),
                                   name="lug / strap pulls here", hoverinfo="name",
                                   showlegend=False))
    # THE STRAPS, over the device (device= is the whole point of this render)
    traces += strap_traces(ref, q, A, device=device)

    _o, e_d, e_r, e_o = hand_axes(ref, q)
    eye = 2.4 * e_r + 0.18 * e_o                   # side-on (pure radial), dorsal up
    fig = go.Figure(traces)
    fig.update_layout(
        title="ExoKey — the STRAP, routed OVER the gauntlet (the convex hull of skin ∪ device).<br>"
              "<sub>Pink bands are the straps; the dark tubes are the gauntlet; diamonds are the "
              "anchor feet the watch-lugs sit on. The band bulges over the structure it holds down.</sub>",
        scene=dict(aspectmode="data", xaxis_visible=False, yaxis_visible=False, zaxis_visible=False,
                   camera=dict(eye=dict(x=eye[0], y=eye[1], z=eye[2]),
                               up=dict(x=e_o[0], y=e_o[1], z=e_o[2]))),
        margin=dict(l=0, r=0, t=60, b=0), template="plotly_white", showlegend=False)
    fig.write_html("out/strap.html", include_plotlyjs="cdn",
                   config={"displayModeBar": False, "responsive": True})
    print("wrote out/strap.html")


if __name__ == "__main__":
    main()

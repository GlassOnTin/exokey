"""GAUNTLET ON THE OUTSIDE OF THE STRAP, rendered.  PYTHONPATH=. .venv/bin/python scripts/onstrap_view.py

The new layering (§8.15j): hand -> soft strap (innermost, the only thing that touches the hand) ->
gauntlet (outermost). The strap is the hull of the LIMB now, hand-hugging; the lattice rides on its
outer face. The hand never meets a hard bit.
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


def tube_mesh(nodes, bars, live, radii, n=8, color="#9a8f77", opacity=1.0):
    V, I, J, K = [], [], [], []
    for kk, e in enumerate(live):
        a = np.asarray(nodes[bars[e][0]], float)
        b = np.asarray(nodes[bars[e][1]], float)
        ax = b - a
        L = float(np.linalg.norm(ax))
        if L < 1e-9:
            continue
        ax /= L
        u = np.cross(ax, [0., 0., 1.])
        if np.linalg.norm(u) < 1e-6:
            u = np.cross(ax, [0., 1., 0.])
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
    return go.Mesh3d(x=V[:, 0], y=V[:, 1], z=V[:, 2], i=I, j=J, k=K, color=color, opacity=opacity,
                     flatshading=True, lighting=dict(ambient=0.55, diffuse=0.6),
                     name="gauntlet (outer)", hoverinfo="skip", showlegend=False)


def main():
    ref = hands()[50]
    x = pickle.load(open("out/final_design.pkl", "rb"))["x"]
    q = ref.compose({f: posture(ref, f, tp_of(x, f), tm_of(x, f), float(x.get(f"ab_{f}", 0.0)))
                     for f in FINGERS})
    z = np.load("out/bone.npz", allow_pickle=True)
    nodes = z["nodes"]
    bars = [tuple(bb) for bb in z["bars"]]
    live = [int(e) for e in z["live"]]
    A = nodes[[int(i) for i in z["anchors"]]]
    o, e_d, e_r, e_o = hand_axes(ref, q)

    traces = [skin_trace(ref, q, opacity=0.16)]
    # the strap: the hull of the LIMB (device=None), hand-hugging, INNERMOST -- against the skin
    traces += strap_traces(ref, q, A, device=None)
    # the gauntlet: OUTERMOST, translucent so the strap shows under it
    traces.append(tube_mesh(nodes, bars, live, z["radii"], opacity=0.45))

    eye = 2.2 * e_r + 0.35 * e_o - 0.15 * e_d          # side-on: see strap inside, gauntlet outside
    fig = go.Figure(traces)
    fig.update_layout(
        title="ExoKey — the gauntlet on the OUTSIDE of the strap (§8.15j). Hand → soft strap (pink, "
              "the only<br><sub>thing that touches the hand) → gauntlet (tan, outermost). The strap "
              "cushions, tethers, and spreads; no hard bit meets the hand, and the gate still holds "
              "(499 µm).</sub>",
        scene=dict(aspectmode="data", xaxis_visible=False, yaxis_visible=False, zaxis_visible=False,
                   camera=dict(eye=dict(x=eye[0], y=eye[1], z=eye[2]),
                               up=dict(x=e_o[0], y=e_o[1], z=e_o[2]))),
        margin=dict(l=0, r=0, t=60, b=0), template="plotly_white", showlegend=False)
    fig.write_html("out/onstrap.html", include_plotlyjs="cdn",
                   config={"displayModeBar": False, "responsive": True})
    print("wrote out/onstrap.html")


if __name__ == "__main__":
    main()

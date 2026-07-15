"""THE SANDWICH, layered.  PYTHONPATH=. .venv/bin/python scripts/sandwich_view.py

The point of the render: the hand does NOT meet tube-ends. It meets a smooth INNER BEARING FACE (the
pad), which spreads the strap preload and any knock over the dorsal skin; the topology-optimised
lattice is the CORE that sits on it. Three layers, skin-side up: skin -> bearing pad -> lattice.
"""
from __future__ import annotations

import pickle

import numpy as np
import plotly.graph_objects as go

from design.vector import posture, tm_of, tp_of
from hand.myohand import FINGERS
from opt.problem import hands
from structure.anchor import bearing_surface
from structure.frame import hand_axes
from viz.scene import skin_trace


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
                     name="lattice core", hoverinfo="skip", showlegend=False)


def main():
    ref = hands()[50]
    x = pickle.load(open("out/final_design.pkl", "rb"))["x"]
    q = ref.compose({f: posture(ref, f, tp_of(x, f), tm_of(x, f), float(x.get(f"ab_{f}", 0.0)))
                     for f in FINGERS})
    z = np.load("out/bone.npz", allow_pickle=True)
    nodes = z["nodes"]
    bars = [tuple(bb) for bb in z["bars"]]
    live = [int(e) for e in z["live"]]
    o, e_d, e_r, e_o = hand_axes(ref, q)

    traces = [skin_trace(ref, q, opacity=0.16)]

    # THE INNER BEARING FACE (the pad): the DORSAL SKIN itself over the bearing region, offset
    # 2 mm proud. Reusing the skin mesh means the pad is as smooth as the hand it copies -- a
    # continuous sheet, not the fragmented shards a raw triangulation of the wrapped patch gave.
    from hand.flesh import skin
    V, F = skin(ref, q)
    Pb, *_ = bearing_surface(ref, q)
    d_b = (Pb - o) @ e_d
    dlo, dhi = d_b.min() - 0.004, d_b.max() + 0.004
    fc = V[F].mean(axis=1)
    fn = np.cross(V[F][:, 1] - V[F][:, 0], V[F][:, 2] - V[F][:, 0])
    fn /= np.linalg.norm(fn, axis=1, keepdims=True) + 1e-12
    keep = ((fn @ e_o) > 0.35) & ((fc - o) @ e_d > dlo) & ((fc - o) @ e_d < dhi)
    Fpad = F[keep]
    Vp = V + 0.002 * e_o                              # the pad sits 2 mm proud of the skin
    traces.append(go.Mesh3d(x=Vp[:, 0], y=Vp[:, 1], z=Vp[:, 2],
                            i=Fpad[:, 0], j=Fpad[:, 1], k=Fpad[:, 2],
                            color="#2b8cbe", opacity=0.95, flatshading=False,
                            lighting=dict(ambient=0.55, diffuse=0.7, specular=0.25),
                            name="bearing pad (inner face — the hand touches THIS)",
                            hoverinfo="name", showlegend=False))

    # THE CORE: the lattice, translucent so the pad shows under it
    traces.append(tube_mesh(nodes, bars, live, z["radii"], opacity=0.5))

    eye = 2.1 * e_r + 0.9 * e_o - 0.2 * e_d       # side-on, dorsal up: see the pad under the core
    fig = go.Figure(traces)
    fig.update_layout(
        title="ExoKey — the SANDWICH, layered. The hand meets a smooth <b>bearing pad</b> (blue), "
              "which spreads<br><sub>the strap preload and any knock over the dorsal skin; the "
              "lattice (tan, translucent) is the CORE that sits on the pad. No tube-end touches "
              "the hand.</sub>",
        scene=dict(aspectmode="data", xaxis_visible=False, yaxis_visible=False, zaxis_visible=False,
                   camera=dict(eye=dict(x=eye[0], y=eye[1], z=eye[2]),
                               up=dict(x=e_o[0], y=e_o[1], z=e_o[2]))),
        margin=dict(l=0, r=0, t=60, b=0), template="plotly_white", showlegend=False)
    fig.write_html("out/sandwich.html", include_plotlyjs="cdn",
                   config={"displayModeBar": False, "responsive": True})
    print("wrote out/sandwich.html")


if __name__ == "__main__":
    main()

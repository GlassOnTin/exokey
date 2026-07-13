"""THE GROWN BONE STRUCTURE.  PYTHONPATH=. .venv/bin/python scripts/lattice_view.py

Every strut is drawn as a tube and coloured by how hard it is working (strain energy per unit
volume). Nothing here was drawn by hand: the shape is whatever survived Wolff's law on a lattice
that filled the whole space the gauntlet was allowed to occupy.
"""
from __future__ import annotations

import pickle

import numpy as np
import plotly.graph_objects as go

from design.vector import posture, tm_of, tp_of
from hand.myohand import FINGERS
from opt.problem import hands
from structure.frame import hand_axes
from structure.lattice import BAR_R, solve
from viz.scene import skin_trace


def tubes(nodes, bars, live, se, r, n=7):
    """Each surviving strut as a round tube, coloured by its strain-energy density."""
    V, F, C = [], [], []
    lo, hi = np.percentile([se[e] for e in live], [5, 95])
    for e in live:
        a, b = nodes[bars[e][0]], nodes[bars[e][1]]
        ax = b - a
        L = np.linalg.norm(ax)
        if L < 1e-6:
            continue
        ax = ax / L
        u = np.cross(ax, [0, 0, 1.0])
        if np.linalg.norm(u) < 1e-6:
            u = np.cross(ax, [0, 1.0, 0])
        u /= np.linalg.norm(u)
        v = np.cross(ax, u)
        base = len(V)
        for k in range(n):
            th = 2 * np.pi * k / n
            d = r * (np.cos(th) * u + np.sin(th) * v)
            V += [a + d, b + d]
        c = float(np.clip((se[e] - lo) / (hi - lo + 1e-30), 0, 1))
        C += [c] * (2 * n)
        for k in range(n):
            p, qq = base + 2 * k, base + 2 * ((k + 1) % n)
            F += [(p, p + 1, qq + 1), (p, qq + 1, qq)]
    V = np.array(V)
    F = np.array(F)
    return go.Mesh3d(x=V[:, 0], y=V[:, 1], z=V[:, 2],
                     i=F[:, 0], j=F[:, 1], k=F[:, 2],
                     intensity=C, colorscale="Inferno", cmin=0, cmax=1,
                     showscale=True, flatshading=True,
                     colorbar=dict(title="load<br>carried", thickness=12, len=0.5, x=0.93),
                     name="bone", hoverinfo="skip")


def main():
    d = pickle.load(open("out/pareto.pkl", "rb"))
    X, Fp = d["X"], np.atleast_2d(d["F"])
    Fn = (Fp - Fp.min(0)) / (Fp.max(0) - Fp.min(0) + 1e-12)
    x = X[int(np.argmin((Fn ** 2).sum(1)))]
    h = hands()[50]
    q = h.compose({f: posture(h, f, tp_of(x, f), tm_of(x, f), x.get(f"ab_{f}", 0.0))
                   for f in FINGERS})

    z = np.load("out/lattice.npz", allow_pickle=True)
    nodes = z["nodes"]
    bars = [tuple(b) for b in z["bars"]]
    live = [int(e) for e in z["live"]]
    btn = {f: int(i) for f, i in zip(z["fingers"], z["buttons"])}
    from structure.lattice import ground
    _n, _b, _btn, loads, anchor_k, anchor_n = ground(h, q)

    w, se, mass, tension = solve(nodes, bars, live, btn, loads, anchor_k, anchor_n)

    traces = []
    sk = skin_trace(h, q, opacity=0.20)
    if sk is not None:
        traces.append(sk)
    traces.append(tubes(nodes, bars, live, se, float(BAR_R)))
    for f, i in btn.items():
        p = nodes[i]
        traces.append(go.Scatter3d(x=[p[0]], y=[p[1]], z=[p[2]], mode="markers",
                                   marker=dict(size=6, color="#00d0ff", symbol="diamond"),
                                   name=f"{f} button", hoverinfo="name", showlegend=False))
    # only the anchor nodes the GROWN structure actually stands on
    used = {i for e in live for i in bars[e]}
    A = nodes[[int(i) for i in z["anchors"] if int(i) in used]]
    if len(A):
        traces.append(go.Scatter3d(x=A[:, 0], y=A[:, 1], z=A[:, 2], mode="markers",
                                   marker=dict(size=4, color="#b03060", symbol="x"),
                                   name="anchor", hoverinfo="name", showlegend=False))

    _o, _e_d, e_r, e_o = hand_axes(h, q)
    eye = 1.55 * e_o + 0.62 * e_r
    fig = go.Figure(traces)
    fig.update_layout(
        title=f"ExoKey — the gauntlet, GROWN ON A FREE-FORM DOMAIN.  "
              f"{int(z['bars0'])} candidate struts → <b>{len(live)}</b> "
              f"({100*(1-len(live)/int(z['bars0'])):.1f}% deleted).  "
              f"{float(z['mass0'])*1000:.0f} g → <b>{mass*1000:.1f} g</b>, "
              f"buttons steady at {w*1e6:.0f} µm (gate 500 µm), strap carries {tension:.2f} N.<br>"
              f"<sub>Nothing was drawn by hand. Bright = carrying load. "
              f"Flesh can only PUSH; the strap supplies the pull.</sub>",
        scene=dict(aspectmode="data", xaxis_visible=False, yaxis_visible=False,
                   zaxis_visible=False,
                   camera=dict(eye=dict(x=eye[0], y=eye[1], z=eye[2]),
                               up=dict(x=e_r[0], y=e_r[1], z=e_r[2]))),
        margin=dict(l=0, r=0, t=70, b=0), template="plotly_white", showlegend=False)
    fig.write_html("out/lattice.html", include_plotlyjs="cdn")
    print(f"  {len(live)} struts, {mass*1000:.1f} g, buttons {w*1e6:.0f} um, "
          f"strap {tension:.2f} N")
    print("\nbrowser view: out/lattice.html")


if __name__ == "__main__":
    main()

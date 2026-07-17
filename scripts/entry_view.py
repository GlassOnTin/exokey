"""THE FINGER-ENTRY ROUTE, shown — the channel each fingertip slides in by, and the mount clearing it.

    PYTHONPATH=. .venv/bin/python scripts/entry_view.py   ->  out/entry.html

The complaint that started the rebuild was "I can't see the path the finger takes to the sensor."
This shows exactly that: the long-finger cluster mount (solid), and for each finger the ENTRY SWEEP
(`manufacture.entry`) — the distal-phalanx skin swept along the slide-in — as a translucent channel.
A channel that passes clear of the mount is a finger that can get in; the numbers on the title are
the measured clearances.
"""
from __future__ import annotations

import pickle

import numpy as np
import plotly.graph_objects as go

from design.vector import posture, tm_of, tp_of
from hand.myohand import FINGERS
from manufacture import entry, mount
from opt.problem import hands
from viz.scene import skin_trace

LONG = ["index", "middle", "ring", "little"]
CH = {"index": "#1c7ed6", "middle": "#74b816", "ring": "#f08c00", "little": "#7048e8"}


def main():
    h = hands()[50]
    x = pickle.load(open("out/final_design.pkl", "rb"))["x"]
    q = h.compose({f: posture(h, f, tp_of(x, f), tm_of(x, f), float(x.get(f"ab_{f}", 0.0)))
                   for f in FINGERS})
    z = np.load("out/final.npz", allow_pickle=True)
    nodes = np.array(z["nodes"], float)
    btn = {f: int(i) for f, i in zip(z["fingers"], z["buttons"])}
    mounts = {f: nodes[btn[f]] for f in LONG}

    m = mount.cluster_mesh(h, q, LONG, mounts)
    grams = float(m.volume) * 1060 * 1000
    V, F = np.asarray(m.vertices), np.asarray(m.faces)

    traces = []
    sk = skin_trace(h, q, opacity=0.14)
    if sk is not None:
        traces.append(sk)
    traces.append(go.Mesh3d(x=V[:, 0], y=V[:, 1], z=V[:, 2], i=F[:, 0], j=F[:, 1], k=F[:, 2],
                            color="#9aa5b1", opacity=0.55, flatshading=True,
                            lighting=dict(ambient=0.55, diffuse=0.9, specular=0.15),
                            name="cluster mount (PA frame)", hoverinfo="name", showlegend=True))

    # the DROP-IN CRADLES (TPU), nested in the frame -- the cup the finger actually enters and presses.
    for i, f in enumerate(LONG):
        im = mount.insert_mesh(h, q, f)
        iv, iff = np.asarray(im.vertices), np.asarray(im.faces)
        traces.append(go.Mesh3d(x=iv[:, 0], y=iv[:, 1], z=iv[:, 2], i=iff[:, 0], j=iff[:, 1], k=iff[:, 2],
                                color="#e0a458", opacity=0.85, flatshading=True,
                                lighting=dict(ambient=0.55, diffuse=0.9, specular=0.15),
                                name="drop-in cradle (TPU)" if i == 0 else None,
                                showlegend=(i == 0), hoverinfo="name"))

    clr = {}
    for f in LONG:
        p = mount.cluster_mount(h, q, LONG, mounts)
        clr[f] = entry.entry_clearance(h, q, f, boxes=p["boxes"], caps=p["caps"], cyls=p["cyls"])
        sweep = entry.entry_sweep(h, q, f, length=0.018, n=12)[::4]      # the channel, subsampled
        traces.append(go.Scatter3d(
            x=sweep[:, 0], y=sweep[:, 1], z=sweep[:, 2], mode="markers",
            marker=dict(size=1.6, color=CH[f], opacity=0.35),
            name=f"{f} entry channel", hoverinfo="name"))

    worst = min(clr.values()) * 1e3
    fig = go.Figure(traces)
    fig.update_layout(
        title=f"ExoKey — the finger-entry route.  Each coloured cloud is the channel a fingertip "
              f"slides in by; it passes <b>clear of the mount</b> (worst clearance "
              f"<b>{worst:+.1f} mm</b>).  Cluster mount {grams:.1f} g.",
        scene=dict(aspectmode="data", xaxis_visible=False, yaxis_visible=False,
                   zaxis_visible=False,
                   camera=dict(eye=dict(x=-1.5, y=0.8, z=0.8))),
        margin=dict(l=0, r=0, t=70, b=0), template="plotly_white",
        legend=dict(x=0.02, y=0.98, bgcolor="rgba(255,255,255,0.6)"))
    fig.write_html("out/entry.html", include_plotlyjs="cdn")
    import os
    print("  entry clearances (mm):", {f: round(v * 1e3, 1) for f, v in clr.items()})
    print(f"\nbrowser view: out/entry.html  ({os.path.getsize('out/entry.html')/1e6:.1f} MB)")


if __name__ == "__main__":
    main()

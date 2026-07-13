"""What the gauntlet grew into. The domain in ghost, the surviving structure solid.

    PYTHONPATH=. .venv/bin/python scripts/gauntlet_view.py   ->  out/gauntlet.html

The user: "the solution will begin looking natural and like an alien bone structure when we get
into the full structural optimisation."

That is Wolff's law. Topology optimisation lays material along the STRESS TRAJECTORIES and
deletes everything else -- and bone remodels along the lines of principal stress. Same problem,
same shapes. It does not look designed; it looks grown.
"""
from __future__ import annotations

import numpy as np
import plotly.graph_objects as go

from design.vector import posture, tm_of, tp_of
from hand.myohand import FINGERS
from opt.problem import hands
from viz.scene import _mesh_traces, skin_trace


def tri(nodes, quads, sel, colour, opacity, name):
    V, F = [], []
    for qd in [quads[e] for e in sel]:
        n = len(V)
        V += [nodes[k] for k in qd]
        F += [(n, n + 1, n + 2), (n, n + 2, n + 3)]
    if not V:
        return None
    V, F = np.array(V), np.array(F)
    return go.Mesh3d(x=V[:, 0], y=V[:, 1], z=V[:, 2], i=F[:, 0], j=F[:, 1], k=F[:, 2],
                     color=colour, opacity=opacity, flatshading=True,
                     lighting=dict(ambient=0.45, diffuse=0.95, specular=0.3),
                     name=name, hoverinfo="name", showlegend=False)


def main():
    import pickle

    d = np.load("out/gauntlet.npz")
    nodes, quads = d["nodes"], [tuple(q) for q in d["quads"]]
    live = set(int(e) for e in d["live"])
    dead = [e for e in range(len(quads)) if e not in live]

    with open("out/pareto.pkl", "rb") as fh:
        P = pickle.load(fh)
    X, F = P["X"], np.atleast_2d(P["F"])
    Fn = (F - F.min(0)) / (F.max(0) - F.min(0) + 1e-12)
    x = X[int(np.argmin((Fn**2).sum(1)))]
    h = hands()[50]
    q = h.compose({f: posture(h, f, tp_of(x, f), tm_of(x, f), x.get(f"ab_{f}", 0.0))
                   for f in FINGERS})

    # THE SKIN, not the skeleton. Until now every render drew bones and capsules -- which is
    # why every geometry error had to be caught by a number instead of by eye.
    traces = []
    sk = skin_trace(h, q, opacity=0.22)
    if sk is not None:
        traces.append(sk)
    traces += _mesh_traces(h, q, opacity=0.045)      # the bones, faint, underneath
    t = tri(nodes, quads, dead, "#b9c2c9", 0.05, "deleted (the domain)")
    if t is not None:
        traces.append(t)
    t = tri(nodes, quads, sorted(live), "#12639b", 1.0, "the gauntlet")
    if t is not None:
        traces.append(t)
    for i in d["wells"]:
        p = nodes[int(i)]
        traces.append(go.Scatter3d(x=[p[0]], y=[p[1]], z=[p[2]], mode="markers",
                                   marker=dict(size=7, color="#111", symbol="diamond"),
                                   name="button", hoverinfo="name", showlegend=False))
    for i in d["strap"]:
        p = nodes[int(i)]
        traces.append(go.Scatter3d(x=[p[0]], y=[p[1]], z=[p[2]], mode="markers",
                                   marker=dict(size=4, color="#b03060", symbol="x"),
                                   name="strap", hoverinfo="name", showlegend=False))

    fig = go.Figure(traces)
    fig.update_layout(
        title=f"ExoKey — the gauntlet, grown.  "
              f"{float(d['mass0'])*1000:.1f} g solid → <b>{float(d['mass'])*1000:.1f} g</b> "
              f"({100*(1-float(d['mass'])/float(d['mass0'])):.0f}% removed), "
              f"buttons steady at {float(d['defl'])*1e6:.0f} µm (gate 500 µm).  "
              f"Grey = what it was allowed to be; blue = what carries load.",
        scene=dict(aspectmode="data", xaxis_visible=False,
                   yaxis_visible=False, zaxis_visible=False, camera=dict(eye=dict(x=-1.65, y=0.66, z=0.67), up=dict(x=-0.24, y=-0.06, z=0.97),
                               center=dict(x=0, y=0, z=0))),
        margin=dict(l=0, r=0, t=60, b=0), template="plotly_white", showlegend=False)
    fig.write_html("out/gauntlet.html", include_plotlyjs="cdn")
    print(f"  solid {float(d['mass0'])*1000:.1f} g -> grown {float(d['mass'])*1000:.1f} g "
          f"({100*(1-float(d['mass'])/float(d['mass0'])):.0f}% removed)")
    print(f"  buttons {float(d['defl'])*1e6:.0f} um (gate 500)")
    print("\nbrowser view: out/gauntlet.html")


if __name__ == "__main__":
    main()

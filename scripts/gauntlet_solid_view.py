"""The PRINTABLE SOLID -- the marching-cubes STL, sensor mounts and all, on the hand.

    PYTHONPATH=. .venv/bin/python scripts/gauntlet_solid_view.py   ->  out/gauntlet_solid.html

The skeleton views (`gauntlet_view.py`, the stage renders) draw the wire structure from the `.npz`
files. The sensor MODULES exist only in the STL export path (`manufacture/wellmod` -> `export_stl`),
so no skeleton view shows them. This one reads `out/gauntlet.stl` back and renders the actual solid
you would print -- the finger-well frames, the Hall seats, the wire grooves and the wrist housing --
ghosted over the hand for scale. Run `scripts/export_stl.py` first to (re)build the STL.
"""
from __future__ import annotations

import os
import pickle

import numpy as np
import plotly.graph_objects as go

from design.vector import posture, tm_of, tp_of
from hand.myohand import FINGERS
from opt.problem import hands
from structure.frame import MATERIALS
from viz.scene import _mesh_traces, skin_trace

STL = "out/gauntlet.stl"


def main():
    if not os.path.exists(STL):
        raise SystemExit(f"no {STL} -- run: PYTHONPATH=. .venv/bin/python scripts/export_stl.py")

    import trimesh
    m = trimesh.load(STL, process=False)
    V, F = np.asarray(m.vertices), np.asarray(m.faces)
    grams = float(m.volume) * MATERIALS["cf_pa12"]["rho"] * 1000

    # the hand, in the same world frame the STL was built in, for scale and placement.
    traces = []
    if os.path.exists("out/final_design.pkl"):
        h = hands()[50]
        x = pickle.load(open("out/final_design.pkl", "rb"))["x"]
        q = h.compose({f: posture(h, f, tp_of(x, f), tm_of(x, f), float(x.get(f"ab_{f}", 0.0)))
                       for f in FINGERS})
        sk = skin_trace(h, q, opacity=0.16)
        if sk is not None:
            traces.append(sk)
        traces += _mesh_traces(h, q, opacity=0.04)          # bones, faint, underneath

    traces.append(go.Mesh3d(
        x=V[:, 0], y=V[:, 1], z=V[:, 2], i=F[:, 0], j=F[:, 1], k=F[:, 2],
        color="#12639b", opacity=1.0, flatshading=True,
        lighting=dict(ambient=0.45, diffuse=0.95, specular=0.3),
        name="printed solid", hoverinfo="name", showlegend=False))

    fig = go.Figure(traces)
    fig.update_layout(
        title=f"ExoKey — the printable solid, sensor mounts included.  "
              f"<b>{grams:.1f} g</b> CF-PA12, watertight — the finger-well frames, Hall seats, "
              f"wire grooves and wrist housing are part of the one print.",
        scene=dict(aspectmode="data", xaxis_visible=False, yaxis_visible=False,
                   zaxis_visible=False,
                   camera=dict(eye=dict(x=-1.65, y=0.66, z=0.67),
                               up=dict(x=-0.24, y=-0.06, z=0.97),
                               center=dict(x=0, y=0, z=0))),
        margin=dict(l=0, r=0, t=60, b=0), template="plotly_white", showlegend=False)
    fig.write_html("out/gauntlet_solid.html", include_plotlyjs="cdn")
    print(f"  {len(F)} faces, {grams:.1f} g CF-PA12 (watertightness is verified in export_stl)")
    print("\nbrowser view: out/gauntlet_solid.html")


if __name__ == "__main__":
    main()

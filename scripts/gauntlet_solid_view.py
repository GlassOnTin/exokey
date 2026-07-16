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

from design.params import MAGNET_L
from design.vector import posture, tm_of, tp_of
from hand.myohand import FINGERS
from manufacture import wellmod as wm
from opt.problem import hands
from structure.frame import MATERIALS
from viz.scene import _mesh_traces, skin_trace

STL = "out/gauntlet.stl"
CELL = 0.0010          # decimation cell (m). The full 354k-face STL makes a 52 MB web page;
                       # clustering vertices to a 1 mm grid keeps the shape at ~1/5 the faces.


def _decimate(V, F, cell):
    """Vertex-clustering decimation (no dep): snap vertices to a `cell` grid, merge each cluster to
    its centroid, drop faces that collapse. Crude but plenty for a shape overview, and it keeps the
    bounding box exact. ponytail: a few lines of numpy instead of a fast_simplification dependency."""
    key = np.floor((V - V.min(0)) / cell).astype(np.int64)
    _, inv = np.unique(key, axis=0, return_inverse=True)
    inv = np.asarray(inv).ravel()
    n = int(inv.max()) + 1
    nV = np.zeros((n, 3))
    cnt = np.zeros(n)
    np.add.at(nV, inv, V)
    np.add.at(cnt, inv, 1)
    nV /= cnt[:, None]
    nF = inv[F]
    good = (nF[:, 0] != nF[:, 1]) & (nF[:, 1] != nF[:, 2]) & (nF[:, 0] != nF[:, 2])
    return nV, nF[good]


def main():
    if not os.path.exists(STL):
        raise SystemExit(f"no {STL} -- run: PYTHONPATH=. .venv/bin/python scripts/export_stl.py")

    import trimesh
    m = trimesh.load(STL, process=True)          # merge the STL's per-triangle verts before decimating
    grams = float(m.volume) * MATERIALS["cf_pa12"]["rho"] * 1000
    Vd, Fd = _decimate(np.asarray(m.vertices), np.asarray(m.faces), CELL)
    # ⚠ clustering leaves the face winding inconsistent, so flat-shading normals point every which
    # way and the part renders BLACK. Rebuild through trimesh and fix the winding -> clean shading.
    md = trimesh.Trimesh(Vd, Fd, process=True)
    md.fix_normals()
    V, F = np.asarray(md.vertices), np.asarray(md.faces)

    # the hand, in the same world frame the STL was built in, for scale -- and the magnet/Hall
    # positions, which live only in the module geometry (manufacture.wellmod), not in the STL.
    traces = []
    mag, hall = [], []
    if os.path.exists("out/final_design.pkl"):
        h = hands()[50]
        x = pickle.load(open("out/final_design.pkl", "rb"))["x"]
        q = h.compose({f: posture(h, f, tp_of(x, f), tm_of(x, f), float(x.get(f"ab_{f}", 0.0)))
                       for f in FINGERS})
        sk = skin_trace(h, q, opacity=0.16)
        if sk is not None:
            traces.append(sk)
        traces += _mesh_traces(h, q, opacity=0.04)          # bones, faint, underneath
        for f in FINGERS:
            wf = h.well_frame(q, f)
            fl, ax = np.asarray(wf["floor"]), np.asarray(wf["axis"])
            cc = np.asarray(wf["pos"]) - 0.5 * wf["half"] * ax
            s = wm._stack(wf)
            mag.append(cc + (s["cup_palmar"] + wm.DOME_T - 0.5 * float(MAGNET_L)) * fl)
            hall.append(cc + s["hall"] * fl)

    # THE SOLID, translucent so the drop-in parts show through it, light so the shading reads.
    traces.append(go.Mesh3d(
        x=V[:, 0], y=V[:, 1], z=V[:, 2], i=F[:, 0], j=F[:, 1], k=F[:, 2],
        color="#c7b89a", opacity=0.5, flatshading=True,
        lighting=dict(ambient=0.55, diffuse=0.9, specular=0.15),
        name="printed solid (CF-PA12)", hoverinfo="name", showlegend=True))

    # THE MAGNETS and HALL SENSORS, in contrasting colours -- the drop-in parts, where they sit.
    if mag:
        mag, hall = np.array(mag), np.array(hall)
        traces.append(go.Scatter3d(
            x=mag[:, 0], y=mag[:, 1], z=mag[:, 2], mode="markers",
            marker=dict(size=6, color="#e8590c", symbol="diamond", line=dict(width=1, color="#7a2e06")),
            name="magnet (Ø3×1 N42)", hoverinfo="name"))
        traces.append(go.Scatter3d(
            x=hall[:, 0], y=hall[:, 1], z=hall[:, 2], mode="markers",
            marker=dict(size=6, color="#1098ad", symbol="square", line=dict(width=1, color="#0b525f")),
            name="Hall sensor", hoverinfo="name"))

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
        margin=dict(l=0, r=0, t=60, b=0), template="plotly_white",
        showlegend=True, legend=dict(x=0.02, y=0.98, bgcolor="rgba(255,255,255,0.6)"))
    fig.write_html("out/gauntlet_solid.html", include_plotlyjs="cdn")
    sz = os.path.getsize("out/gauntlet_solid.html") / 1e6
    print(f"  {len(F)} faces after {CELL*1e3:.1f} mm decimation, {grams:.1f} g CF-PA12 "
          f"(mass + watertightness are from the full STL, verified in export_stl)")
    print(f"\nbrowser view: out/gauntlet_solid.html  ({sz:.1f} MB)")


if __name__ == "__main__":
    main()

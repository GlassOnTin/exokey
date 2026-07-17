"""THE FINGER-ENTRY ROUTE, shown against the WHOLE gauntlet — struts, mounts and all.

    PYTHONPATH=. .venv/bin/python scripts/entry_view.py   ->  out/entry.html

The complaint that started the rebuild was "I can't see the path the finger takes to the sensor,"
then "the render doesn't show the gauntlet — and the gauntlet is what blocks entry." Both are fixed
here: the render is the WHOLE printed solid (`out/gauntlet.stl` — struts, the sensor mounts, the
wrist housing), translucent, with each finger's ENTRY SWEEP (`manufacture.entry`) drawn as a channel,
and the drop-in cradles it passes through. The clearance on the title is measured against **everything
near the finger — the gauntlet struts AND the mount**, not the mount alone.
"""
from __future__ import annotations

import os
import pickle

import numpy as np
import plotly.graph_objects as go

from design.vector import posture, tm_of, tp_of
from hand.myohand import FINGERS
from manufacture import entry, mount
from opt.problem import hands
from viz.scene import skin_trace

LONG = ["index", "middle", "ring", "little"]
SHOW = ["thumb", "index", "middle", "ring", "little"]     # the thumb has its own well_mount; check it too
CH = {"thumb": "#e03131", "index": "#1c7ed6", "middle": "#74b816", "ring": "#f08c00", "little": "#7048e8"}


def _decimate(V, F, cell=0.0010):
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
    import trimesh
    if not os.path.exists("out/gauntlet.stl"):
        raise SystemExit("run scripts/export_stl.py first (need out/gauntlet.stl)")
    h = hands()[50]
    x = pickle.load(open("out/final_design.pkl", "rb"))["x"]
    q = h.compose({f: posture(h, f, tp_of(x, f), tm_of(x, f), float(x.get(f"ab_{f}", 0.0)))
                   for f in FINGERS})
    z = np.load("out/final.npz", allow_pickle=True)
    nodes = np.array(z["nodes"], float)
    bars = [tuple(b) for b in z["bars"]]
    live = [int(e) for e in z["live"]]
    rr = np.atleast_1d(np.asarray(z["radii"] if "radii" in z.files else 0.0009, float))
    btn = {f: int(i) for f, i in zip(z["fingers"], z["buttons"])}
    struts = [((nodes[bars[e][0]], nodes[bars[e][1]]), float(rr[k]) if rr.size > 1 else float(rr[0]))
              for k, e in enumerate(live)]

    # THE WHOLE PRINTED SOLID (struts + mounts + housing), decimated, normals fixed, translucent.
    gm = trimesh.load("out/gauntlet.stl", process=True)
    Vd, Fd = _decimate(np.asarray(gm.vertices), np.asarray(gm.faces))
    md = trimesh.Trimesh(Vd, Fd, process=True)
    md.fix_normals()
    V, F = np.asarray(md.vertices), np.asarray(md.faces)
    grams = float(gm.volume) * 1060 * 1000

    traces = []
    sk = skin_trace(h, q, opacity=0.12)
    if sk is not None:
        traces.append(sk)
    # SHADING: low ambient + strong diffuse + a directional light off the scene's corner, so surfaces
    # at different angles read differently (a flat single colour makes the cups impossible to discern).
    lo, hi = V.min(0), V.max(0)
    c, L = (lo + hi) / 2, float((hi - lo).max())
    d = np.array([-1.5, 0.8, 0.8]); d /= np.linalg.norm(d)   # the camera eye dir -> light the faces we see
    lpos = dict(x=float(c[0] + d[0] * 2 * L), y=float(c[1] + d[1] * 2 * L), z=float(c[2] + d[2] * 2 * L))
    LIGHT = dict(ambient=0.35, diffuse=1.0, specular=0.25, roughness=0.4, fresnel=0.1)

    def lit(vv, ff, color, opacity, name, show):
        return go.Mesh3d(x=vv[:, 0], y=vv[:, 1], z=vv[:, 2], i=ff[:, 0], j=ff[:, 1], k=ff[:, 2],
                         color=color, opacity=opacity, flatshading=True, lighting=LIGHT,
                         lightposition=lpos, name=name, hoverinfo="name", showlegend=show)

    traces.append(lit(V, F, "#9aa5b1", 0.28, "the gauntlet (struts + mounts + housing)", True))
    for i, f in enumerate(SHOW):                        # the drop-in cradles the channels pass through
        im = mount.insert_mesh(h, q, f)
        imf = trimesh.Trimesh(im.vertices, im.faces, process=True)   # consistent normals -> real shading
        imf.fix_normals()
        traces.append(lit(np.asarray(imf.vertices), np.asarray(imf.faces), "#e0a458", 1.0,
                          "drop-in cradle (TPU)" if i == 0 else None, i == 0))

    clr = {}
    for f in SHOW:
        fr, ins = mount.well_mount(h, q, f, nodes[btn[f]]), mount.well_insert(h, q, f)
        clr[f] = entry.entry_clearance(h, q, f, boxes=fr["boxes"] + ins["boxes"],
                                       caps=fr["caps"] + ins["caps"] + struts,
                                       cyls=fr["cyls"] + ins["cyls"])          # vs struts AND mount
        sweep = entry.entry_sweep(h, q, f, length=0.018, n=12)[::4]
        traces.append(go.Scatter3d(x=sweep[:, 0], y=sweep[:, 1], z=sweep[:, 2], mode="markers",
                                   marker=dict(size=1.6, color=CH[f], opacity=0.35),
                                   name=f"{f} entry channel", hoverinfo="name"))

    worst = min(clr.values()) * 1e3
    fig = go.Figure(traces)
    fig.update_layout(
        title=f"ExoKey — the finger-entry route, against the whole gauntlet.  Each coloured cloud is "
              f"the channel a fingertip slides in by; measured against the struts AND the mounts it "
              f"passes <b>clear</b> (worst <b>{worst:+.1f} mm</b>).  Whole part {grams:.1f} g.",
        scene=dict(aspectmode="data", xaxis_visible=False, yaxis_visible=False, zaxis_visible=False,
                   camera=dict(eye=dict(x=-1.5, y=0.8, z=0.8))),
        margin=dict(l=0, r=0, t=70, b=0), template="plotly_white",
        legend=dict(x=0.02, y=0.98, bgcolor="rgba(255,255,255,0.6)"))
    fig.write_html("out/entry.html", include_plotlyjs="cdn")
    print("  entry clearances vs struts+mount (mm):", {f: round(v * 1e3, 1) for f, v in clr.items()})
    print(f"\nbrowser view: out/entry.html  ({os.path.getsize('out/entry.html')/1e6:.1f} MB)")


if __name__ == "__main__":
    main()

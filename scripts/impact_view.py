"""THE IMPACT-AWARE GAUNTLET, drawn at its real thicknesses.

    PYTHONPATH=. .venv/bin/python scripts/impact_view.py

The structure grown WITH a 50 N knock in the load set and co-sized for the gate AND the knock
(`scripts/impact_opt.py` -> out/impact_opt.npz). Set it beside the keypress bone (out/sized.html) and
the difference is the whole finding: this one is BROAD and REDUNDANT, many members sharing the blow,
because a localised knock wants that -- not a few fat members. Each strut is drawn at the radius the
combined gate+impact sizing chose for it, and coloured by that radius: the bright, thick members are
the knock's load path.
"""
from __future__ import annotations

import pickle

import numpy as np
import plotly.graph_objects as go

from design.vector import action_dirs, evaluate, posture, tm_of, tp_of
from hand.myohand import FINGERS
from opt.problem import hands
from structure.frame import hand_axes
from structure.spline import TENSION, curves
from viz.scene import skin_trace, strap_traces, well_traces

from scripts.lattice_view import tubes
from scripts.sized_view import _mobile

SMOOTH = True     # spline the load paths (curves()), the smoothing the keypress bone got and this
                  # render first skipped -- so a grid-staircased path is drawn as the curve it is.


def main():
    H = hands()
    h = H[50]
    fd = pickle.load(open("out/final_design.pkl", "rb"))
    x = fd["x"]
    evaluate(x, H)
    q = h.compose({f: posture(h, f, tp_of(x, f), tm_of(x, f), float(x.get(f"ab_{f}", 0.0)))
                   for f in FINGERS})

    z = np.load("out/impact_opt.npz", allow_pickle=True)
    nodes = z["nodes"]
    bars = [tuple(b) for b in z["bars"]]         # already the live bars, in order
    live = list(range(len(bars)))
    radii = z["radii"]                           # POSITIONAL: radii[k] <-> bars[k]
    mass, w_um, knock = float(z["mass"]), float(z["button_um"]), float(z["knock_mpa"])   # grams, µm, MPa
    m_bolt = float(z["mass_bolton"])                                                      # grams

    # colour by radius rank -- the sizer put the metal where the knock needs it, so thick == the
    # blow's load path. (tubes() takes a {bar: value} map and colours by its percentile.)
    se = {e: float(radii[e]) for e in live}

    # SMOOTH: draw each member as its cubic-spline load path, not a grid staircase. curves() keeps the
    # node indices (so the FEA nodes -- the verified structure -- are unchanged; only the DRAWN path
    # curves), and owner[j] maps sub-beam j back to its member so radius and colour carry through. A
    # dense lattice branches a lot (branches stay straight); the continuing trunks are what curve.
    if SMOOTH:
        cn, cb, owner = curves(nodes, bars, live, tension=float(TENSION) * 0.3)
        clive = list(range(len(cb)))
        crad = radii[owner]
        cse = {j: float(radii[owner[j]]) for j in clive}
        tn, tb, tl, ts, tr, sides = cn, cb, clive, cse, crad, 5
    else:
        tn, tb, tl, ts, tr, sides = nodes, bars, live, se, radii, 7

    traces = []
    sk = skin_trace(h, q, opacity=0.16)
    if sk is not None:
        traces.append(sk)
    traces.append(tubes(tn, tb, tl, ts, tr, n=sides))

    cups = []
    for f in FINGERS:
        wf = h.well_frame(q, f)
        cups.append(dict(pos=wf["pos"], axis=wf["axis"], floor=wf["floor"],
                         lateral=wf["lateral"], half=wf["half"], radius=wf["radius"],
                         finger=f, label=f"{f} well", dirs=dict(action_dirs(h, q, f))))
    traces += well_traces(cups)

    used = {i for e in live for i in bars[e]}
    A = nodes[[int(i) for i in z["anchors"] if int(i) in used]]
    if len(A):
        traces += strap_traces(h, q, A)

    _o, e_d, e_r, e_o = hand_axes(h, q)
    eye = 1.65 * e_o - 0.88 * e_d + 0.22 * e_r
    fig = go.Figure(traces)
    fig.update_layout(
        title=f"ExoKey — the IMPACT-AWARE gauntlet.  <b>{len(live)}</b> struts, <b>{mass:.1f} g</b>, "
              f"survives a 50 N knock at {knock:.0f} MPa, keys {w_um:.0f} µm (gate 500).<br>"
              f"<sub>Grown WITH the knock and co-sized for gate + impact: broad and load-sharing, "
              f"{100*(1-mass/m_bolt):.0f}% lighter than thickening the keypress bone ({m_bolt:.1f} g). "
              f"Radii {radii.min()*1e3:.2f}–{radii.max()*1e3:.2f} mm; bright = the knock's load path.</sub>",
        scene=dict(aspectmode="data", xaxis_visible=False, yaxis_visible=False, zaxis_visible=False,
                   camera=dict(eye=dict(x=eye[0], y=eye[1], z=eye[2]),
                               up=dict(x=e_d[0], y=e_d[1], z=e_d[2]))),
        margin=dict(l=0, r=0, t=70, b=0), template="plotly_white", showlegend=False)
    # tubes() labels its scale "load carried"; here we colour by RADIUS (which the gate+impact sizing
    # set, so on this FSD structure it IS the knock's demand) -- relabel it so the picture is honest.
    fig.update_traces(selector=dict(type="mesh3d", showscale=True),
                      colorbar_title_text="radius<br>(percentile)")

    orbit = open("scripts/_orbit.js").read()
    fig.write_html("out/impact.html", include_plotlyjs="cdn", post_script=orbit,
                   config={"responsive": True, "displayModeBar": False})
    _mobile("out/impact.html")
    print(f"  {len(live)} struts, {mass:.1f} g, knock {knock:.0f} MPa, keys {w_um:.0f} um")
    print(f"  radii {radii.min()*1e3:.2f}-{radii.max()*1e3:.2f} mm "
          f"(p10 {np.percentile(radii,10)*1e3:.2f}, p90 {np.percentile(radii,90)*1e3:.2f})")
    print(f"  vs the keypress bone thickened for the knock: {m_bolt:.1f} g "
          f"-> {100*(1-mass/m_bolt):.0f}% lighter")
    print("\nbrowser view: out/impact.html")


if __name__ == "__main__":
    main()

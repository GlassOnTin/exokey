"""THE MINIMAL-COPPER HARNESS BUS (§8.15l qqq-2).

    PYTHONPATH=. .venv/bin/python scripts/harness_view.py   ->  out/harness.html

The wires do not run point-to-point: the sensors are I²C, so VDD/GND are shared and SDA/SCL are a bus.
So the harness is a **shared Steiner tree over the gauntlet struts** (`mount.harness_bus`), not five
independent runs. This shows that tree, coloured by how many conductors share each groove, against the
routable struts — with the old five-run routing (`mount.harness_routes`) on a legend toggle, so the
sharing is visible. Sensors in red, the wrist anchors the MCU sits on in green.
"""
from __future__ import annotations

import pickle

import numpy as np
import plotly.graph_objects as go

from design.vector import posture, tm_of, tp_of
from hand.myohand import FINGERS
from manufacture import mount
from opt.problem import hands
from viz.scene import skin_trace

WIRE_COLOUR = {2: "#2f9e44", 4: "#1c4fd6", 6: "#e8590c"}     # power-only / one bus / both buses


def main():
    h = hands()[50]
    x = pickle.load(open("out/final_design.pkl", "rb"))["x"]
    q = h.compose({f: posture(h, f, tp_of(x, f), tm_of(x, f), float(x.get(f"ab_{f}", 0.0)))
                   for f in FINGERS})
    z = np.load("out/final.npz", allow_pickle=True)
    nodes = np.array(z["nodes"], float)
    bars = [tuple(b) for b in z["bars"]]
    live = [int(e) for e in z["live"]]
    btn = {f: int(i) for f, i in zip(z["fingers"], z["buttons"])}
    anchors = [int(a) for a in z["anchors"]]

    def seg_trace(segs, colour, width, name, show=True, visible=True):
        xs, ys, zs = [], [], []
        for i, j in segs:
            xs += [nodes[i][0], nodes[j][0], None]
            ys += [nodes[i][1], nodes[j][1], None]
            zs += [nodes[i][2], nodes[j][2], None]
        return go.Scatter3d(x=xs, y=ys, z=zs, mode="lines", line=dict(color=colour, width=width),
                            name=name, hoverinfo="name", showlegend=show, visible=visible)

    bus = mount.harness_bus(nodes, bars, live, btn, anchors)
    bus_len = sum(float(np.linalg.norm(nodes[i] - nodes[j])) for i, j, _ in bus)
    copper = sum(float(np.linalg.norm(nodes[i] - nodes[j])) * nw for i, j, nw in bus)
    routes = mount.harness_routes(nodes, bars, live, btn, anchors)
    route_segs = [(r[k], r[k + 1]) for r in routes for k in range(len(r) - 1)]
    base_len = sum(float(np.linalg.norm(nodes[a] - nodes[b])) for a, b in route_segs)

    traces = []
    sk = skin_trace(h, q, opacity=0.10)
    if sk is not None:
        traces.append(sk)
    traces.append(seg_trace([bars[e] for e in live], "#e0e0e0", 1,
                            "gauntlet struts (routable graph)"))                      # faint context
    traces.append(seg_trace(route_segs, "#f0a0a0", 4,
                            f"OLD: five independent runs ({base_len*1e3:.0f} mm)",
                            visible="legendonly"))                                    # toggle to compare

    by_nw = {}
    for i, j, nw in bus:
        by_nw.setdefault(nw, []).append((i, j))
    for nw in sorted(by_nw):
        traces.append(seg_trace(by_nw[nw], WIRE_COLOUR.get(nw, "#1c4fd6"), 3 + nw,
                                f"bus groove — {nw} wires ({len(by_nw[nw])} seg)"))

    S = np.array([nodes[btn[f]] for f in FINGERS])
    A = nodes[np.array(anchors)]
    traces.append(go.Scatter3d(x=S[:, 0], y=S[:, 1], z=S[:, 2], mode="markers",
                               marker=dict(size=5, color="#d6336c"), name="sensors (5)",
                               hoverinfo="name"))
    traces.append(go.Scatter3d(x=A[:, 0], y=A[:, 1], z=A[:, 2], mode="markers",
                               marker=dict(size=2.5, color="#2f9e44"), name="wrist anchors (MCU)",
                               hoverinfo="name"))

    fig = go.Figure(traces)
    fig.update_layout(
        title=f"ExoKey — the minimal-copper harness.  The wires ride a <b>shared bus</b> (a Steiner "
              f"tree over the struts), <b>{bus_len*1e3:.0f} mm</b> against <b>{base_len*1e3:.0f} mm</b> "
              f"of five point-to-point runs — <b>−{100*(1-copper/(4*base_len)):.0f}% copper</b>.  "
              f"Toggle 'OLD' in the legend to see the sharing.",
        scene=dict(aspectmode="data", xaxis_visible=False, yaxis_visible=False, zaxis_visible=False,
                   camera=dict(eye=dict(x=-1.5, y=0.8, z=0.8))),
        margin=dict(l=0, r=0, t=70, b=0), template="plotly_white",
        legend=dict(x=0.02, y=0.98, bgcolor="rgba(255,255,255,0.6)"))
    fig.write_html("out/harness.html", include_plotlyjs="cdn")
    print(f"  bus {bus_len*1e3:.0f} mm ({len(bus)} seg) vs {base_len*1e3:.0f} mm independent; "
          f"copper {copper*1e3:.0f} vs {4*base_len*1e3:.0f} mm-eq")
    print("browser view: out/harness.html")


if __name__ == "__main__":
    main()

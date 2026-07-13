"""Stage 3 -- the exoskeleton structure, loaded and rendered.

Builds the frame around the hand posed on its Stage-2 optimal keys, presses them, and
reports the four things the outer loop will need: mass, stress utilisation, key
deflection, and clearance from the flesh.

Also runs the sensitivity the plan demands: soft-tissue support stiffness is poorly
characterised (10-50 N/mm in the literature), so sweep it and REPORT HOW FAR THE ANSWER
MOVES rather than pretending to a number.

Writes out/stage3.html.
"""
from __future__ import annotations

import numpy as np
import plotly.graph_objects as go

from hand.myohand import FINGERS, MyoHand
from structure.frame import (
    DEFLECTION_MAX,
    MATERIALS,
    SAFETY_FACTOR,
    build_exo,
    clearance,
    solve,
)
from viz.scene import _mesh_traces, _pad_traces, well_traces

KIND_STYLE = {"alu": 9, "nylon": 6, "strap": 4, "clip": 5}


def main():
    h = MyoHand()
    d = np.load("out/optimal_keys.npz", allow_pickle=True)
    press = float(d["press_N"])
    q = d["q_opt"]
    keys = {f: (d["pos"][i], d["normal"][i]) for i, f in enumerate(d["fingers"])}

    exo = build_exo(h, q, keys)
    print(f"frame: {len(exo.nodes)} nodes, {len(exo.members)} members, "
          f"{len(exo.supports)} soft-tissue contacts")
    print(f"mass:  {exo.mass()*1000:.1f} g "
          f"(al6061 strip {DEFAULTS_MM()}, pa12 stalks, nylon webbing)\n")

    # ---- clearance -------------------------------------------------------------------
    gaps = clearance(h, q, exo)
    worst = min(gaps, key=gaps.get)
    print(f"clearance (rigid members vs flesh): worst = {worst} at {gaps[worst]*1000:+.1f} mm")
    for k, v in sorted(gaps.items(), key=lambda kv: kv[1])[:3]:
        print(f"    {k:16s} {v*1000:+7.1f} mm")

    # ---- load cases ------------------------------------------------------------------
    print(f"\nload cases at {press} N/key:")
    print(f"{'chord':22s} {'max util':>9s} {'worst member':>16s} {'max defl':>10s}  ok")
    print("-" * 66)
    chords = [[(f, 0)] for f in FINGERS] + [[(f, 0) for f in FINGERS]]
    for c in chords:
        r = solve(exo, c, press_N=press)
        label = "+".join(f for f, _ in c) if len(c) < 5 else "ALL FIVE (worst case)"
        flag = "ok" if r["ok"] else ("YIELD" if r["max_util"] > 1 else "MUSHY")
        print(f"{label:22s} {r['max_util']:9.3f} {r['worst_member']:>16s} "
              f"{r['max_deflection']*1000:8.3f}mm  {flag}")

    full = solve(exo, [(f, 0) for f in FINGERS], press_N=press)
    print(f"\n  gate: utilisation <= 1.0 (yield/{SAFETY_FACTOR:.0f}), "
          f"deflection <= {DEFLECTION_MAX*1000:.1f} mm")
    print("  per-key deflection: "
          + ", ".join(f"{f} {full['deflection'][(f, 0)]*1000:.3f}mm" for f in FINGERS))

    # ---- the sensitivity the plan asks for -------------------------------------------
    print("\nsoft-tissue support stiffness sweep (literature: 10-50 N/mm, poorly known):")
    print(f"{'k (N/mm)':>9s} {'max defl':>10s} {'max util':>9s}")
    ks = [10e3, 25e3, 50e3, 100e3]
    res = [solve(exo, [(f, 0) for f in FINGERS], press_N=press, k_soft=k) for k in ks]
    for k, r in zip(ks, res):
        print(f"{k/1e3:9.0f} {r['max_deflection']*1000:8.3f}mm {r['max_util']:9.3f}")
    dl = [r["max_deflection"] for r in res]
    ut = [r["max_util"] for r in res]
    print(f"\n  over the 10-50 N/mm band: deflection moves "
          f"{max(dl[:3])/min(dl[:3]):.2f}x, utilisation moves {max(ut[:3])/min(ut[:3]):.2f}x")
    print("  => deflection is sensitive to it; stress is not. Any deflection number "
          "here carries that band.")

    # ---- browser view ----------------------------------------------------------------
    traces = _mesh_traces(h, q, opacity=0.18) + _pad_traces(h, q)
    u = full["util"]
    umax = max(u.values())
    for m in exo.members:
        a, b = exo.nodes[m.i], exo.nodes[m.j]
        frac = u[m.name] / max(umax, 1e-9)
        col = f"rgb({int(40+215*frac)},{int(90+60*(1-frac))},{int(200*(1-frac))})"
        traces.append(
            go.Scatter3d(
                x=[a[0], b[0]], y=[a[1], b[1]], z=[a[2], b[2]], mode="lines",
                line=dict(color=col, width=KIND_STYLE[m.kind],
                          dash="dot" if m.kind == "strap" else "solid"),
                name=f"{m.name} ({m.kind})",
                text=[f"{m.name}<br>{m.material}<br>util {u[m.name]:.3f}<br>"
                      f"σ {full['stress'][m.name]/1e6:.1f} MPa"] * 2,
                hoverinfo="text", showlegend=False,
            )
        )
    S = np.array([exo.nodes[n] for n in exo.supports])
    traces.append(go.Scatter3d(
        x=S[:, 0], y=S[:, 1], z=S[:, 2], mode="markers",
        marker=dict(size=7, symbol="x", color="#b03060"),
        name="soft-tissue supports", text=exo.supports, hoverinfo="text"))
    traces += well_traces([
        dict(**h.well_frame(q, f), finger=f, label=f) for f in FINGERS
    ])

    # HONESTY IN THE TITLE. These keys come from STAGE 2, which optimises each finger
    # INDEPENDENTLY -- it places the index's well knowing nothing about the middle finger. So
    # overlapping wells are its EXPECTED output, not a bug, and resolving them is precisely
    # what Stage 4's constraints exist to do. But the render did not SAY so, and a
    # known-infeasible layout read as a proposal. It says it now.
    from design.vector import WELL_WALL, _seg_seg_dist, well_channel

    ch = {f: well_channel(h, q, f) for f in FINGERS}
    worst_ov, worst_pair = -np.inf, None
    for a in range(len(FINGERS)):
        for b in range(a + 1, len(FINGERS)):
            fa, fb = FINGERS[a], FINGERS[b]
            da, pa, ra = ch[fa]
            db, pb, rb = ch[fb]
            ov = (ra + rb + 2 * WELL_WALL) - _seg_seg_dist(da, pa, db, pb)
            if ov > worst_ov:
                worst_ov, worst_pair = ov, (fa, fb)
    warn = (f"<br><b>WELLS OVERLAP by {worst_ov*1000:.1f} mm "
            f"({worst_pair[0]}/{worst_pair[1]}) — EXPECTED.</b> Stage 2 places each finger's "
            f"well INDEPENDENTLY of the others; Stage 4 is what resolves it."
            if worst_ov > 0 else "<br>wells clear.")

    fig = go.Figure(traces)
    fig.update_layout(
        title=f"ExoKey Stage 3 — structure on the STAGE-2 keys. "
              f"{exo.mass()*1000:.0f} g, max util {full['max_util']:.2f}, "
              f"max key deflection {full['max_deflection']*1000:.2f} mm "
              f"(all five pressing @ {press} N). Members coloured by stress utilisation." + warn,
        scene=dict(aspectmode="data", xaxis_title="x (m)", yaxis_title="y (m)",
                   zaxis_title="z (m)"),
        margin=dict(l=0, r=0, t=60, b=0), template="plotly_white",
    )
    fig.write_html("out/stage3.html", include_plotlyjs="cdn")
    print("\nbrowser view: out/stage3.html")


def DEFAULTS_MM():
    from structure.frame import DEFAULTS

    b, t = DEFAULTS["sec_alu"]
    return f"{b*1000:.0f}x{t*1000:.0f}mm"


if __name__ == "__main__":
    main()

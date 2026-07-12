"""Render a chosen design off the Pareto front, on the hand, with the structure loaded.

    .venv/bin/python scripts/stage4_view.py [--pick knee|effort|mass|crisp]

A 3-D scatter of a Pareto front tells you the trade-off. It does NOT tell you whether the
device is something a person would wear, and that is the judgement the whole project needs
a human for. So put the winner back on the hand and look at it.
"""
from __future__ import annotations

import argparse
import pickle

import numpy as np
import plotly.graph_objects as go

from design.vector import evaluate, keys_on_reference
from hand.myohand import FINGERS, MyoHand
from hand.scaling import population
from opt.problem import CONSTRAINT_NAMES, OBJECTIVE_NAMES
from viz.scene import FINGER_COLOR, _mesh_traces, keycap_traces

KIND_STYLE = {"alu": 9, "nylon": 6, "strap": 4, "clip": 5}


def pick(F: np.ndarray, how: str) -> int:
    if how == "effort":
        return int(np.argmin(F[:, 0]))
    if how == "mass":
        return int(np.argmin(F[:, 1]))
    if how == "crisp":
        return int(np.argmin(F[:, 2]))
    # knee: closest to the ideal point after normalising each objective to [0,1]
    lo, hi = F.min(0), F.max(0)
    Z = (F - lo) / np.where(hi - lo > 0, hi - lo, 1.0)
    return int(np.argmin(np.linalg.norm(Z, axis=1)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pick", default="knee", choices=["knee", "effort", "mass", "crisp"])
    args = ap.parse_args()

    with open("out/pareto.pkl", "rb") as fh:
        d = pickle.load(fh)
    F, X = np.atleast_2d(d["F"]), d["X"]
    i = pick(F, args.pick)
    x = X[i]

    hands = {p: MyoHand(scale=s) for p, s in population((5, 50, 95)).items()}
    ref = hands[50]
    r = evaluate(x, hands)

    print(f"chosen design ({args.pick}) of {len(F)} on the front\n")
    for nm, v in zip(OBJECTIVE_NAMES, r["F"]):
        print(f"  {nm:20s} {v:12.4g}")
    print(f"  {'total keys':20s} {r['total_keys']:12d}   "
          + " ".join(f"{f}:{r['n_keys'][f]}" for f in FINGERS))
    print(f"  {'switch force':20s} {r['press_N']:12.2f} N")
    print(f"  {'frame':20s} {x['material']}, "
          f"{x['alu_w']*1000:.1f} x {x['alu_t']*1000:.2f} mm strip")
    print(f"\n  constraints (<= 0):")
    for nm, v in zip(CONSTRAINT_NAMES, r["G"]):
        print(f"    {nm:14s} {v:+9.4f}  {'ok' if v <= 0 else 'VIOLATED'}")
    print(f"\n  effort/char by hand (5th, 50th, 95th): "
          + ", ".join(f"{v:.3e}" for v in r["char_effort_by_hand"]))

    # ---- render: hand at rest, the device on it, keys coloured by finger ---------------
    exo, keys = r["exo"], r["keys_ref"]
    st = r["struct"]
    traces = _mesh_traces(ref, ref.q_neutral, opacity=0.16)

    u = st["util"]
    umax = max(u.values())
    for m in exo.members:
        a, b = exo.nodes[m.i], exo.nodes[m.j]
        frac = u[m.name] / max(umax, 1e-9)
        col = f"rgb({int(40+215*frac)},{int(90+60*(1-frac))},{int(200*(1-frac))})"
        traces.append(go.Scatter3d(
            x=[a[0], b[0]], y=[a[1], b[1]], z=[a[2], b[2]], mode="lines",
            line=dict(color=col, width=KIND_STYLE[m.kind],
                      dash="dot" if m.kind == "strap" else "solid"),
            text=[f"{m.name}<br>{m.material}<br>util {u[m.name]:.3f}"] * 2,
            hoverinfo="text", showlegend=False))

    # Real low-profile keycaps, not markers: the cap's top face is the surface the pad
    # lands on, so obliquity is visible instead of hidden.
    caps = [
        dict(pos=keys[(f, k)][0], normal=keys[(f, k)][1], finger=f,
             label=f"{f} row {k}")
        for f in FINGERS for k in range(r["n_keys"][f])
    ]
    traces += keycap_traces(caps)
    for f in FINGERS:  # legend entries only
        traces.append(go.Scatter3d(
            x=[keys[(f, 0)][0][0]], y=[keys[(f, 0)][0][1]], z=[keys[(f, 0)][0][2]],
            mode="markers", marker=dict(size=0.1, color=FINGER_COLOR[f]),
            name=f"{f} ({r['n_keys'][f]} keys)"))

    # How squarely does each pad meet its cap? For the thumb this is ~80 deg -- a direct,
    # visible consequence of MyoHand having no adductor pollicis.
    print("\n  pad-to-cap obliquity (0 deg = pad flat on the cap):")
    for f in FINGERS:
        pos, nrm = keys[(f, 0)]
        post = ref.press(f, pos, nrm, press_N=r["press_N"], q0=ref.q_neutral)
        _, pn = ref.pad_pose(post.q, f)
        ang = np.rad2deg(np.arccos(np.clip(-(pn @ nrm), -1, 1)))
        print(f"    {f:8s} {ang:5.0f} deg" + ("   <-- oblique: needs an angled cap"
                                              if ang > 45 else ""))

    S = np.array([exo.nodes[n] for n in exo.supports])
    traces.append(go.Scatter3d(
        x=S[:, 0], y=S[:, 1], z=S[:, 2], mode="markers",
        marker=dict(size=7, symbol="x", color="#b03060"), name="soft-tissue supports"))

    fig = go.Figure(traces)
    fig.update_layout(
        title=(f"ExoKey Stage 4 — {args.pick} design. {r['total_keys']} keys, "
               f"{r['F'][1]:.0f} g, effort/char {r['F'][0]:.2e}, "
               f"max deflection {r['F'][2]:.2f} mm. Feasible for the 5th–95th percentile hand."),
        scene=dict(aspectmode="data", xaxis_title="x (m)", yaxis_title="y (m)",
                   zaxis_title="z (m)"),
        template="plotly_white", margin=dict(l=0, r=0, t=60, b=0))
    fig.write_html("out/stage4.html", include_plotlyjs="cdn")
    print("\nbrowser view: out/stage4.html")


if __name__ == "__main__":
    main()

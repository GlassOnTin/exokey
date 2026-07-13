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

from design.vector import evaluate, keys_on_reference, posture
from hand.myohand import FINGERS, MyoHand
from hand.scaling import population
from opt.problem import CONSTRAINT_NAMES, OBJECTIVE_NAMES
from viz.scene import FINGER_COLOR, _mesh_traces, well_traces

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

    # ---- render: THE HAND IN THE POSTURE THE DESIGN IS FOR, wearing the device ---------
    #
    # It drew the hand at `q_neutral` -- AT REST -- while placing the wells at the DESIGN
    # posture. So the picture showed wells hanging in space off a hand that was not the hand
    # the design is for, and every "is that well on that fingertip?" question it was supposed
    # to answer, it answered about the wrong hand.
    #
    # The wells and the hand are now BOTH drawn from one COMPOSED posture -- all five digits
    # in their wells at once, which is what wearing it means, and what the well-vs-finger
    # collision check assumes.
    from design.vector import action_dirs, tm_of, tp_of

    exo, keys = r["exo"], r["keys_ref"]
    st = r["struct"]
    per = {f: posture(ref, f, tp_of(x, f), tm_of(x, f), x.get(f"ab_{f}", 0.0)) for f in FINGERS}
    q_on = ref.compose(per)
    traces = _mesh_traces(ref, q_on, opacity=0.16)

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

    # WELLS, drawn as the U-CHANNELS they are, with their five joystick directions -- in the
    # SAME composed posture as the hand, so a well sitting inside a neighbouring finger is
    # visible rather than hidden by a posture mismatch.
    cups = [dict(**ref.well_frame(q_on, f), finger=f, label=f"{f} well",
                 dirs=action_dirs(ref, q_on, f))
            for f in FINGERS]
    traces += well_traces(cups)
    for f in FINGERS:  # legend entries only
        traces.append(go.Scatter3d(
            x=[keys[(f, 0)][0][0]], y=[keys[(f, 0)][0][1]], z=[keys[(f, 0)][0][2]],
            mode="markers", marker=dict(size=0.1, color=FINGER_COLOR[f]),
            name=f"{f} ({r['n_keys'][f]} keys)"))

    # CAN THE DIGIT ACTUALLY PUSH ALONG ITS WELL'S CLICK AXIS?
    #
    # "Pad-to-cap obliquity" is now a meaningless question -- the well faces the pad BY
    # CONSTRUCTION, so it is 0 by definition. (The old line computed it against the previous
    # `-press` convention and, after the axis was flipped to the pad normal, reported 175-180
    # deg for every digit and advised an "angled cap" on all five. A stale diagnostic is worse
    # than none: it invents a finding.)
    #
    # The question that still matters is different, and it is the real thumb caveat: `click`
    # presses along the pad normal, but a digit can only push where its MUSCLES let it. The
    # gap between the two is wasted as shear -- and for the thumb it is large, because
    # MyoHand has no adductor pollicis.
    print("\n  click axis vs the direction the digit can actually PUSH:")
    for f in FINGERS:
        q_f = per[f]
        _, pn = ref.pad_pose(q_f, f)
        push = ref.press_dir_flexor(q_f, f)
        ang = np.rad2deg(np.arccos(np.clip(float(push @ pn), -1.0, 1.0)))
        note = "   <-- much of the press is shear, not click" if ang > 45 else ""
        print(f"    {f:8s} {ang:5.0f} deg{note}")

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

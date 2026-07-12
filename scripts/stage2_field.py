"""Stage 2 (coarse) -- the effort landscape over each finger's reachable workspace.

The question this answers is the one the whole project rests on: DOES KEY POSITION
MATTER? If effort barely moves as a key is placed around the workspace, there is nothing
for an optimiser to find and no reason to build one. v1 never asked, because its "effort"
was a sum of constraint violations and could not have answered.

Method. Rather than scatter keys into space and ask whether a finger can reach them
(most cannot, and the failures tell us nothing), we sweep the finger's own flexion
posture and read the pad pose off it. Every sample is then reachable BY CONSTRUCTION,
the key normal is the natural pad normal at that posture, and the grid axes are directly
interpretable: "how curled is the finger when it hits this key".

DIP is tied to PIP at 2/3, the standard finger flexion synergy -- the DIP is not
independently controlled in normal use.

Writes out/stage2_field.html.
"""
from __future__ import annotations

import numpy as np
import plotly.graph_objects as go

from hand.myohand import FINGERS, FLEXION_JOINTS, MyoHand
from viz.scene import FINGER_COLOR, _mesh_traces, _pad_traces

PRESS_N = 0.5
N_GRID = 9  # 9x9 postures per finger
SWITCH_TRAVEL = 0.003  # m; a mechanical switch actuates at ~2 mm, bottoms at ~4 mm


def sweep(h: MyoHand, finger: str, n: int = N_GRID) -> list[dict]:
    """Effort over (proximal x middle) flexion; the distal joint follows at 2/3.

    Joints come from FLEXION_JOINTS by name, and the span is clamped to the flexion side
    (see MyoHand.flexion_span). Indexing them out of DIGIT_JOINTS by position instead swept
    the THUMB's abduction axis as if it were flexion, and drove its IP joint to -72 deg --
    hyperextended -- which then won on effort. A key must never demand a hyperextended digit.
    """
    prox, mid, dist = FLEXION_JOINTS[finger]
    a_p, _, lim_p = h.flexion_span(prox)
    a_m, _, lim_m = h.flexion_span(mid)
    a_d, _, lim_d = h.flexion_span(dist)

    # Work in the FLEXION FRACTION t (0 straight, 1 fully flexed) and scale by the signed
    # limit. Never np.clip() against (lo, hi) here: for the thumb those are (0.0, -75 deg),
    # so a_min > a_max and numpy silently returns a_max -- which pinned the thumb's IP at
    # FULL flexion in all 81 samples, left it no headroom, and made every thumb key
    # unpressable.
    out = []
    for f_p in np.linspace(0.05, 0.95, n):  # stay off the exact limits
        for f_m in np.linspace(0.05, 0.95, n):
            q = h.q_neutral.copy()
            q[a_p] = f_p * lim_p
            q[a_m] = f_m * lim_m
            q[a_d] = min(1.0, (2.0 / 3.0) * f_m) * lim_d

            pos, nrm = h.pad_pose(q, finger)
            a, effort, resid, floor, scale = h.solve_activations(q, finger, PRESS_N, -nrm)
            travel = h.press_travel(q, finger, -nrm)
            out.append(
                dict(
                    finger=finger, pos=pos, normal=-nrm, q=q,
                    mcp=f_p, pip=f_m, effort=float(effort), max_act=float(a.max()),
                    travel=float(travel),
                    # HARD feasibility. Effort alone is degenerate: it is minimised by a
                    # finger doing nothing at its flexion limit, which cannot press a key.
                    ok=bool(travel >= SWITCH_TRAVEL and a.max() <= 0.95),
                )
            )
    return out


def main():
    h = MyoHand()
    print(f"press = {PRESS_N} N, gravity {'ON' if h.gravity else 'OFF'}, "
          f"{N_GRID}x{N_GRID} postures/finger\n")

    samples = {}
    print(f"{'finger':8s} {'pressable':>10s} {'effort min':>11s} {'effort max':>11s} "
          f"{'spread':>9s}  (feasible keys only)")
    print("-" * 78)
    for f in FINGERS:
        s = sweep(h, f)
        samples[f] = s
        good = [x for x in s if x["ok"]]
        e = np.array([x["effort"] for x in good])
        print(f"{f:8s} {len(good):5d}/{len(s):<4d} {e.min():11.3e} {e.max():11.3e} "
              f"{e.max()/max(e.min(),1e-12):8.0f}x")

    # ---- the number the project hinges on -------------------------------------------
    print()
    for f in FINGERS:
        e = np.array([x["effort"] for x in samples[f] if x["ok"]])
        lo, hi = np.percentile(e, [10, 90])
        print(f"  {f:8s} 10th-90th pct effort: {lo:.3e} .. {hi:.3e}  "
              f"({hi/max(lo,1e-12):.1f}x within the finger)")

    # ---- key selection: COUPLED across the four fingers ------------------------------
    #
    # Choosing each finger's key by its own argmin is wrong twice over.
    #
    # 1. Effort alone is degenerate. It is minimised by a finger doing NOTHING, at its
    #    flexion limit, where the flexors go slack -- and a finger at its limit cannot
    #    press a key. The travel gate (>= SWITCH_TRAVEL) is a hard constraint for exactly
    #    this reason, but on its own the optimum just parks ON the constraint (the ring
    #    came out at 3.2 mm of travel against a 3.0 mm gate).
    #
    # 2. MyoHand has NO ENSLAVEMENT. Its FDP2..FDP5 moment arms are strictly diagonal --
    #    four independent actuators, no shared muscle belly, no tendon coupling -- so
    #    curling the ring fully while its neighbours stay relaxed is free HERE and
    #    impossible in a real hand, where the ring is the least independent digit. Left
    #    alone the optimiser exploits that and returns a hand nobody can make (a 68
    #    percentage-point MCP spread across the four fingers).
    #
    # So the four fingers are constrained to a COMMON flexion posture -- common drive, the
    # crudest honest stand-in for enslavement, and also what a key row physically wants.
    # It is conservative: real fingers do individuate somewhat. Modelling enslavement
    # properly needs a hand whose long flexors actually share a belly.
    # The thumb has its own flexor and its own independence, so it is chosen separately.
    grid = {(round(x["mcp"], 4), round(x["pip"], 4)): x for x in samples["index"]}
    fingers4 = [f for f in FINGERS if f != "thumb"]
    coupled, best_cost = None, np.inf
    for cell in grid:
        row = [next(x for x in samples[f]
                    if (round(x["mcp"], 4), round(x["pip"], 4)) == cell) for f in fingers4]
        if not all(x["ok"] for x in row):
            continue
        cost = sum(x["effort"] for x in row)
        if cost < best_cost:
            best_cost, coupled = cost, dict(zip(fingers4, row))

    if coupled is None:
        raise RuntimeError("no common finger posture presses all four keys")

    best = dict(coupled)
    good_t = [x for x in samples["thumb"] if x["ok"]]
    best["thumb"] = good_t[int(np.argmin([x["effort"] for x in good_t]))]
    q_opt = h.compose({f: best[f]["q"] for f in FINGERS})

    print("\noptimal PRESSABLE key per finger (hand posed here in the view):")
    for f in FINGERS:
        b = best[f]
        print(f"  {f:8s} MCP {b['mcp']*100:3.0f}%  PIP {b['pip']*100:3.0f}%   "
              f"Σa³={b['effort']:.2e}  travel={b['travel']*1000:4.1f}mm  "
              f"max act={b['max_act']:.3f}")
    sp = [best[f]["mcp"] for f in FINGERS if f != "thumb"]
    print(f"  MCP flexion spread across the four fingers: "
          f"{(max(sp)-min(sp))*100:.0f} percentage points")

    np.savez(
        "out/optimal_keys.npz",
        fingers=np.array(FINGERS),
        pos=np.array([best[f]["pos"] for f in FINGERS]),
        normal=np.array([best[f]["normal"] for f in FINGERS]),
        effort=np.array([best[f]["effort"] for f in FINGERS]),
        travel=np.array([best[f]["travel"] for f in FINGERS]),
        q_opt=q_opt,
        press_N=PRESS_N,
    )

    # ---- browser view ---------------------------------------------------------------
    traces = _mesh_traces(h, q_opt, opacity=0.20)  # hand ON the keys
    traces += _pad_traces(h, q_opt)
    all_e = np.concatenate([[x["effort"] for x in samples[f]] for f in FINGERS])
    for f in FINGERS:
        s = samples[f]
        P = np.array([x["pos"] for x in s])
        E = np.array([x["effort"] for x in s])
        traces.append(
            go.Scatter3d(
                x=P[:, 0], y=P[:, 1], z=P[:, 2], mode="markers", name=f,
                marker=dict(
                    size=4, color=np.log10(np.maximum(E, 1e-9)), colorscale="Viridis",
                    showscale=(f == "index"), cmin=np.log10(max(all_e.min(), 1e-9)),
                    cmax=np.log10(all_e.max()),
                    colorbar=dict(title="log10 Σa³"),
                ),
                text=[f"{f}<br>MCP {x['mcp']*100:.0f}% PIP {x['pip']*100:.0f}%"
                      f"<br>Σa³={x['effort']:.2e}<br>max act={x['max_act']:.3f}" for x in s],
                hoverinfo="text",
            )
        )
        # mark the cheapest PRESSABLE key for this finger
        b = best[f]
        traces.append(
            go.Scatter3d(
                x=[b["pos"][0]], y=[b["pos"][1]], z=[b["pos"][2]], mode="markers",
                marker=dict(size=11, symbol="diamond", color=FINGER_COLOR[f],
                            line=dict(width=2, color="#000")),
                name=f"{f} cheapest", showlegend=False,
            )
        )

    fig = go.Figure(traces)
    fig.update_layout(
        title=f"ExoKey Stage 2 — effort over the reachable workspace ({PRESS_N} N press). "
              f"Hand posed ON its optimal keys (diamonds).",
        scene=dict(aspectmode="data", xaxis_title="x (m)", yaxis_title="y (m)",
                   zaxis_title="z (m)"),
        margin=dict(l=0, r=0, t=40, b=0), template="plotly_white",
    )
    import os
    os.makedirs("out", exist_ok=True)
    fig.write_html("out/stage2_field.html", include_plotlyjs="cdn")
    print(f"\nbrowser view: out/stage2_field.html "
          f"({sum(len(samples[f]) for f in FINGERS)} key poses)")


if __name__ == "__main__":
    main()

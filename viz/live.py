"""LIVE VIEW: the optimiser streams into the render, and you watch it work.

THE USER, and it is a correction to how this whole project has been run:

    "I usually prefer to do this sort of project with a tight coupled visualisation, or
     sometimes even visualisation being first and optimisation event coupled to it."

They are right, and I have had it backwards. Every substantial bug in this project was caught
by LOOKING -- the fingernail, the thumb's sign, the wells intersecting, the bone not fitting
the well, the structure being a ball in the palm. Not one of them was caught by a test I had
written in advance, because you cannot write a test for a mistake you do not know you are
making. The picture is the only instrument that finds the unknown unknowns.

And yet every run has been: optimise for 20 minutes, then render a static page afterwards. The
picture arrived AFTER the decision. That is the wrong order.

THE DESIGN, and it is deliberately the laziest thing that works:

  * the HAND MESH is written ONCE (it is 4 MB and it never changes)
  * each generation, the optimiser writes `out/live.json` -- just the DEVICE: wells, frame
    nodes, members, and the numbers. A few kB.
  * the page polls it and calls Plotly.react, which redraws only what changed.

No websockets, no server process, no new dependency. `python3 -m http.server` and a `fetch`.
The moment it needs more than that, it can have more than that.
"""
from __future__ import annotations

import json
import os

import numpy as np

LIVE = "out/live.json"


def scene(h, x, gen: int = -1, note: str = "") -> dict:
    """THE DEVICE, as Plotly traces. Small enough to write every few generations.

    ⚠ THIS USED TO DRAW THE PALMAR BOX -- and it went on trying to for a whole 62-minute run,
    failing every time with `KeyError: 'alu_w'`, because that variable was deleted along with the
    architecture it shaped. The exception was caught so it could not kill the optimiser, and the
    3D panel therefore sat empty for 40 generations while the front streamed happily beside it.
    A render that fails silently is worse than no render: it looks like nothing is happening.

    What it draws now is what the optimiser is actually deciding: the hand, the five wells, and
    the GROWN GAUNTLET -- Wolff's law on a free-form lattice, coarse (the same growth the GA is
    steering by), coloured by how hard each strut is working.
    """
    from design.params import DEFLECTION_MAX
    from design.qwerty import best_action_map, used_actions
    from design.vector import PRESS_N, evaluate, posture, tm_of, tp_of
    from hand.myohand import FINGERS
    from opt.problem import hands
    from structure.frame import hand_axes
    from structure.lattice import grow

    per = {f: posture(h, f, tp_of(x, f), tm_of(x, f), x.get(f"ab_{f}", 0.0)) for f in FINGERS}
    q_on = h.compose(per)

    r = evaluate(x, hands())
    wired = used_actions(r["action_map"])
    nodes, bars, live, btn, cases, ak, an, hist, pc, _sh, _ls = grow(
        h, q_on, wired=wired, gate=float(DEFLECTION_MAX), pitch=0.008, rate=0.20)

    seg = _bone_segments(h, q_on)
    bx, by, bz = [], [], []
    for k in range(0, len(seg), 2):
        bx += [seg[k][0], seg[k + 1][0], None]
        by += [seg[k][1], seg[k + 1][1], None]
        bz += [seg[k][2], seg[k + 1][2], None]
    traces = [dict(type="scatter3d", x=bx, y=by, z=bz, mode="lines",
                   line=dict(color="#c8b8a8", width=9), hoverinfo="skip", name="hand")]

    # the grown bone, as line segments (a tube mesh is too heavy to write every few generations)
    sx, sy, sz = [], [], []
    for e in live:
        a, b = nodes[bars[e][0]], nodes[bars[e][1]]
        sx += [float(a[0]), float(b[0]), None]
        sy += [float(a[1]), float(b[1]), None]
        sz += [float(a[2]), float(b[2]), None]
    traces.append(dict(type="scatter3d", x=sx, y=sy, z=sz, mode="lines",
                       line=dict(color="#12639b", width=5), hoverinfo="skip", name="gauntlet"))

    P = np.array([nodes[btn[f]] for f in FINGERS])
    traces.append(dict(type="scatter3d", x=[float(v) for v in P[:, 0]],
                       y=[float(v) for v in P[:, 1]], z=[float(v) for v in P[:, 2]],
                       mode="markers", marker=dict(size=5, color="#e8590c"),
                       hoverinfo="skip", name="buttons"))

    # ⚠ THE STRAPS, AND THE NODES THEY ACTUALLY PULL ON.
    #
    # The user, watching the live run: "the straps no longer appear connected to the elements? Is
    # that just a rendering issue?" It was not. The wrist band had been placed 31 mm PROXIMAL of
    # the hand -- out on the forearm, where the gauntlet does not go -- so the structure reached it
    # at ONE node. A band the structure cannot reach is not a strap; it is a decoration the solver
    # is nonetheless leaning on.
    #
    # So the live view now draws BOTH: the bands, and the structural nodes the strap is pulling
    # on. If those markers are not sitting ON a band, the anchor is fiction, and you can see it.
    from structure.anchor import strap_bands, under_strap

    A = [nodes[i] for i in sorted(ak)]
    held = {i for e in live for i in bars[e]}
    pulled = sorted(under_strap(h, q_on, nodes, sorted(ak)) & held)
    if pulled:
        Q = np.array([nodes[i] for i in pulled])
        traces.append(dict(type="scatter3d", x=[float(v) for v in Q[:, 0]],
                           y=[float(v) for v in Q[:, 1]], z=[float(v) for v in Q[:, 2]],
                           mode="markers", marker=dict(size=7, color="#b03060",
                                                       symbol="diamond"),
                           hoverinfo="skip", name="strap pulls here"))
    # ⚠ THE BAND IS THE HAND'S OWN CROSS-SECTION, not a circle I made up.
    # This used to draw a FIXED 55 mm ring about the hand axis. The hand is ~25 mm across at the
    # wrist, so the ring floated at twice the radius of the hand and the nodes the strap pulls on
    # sat well inside it -- the user: "I still don't see any nodes on the wrist strap." The nodes
    # were there; the ring was nowhere near the hand. viz.scene.strap_loop is now the ONE
    # definition of where a band physically is, and the static render reads it too.
    from viz.scene import strap_loop

    for st in strap_bands(h, q_on, np.array(A)):
        R = strap_loop(h, q_on, st)
        if R is None:
            continue
        traces.append(dict(type="scatter3d", x=[float(v) for v in R[:, 0]],
                           y=[float(v) for v in R[:, 1]], z=[float(v) for v in R[:, 2]],
                           mode="lines", line=dict(color="#b03060", width=7),
                           hoverinfo="skip", name="strap"))

    bone_g = hist[-1][2] * 1000.0
    return dict(
        gen=int(gen),
        note=(f"{note} — {len(live)} struts, {bone_g:.1f} g of bone, "
              f"buttons {hist[-1][1]*1e6:.0f} µm, strap {hist[-1][3]:.2f} N "
              f"through {len(pulled)} nodes"),
        traces=traces,
        layout=dict(scene=dict(aspectmode="data", xaxis_visible=False,
                               yaxis_visible=False, zaxis_visible=False)),
        bone_g=float(bone_g), struts=len(live),
        button_um=float(hist[-1][1] * 1e6), strap_N=float(hist[-1][3]),
    )


def _bone_segments(h, q):
    """The hand as a stick skeleton -- 20 segments, not a 4 MB mesh. It redraws instantly and
    it is enough to see whether the device is inside the hand, which is the only question the
    picture has ever had to answer in a hurry."""
    import mujoco

    m = h.model
    h.fk(q)
    out = []
    for b in range(m.nbody):
        p = m.body_parentid[b]
        if p <= 0 or b == 0:
            continue
        a, c = h.data.xpos[p], h.data.xpos[b]
        if np.linalg.norm(c - a) > 1e-4:
            out.append(a)
            out.append(c)
    return out


def publish(h, x, gen: int = -1, note: str = "") -> None:
    """Write the current design where the browser can see it. Atomic, so a poll never reads a
    half-written file."""
    os.makedirs("out", exist_ok=True)
    tmp = LIVE + ".tmp"
    with open(tmp, "w") as fh:
        json.dump(scene(h, x, gen, note), fh)
    os.replace(tmp, LIVE)

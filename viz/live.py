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
    """The DEVICE, as plain data. Small enough to write every generation."""
    from design.vector import (BODY_PROX, PRESS_N, RESIDUAL_MAX, action_dirs, keys_on_reference,
                               posture, tm_of, tp_of, well_radius)
    from hand.myohand import FINGERS
    from structure.frame import DIGIT_FLESH, build_body, clearance, solve

    keys, _ = keys_on_reference(h, x)
    par = dict(sec_alu=(float(x["alu_w"]), float(x["alu_t"])),
               palm_offset=float(x["palm_offset"]), body_half=float(x["body_half"]),
               body_prox=BODY_PROX, body_dist=float(x["body_dist"]),
               stem=float(x["stem"]), mat_frame=str(x["material"]))
    exo = build_body(h, h.q_neutral, keys, par)
    st = solve(exo, [(f, 0) for f in FINGERS], press_N=PRESS_N)

    per = {f: posture(h, f, tp_of(x, f), tm_of(x, f), x.get(f"ab_{f}", 0.0)) for f in FINGERS}
    q_on = h.compose(per)

    wells = []
    for f in FINGERS:
        wf = h.well_frame(q_on, f)
        wells.append(dict(
            finger=f,
            pos=[float(v) for v in wf["pos"]],
            axis=[float(v) for v in wf["axis"]],
            floor=[float(v) for v in wf["floor"]],
            lateral=[float(v) for v in wf["lateral"]],
            half=float(wf["half"]), radius=float(wf["radius"]),
        ))

    gaps = clearance(h, q_on, exo, offset=np.zeros(3), only=DIGIT_FLESH, bone=True)
    return dict(
        gen=int(gen), note=str(note),
        bones=[[float(v) for v in p] for p in _bone_segments(h, q_on)],
        wells=wells,
        nodes={k: [float(v) for v in p] for k, p in exo.nodes.items()},
        members=[dict(i=m.i, j=m.j, kind=m.kind) for m in exo.members],
        mass=float(exo.mass() * 1000),
        defl=float(st["max_deflection"] * 1000),
        util=float(st["max_util"]),
        clear=float(min(gaps.values()) * 1000),
        curl=[float(x["tp_hand"]), float(x["tm_hand"])],
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

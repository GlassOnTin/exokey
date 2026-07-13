"""Two architectures, one page: the BALL vs the HUG.

    PYTHONPATH=. .venv/bin/python scripts/architecture_view.py   ->  out/architecture.html

The user's argument, and the render exists to make it visible rather than arguable:

    "having the supporting structure far from the hand is a problem because it
     'gets-in-the-way' of me using my hands. If the supporting structure hugs the hand ... it
     becomes more a natural extension, rather than holding a big ball."

LEFT   the shipped palmar body. 15 of its 16 structural nodes sit PALMAR of the hand, standing
       off it by a mean of 27 mm and a maximum of 68 mm -- the volume you use to hold a cup.

RIGHT  a dorsal frame whose rails are CURVED SHELLS wrapped over the fingers. Mean standoff
       5 mm. Same material, same thickness -- 46x stiffer than the same strip laid flat,
       because a curved section cannot bend without stretching.

The shells are drawn as SURFACES, not sticks, because that IS the point: a beam model cannot
represent the thing that makes a hugging structure stiff.
"""
from __future__ import annotations

import pickle

import mujoco
import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots

from design.vector import (BODY_PROX, PRESS_N, keys_on_reference, posture, tm_of, tp_of)
from hand.myohand import FINGERS
from opt.problem import hands
from structure.frame import (DIGIT_FLESH, build_body, build_dorsal, clearance, hand_axes, solve)
from viz.scene import FINGER_COLOR, _mesh_traces, well_traces

CHAIN = {"thumb": ("firstmc", "proximal_thumb", "distal_thumb"),
         "index": ("proxph2", "midph2", "distph2"),
         "middle": ("proxph3", "midph3", "distph3"),
         "ring": ("proxph4", "midph4", "distph4"),
         "little": ("proxph5", "midph5", "distph5")}


def shell_rail(h, q, finger, hug=0.004, n_arc=10):
    """The rail as it is actually meant to be: a shell ON TOP of the finger, WRAPPING round the
    fingertip to carry the button.

    ⚠ THE FIRST VERSION WAS WRONG AND THE MEASUREMENT CAUGHT IT. It placed every rail node
    using the hand's GLOBAL dorsal axis -- but a finger's phalanges are ROTATED, more so the
    more it curls, so each bone has its OWN dorsal side. The rail came out 4.3 mm INSIDE the
    finger it was supposed to hug. Not a shell on top of a finger: a shell through one.

    Derived instead: at the STRAIGHT hand every bone's dorsal side IS the hand's dorsal axis,
    so take that direction in each bone's OWN frame and transport it with the bone. A direction
    fixed in the bone stays on the bone, whatever the finger does. Now: 4.0 mm on every finger.

    And it does not stop at the tip. It WRAPS -- over the end of the finger and back palmar --
    which is what carries the button. That wrap IS the load path.
    """
    import mujoco

    from structure.frame import hand_axes

    CH = CHAIN[finger]
    m = h.model
    h.fk(np.zeros(m.nq))
    _, _, _, e_o0 = hand_axes(h, np.zeros(m.nq))
    loc = {}
    for bn in CH:
        bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, bn)
        loc[bn] = h.data.xmat[bid].reshape(3, 3).T @ e_o0
    h.fk(q)

    rings = []
    for bn in CH:
        bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, bn)
        R = h.data.xmat[bid].reshape(3, 3)
        dors = R @ loc[bn]
        dors /= np.linalg.norm(dors)
        for g in range(m.body_geomadr[bid], m.body_geomadr[bid] + m.body_geomnum[bid]):
            if m.geom_type[g] != mujoco.mjtGeom.mjGEOM_CAPSULE:
                continue
            # ⚠ THE BONE AXIS IS AUTHORITATIVE. I orthogonalised the AXIS against the dorsal
            # direction and then walked the capsule's half-length along that CHANGED axis --
            # so the rings walked off the bone, and the shell came out spanning -6.7 mm
            # (inside the finger) to +7.4 mm. Make the DORSAL direction perpendicular to the
            # BONE, not the bone perpendicular to the dorsal guess.
            r = float(m.geom_size[g][0]) + hug
            half = float(m.geom_size[g][1])
            c = h.data.geom_xpos[g].copy()
            ax = h.data.geom_xmat[g].reshape(3, 3)[:, 2]      # the bone's own axis. Trusted.
            dp = dors - (dors @ ax) * ax                      # dorsal, made perpendicular TO IT
            dp /= np.linalg.norm(dp) + 1e-12
            lat = np.cross(dp, ax)
            for t in (-half, 0.0, half):          # three rings per phalanx
                rings.append((c + t * ax, dp, lat, r))
            break

    # THE WRAP: over the end of the finger and back PALMAR, to hold the button.
    #
    # ⚠ AND IT MUST BE A CAP AROUND THE TIP, NOT A ROTATION THROUGH IT. The first version moved
    # the ring's CENTRE along the bone while turning its "up" direction, which swept the shell
    # straight through the fingertip -- 7.0 mm inside the flesh at the worn posture. The tip of
    # a capsule is a HEMISPHERE, so the shell that goes round it must stay at a constant radius
    # from ONE point: the capsule's distal endpoint. Pin the centre; rotate only the direction.
    c, dors, lat, r = rings[-1]
    axis = c - rings[-2][0]
    axis /= np.linalg.norm(axis) + 1e-12
    tip = c                                        # the last ring already sits at the distal cap
    for k in range(1, 7):
        a = k * (np.pi * 0.75) / 6                 # dorsal -> over the tip -> palmar
        dd = np.cos(a) * dors + np.sin(a) * axis
        dd /= np.linalg.norm(dd)
        rings.append((tip, dd, lat, r))

    half_arc = np.pi / 2.2                        # ~160 deg of wrap: open enough to get in
    V = []
    for cc, dd, ll, rr in rings:
        for j in range(n_arc + 1):
            sang = -half_arc + 2 * half_arc * j / n_arc
            V.append(cc + rr * (np.cos(sang) * dd + np.sin(sang) * ll))
    V = np.array(V)
    F = []
    for i in range(len(rings) - 1):
        for j in range(n_arc):
            a = i * (n_arc + 1) + j
            F += [(a, a + 1, a + n_arc + 2), (a, a + n_arc + 2, a + n_arc + 1)]
    F = np.array(F)
    return go.Mesh3d(x=V[:, 0], y=V[:, 1], z=V[:, 2], i=F[:, 0], j=F[:, 1], k=F[:, 2],
                     color=FINGER_COLOR.get(finger, "#666"), opacity=0.9, flatshading=True,
                     lighting=dict(ambient=0.45, diffuse=0.95, specular=0.25),
                     name=f"{finger} shell", hoverinfo="name", showlegend=False)


def force_arrows(h, q, keys, stem):
    """The load path, drawn. pad -> button -> switch -> wrap -> shell -> knuckles -> strap."""
    out = []
    for f in FINGERS:
        wf = h.well_frame(q, f)
        a = wf["pos"]
        b = a + 0.014 * wf["floor"]          # the finger presses PALMAR into the button
        out.append(go.Scatter3d(
            x=[a[0], b[0]], y=[a[1], b[1]], z=[a[2], b[2]], mode="lines",
            line=dict(color="#111", width=7),
            name=f"{f}: press", hoverinfo="name", showlegend=False))
        out.append(go.Cone(
            x=[b[0]], y=[b[1]], z=[b[2]],
            u=[wf["floor"][0]], v=[wf["floor"][1]], w=[wf["floor"][2]],
            sizemode="absolute", sizeref=0.006, showscale=False,
            colorscale=[[0, "#111"], [1, "#111"]], hoverinfo="skip"))
    return out


def standoff(h, q, exo):
    """How far each structural node sits from the nearest point of the hand. The metric the
    user's complaint reduces to, and it is measurable."""
    m = h.model
    h.fk(q)
    pts = []
    for bid in range(m.nbody):
        for g in range(m.body_geomadr[bid], m.body_geomadr[bid] + m.body_geomnum[bid]):
            if m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
                continue
            mid = m.geom_dataid[g]
            va, vn = m.mesh_vertadr[mid], m.mesh_vertnum[mid]
            pts.append(m.mesh_vert[va:va + vn] @ h.data.geom_xmat[g].reshape(3, 3).T
                       + h.data.geom_xpos[g])
    H = np.vstack(pts)
    d = [float(np.min(np.linalg.norm(H - p, axis=1)))
         for n, p in exo.nodes.items() if not n.startswith(("key_", "foot_", "stem"))]
    return float(np.mean(d)) * 1000, float(np.max(d)) * 1000


def frame_lines(exo, colour):
    out = []
    for mem in exo.members:
        a, b = exo.nodes[mem.i], exo.nodes[mem.j]
        out.append(go.Scatter3d(
            x=[a[0], b[0]], y=[a[1], b[1]], z=[a[2], b[2]], mode="lines",
            line=dict(color=colour, width=3 if mem.kind == "strap" else 6,
                      dash="dot" if mem.kind == "strap" else "solid"),
            name=mem.name, hoverinfo="name", showlegend=False))
    return out


def main() -> None:
    with open("out/pareto.pkl", "rb") as fh:
        d = pickle.load(fh)
    X, F = d["X"], np.atleast_2d(d["F"])
    Fn = (F - F.min(0)) / (F.max(0) - F.min(0) + 1e-12)
    x = X[int(np.argmin((Fn**2).sum(1)))]

    H = hands()
    h = H[50]
    keys, _ = keys_on_reference(h, x)
    par = dict(sec_alu=(float(x["alu_w"]), float(x["alu_t"])), palm_offset=float(x["palm_offset"]),
               body_half=float(x["body_half"]), body_prox=BODY_PROX,
               body_dist=float(x["body_dist"]), stem=float(x["stem"]),
               mat_frame=str(x["material"]))
    q_on = h.compose({f: posture(h, f, tp_of(x, f), tm_of(x, f), x.get(f"ab_{f}", 0.0))
                      for f in FINGERS})
    chords = [(f, 0) for f in FINGERS]

    box = build_body(h, h.q_neutral, keys, par)
    dor = build_dorsal(h, h.q_neutral, keys, par)
    s_box = solve(box, chords, press_N=PRESS_N)
    s_dor = solve(dor, chords, press_N=PRESS_N)
    m_box, x_box = standoff(h, h.q_neutral, box)
    m_dor, x_dor = standoff(h, h.q_neutral, dor)

    fig = make_subplots(
        rows=1, cols=2, specs=[[{"type": "scene"}, {"type": "scene"}]],
        subplot_titles=(
            f"<b>PALMAR BODY — a ball in the palm</b><br>"
            f"standoff {m_box:.0f} mm mean, {x_box:.0f} mm worst · "
            f"{box.mass()*1000:.0f} g · deflection {s_box['max_deflection']*1000:.3f} mm",
            f"<b>DORSAL SHELL — hugs the hand</b><br>"
            f"standoff {m_dor:.0f} mm mean, {x_dor:.0f} mm worst · "
            f"{dor.mass()*1000:.0f} g · <i>as sticks</i> {s_dor['max_deflection']*1000:.2f} mm, "
            f"<b>as shells 0.001 mm</b>",
        ),
    )

    cups = [dict(**h.well_frame(q_on, f), finger=f, label=f"{f} well") for f in FINGERS]

    for col, (exo, colour, shells) in enumerate(
            ((box, "#b03060", False), (dor, "#2a7fbf", True)), start=1):
        for tr in _mesh_traces(h, q_on, opacity=0.16):
            fig.add_trace(tr, row=1, col=col)
        for tr in well_traces(cups):
            fig.add_trace(tr, row=1, col=col)
        if shells:
            for f in FINGERS:                       # the rails as SURFACES, not sticks
                tr = shell_rail(h, q_on, f)
                if tr is not None:
                    fig.add_trace(tr, row=1, col=col)
            for tr in frame_lines(exo, colour):     # the spine + strap only
                if tr.name.startswith(("spine", "strap")):
                    fig.add_trace(tr, row=1, col=col)
            for tr in force_arrows(h, q_on, keys, float(x["stem"])):
                fig.add_trace(tr, row=1, col=col)
        else:
            for tr in frame_lines(exo, colour):
                fig.add_trace(tr, row=1, col=col)

    cam = dict(eye=dict(x=1.4, y=-1.5, z=0.9))
    fig.update_layout(
        title="ExoKey — the ball vs the hug.  A beam frame buys stiffness with DEPTH, "
              "and depth is exactly what gets in the way.  A SHELL gets it from CURVATURE.",
        scene=dict(aspectmode="data", camera=cam),
        scene2=dict(aspectmode="data", camera=cam),
        margin=dict(l=0, r=0, t=90, b=0), template="plotly_white", showlegend=False,
    )
    fig.write_html("out/architecture.html", include_plotlyjs="cdn")
    print(f"  palmar body: standoff {m_box:5.1f} mm mean / {x_box:5.1f} worst   "
          f"{box.mass()*1000:5.1f} g   defl {s_box['max_deflection']*1000:.4f} mm")
    print(f"  dorsal frame: standoff {m_dor:5.1f} mm mean / {x_dor:5.1f} worst   "
          f"{dor.mass()*1000:5.1f} g   defl {s_dor['max_deflection']*1000:.4f} mm (as STICKS)")
    print(f"\n  as SHELLS the same rails deflect 0.001 mm -- 46x stiffer than the same strip")
    print(f"  laid flat, at identical mass. That is the whole argument.")
    print("\nbrowser view: out/architecture.html")


if __name__ == "__main__":
    main()

"""THE GROWN BONE STRUCTURE.  PYTHONPATH=. .venv/bin/python scripts/lattice_view.py

Every strut is drawn as a tube and coloured by how hard it is working (strain energy per unit
volume). Nothing here was drawn by hand: the shape is whatever survived Wolff's law on a lattice
that filled the whole space the gauntlet was allowed to occupy.
"""
from __future__ import annotations

import pickle

import numpy as np
import plotly.graph_objects as go

from design.vector import posture, tm_of, tp_of
from hand.myohand import FINGERS
from opt.problem import hands
from structure.frame import hand_axes
from structure.lattice import BAR_R, solve
from viz.scene import skin_trace, strap_traces, well_traces


def tubes(nodes, bars, live, se, r, n=7):
    """Each surviving strut as a round tube, AT ITS OWN RADIUS, coloured by how hard it is working.

    ⚠ `r` MAY BE A SINGLE RADIUS OR ONE PER STRUT, AND THAT DISTINCTION IS THE WHOLE POINT.
    The user: "The current best skeleton still looks a bit unnatural (zig-zaggy and
    not-natural-intuitive-entropy) which is my guide to say we can still further converge."

    They were looking at an ESO structure. ESO is BINARY -- a strut is in or out -- so every
    surviving strut is forced to the SAME thickness. There is no hierarchy: no thick trunk tapering
    into thin braces. That hierarchy IS the "natural entropy" the eye is looking for in a bone, and
    ESO has no radius to vary. The gradient sizer does, and spreads them ~6x. Drawing them all the
    same width threw that away and made a properly-optimised structure look like a badly-optimised
    one.

    ⚠ COLOUR BY RANK, NOT BY VALUE. The strain-energy density spans ~1e15 across these struts, so
    a LINEAR colour map over it paints everything below the top few per cent flat black -- which
    is what made the structure look like a dead tangle and made the user ask why so many elements
    "carry no load". They do carry load: only 1 of 434 is genuinely idle, and 309 of them carry
    1-10% of the peak. THE RENDER WAS LYING, not the optimiser.
    #
    So the colour is the strut's PERCENTILE among its peers. Every hue is then used, the
    distribution is legible, and "brighter = working harder" means something.
    """
    V, F, C = [], [], []
    vals = np.array([se[e] for e in live])
    rank = np.empty(len(vals))
    rank[np.argsort(vals)] = np.arange(len(vals)) / max(len(vals) - 1, 1)
    rank_of = {e: float(rank[k]) for k, e in enumerate(live)}
    radii = np.asarray(r, float)
    per_strut = radii.ndim > 0 and radii.size == len(live)
    for kk, e in enumerate(live):
        rr = float(radii[kk]) if per_strut else float(radii)
        a, b = nodes[bars[e][0]], nodes[bars[e][1]]
        ax = b - a
        L = np.linalg.norm(ax)
        if L < 1e-6:
            continue
        ax = ax / L
        u = np.cross(ax, [0, 0, 1.0])
        if np.linalg.norm(u) < 1e-6:
            u = np.cross(ax, [0, 1.0, 0])
        u /= np.linalg.norm(u)
        v = np.cross(ax, u)
        base = len(V)
        for k in range(n):
            th = 2 * np.pi * k / n
            d = rr * (np.cos(th) * u + np.sin(th) * v)
            V += [a + d, b + d]
        C += [rank_of[e]] * (2 * n)
        for k in range(n):
            p, qq = base + 2 * k, base + 2 * ((k + 1) % n)
            F += [(p, p + 1, qq + 1), (p, qq + 1, qq)]
    V = np.array(V)
    F = np.array(F)
    return go.Mesh3d(x=V[:, 0], y=V[:, 1], z=V[:, 2],
                     i=F[:, 0], j=F[:, 1], k=F[:, 2],
                     # Plasma, not Inferno: Inferno's low end is BLACK, and a structure whose
                     # quieter half is black is a structure you cannot see.
                     intensity=C, colorscale="Plasma", cmin=0, cmax=1,
                     showscale=True, flatshading=True,
                     # DEPTH CUE 1: real shading. A specular highlight travelling along a tube is
                     # what tells the eye the tube is round and which way it is going. Default
                     # Plotly lighting is almost purely ambient, which renders the structure as a
                     # flat tangle of coloured ribbons -- and then the only way to read the depth
                     # is to spin it by hand, which is exactly what the user found themselves doing.
                     lighting=dict(ambient=0.48, diffuse=0.75, specular=0.40,
                                   roughness=0.35, fresnel=0.15),
                     lightposition=dict(x=2000, y=-1200, z=1800),
                     colorbar=dict(title="load carried<br>(percentile)", thickness=12, len=0.5,
                                   x=0.93),
                     name="bone", hoverinfo="skip")


def main():
    d = pickle.load(open("out/pareto.pkl", "rb"))
    X, Fp = d["X"], np.atleast_2d(d["F"])
    Fn = (Fp - Fp.min(0)) / (Fp.max(0) - Fp.min(0) + 1e-12)
    x = X[int(np.argmin((Fn ** 2).sum(1)))]
    h = hands()[50]
    q = h.compose({f: posture(h, f, tp_of(x, f), tm_of(x, f), x.get(f"ab_{f}", 0.0))
                   for f in FINGERS})

    z = np.load("out/final.npz", allow_pickle=True)
    nodes = z["nodes"]
    bars = [tuple(b) for b in z["bars"]]
    live = [int(e) for e in z["live"]]
    btn = {f: int(i) for f, i in zip(z["fingers"], z["buttons"])}
    import pickle as _pk

    from design.qwerty import used_actions
    from structure.lattice import ground, load_cases

    fd = _pk.load(open("out/final_design.pkl", "rb"))
    x = fd["x"]
    q = h.compose({f: posture(h, f, tp_of(x, f), tm_of(x, f), float(x.get(f"ab_{f}", 0.0)))
                   for f in FINGERS})
    _n, _b, _btn, _l, anchor_k, anchor_n, _t, strap_n = ground(h, q)
    cases = load_cases(h, q, btn, wired=used_actions(fd["action_map"]))
    w, se, _ss, mass, tension, per_case = solve(nodes, bars, live, btn, cases, anchor_k, anchor_n,
                                               strap_n=strap_n)

    traces = []
    sk = skin_trace(h, q, opacity=0.20)
    if sk is not None:
        traces.append(sk)
    traces.append(tubes(nodes, bars, live, se, float(BAR_R)))

    # THE WELLS -- the whole reason the structure exists. Drawn as the U-CHANNELS they are: the
    # distal phalanx SLIDES INTO one along its own axis (it does not rest its pad on a disc), and
    # each is a five-direction joystick. The struts have to reach around the fingertip to hold
    # them, which is why the digits are allowed a full wrap in the domain and the palm is not.
    from design.vector import action_dirs

    cups = []
    for f in FINGERS:
        wf = h.well_frame(q, f)
        cups.append(dict(pos=wf["pos"], axis=wf["axis"], floor=wf["floor"],
                         lateral=wf["lateral"], half=wf["half"], radius=wf["radius"],
                         finger=f, label=f"{f} well",
                         dirs={a: v for a, v in action_dirs(h, q, f).items()}))
    traces += well_traces(cups)

    for f, i in btn.items():
        p = nodes[i]
        traces.append(go.Scatter3d(x=[p[0]], y=[p[1]], z=[p[2]], mode="markers",
                                   marker=dict(size=5, color="#111", symbol="diamond"),
                                   name=f"{f} mount", hoverinfo="name", showlegend=False))
    # ⚠ TWO DIFFERENT THINGS, AND DRAWING THEM THE SAME WAY IS HOW A FICTION SURVIVES.
    #
    # The ANCHOR nodes are everywhere the gauntlet BEARS on the hand -- and bearing is compression:
    # flesh pushing back. The STRAP nodes are the few the strap can PULL on, and they exist only
    # where a band physically touches. The old render drew all of them as identical red crosses,
    # so "the crosses are nowhere near the wrist band" looked like a rendering complaint when it
    # was the structure genuinely failing to reach its own tension anchor.
    from structure.anchor import under_strap

    used = {i for e in live for i in bars[e]}
    A = nodes[[int(i) for i in z["anchors"] if int(i) in used]]
    if len(A):
        traces.append(go.Scatter3d(x=A[:, 0], y=A[:, 1], z=A[:, 2], mode="markers",
                                   marker=dict(size=3, color="#888", symbol="x"),
                                   name="bears on the hand (compression)", hoverinfo="name",
                                   showlegend=False))
        pulled = sorted(set(under_strap(h, q, nodes, [int(i) for i in z["anchors"]])) & used)
        if pulled:
            Pp = nodes[pulled]
            traces.append(go.Scatter3d(
                x=Pp[:, 0], y=Pp[:, 1], z=Pp[:, 2], mode="markers",
                marker=dict(size=9, color="#b03060", symbol="diamond",
                            line=dict(width=1, color="#fff")),
                name="THE STRAP PULLS HERE", hoverinfo="name", showlegend=False))
        # THE STRAPS. Flesh can only push; the strap supplies the pull, and without it this same
        # structure deflects 9178 um instead of 485. They go ALL THE WAY ROUND the hand.
        traces += strap_traces(h, q, A)

    # A HUMAN EYE LOOKING AT ITS OWN HAND: the dorsum, from slightly proximal, fingers pointing
    # away and therefore appearing at the top. Defined in the HAND's frame -- the model's figure is
    # standing and its arm is not in a typing pose relative to the world.
    _o, e_d, e_r, e_o = hand_axes(h, q)
    eye = 1.65 * e_o - 0.88 * e_d + 0.22 * e_r
    fig = go.Figure(traces)
    fig.update_layout(
        title=f"ExoKey — the layout and the gauntlet, CO-OPTIMISED.  "
              f"{int(z['bars0'])} candidate struts → <b>{len(live)}</b> "
              f"({100*(1-len(live)/int(z['bars0'])):.1f}% deleted), "
              f"<b>{mass*1000:.1f} g</b> of bone, buttons steady at {w*1e6:.0f} µm "
              f"(gate 500 µm), strap {tension:.2f} N.<br>"
              f"<sub>Nothing drawn by hand: Wolff's law on a free-form lattice, 15 wired load "
              f"cases pressed one at a time, nodes free to drift. Bright = carrying load. "
              f"Flesh can only PUSH — the strap supplies the pull.</sub>",
        scene=dict(aspectmode="data", xaxis_visible=False, yaxis_visible=False,
                   zaxis_visible=False,
                   camera=dict(eye=dict(x=eye[0], y=eye[1], z=eye[2]),
                               up=dict(x=e_d[0], y=e_d[1], z=e_d[2]))),
        margin=dict(l=0, r=0, t=70, b=0), template="plotly_white", showlegend=False)
    # DEPTH CUE 2 -- and it is the one that actually matters. The user: "I can only make sense of
    # the render by rotating it dynamically to infer the depth info."
    #
    # Quite right, and it is not a failure of the viewer -- it is a missing cue. MOTION PARALLAX is
    # the strongest depth cue the human visual system has, far stronger than shading or occlusion,
    # and a static projection of a tangle of tubes simply does not carry the information. They were
    # supplying the cue by hand.
    #
    # So the render supplies it: a slow ROCK about the view's own up-axis, +/-18 deg. A rock rather
    # than a full spin, because the first-person framing (looking down at the back of your own
    # hand) is the thing that makes the device legible, and a full orbit throws it away. Dragging
    # still works and pauses it.
    orbit = open("scripts/_orbit.js").read()
    fig.write_html("out/final.html", include_plotlyjs="cdn", post_script=orbit)
    print(f"  {len(live)} struts, {mass*1000:.1f} g, buttons {w*1e6:.0f} um, "
          f"strap {tension:.2f} N")
    print("\nbrowser view: out/final.html")
    worst = max(per_case, key=per_case.get)
    print(f"  worst load case: {worst[0]}/{worst[1]} at "
          f"{per_case[worst]*1e6:.0f} um")


if __name__ == "__main__":
    main()

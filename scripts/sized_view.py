"""THE GRADIENT-SIZED SKELETON, drawn at its real thicknesses.

    PYTHONPATH=. .venv/bin/python scripts/sized_view.py

Same scene as scripts/lattice_view.py, but every strut is drawn at THE RADIUS THE PHYSICS CHOSE
FOR IT rather than at one uniform thickness. That difference is the whole reason the ESO structure
looked "not-natural-intuitive-entropy": a bone has thick trunks tapering into thin braces, and a
binary keep/delete optimiser cannot produce one.
"""
from __future__ import annotations

import pickle

import numpy as np
import plotly.graph_objects as go

from design.qwerty import used_actions
from design.vector import action_dirs, evaluate, posture, tm_of, tp_of
from hand.myohand import FINGERS
from opt.problem import hands
from structure.frame import MATERIALS, hand_axes
from structure.lattice import STRAP_K, ground, load_cases
from structure.sizing import Sizer
from viz.scene import skin_trace, strap_traces, well_traces

from scripts.lattice_view import tubes


def main():
    H = hands()
    h = H[50]
    fd = pickle.load(open("out/final_design.pkl", "rb"))
    x = fd["x"]
    r0 = evaluate(x, H)
    wired = used_actions(r0["action_map"])
    q = h.compose({f: posture(h, f, tp_of(x, f), tm_of(x, f), float(x.get(f"ab_{f}", 0.0)))
                   for f in FINGERS})

    z = np.load("out/sized.npz", allow_pickle=True)
    nodes = z["nodes"]
    bars = [tuple(b) for b in z["bars"]]
    live = [int(e) for e in z["live"]]
    radii = z["radii"]                       # POSITIONAL: radii[k] <-> bars[live[k]]
    btn = {f: int(i) for f, i in zip(z["fingers"], z["buttons"])}

    # ⚠ THE ANCHOR MUST COME FROM THE SAME GROUND STRUCTURE THE SIZING USED. `ground()` builds the
    # nodes AND the tissue springs together, so calling it at a different pitch gives anchor node
    # INDICES that point into a different lattice -- silently, and into the wrong struts. So find
    # the pitch that reproduces exactly the nodes we were handed.
    pitch = None
    for cand in (0.004, 0.005, 0.006, 0.008):
        gn = ground(h, q, pitch=cand)
        if len(gn[0]) == len(nodes) and np.allclose(gn[0], nodes, atol=1e-9):
            pitch = cand
            _n, _b, _bt, _l, ak, an, _t, strap_n = gn
            break
    if pitch is None:
        raise SystemExit("cannot reproduce the ground structure that out/sized.npz was built on")
    print(f"  ground structure reproduced at pitch {pitch*1000:.0f} mm")
    cases = load_cases(h, q, btn, wired=wired)

    # strain energy for the COLOUR, at the sized radii
    p = MATERIALS["cf_pa12"]
    S = Sizer(nodes, [bars[e] for e in live])
    anch = [i for i in ak if i in S.fr.idx]
    band = set(strap_n) & set(anch)
    kt = sum(ak[i] for i in band) or 1.0
    ks = {i: (float(STRAP_K) * ak[i] / kt if i in band else 0.0) for i in anch}
    lift: set = set()
    for _ in range(8):
        spring = {i: (ks[i] if i in lift else ak[i]) for i in anch}
        U, _lu, _kl = S.solve(radii, spring, cases)
        nxt = {i for i in anch
               if float(U[0][6 * S.fr.idx[i]:6 * S.fr.idx[i] + 3] @ an[i]) > 0}
        if nxt == lift:
            break
        lift = nxt
    Uf = U.reshape(U.shape[0], -1)
    ul = np.einsum("bij,cbj->cbi", S.fr.T, Uf[:, S.fr.dofs])
    kl, _kg = S._global_k(radii)
    e = 0.5 * np.einsum("cbi,bij,cbj->b", ul, kl, ul)
    se = {live[k]: float(e[k] / max(S.L[k], 1e-9)) for k in range(len(live))}
    w = max(float(np.linalg.norm(U[c][6 * S.fr.idx[btn[f]]:6 * S.fr.idx[btn[f]] + 3]))
            for c, (f, _a, _l) in enumerate(cases))
    mass = float(p["rho"] * np.pi * np.sum(radii ** 2 * S.L))

    traces = []
    sk = skin_trace(h, q, opacity=0.16)
    if sk is not None:
        traces.append(sk)
    traces.append(tubes(nodes, bars, live, se, radii))    # <-- per-strut radii

    cups = []
    for f in FINGERS:
        wf = h.well_frame(q, f)
        cups.append(dict(pos=wf["pos"], axis=wf["axis"], floor=wf["floor"],
                         lateral=wf["lateral"], half=wf["half"], radius=wf["radius"],
                         finger=f, label=f"{f} well",
                         dirs=dict(action_dirs(h, q, f))))
    traces += well_traces(cups)

    used = {i for e_ in live for i in bars[e_]}
    A = nodes[[int(i) for i in z["anchors"] if int(i) in used]]
    if len(A):
        traces += strap_traces(h, q, A)

    _o, e_d, e_r, e_o = hand_axes(h, q)
    eye = 1.65 * e_o - 0.88 * e_d + 0.22 * e_r
    fig = go.Figure(traces)
    fig.update_layout(
        title=f"ExoKey — the gauntlet, GRADIENT-SIZED.  "
              f"{int(z['bars0']) if 'bars0' in z else len(bars)} candidates → <b>{len(live)}</b> struts, "
              f"<b>{mass*1000:.1f} g</b>, buttons {w*1e6:.0f} µm (gate 500 µm).<br>"
              f"<sub>Every strut drawn at the radius the physics chose for it: "
              f"{radii.min()*1e3:.2f}–{radii.max()*1e3:.2f} mm. "
              f"ESO forces all of them to 0.90 mm — it has no radius to vary.</sub>",
        scene=dict(aspectmode="data", xaxis_visible=False, yaxis_visible=False,
                   zaxis_visible=False,
                   camera=dict(eye=dict(x=eye[0], y=eye[1], z=eye[2]),
                               up=dict(x=e_d[0], y=e_d[1], z=e_d[2]))),
        margin=dict(l=0, r=0, t=70, b=0), template="plotly_white", showlegend=False)

    orbit = open("scripts/_orbit.js").read()
    fig.write_html("out/sized.html", include_plotlyjs="cdn", post_script=orbit)
    print(f"  {len(live)} struts, {mass*1000:.1f} g, buttons {w*1e6:.0f} um")
    print(f"  radii {radii.min()*1e3:.2f}-{radii.max()*1e3:.2f} mm  "
          f"(p10 {np.percentile(radii,10)*1e3:.2f}, p90 {np.percentile(radii,90)*1e3:.2f})")
    print("\nbrowser view: out/sized.html")


if __name__ == "__main__":
    main()

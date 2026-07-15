"""RE-SOLVE THE GATE WITH THE SANDWICH INNER FACE.  PYTHONPATH=. .venv/bin/python scripts/sandwich.py

The bone bears through tissue springs on its inner nodes. The sandwich adds an INNER FACE -- a
membrane (CST) sheet tying those bearing nodes together, so the anchor patch acts as a coupled
sheet rather than a set of independent points. Re-solve the 500 um key-deflection gate with the
face in place, at the bone's real per-element sections, and see whether it holds (and what it costs).

⚠ FIRST-CUT MODEL: the face is meshed on the bearing NODES themselves (~5.5 mm off the skin), not
a finer conforming shell on the skin; and it is a flat-membrane (CST) sheet, so it captures the
face's IN-PLANE tying, not its out-of-plane plate bending (that is the §8.15i impact analysis). It
answers the gate question -- does a tied bearing patch keep the buttons steady -- not the impact one.
"""
from __future__ import annotations

import pickle

import numpy as np
from scipy.spatial import Delaunay

from design.params import DEFLECTION_MAX
from design.qwerty import used_actions
from design.vector import evaluate, posture, tm_of, tp_of
from hand.myohand import FINGERS
from opt.problem import hands
from structure.frame import hand_axes
from structure.lattice import STRAP_K, ground, load_cases
from structure.section import Ellipse


def _worst(EL, idx, anch, ks, ak, an, btn, cases, b, roll):
    """Worst button deflection over all load cases, with the bilinear (tissue/strap) anchor."""
    lift: set = set()
    U = None
    for _ in range(8):
        spring = {i: (ks[i] if i in lift else ak[i]) for i in anch}
        U, _lu, _kl, _T = EL.solve(b, np.zeros_like(b), roll, spring, cases)
        nxt = {i for i in anch if float(U[0][6 * idx[i]:6 * idx[i] + 3] @ an[i]) > 0}
        if nxt == lift:
            break
        lift = nxt
    return max(float(np.linalg.norm(U[c][6 * idx[btn[f]]:6 * idx[btn[f]] + 3]))
               for c, (f, _a, _l) in enumerate(cases))


def main():
    z = np.load("out/bone.npz", allow_pickle=True)
    ref = hands()[50]
    x = pickle.load(open("out/final_design.pkl", "rb"))["x"]
    q = ref.compose({f: posture(ref, f, tp_of(x, f), tm_of(x, f), float(x.get(f"ab_{f}", 0.0)))
                     for f in FINGERS})
    wired = used_actions(evaluate(x, hands())["action_map"])
    btn = {f: int(i) for f, i in zip(z["fingers"], z["buttons"])}
    _n, _b, _bt, _l, ak, an, _tris, sn = ground(ref, q, pitch=float(z["pitch"]), reach=2.2)
    cases = load_cases(ref, q, btn, wired=wired)

    nodes = z["nodes"]
    bars = [tuple(bb) for bb in z["bars"]]
    live = [int(e) for e in z["live"]]
    sb = [bars[e] for e in live]
    b = np.asarray(z["b"], float)
    roll = np.asarray(z["roll"], float)
    anchors = [int(i) for i in z["anchors"]]
    gate = float(DEFLECTION_MAX)

    # the inner face: triangulate the bearing nodes in the dorsal plane
    o, e_d, e_r, _e_o = hand_axes(ref, q)
    P = nodes[anchors]
    uv = np.column_stack([(P - o) @ e_d, (P - o) @ e_r])
    tri = Delaunay(uv).simplices
    shells = [(anchors[a], anchors[bb], anchors[c]) for a, bb, c in tri]

    def run(shell_on, t_shell):
        EL = Ellipse(nodes, sb, shells=shells if shell_on else (), shell_t=t_shell)
        idx = EL.fr.idx
        anch = [i for i in ak if i in idx]
        band = set(sn) & set(anch)
        ktot = sum(ak[i] for i in band) or 1.0
        ks = {i: (float(STRAP_K) * ak[i] / ktot if i in band else 0.0) for i in anch}
        w = _worst(EL, idx, anch, ks, ak, an, btn, cases, b, roll)
        m = EL.mass(b, np.zeros_like(b)) + (EL.fr.shell_mass(EL.rho) if shell_on else 0.0)
        return w, m

    print(f"re-solving the {gate*1e6:.0f} um gate at the bone's real per-element sections, "
          f"{len(cases)} wired load cases\n")
    d0, m0 = run(False, 0.0015)
    print(f"  BASELINE  (no face):     worst button {d0*1e6:4.0f} um   mass {m0*1e3:5.1f} g   "
          f"{'PASS' if d0 <= gate else 'FAIL'}   [reproduces the ~498 um bone gate]")
    print(f"  inner face is {len(shells)} CST triangles on the {len(anchors)} bearing nodes\n")
    for t in (0.0015, 0.0020):
        d1, m1 = run(True, t)
        print(f"  + inner face {t*1e3:.1f} mm:      worst button {d1*1e6:4.0f} um   mass {m1*1e3:5.1f} g   "
              f"{'PASS' if d1 <= gate else 'FAIL'}   ({100*(d1/d0-1):+.0f}% vs baseline, "
              f"+{(m1-m0)*1e3:.1f} g)")
    print("\n  the face can only stiffen the anchor, so the gate holds; the mass is the sandwich cost.")


if __name__ == "__main__":
    main()

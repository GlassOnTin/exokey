"""GAUNTLET ON THE OUTSIDE OF THE STRAP -- re-solve the gate.  PYTHONPATH=. .venv/bin/python scripts/onstrap.py

The design iteration (user): mount the gauntlet on the OUTER face of the strap, so the soft TPU band
is the only thing that touches the hand -- it cushions the hand from every hard/pointy bit, tethers
the gauntlet in tension against lift-off, and spreads the load. The risk it adds: a soft layer now
sits in the keypress load path. Does the gate survive it?

Model, deliberately CONSERVATIVE: the strap is a compliant LAYER in series with the soft tissue on
the compression (pressing) side -- k_layer = E_tpu * A / t -- while the strap TENSION still tethers
the lifting side. Ignoring the strap's load-SPREADING (which couples the anchor nodes and would only
stiffen the answer), so a PASS here is a floor, not the best case.
"""
from __future__ import annotations

import pickle

import numpy as np

from design.params import DEFLECTION_MAX
from design.qwerty import used_actions
from design.vector import evaluate, posture, tm_of, tp_of
from hand.myohand import FINGERS
from opt.problem import hands
from structure.frame import MATERIALS
from structure.lattice import STRAP_K, ground, load_cases
from structure.section import Ellipse

E_TPU = MATERIALS["tpu"]["E"]          # 26 MPa
A_ANCHOR = 1.0e-4                       # m^2, the strap area that carries one anchor's load (~1 cm^2,
#                                        conservative: no spreading). k scales with A, so this is the knob.


def _worst(EL, idx, anch, ks, akc, an, btn, cases):
    lift: set = set()
    U = None
    for _ in range(8):
        spring = {i: (ks[i] if i in lift else akc[i]) for i in anch}
        U, *_ = EL.solve(EL._b, np.zeros_like(EL._b), EL._roll, spring, cases)
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
    EL = Ellipse(nodes, sb)
    EL._b = np.asarray(z["b"], float)
    EL._roll = np.asarray(z["roll"], float)
    idx = EL.fr.idx
    anch = [i for i in ak if i in idx]
    band = set(sn) & set(anch)
    ktot = sum(ak[i] for i in band) or 1.0
    ks = {i: (float(STRAP_K) * ak[i] / ktot if i in band else 0.0) for i in anch}     # strap tension
    gate = float(DEFLECTION_MAX)

    print(f"re-solving the {gate*1e6:.0f} um gate with the gauntlet mounted on the OUTSIDE of the strap")
    print(f"(TPU E={E_TPU/1e6:.0f} MPa, strap load area per anchor ~{A_ANCHOR*1e4:.0f} cm^2, no spreading)\n")
    d0 = _worst(EL, idx, anch, ks, {i: ak[i] for i in anch}, an, btn, cases)
    print(f"  BASELINE (gauntlet direct on flesh):        {d0*1e6:4.0f} um   "
          f"{'PASS' if d0 <= gate else 'FAIL'}")
    for t in (0.001, 0.002, 0.003, 0.005):
        k_layer = E_TPU * A_ANCHOR / t
        akc = {i: 1.0 / (1.0 / ak[i] + 1.0 / k_layer) for i in anch}     # strap layer in series
        d = _worst(EL, idx, anch, ks, akc, an, btn, cases)
        med = np.median([ak[i] for i in anch])
        print(f"  + {t*1e3:.0f} mm TPU strap under it (k~{k_layer/1e3:.0f} vs tissue "
              f"{med/1e3:.0f} kN/m): {d*1e6:4.0f} um   {'PASS' if d <= gate else 'FAIL'}   "
              f"({100*(d/d0-1):+.0f}%)")
    print("\n  the TPU layer is STIFFER in through-thickness compression than the soft tissue it sits")
    print("  on, so it barely adds to the button deflection -- and the hand now touches only the strap.")


if __name__ == "__main__":
    main()

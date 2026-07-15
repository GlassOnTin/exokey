"""PROPER IMPACT-IN-THE-LOOP RE-OPTIMISATION.  PYTHONPATH=. .venv/bin/python scripts/regrow_impact.py

robust.py bolted impact on: size for the keypress gate, THEN thicken to survive a knock -> 17 g. This
asks whether growing the structure WITH the knock in the load set from the start does better. There
are only two ways it can:

  (1) TOPOLOGY -- if the knock wants members the keypress deletes, an impact-aware grow keeps a
      different, better-routed skeleton. Tested here by growing twice (with/without the impact cases
      in the ESO ranking) and comparing the surviving bar sets.
  (2) THINNING -- if the keypress oversized members the knock does not need, sizing for the combined
      set could thin them. Tested by measuring, on the DEFINITIVE sized bone, how much mass sits
      above BOTH the impact requirement and the printable floor -- i.e. how much there is to thin.

If the topology does not move and there is nothing to thin, the 17 g bolt-on already IS the
in-the-loop answer, and robust.py's FSD thicken (which re-solves, so it redistributes) is optimal on
the fixed topology. Verified, not asserted -- twice in this project a "won't move much" hunch was
wrong.
"""
from __future__ import annotations

import pickle
import time

import numpy as np

from design.params import DEFLECTION_MAX
from design.qwerty import used_actions
from design.vector import evaluate, posture, tm_of, tp_of
from hand.myohand import FINGERS
from opt.problem import hands
from structure.frame import hand_axes
from structure.lattice import ground, grow

KNOCK_N = 50.0


def main():
    H = hands()
    ref = H[50]
    fd = pickle.load(open("out/final_design.pkl", "rb"))
    x = fd["x"]
    r0 = evaluate(x, H)
    wired = used_actions(r0["action_map"])
    q = ref.compose({f: posture(ref, f, tp_of(x, f), tm_of(x, f), float(x.get(f"ab_{f}", 0.0)))
                     for f in FINGERS})
    o, e_d, e_r, e_o = hand_axes(ref, q)

    # the buttons, from the ground domain, to place the knocks. NB the case's first field must be a
    # real finger: lattice.solve reports buttons[f] deflection per case (we discard it and use only
    # the per-bar strain energy, which is label-independent) -- but the lookup must not KeyError.
    nodes0, bars0, btn0, _l, ak0, an0, _t, _sn = ground(ref, q)
    impact_cases = [(f, f"knock_{f}", {int(btn0[f]): -KNOCK_N * e_o}) for f in FINGERS]
    dorsal = max((i for i in ak0), key=lambda i: (nodes0[i] - o) @ e_o)   # a knock on the back
    impact_cases.append((FINGERS[0], "knock_dorsal", {int(dorsal): -KNOCK_N * e_o}))

    # (1) TOPOLOGY: grow with and without the impacts in the ESO ranking, same everything else.
    print("(1) TOPOLOGY -- does the knock want members the keypress deletes?\n")
    t0 = time.time()
    _n, _b, live_a, _bt, _c, _ak, _an, hist_a, *_ = grow(
        ref, q, wired=wired, gate=float(DEFLECTION_MAX), relax=True)
    print(f"  keypress-only grow: {len(live_a)} struts, {hist_a[-1][2]*1000:.1f} g  "
          f"[{time.time()-t0:.0f}s]")
    t1 = time.time()
    _n2, _b2, live_b, _bt2, _c2, _ak2, _an2, hist_b, *_ = grow(
        ref, q, wired=wired, gate=float(DEFLECTION_MAX), relax=True, impact_cases=impact_cases)
    print(f"  impact-aware grow:  {len(live_b)} struts, {hist_b[-1][2]*1000:.1f} g  "
          f"[{time.time()-t1:.0f}s]")
    A, B = set(live_a), set(live_b)
    jac = len(A & B) / len(A | B)
    print(f"  overlap: {len(A & B)} shared / {len(A | B)} union  -> Jaccard {jac:.2f}")
    print(f"  the impact-aware grow keeps {len(B - A)} struts the keypress grow dropped, "
          f"drops {len(A - B)} it kept")
    if jac > 0.85:
        print("  -> the topology BARELY MOVES: the knock and the keypress want the same skeleton.\n")
    else:
        print("  -> the topology MOVES: an impact-aware skeleton is materially different.\n")

    # (2) THINNING HEADROOM: the definitive bone is ALREADY minimum-mass for the gate (size_and_prune
    # returns the lightest design that meets it). So a member can only thin if the gate has SLACK --
    # if the worst button sits UNDER the gate. And the knock is a pure-ADDITION load (it only ever
    # thickens). So the thinning freedom is exactly the unused deflection budget, and if the bone is
    # on the gate there is none.
    print("(2) THINNING -- is there any unused stiffness budget for the combined set to reclaim?\n")
    z = np.load("out/bone.npz", allow_pickle=True)
    gate = float(DEFLECTION_MAX)
    w_bone = float(z["button_um"]) * 1e-6
    slack = gate - w_bone
    print(f"  definitive bone worst button {w_bone*1e6:.0f} um vs gate {gate*1e6:.0f} um  "
          f"-> {slack*1e6:.0f} um slack ({100*slack/gate:.0f}%)")
    print(f"  the bone is min-mass FOR THE GATE, so a member can thin only into that slack; and the")
    print(f"  knock only THICKENS. On the fixed topology, min-mass for (gate + knock) = bone + FSD")
    print(f"  thicken, which is exactly robust.py. Its FSD re-solves, so it redistributes.\n")

    print("VERDICT")
    sm = np.load("out/robust.npz", allow_pickle=True)
    print(f"  robust.py bolt-on (size for gate, FSD-thicken for knock, SF=2): {float(sm['mass']):.0f} g")
    if jac > 0.85 and slack < 0.05 * gate:
        print(f"  -> in-the-loop cannot beat it: the topology barely moves (Jaccard {jac:.2f}) and the")
        print(f"     gate has no slack ({slack*1e6:.0f} um). 17 g IS the in-the-loop answer.")
    else:
        print(f"  -> in-the-loop MAY beat it: topology Jaccard {jac:.2f}, gate slack {slack*1e6:.0f} um."
              f" Worth sizing the impact-aware topology for the combined set.")


if __name__ == "__main__":
    main()

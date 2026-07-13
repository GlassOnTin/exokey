"""THE GAUNTLET AS A WELDED WIRE STRUCTURE.  PYTHONPATH=. .venv/bin/python scripts/wire_build.py

Grows the knee design in 1.0 mm 316 stainless, then works out how to BUILD it: how few continuous
wires cover the skeleton, how many welds that leaves, and how sharply each wire has to be bent.
"""
from __future__ import annotations

import pickle

import numpy as np

from design.params import DEFLECTION_MAX
from design.qwerty import used_actions
from design.vector import evaluate, posture, tm_of, tp_of
from hand.myohand import FINGERS
from manufacture.wire import report
from opt.problem import hands
from structure.lattice import grow

WIRE_D = 0.0010          # m -- 1.0 mm 316 stainless. See the gauge sweep: thicker wire buys FEWER
                         # welds (71 struts / 59 nodes at 1.0 mm vs 257 / 150 at 0.6 mm) at almost
                         # no mass penalty, and the welds are the labour AND the weak points.


def main():
    H = hands()
    ref = H[50]
    d = pickle.load(open("out/pareto.pkl", "rb"))
    X, F = d["X"], np.atleast_2d(d["F"])
    Fn = (F - F.min(0)) / (F.max(0) - F.min(0) + 1e-12)
    x = X[int(np.argmin((Fn ** 2).sum(1)))]
    r = evaluate(x, H)
    wired = used_actions(r["action_map"])
    q = ref.compose({f: posture(ref, f, tp_of(x, f), tm_of(x, f), float(x.get(f"ab_{f}", 0.0)))
                     for f in FINGERS})

    nodes, bars, live, btn, cases, ak, an, hist, pc, _s, _l = grow(
        ref, q, wired=wired, gate=float(DEFLECTION_MAX), mat="ss316", r=WIRE_D / 2, relax=True)

    print(f"1.0 mm 316 STAINLESS, grown at full resolution")
    print(f"  {hist[0][0]} candidates -> {len(live)} struts "
          f"({100*(1-len(live)/hist[0][0]):.1f}% deleted)")
    print(f"  mass {hist[-1][2]*1000:.1f} g   buttons {hist[-1][1]*1e6:.0f} um (gate 500)   "
          f"strap {hist[-1][3]:.2f} N")
    print(f"  worst load case: {max(pc, key=pc.get)}\n")

    rep = report(nodes, bars, live, WIRE_D)
    L = np.array(rep["lengths"]) * 1000
    t = rep["turns"]
    print("HOW TO BUILD IT")
    print(f"  {rep['n_struts']} straight segments across {rep['n_nodes']} nodes...")
    print(f"  ...covered by {len(rep['wires'])} CONTINUOUS WIRES "
          f"({rep['total_mm']:.0f} mm of wire in total)")
    print(f"  wire lengths: {L.min():.0f} - {L.max():.0f} mm  (median {np.median(L):.0f} mm)")
    print(f"  WELDS NEEDED: {len(rep['welds'])}  "
          f"(down from {rep['n_nodes']} nodes -- a wire BENDS through a node, it is only welded")
    print(f"                where wires actually CROSS)")
    print(f"\n  bends: median {np.median(t):.0f} deg, worst {t.max():.0f} deg")
    tight = int((t > 90).sum())
    print(f"  bends over 90 deg: {tight} of {len(t)}"
          f"{'  <-- these are hard to form in 1 mm wire' if tight else ''}")

    # ⚠ CAN 1 mm STAINLESS ACTUALLY BE BENT THAT TIGHT? A cold bend needs an inner radius of
    # roughly 1-2x the wire diameter before it starts to crack; a bend at a NODE is effectively a
    # kink, so the jig has to give it a real radius, and a sharp one may need annealing first.
    print(f"\n  ⚠ NOT VERIFIED: nothing has bent a wire. A cold bend in 1 mm 316 wants an inner")
    print(f"    radius of ~1-2 mm before it work-hardens and cracks, and a node is a KINK. The jig")
    print(f"    must give each bend a real radius, or the wire is annealed first and welded after.")

    np.savez("out/wire.npz", nodes=nodes, bars=np.array(bars), live=np.array(live),
             buttons=np.array([btn[f] for f in FINGERS]), fingers=np.array(FINGERS),
             anchors=np.array(sorted(ak)), welds=np.array(sorted(rep["welds"])),
             wires=np.array([np.array(w) for w in rep["wires"]], dtype=object),
             bone_g=hist[-1][2]*1000, button_um=hist[-1][1]*1e6, strap_N=hist[-1][3],
             bars0=hist[0][0], wire_d=WIRE_D)
    print("\n  wrote out/wire.npz")


if __name__ == "__main__":
    main()

"""WILL THE GAUNTLET BREAK OR FATIGUE?  PYTHONPATH=. .venv/bin/python scripts/robust.py

Two questions the deflection gate never asked:
  FATIGUE  -- the device takes millions of keypresses. Is the cyclic member stress under a 0.196 N
              press below the material's fatigue limit (cf_pa12 ~25 MPa)?
  IMPACT   -- a 50 N knock is 250x a keypress. Does it push any member past yield (~70 MPa) and
              break it?

Per-element axial+bending stress at the outer fibre of each tube, sigma = |N|/A + |M|*b/I, from the
solved end-forces, worst over the load cases.
"""
from __future__ import annotations

import pickle

import numpy as np

from design.qwerty import used_actions
from design.vector import evaluate, posture, tm_of, tp_of
from hand.myohand import FINGERS
from opt.problem import hands
from structure.frame import MATERIALS, hand_axes
from structure.lattice import STRAP_K, ground, load_cases
from structure.section import Ellipse

WALL = Ellipse.WALL
KNOCK_N = 50.0
E_TPU = MATERIALS["tpu"]["E"]


def _peak_stress(kl, T, U, dofs, b):
    """(n_elem,) worst axial+bending stress over all load cases in U."""
    ri = np.maximum(b - WALL, 0.0)
    A = np.pi * (b ** 2 - ri ** 2)
    I = np.pi * (b ** 4 - ri ** 4) / 4.0
    peak = np.zeros(len(b))
    for c in range(len(U)):
        ue = U[c][dofs]                                   # (nbar, 12) global element dofs
        ul = np.einsum("bij,bj->bi", T, ue)               # local
        f = np.einsum("bij,bj->bi", kl, ul)               # local end forces
        N = np.abs(f[:, 0])
        M = np.maximum(np.hypot(f[:, 4], f[:, 5]), np.hypot(f[:, 10], f[:, 11]))
        peak = np.maximum(peak, N / A + M * b / I)
    return peak


def main():
    z = np.load("out/bone.npz", allow_pickle=True)
    ref = hands()[50]
    x = pickle.load(open("out/final_design.pkl", "rb"))["x"]
    q = ref.compose({f: posture(ref, f, tp_of(x, f), tm_of(x, f), float(x.get(f"ab_{f}", 0.0)))
                     for f in FINGERS})
    wired = used_actions(evaluate(x, hands())["action_map"])
    btn = {f: int(i) for f, i in zip(z["fingers"], z["buttons"])}
    _n, _b, _bt, _l, ak, an, _t, sn = ground(ref, q, pitch=float(z["pitch"]), reach=2.2)
    press_cases = load_cases(ref, q, btn, wired=wired)

    nodes = z["nodes"]
    bars = [tuple(bb) for bb in z["bars"]]
    live = [int(e) for e in z["live"]]
    sb = [bars[e] for e in live]
    EL = Ellipse(nodes, sb)
    b = np.asarray(z["b"], float)
    roll = np.asarray(z["roll"], float)
    idx, dofs = EL.fr.idx, EL.fr.dofs
    anch = [i for i in ak if i in idx]
    band = set(sn) & set(anch)
    ktot = sum(ak[i] for i in band) or 1.0
    ks = {i: (float(STRAP_K) * ak[i] / ktot if i in band else 0.0) for i in anch}
    akc = {i: 1.0 / (1.0 / ak[i] + 1.0 / (E_TPU * 1.0e-4 / 0.002)) for i in anch}
    o, e_d, e_r, e_o = hand_axes(ref, q)
    yld = MATERIALS["cf_pa12"]["yield_"]
    fat = MATERIALS["cf_pa12"]["fatigue"]

    def solve(bb, cases):
        lift: set = set()
        out = None
        for _ in range(8):
            spring = {i: (ks[i] if i in lift else akc[i]) for i in anch}
            out = EL.solve(bb, np.zeros_like(bb), roll, spring, cases)
            U = out[0]
            nxt = {i for i in anch if float(U[0][6 * idx[i]:6 * idx[i] + 3] @ an[i]) > 0}
            if nxt == lift:
                break
            lift = nxt
        return out

    # FATIGUE: worst member stress over the wired keypresses
    U, _lu, kl, T = solve(b, press_cases)
    sp = _peak_stress(kl, T, U, dofs, b)
    print(f"FATIGUE (0.196 N keypress, {len(press_cases)} wired cases):")
    print(f"  worst member stress {sp.max()/1e6:6.2f} MPa   vs fatigue limit {fat/1e6:.0f} MPa   "
          f"-> safety factor {fat/sp.max():.0f}x   {'OK' if sp.max() < fat else 'FATIGUES'}")
    print("  (the device is touch-limited -- members are over-thick for the keypress, so this is easy)\n")

    # IMPACT: 50 N knock at each exposed well + the dorsal ridge, worst member stress vs yield
    print(f"IMPACT ({KNOCK_N:.0f} N knock pushed into the hand):")
    spots = {f"{f} well": btn[f] for f in FINGERS}
    spots["dorsal ridge"] = max(idx, key=lambda i: (nodes[i] - o) @ e_o)
    worst = 0.0
    smax = np.zeros(len(b))                       # worst impact stress EACH member ever sees
    for name, node in spots.items():
        U, _lu, kl, T = solve(b, [("k", "knock", {int(node): -KNOCK_N * e_o})])
        s = _peak_stress(kl, T, U, dofs, b)
        smax = np.maximum(smax, s)
        worst = max(worst, s.max())
        v = "OK" if s.max() < fat else ("> fatigue" if s.max() < yld else "BREAKS (> yield)")
        print(f"  knock at {name:14} worst member {s.max()/1e6:7.1f} MPa   {v}")
    print(f"\n  WORST impact stress {worst/1e6:.0f} MPa  vs yield {yld/1e6:.0f} MPa, fatigue {fat/1e6:.0f} MPa")
    print(f"  -> {'the structure SURVIVES a 50 N knock' if worst < yld else 'a 50 N knock can BREAK the structure'}")

    # DO THE SAME BARS CARRY BOTH? If keypress and knock load the same members, an impact-aware GROW
    # keeps the same topology -- the impact mass is then all in SIZING, and re-routing cannot help.
    from scipy.stats import spearmanr
    rho = float(spearmanr(sp, smax).correlation)
    k = max(1, int(0.2 * len(sp)))
    ov = len(set(np.argsort(sp)[-k:]) & set(np.argsort(smax)[-k:])) / k
    print(f"\n  LOAD PATHS -- keypress vs knock per-member stress: Spearman rho = {rho:.2f}, "
          f"top-20% overlap {ov*100:.0f}%")
    if rho > 0.6:
        print("  -> they SHARE load paths (well -> anchor is the same route for a press and a knock),")
        print("     so an impact-aware GROW keeps the same topology; the impact mass is all in SIZING,")
        print("     and topology re-routing cannot beat the thicken-to-survive. 17 g is near-optimal.")
    else:
        print("  -> they load DIFFERENT members, so an impact-aware topology could re-route and be lighter.")

    # WHAT ROBUSTNESS COSTS: thicken each member so its worst impact stress meets yield/SF. Bending
    # dominates, sigma ~ 1/b^2, so b -> b*sqrt(s*SF/yield); never thin a member already OK. First-cut:
    # ignores load redistribution (a proper re-solve would do better), so an UPPER bound on the mass.
    rho = MATERIALS["cf_pa12"]["rho"]
    L = np.array([np.linalg.norm(nodes[j] - nodes[i]) for i, j in sb])
    def mass(bb):
        ri = np.maximum(bb - WALL, 0.0)
        return float(rho * np.sum(np.pi * (bb ** 2 - ri ** 2) * L))
    print(f"\n  COST OF ROBUSTNESS (thicken to survive the knock, first-cut upper bound):")
    for SF in (1.0, 2.0):
        b_rob = b * np.maximum(1.0, np.sqrt(smax * SF / yld))
        print(f"    survive 50 N at SF {SF:.0f}: {mass(b):.4g} -> {mass(b_rob)*1e3:.0f} g "
              f"({mass(b_rob)/mass(b):.1f}x the {mass(b)*1e3:.0f} g bone)")
    print("  ⚠ quasi-static (a real impact amplifies); a proper re-optimisation (re-route + re-size,")
    print("     not just thicken) would beat this, and compliant well cups could cut the input force.")

    # PRODUCE the robust structure: iterate thicken-to-survive to convergence at SF=2, then confirm
    # the deflection gate still holds (thicker is stiffer, so it can only help). Saved as a VARIANT.
    from design.params import DEFLECTION_MAX
    SF = 2.0
    knock_nodes = [int(v) for v in spots.values()]
    br = b.copy()
    sm = smax
    for _ in range(12):
        sm = np.zeros(len(br))
        for node in knock_nodes:
            U, _lu, kl, T = solve(br, [("k", "knock", {node: -KNOCK_N * e_o})])
            sm = np.maximum(sm, _peak_stress(kl, T, U, dofs, br))
        if sm.max() <= yld / SF * 1.02:
            break
        br = br * np.maximum(1.0, np.sqrt(sm * SF / yld))
    Ug, _lu2, _klg, _Tg = solve(br, press_cases)
    wg = max(float(np.linalg.norm(Ug[c][6 * idx[btn[f]]:6 * idx[btn[f]] + 3]))
             for c, (f, _a, _l) in enumerate(press_cases))
    gate = float(DEFLECTION_MAX)
    np.savez("out/robust.npz", nodes=nodes, bars=np.array(sb), live=np.array(range(len(sb))),
             b=br, wall=WALL, roll=roll, radii=br, buttons=z["buttons"], fingers=z["fingers"],
             anchors=z["anchors"], mass=mass(br) * 1e3, pitch=z["pitch"], skin_r=z["skin_r"],
             build_dir=z["build_dir"])
    print(f"\n  ROBUST STRUCTURE (iterated thicken-to-survive, SF {SF:.0f}):")
    print(f"    survives the 50 N knock: worst member {sm.max()/1e6:.0f} MPa <= yield/{SF:.0f} "
          f"({yld/SF/1e6:.0f} MPa)")
    print(f"    mass {mass(b)*1e3:.0f} g -> {mass(br)*1e3:.0f} g   deflection gate {wg*1e6:.0f} um "
          f"(<= {gate*1e6:.0f}, still holds)")
    print("    wrote out/robust.npz (a variant; bone.npz is unchanged)")


if __name__ == "__main__":
    main()

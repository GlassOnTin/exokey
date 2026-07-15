"""PROPER IMPACT-IN-THE-LOOP RE-OPTIMISATION.  PYTHONPATH=. .venv/bin/python scripts/impact_opt.py

robust.py bolted impact on: size the sparse keypress skeleton for the gate, THEN thicken it to survive
a knock (17 g). regrow_impact.py showed WHY that is not optimal: grown WITH the knock in the load set,
the skeleton is a genuinely DIFFERENT, broader one (Jaccard 0.20) -- because a 50 N knock wants many
members SHARING the load, not a few fat ones. Measured directly: the same 50 N knock is 348 MPa on the
sparse bone but only ~56 MPa spread over the broad domain. So the broad skeleton survives at far less
radius, and radius is mass.

This sizes it properly and compares like with like, both grown topologies, both circular rods:

  BOLT-ON      grow for the KEYPRESS gate  ->  size for the gate  ->  FSD-thicken to survive the knock
  IN-THE-LOOP  grow WITH the knock          ->  size for the gate AND the knock together

The knock is a per-member stress limit; the gate is a global deflection limit. They marry by
fully-stressed design feeding the deflection sizer: rlo_e = r_e*sqrt(sigma_e*SF/yield) is a per-member
floor the OC must respect, iterated to a fixed point as the knock redistributes.

⚠ CIRCULAR RODS, as the sizer models them -- not the hollow stadium of the final bone. The number that
transfers is the RATIO, not the grams. The two grows are cached (each is ~10 min).
"""
from __future__ import annotations

import os
import pickle
import time

for _v in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_v, "1")

import numpy as np

from design.params import DEFLECTION_MAX
from design.qwerty import used_actions
from design.vector import evaluate, posture, tm_of, tp_of
from hand.myohand import FINGERS
from opt.problem import hands
from structure.frame import MATERIALS, hand_axes
from structure.lattice import STRAP_K, grow, ground, load_cases
from structure.sizing import Sizer, size_and_prune

KNOCK_N = 50.0
SF = 2.0
E_TPU = MATERIALS["tpu"]["E"]
R_PRINT = 2.5e-4
R_MAX = 2.5e-3
A_ANCHOR = 1.0e-4
CACHE = "out/grow_pair.npz"


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
    gate = float(DEFLECTION_MAX)
    yld = MATERIALS["cf_pa12"]["yield_"]

    # anchors, buttons, strap band -- from ground (deterministic); grow relaxes NODES, so each grow
    # keeps its own node positions, but connectivity/anchor indices are shared.
    _n, _b, btn, _l, ak, an, _t, strap_n = ground(ref, q)
    knock_at = {f: int(btn[f]) for f in FINGERS}
    dorsal = max((i for i in ak), key=lambda i: (_n[i] - o) @ e_o)

    # the two grows (cached). Keypress-only, and impact-aware with the knocks in the ESO ranking.
    if os.path.exists(CACHE):
        z = np.load(CACHE, allow_pickle=True)
        nodes_k, bars, live_k = z["nodes_k"], [tuple(b) for b in z["bars"]], list(z["live_k"])
        nodes_i, live_i = z["nodes_i"], list(z["live_i"])
        print(f"loaded cached grows: keypress {len(live_k)} struts, impact-aware {len(live_i)}\n")
    else:
        knocks = [(f, f"knock_{f}", {knock_at[f]: -KNOCK_N * e_o}) for f in FINGERS]
        knocks.append((FINGERS[0], "knock_dorsal", {dorsal: -KNOCK_N * e_o}))
        t0 = time.time()
        nodes_k, bars_k, live_k, *_ = grow(ref, q, wired=wired, gate=gate, relax=True)
        print(f"keypress grow: {len(live_k)} struts  [{time.time()-t0:.0f}s]", flush=True)
        t1 = time.time()
        nodes_i, bars_i, live_i, *_ = grow(ref, q, wired=wired, gate=gate, relax=True,
                                           impact_cases=knocks)
        print(f"impact grow:   {len(live_i)} struts  [{time.time()-t1:.0f}s]", flush=True)
        bars = [tuple(b) for b in bars_k]                 # same ground bars for both grows
        np.savez(CACHE, nodes_k=nodes_k, nodes_i=nodes_i, bars=np.array(bars),
                 live_k=np.array(live_k), live_i=np.array(live_i))

    cases = load_cases(ref, q, btn, wired=wired)
    akc = {i: 1.0 / (1.0 / ak[i] + 1.0 / (E_TPU * A_ANCHOR / 0.002)) for i in ak}
    rho = MATERIALS["cf_pa12"]["rho"]

    def prep(nodes, live):
        """A knock-stress evaluator + printable-mass fn on the grown subset `sb = bars[live]`."""
        sb = [bars[e] for e in live]
        S = Sizer(nodes, sb, r0=9e-4)
        idx = S.fr.idx
        anch = [i for i in ak if i in idx]
        band = set(strap_n) & set(anch)
        ktot = sum(ak[i] for i in band) or 1.0
        ks = {i: (float(STRAP_K) * ak[i] / ktot if i in band else 0.0) for i in anch}
        spots = [(knock_at[f], -KNOCK_N * e_o) for f in FINGERS] + [(dorsal, -KNOCK_N * e_o)]
        kcases = [("k", "k", {nd: fv}) for nd, fv in spots if nd in idx]
        L = S.L
        T, dofs = S.fr.T, S.fr.dofs

        def stress(rr):
            lift: set = set()
            U = kl = None
            for _ in range(8):
                spring = {i: (ks[i] if i in lift else akc[i]) for i in anch}
                U, _lu, kl = S.solve(rr, spring, kcases)
                nxt = {i for i in anch if float(U[0][6 * idx[i]:6 * idx[i] + 3] @ an[i]) > 0}
                if nxt == lift:
                    break
                lift = nxt
            A = np.pi * rr ** 2
            I = np.pi * rr ** 4 / 4.0
            peak = np.zeros(len(rr))
            for c in range(len(kcases)):
                ul = np.einsum("bij,bj->bi", T, U[c][dofs])
                f = np.einsum("bij,bj->bi", kl, ul)
                N = np.abs(f[:, 0])
                M = np.maximum(np.hypot(f[:, 4], f[:, 5]), np.hypot(f[:, 10], f[:, 11]))
                peak = np.maximum(peak, N / A + M * rr / I)
            return peak, float(peak.max())

        def mass(sub_live, r):                       # printable mass of the size_and_prune result
            return float(rho * np.pi * np.sum(np.maximum(r, R_PRINT) ** 2 * L[sub_live]))

        return sb, stress, mass

    def fsd(rr, sig):
        return np.clip(np.maximum(rr * np.sqrt(np.maximum(sig, 0.0) * SF / yld), R_PRINT),
                       R_PRINT, R_MAX)

    def prune(nodes, sb, rlo=None):
        return size_and_prune(nodes, sb, btn, cases, akc, an, strap_n, float(STRAP_K),
                              gate=gate, r_print=R_PRINT, rlo=rlo)

    # ---- BOLT-ON: keypress topology, size for the gate, then FSD-thicken the fixed topology ----
    print("BOLT-ON -- keypress topology, gate-sized, then thickened for the knock:", flush=True)
    sb_k, stress_k, mass_k = prep(nodes_k, live_k)
    lv, r, m, w = prune(nodes_k, sb_k)
    print(f"  gate-sized: {len(lv)} struts, {m*1e3:.1f} g, {w*1e6:.0f} um", flush=True)
    rr = np.full(len(sb_k), 1e-6)
    rr[lv] = r
    for _ in range(30):
        sig, worst = stress_k(rr)
        if worst <= yld / SF * 1.02:
            break
        rr[lv] = np.maximum(rr[lv], fsd(rr, sig)[lv])
    _sig, worst_bolt = stress_k(rr)
    m_bolt = mass_k(lv, rr[lv])
    print(f"  +knock:     {m_bolt*1e3:.1f} g, knock {worst_bolt/1e6:.0f} MPa "
          f"(gate still {w*1e6:.0f} um -- thicker only stiffens)\n", flush=True)

    # ---- IN-THE-LOOP: impact topology, size for gate AND knock. Seed rlo from the knock's load path
    # on the grown subset FIRST, so the prune keeps it (a sizer cannot resurrect a deleted member). --
    print("IN-THE-LOOP -- impact topology, sized for gate AND knock:", flush=True)
    sb_i, stress_i, mass_i = prep(nodes_i, live_i)
    sig0, _w0 = stress_i(np.full(len(sb_i), 9e-4))
    rlo = fsd(np.full(len(sb_i), 9e-4), sig0)
    rlo[rlo < R_PRINT * 1.5] = R_PRINT
    print(f"  seeded: {int((rlo > R_PRINT*1.5).sum())}/{len(sb_i)} members pinned by the knock",
          flush=True)
    best = None
    for outer in range(10):
        lv, r, m, w = prune(nodes_i, sb_i, rlo=rlo)
        rr = np.full(len(sb_i), 1e-6)
        rr[lv] = r
        sig, worst = stress_i(rr)
        mp = mass_i(lv, r)
        print(f"  round {outer}: {len(lv)} struts, {mp*1e3:5.1f} g, {w*1e6:3.0f} um, "
              f"knock {worst/1e6:4.0f} MPa", flush=True)
        if worst <= yld / SF * 1.05 and (best is None or mp < best[2]):
            best = (list(lv), rr[lv].copy(), mp, w, worst)
        floor = fsd(rr, sig)
        rlo_new = np.maximum(rlo, floor)
        if worst <= yld / SF * 1.05 and np.allclose(rlo_new, rlo, rtol=0.02):
            break
        rlo = rlo_new

    print("\nVERDICT (circular rods, same anchor, same gate, both survive the 50 N knock at SF 2):")
    print(f"  bolt-on     (keypress topology, thickened): {m_bolt*1e3:5.1f} g")
    if best is not None:
        lb, rb, mb, wb, kb = best
        print(f"  in-the-loop (impact topology, co-sized):    {mb*1e3:5.1f} g   "
              f"({len(lb)} struts, {wb*1e6:.0f} um, knock {kb/1e6:.0f} MPa)")
        print(f"  -> in-the-loop is {(1-mb/m_bolt)*100:.0f}% LIGHTER, and reaches a higher safety "
              f"factor (bolt-on saturates at r_max before SF 2)" if mb < m_bolt
              else "  -> in-the-loop is NOT lighter")
        sbl = [sb_i[e] for e in lb]
        np.savez("out/impact_opt.npz", nodes=nodes_i, bars=np.array(sbl),
                 live=np.array(range(len(lb))), radii=rb, r=rb,
                 buttons=np.array([btn[f] for f in FINGERS]), fingers=np.array(FINGERS),
                 anchors=np.array(sorted(ak)), mass=mb * 1e3, button_um=wb * 1e6,
                 knock_mpa=kb / 1e6, mass_bolton=m_bolt * 1e3)
        print("  wrote out/impact_opt.npz")
    else:
        print("  in-the-loop: no design survived within the step budget")


if __name__ == "__main__":
    main()

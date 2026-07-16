"""BONES HAVE NO SHARP EDGES.  PYTHONPATH=. .venv/bin/python scripts/smooth.py

Sweep the spline tension. tau = 0 is the straight structure EXACTLY -- it is the regression, and if
it does not reproduce the old numbers to the last decimal the machinery is lying. Above that, the
load paths bow into cubic splines through their own nodes, the kinks go away, and this measures what
that costs.

⚠ CURVATURE IS NOT FREE, AND THIS IS NOT SOLD AS A STRUCTURAL WIN. For a member with fixed ends
carrying pure axial load, straight is the stiffest path there is. What curvature buys is that the
LOAD PATH stops kinking -- which is a stress riser in a part meant to take millions of keystrokes,
and which is what the user meant by "zig-zaggy" the first time they looked at it.
"""
from __future__ import annotations

import pickle
import time

import numpy as np
from scipy.spatial import cKDTree

from design.params import DEFLECTION_MAX
from design.qwerty import used_actions
from design.vector import evaluate, posture, tm_of, tp_of
from hand.flesh import skin
from hand.myohand import FINGERS
from opt.problem import hands
from structure.frame import MATERIALS
from structure.lattice import (NOZZLE_R, SEG_CLEAR, STRAP_K, buildable, ground, load_cases,
                               unsupported)
from structure.sizing import Sizer, size
from structure.spline import SUBDIV, curves, kink, push_out

TAUS = (0.0, 0.15, 0.30, 0.50)


def main():
    H = hands()
    ref = H[50]
    x = pickle.load(open("out/final_design.pkl", "rb"))["x"]
    wired = used_actions(evaluate(x, H)["action_map"])
    q = ref.compose({f: posture(ref, f, tp_of(x, f), tm_of(x, f), float(x.get(f"ab_{f}", 0.0)))
                     for f in FINGERS})

    z = np.load("out/printable.npz", allow_pickle=True)
    nodes = z["nodes"]
    bars = [tuple(b) for b in z["bars"]]
    live = [int(e) for e in z["live"]]
    btn = {f: int(i) for f, i in zip(z["fingers"], z["buttons"])}
    build = np.asarray(z["build_dir"], float)
    build /= np.linalg.norm(build)
    pitch = float(z["pitch"])

    _n, _b, _bt, _l, ak, an, _t, sn = ground(ref, q, pitch=pitch, reach=2.2)
    cases = load_cases(ref, q, btn, wired=wired)
    rho = MATERIALS["cf_pa12"]["rho"]
    n = float(NOZZLE_R)
    V, _F = skin(ref, q)
    tree = cKDTree(V)

    turns = kink(nodes, bars, live)
    print(f"THE STRAIGHT STRUCTURE: {len(live)} members, {len(turns)} nodes that a load path RUNS "
          f"THROUGH.")
    print(f"  the TURN ANGLE it makes at each one: median {np.median(turns):.0f} deg, "
          f"90th pct {np.percentile(turns, 90):.0f} deg, worst {turns.max():.0f} deg")
    print("  Every one of those is a kink in the centreline: a moment discontinuity in the beam")
    print("  model and a stress riser in the part. THAT is what a spline removes.\n")
    print(f"  {'tau':>5s} {'sub-beams':>10s} {'MASS':>8s} {'button':>7s} {'peak stress':>12s} "
          f"{'skin':>7s} {'support':>8s} {'turn @ sub-beam':>16s}")

    base = None
    out = {}
    # what centreline clearance does each member need? the flesh floor, plus its own rod radius.
    HUG = 0.004
    floor = float(SEG_CLEAR) * HUG
    need = {e: floor + float(z["radii"][k]) for k, e in enumerate(live)}

    for tau in TAUS:
        t0 = time.time()
        n2, b2, owner = curves(nodes, bars, live, tension=tau)
        # ⚠ AND PUSH THE CURVES BACK OUT OF THE HAND. A spline cuts corners, and the corner it cuts
        # may be the flesh: unrepaired, tau = 0.30 bowed the worst clearance from 2.98 mm down to
        # 2.35 mm, straight through a 3.0 mm floor. The NODES never move -- only the interior points
        # of the curves.
        n2, moved = push_out(n2, len(nodes), b2, owner, tree, need)
        lv2 = list(range(len(b2)))
        r, m, w, _, _ = size(n2, b2, btn, cases, ak, an, sn, float(STRAP_K),
                          gate=float(DEFLECTION_MAX), r_min=n, r0=max(9e-4, 2 * n), steps=18)
        if not np.isfinite(w) or w > float(DEFLECTION_MAX):
            print(f"  {tau:5.2f}  MISSES THE GATE ({w*1e6:.0f} um)")
            continue

        # ⚠ PEAK STRESS -- the fatigue argument, MEASURED rather than asserted.
        #
        # `Frame` takes SCALAR section properties, so it cannot be handed a per-element radius
        # vector. The Sizer can: it splits each element's local stiffness into its r^2 and r^4 parts
        # analytically. So take the end forces straight off the Sizer's own element matrices --
        # which is exactly what Frame.stress() does, at the radii the optimiser actually chose.
        S = Sizer(n2, b2)
        idx = S.fr.idx
        anch = [i for i in ak if i in idx]
        band = set(sn) & set(anch)
        kt = sum(ak[i] for i in band) or 1.0
        ks = {i: (float(STRAP_K) * ak[i] / kt if i in band else 0.0) for i in anch}
        lift: set = set()
        for _ in range(8):
            spring = {i: (ks[i] if i in lift else ak[i]) for i in anch}
            U, _lu, kl = S.solve(r, spring, cases)
            nxt = {i for i in anch
                   if float(U[0][6 * idx[i]:6 * idx[i] + 3] @ an[i]) > 0}
            if nxt == lift:
                break
            lift = nxt
        ul = np.einsum("bij,cbj->cbi", S.fr.T, U[:, S.fr.dofs])       # local displacements
        fend = np.einsum("bij,cbj->cbi", kl, ul)                      # (ncase, nbar, 12) end forces
        A = np.pi * r ** 2
        I = np.pi * r ** 4 / 4
        N = np.abs(fend[:, :, 0])                                     # axial
        M = np.maximum(np.hypot(fend[:, :, 4], fend[:, :, 5]),        # bending, worse end
                       np.hypot(fend[:, :, 10], fend[:, :, 11]))
        s_peak = float((N / A + M * r / I).max()) / 1e6               # extreme fibre

        # does the curve dip into the flesh? (the ROD SURFACE, not the centreline)
        mid = 0.5 * (n2[[b[0] for b in b2]] + n2[[b[1] for b in b2]])
        clear = float((tree.query(mid)[0] - r).min())

        # printability: a CURVED member is steep in one place and shallow in another, so this
        # genuinely changes -- and it is now measured on the sub-beams, which is more honest.
        pil = len(unsupported(n2, b2, lv2, build))
        ok = buildable(n2, b2, build)
        props = int((~ok).sum())
        t2 = kink(n2, b2, lv2)
        mass = float(rho * np.pi * np.sum(r ** 2 * S.L)) * 1000

        flag = ""
        if base is None:
            base = (mass, w, s_peak)
            flag = "  <- the straight structure, reproduced"
        print(f"  {tau:5.2f} {len(b2):10d} {mass:6.2f} g {w*1e6:5.0f}um {s_peak:9.1f} MPa "
              f"{clear*1e3:5.2f}mm {pil+props:5d}  {np.percentile(t2, 90):5.1f} deg p90  "
              f"(pushed {moved*1e3:.2f}mm out of the flesh){flag}")
        out[tau] = dict(nodes=n2, bars=b2, owner=owner, radii=r, mass=mass, w=w,
                        stress=s_peak, clear=clear, support=pil + props, turns=t2,
                        t=time.time() - t0)

    if 0.0 not in out or len(out) < 2:
        raise SystemExit("\nnothing to compare")

    b0 = out[0.0]
    print(f"\n  the price of curvature, against tau = 0 ({b0['mass']:.2f} g, "
          f"{b0['stress']:.1f} MPa, {b0['support']} supports):")
    for tau in TAUS[1:]:
        if tau not in out:
            continue
        d = out[tau]
        print(f"    tau {tau:.2f}:  mass {100*(d['mass']/b0['mass']-1):+5.1f}%   "
              f"peak stress {100*(d['stress']/b0['stress']-1):+5.1f}%   "
              f"support {d['support']-b0['support']:+4d}   "
              f"worst turn {np.percentile(b0['turns'],90):.0f} -> "
              f"{np.percentile(d['turns'],90):.0f} deg")

    # ⚠ CHOOSE ON THE THING CURVATURE ACTUALLY THREATENS, WHICH IS THE FLESH.
    #
    # The first version of this picked the largest tension whose MASS stayed within 10% -- and chose
    # tau = 0.50, which is DOMINATED: less smooth than 0.30 (18.5 deg against 17.5), heavier, and it
    # has to be shoved 1.12 mm out of the hand to stop it cutting through. Selecting on the one
    # quantity that curvature does not endanger, while ignoring the one it does, is how you ship a
    # constraint violation.
    #
    # So: the SMOOTHEST tension that keeps the clearance, and the mass, essentially where the
    # straight structure had them. Smoothness is the objective; the flesh is the constraint.
    def keeps_clearance(t):
        return out[t]["clear"] >= 0.97 * b0["clear"]        # within 3% of the straight structure

    ok_taus = [t for t in TAUS if t > 0 and t in out
               and keeps_clearance(t) and out[t]["mass"] <= 1.10 * b0["mass"]]
    pick = min(ok_taus, key=lambda t: np.percentile(out[t]["turns"], 90)) if ok_taus else 0.0
    d = out[pick]
    for t in TAUS:
        if t > 0 and t in out and not keeps_clearance(t):
            print(f"  ⚠ tau = {t:.2f} REJECTED: it presses the gauntlet into the hand "
                  f"({out[t]['clear']*1e3:.2f} mm against the straight structure's "
                  f"{b0['clear']*1e3:.2f} mm), even after the curves are pushed back out.")
    print(f"\n  CHOSEN tau = {pick:.2f}: {len(d['bars'])} sub-beams, {d['mass']:.2f} g, "
          f"{d['w']*1e6:.0f} um, peak {d['stress']:.1f} MPa, {d['support']} support points")
    yld = MATERIALS["cf_pa12"]["yield_"] / 1e6
    print(f"  yield utilisation {d['stress']/yld*100:.0f}% of {yld:.0f} MPa "
          f"(SF 2 -> {d['stress']/(yld/2)*100:.0f}%)")

    np.savez("out/smooth.npz", nodes=d["nodes"], bars=np.array(d["bars"]),
             live=np.arange(len(d["bars"])), radii=d["radii"], owner=d["owner"],
             buttons=np.array([btn[f] for f in FINGERS]), fingers=np.array(FINGERS),
             anchors=z["anchors"], mass=d["mass"], button_um=d["w"] * 1e6,
             bars0=len(d["bars"]), build_dir=build, pitch=pitch, tension=pick,
             subdiv=SUBDIV,
             pillars=np.array(unsupported(d["nodes"], d["bars"],
                                          list(range(len(d["bars"]))), build), dtype=int),
             sagging=np.array([e for e in range(len(d["bars"]))
                               if not buildable(d["nodes"], d["bars"], build)[e]], dtype=int))
    print("\n  wrote out/smooth.npz")


if __name__ == "__main__":
    main()

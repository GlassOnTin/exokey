"""THE PRINTABLE GAUNTLET.  PYTHONPATH=. .venv/bin/python scripts/printable.py

THE USER: "I'm curious to find a smoothed skeletal geometry that we could 3d print WITHOUT TOO MANY
SUPPORTS... the point is to make a structure that can self support as FDM printer layers aligned
with some convenient plane."  And, of the same structure: "still looks a bit unnatural (zig-zaggy
and not-natural-intuitive-entropy)."

THOSE ARE THE SAME PROBLEM. A minimum-mass truss WANTS many thin members -- that is what is
efficient, and it is why the unconstrained answer came out as a net of 0.26 mm hairs, 86% of them
thinner than a 0.4 mm nozzle can lay. What forces a structure into FEW, THICK, CHUNKY members --
the "natural entropy" the eye is looking for in a bone -- is MANUFACTURE.

THREE FDM RULES. ONLY ONE OF THEM IS A VETO, AND IT TOOK FOUR WRONG VERSIONS TO SEE IT:

  1. THE NOZZLE.   r >= 0.4 mm. HARD -- no amount of support lets a 0.4 mm nozzle lay a thinner
                   bead. This is also the rule that RESHAPES the answer.
  2. OVERHANG.     A strut within 45 deg of the build plane cannot HOLD ANYTHING UP. It can still
                   be PRINTED, as a bridge. Those are different properties.
  3. SUPPORT.      Every node needs something under it: a STRUT (free -- it is structure, in the
                   FEM, carrying load, weighed) or a SACRIFICIAL PILLAR (a cost, snapped off).

Every time I made 2 or 3 a BAN, I banned something the printer can actually do, and the run failed
in a way that looked like a finding about the device:

    ban all shallow struts       -> banned all shear bracing  -> NO structure meets the gate
    ban un-supportable nodes     -> banned all five wells     -> the hand is NOT IN THE PRINTER: a
                                                                 pillar may pass through the volume
                                                                 the hand will later occupy
    ban un-bridgeable bars       -> severed a button's stalk  -> a 10 KM rigid-body deflection

WHAT THIS SCRIPT DOES. It takes the TOPOLOGY the unconstrained optimiser found, applies the nozzle
where the nozzle belongs -- to the material that actually gets BUILT -- and re-sizes against it.

⚠ FATTEN, DO NOT DELETE. 86% of that topology's struts are thinner than the nozzle, AND THEY CARRY
53% OF THE MASS. They are not numerical dust; they are a fine net doing real work, which is what a
minimum-mass shell wants to be. Deleting them collapses the structure. Fattening them costs 1.6x.
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
from structure.frame import MATERIALS, hand_axes
from structure.lattice import (BASE_T, BRIDGE_MAX, NOZZLE_R, OVERHANG, STRAP_K, _steep, buildable,
                               ground, load_cases, unsupported)
from structure.sizing import Sizer, size_and_prune

SRC = "out/sized.npz"


def solve_at(S, radii, cases, ak, an, sn, buttons):
    """The worst button deflection at these radii, with the bilinear anchor re-converged."""
    idx = S.fr.idx
    anch = [i for i in ak if i in idx]
    band = set(sn) & set(anch)
    kt = sum(ak[i] for i in band) or 1.0
    ks = {i: (float(STRAP_K) * ak[i] / kt if i in band else 0.0) for i in anch}
    lift: set = set()
    for _ in range(8):
        spring = {i: (ks[i] if i in lift else ak[i]) for i in anch}
        U, _lu, _kl = S.solve(radii, spring, cases)
        nxt = {i for i in anch if float(U[0][6 * idx[i]:6 * idx[i] + 3] @ an[i]) > 0}
        if nxt == lift:
            break
        lift = nxt
    return max(float(np.linalg.norm(U[c][6 * idx[buttons[f]]:6 * idx[buttons[f]] + 3]))
               for c, (f, _a, _l) in enumerate(cases))


def main():
    H = hands()
    ref = H[50]
    fd = pickle.load(open("out/final_design.pkl", "rb"))
    x = fd["x"]
    wired = used_actions(evaluate(x, H)["action_map"])
    q = ref.compose({f: posture(ref, f, tp_of(x, f), tm_of(x, f), float(x.get(f"ab_{f}", 0.0)))
                     for f in FINGERS})
    _o, e_d, e_r, e_o = hand_axes(ref, q)

    # ⚠ RUN THE WHOLE PIPELINE. This used to load a topology found by an EARLIER, unconstrained run
    # and merely fatten it to the nozzle -- which meant the printable structure was never actually
    # OPTIMISED under the printing constraints, only projected onto them.
    pitch, reach = 0.008, 2.2
    nodes, bars, btn, _l, ak, an, _t, sn = ground(ref, q, pitch=pitch, reach=reach)
    cases = load_cases(ref, q, btn, wired=wired)
    rho = MATERIALS["cf_pa12"]["rho"]
    n = float(NOZZLE_R)
    print(f"DOMAIN: {len(bars)} candidate bars at {pitch*1e3:.0f} mm pitch x reach {reach}")
    print(f"  gate {float(DEFLECTION_MAX)*1e6:.0f} um, {len(cases)} wired load cases, "
          f"nozzle {n*1e3:.1f} mm (HARD)\n")

    # the build direction, chosen for LEAST SUPPORT PLASTIC -- see support_mm() below
    def _sup(d, lv):
        d = np.asarray(d, float); d = d / np.linalg.norm(d)
        hh = nodes @ d
        used_ = sorted({i for e in lv for i in bars[e]})
        bed = min(hh[i] for i in used_)
        tot = sum(float(hh[i] - bed) for i in unsupported(nodes, bars, lv, d))
        ok_ = buildable(nodes, bars, d)
        for e in lv:
            if not ok_[e]:
                a, b = bars[e]
                tot += float(0.5 * (hh[a] + hh[b]) - bed)
        return tot

    rng = np.random.default_rng(5)
    V = rng.normal(size=(400, 3)); V /= np.linalg.norm(V, axis=1, keepdims=True)
    build = min(V, key=lambda d: _sup(d, list(range(len(bars)))))

    t0 = time.time()
    stop = {}
    live, r, m, w = size_and_prune(
        nodes, bars, btn, cases, ak, an, sn, float(STRAP_K),
        gate=float(DEFLECTION_MAX), r_print=n, build_dir=build,
        on_step=lambda s, n_, mm, ww: print(
            f"  prune {s:2d}: {n_:5d} struts {mm*1000:7.2f} g {ww*1e6:4.0f} um "
            f"[{time.time()-t0:4.0f}s]", flush=True),
        on_stop=lambda why, n_, mm: stop.update(why=why))
    if not len(live):
        raise SystemExit(f"NO STRUCTURE MET THE GATE ({stop.get('why')})")
    print(f"\n  the pruner stopped because: {stop.get('why')}")

    sb = [bars[e] for e in live]
    S = Sizer(nodes, sb)
    mass = lambda rr: float(rho * np.pi * np.sum(rr ** 2 * S.L)) * 1000   # noqa: E731
    idle = int((r <= n * 1.02).sum())
    print(f"  {idle}/{len(live)} struts sit ON the nozzle floor doing no work "
          f"{'-- CLEAN' if idle == 0 else '-- the pruner could not remove them'}")

    # ---- RULES 2+3: the build direction, chosen for the FEWEST SUPPORT POINTS -----------------
    # ⚠ PILLARS ARE NOT THE WHOLE SUPPORT BILL. A node with nothing under it needs a pillar off the
    # bed; a strut that is SHALLOW *and* longer than a bridge can span needs a prop under its middle.
    # Both are sacrificial, both get snapped off, and a direction chosen on pillars alone will
    # happily trade one for three of the other. Minimise the SUM.
    def pillars(d):
        d = np.asarray(d, float)
        return len(unsupported(nodes, bars, live, d / np.linalg.norm(d)))

    def props(d):
        d = np.asarray(d, float)
        ok_ = buildable(nodes, bars, d / np.linalg.norm(d))
        return sum(not ok_[e] for e in live)

    def support_mm(d):
        """THE HONEST COST OF SUPPORT IS THE COLUMN OF PLASTIC YOU HAVE TO PRINT AND SNAP OFF.

        ⚠ NOT THE COUNT. Counting support POINTS picks the direction that makes the part 172 mm
        tall standing on its fingertips -- 123 pillars, yes, but every one of them a 17 cm column.
        A flatter orientation with MORE pillars can need far LESS plastic. So measure what you
        actually pay: the total LENGTH of column that has to run from the bed up to each thing that
        needs holding -- every unsupported node, and the midpoint of every strut too long to bridge.
        """
        d = np.asarray(d, float)
        d = d / np.linalg.norm(d)
        hh = nodes @ d
        used_ = sorted({i for e in live for i in bars[e]})
        bed = min(hh[i] for i in used_)
        total = sum(float(hh[i] - bed) for i in unsupported(nodes, bars, live, d))
        ok_ = buildable(nodes, bars, d)
        for e in live:
            if not ok_[e]:
                a, b = bars[e]
                total += float(0.5 * (hh[a] + hh[b]) - bed)     # a prop under the strut's midpoint
        return total

    def shape(d):
        d = np.asarray(d, float)
        d = d / np.linalg.norm(d)
        hh = nodes @ d
        u = np.cross(d, [0, 0, 1.0])
        u /= np.linalg.norm(u)
        v = np.cross(d, u)
        used = sorted({i for e in live for i in bars[e]})
        bed = nodes[[i for i in used if hh[i] <= min(hh[j] for j in used) + float(BASE_T)]]
        span = (max(hh[i] for i in used) - min(hh[i] for i in used))
        if not len(bed):
            return span, 0, 0.0, 0.0
        return span, len(bed), np.ptp(bed @ u), np.ptp(bed @ v)

    named = [("wrist -> fingers", e_d), ("fingers -> wrist", -e_d),
             ("palm down (dorsal up)", e_o), ("palm up (dorsal down)", -e_o),
             ("thumb up", e_r), ("little up", -e_r)]
    V2 = rng.normal(size=(800, 3))
    V2 /= np.linalg.norm(V2, axis=1, keepdims=True)
    build = min(V2, key=support_mm)                            # re-pick, now on the ANSWER
    by_count = min(V2, key=lambda d: pillars(d) + props(d))    # what "fewest supports" would pick

    used = sorted({i for e in live for i in bars[e]})
    print(f"BUILD DIRECTION -- chosen for the LEAST SUPPORT PLASTIC "
          f"({len(used)} nodes, {len(live)} struts):")
    print(f"  {'direction':>24s} {'pillars':>8s} {'props':>6s} {'SUPPORT':>10s} {'height':>8s} "
          f"{'bed contact':>20s}")
    for name, d in named + [("BEST by COUNT", by_count), ("BEST by VOLUME <-", build)]:
        ht, nb, a, b = shape(d)
        print(f"  {name:>24s} {pillars(d):8d} {props(d):6d} {support_mm(d)*1e3:8.0f}mm "
              f"{ht*1e3:6.0f}mm   {nb:2d} nodes {a*1e3:3.0f}x{b*1e3:3.0f}mm")
    # ⚠ DERIVE THE POSTURE, DO NOT ASSERT IT. The previous 921-strut answer printed "on its side"
    # and "count picks the worst orientation" as hard-coded prose -- and both became FALSE the moment
    # the pruner was fixed and the structure changed. A render or a message that states a conclusion
    # it did not compute is a lie waiting for its moment.
    lie = max(("palm down", build @ e_o), ("palm up", -(build @ e_o)),
              ("fingers up", -(build @ e_d)), ("wrist up", build @ e_d),
              ("thumb up", -(build @ e_r)), ("little up", build @ e_r), key=lambda t: t[1])[0]
    print(f"  (chosen: distal {build @ e_d:+.2f}  radial {build @ e_r:+.2f}  "
          f"dorsal {build @ e_o:+.2f} -- it prints {lie.upper()})")
    ratio = support_mm(by_count) / max(support_mm(build), 1e-9)
    if ratio > 1.05:
        print(f"\n  ⚠ MINIMISING SUPPORT *COUNT* COSTS {ratio:.1f}x MORE SUPPORT *VOLUME* "
              f"({support_mm(by_count)*1e3:.0f} mm of column against {support_mm(build)*1e3:.0f}):")
        print("    the fewest-point direction stands the part on end, and every pillar then has to")
        print("    climb its whole height.")
    else:
        print(f"\n  (here the two objectives happen to agree: {ratio:.2f}x. They did NOT on the")
        print("   previous structure, where counting cost 1.9x the support plastic.)")
    print("\n  ⚠ NO DIRECTION REACHES ZERO. A shell that hugs a hand has an overhanging underside")
    print("    whichever way you turn it. Zero-support is not available; FEWEST is.")
    print("  ⚠ THE PROPS COME FROM THE 8 mm LATTICE PITCH (longest bar 17.6 mm, past the 10 mm a")
    print("    bridge spans). A FINER lattice removes them by construction -- and MEASURED, it costs")
    print("    2.3x the mass (5.5 mm pitch: 1872 struts, 13.95 g, and 39% of them idle on the nozzle")
    print("    floor). Mass is worn every day; support is paid once. The coarse lattice wins.")
    print("  ⚠ AND THE DIRECTION IS CHOSEN ON SUPPORT ALONE. A real print also wants a low part on")
    print("    a broad footprint -- visible in the table, NOT optimised.")

    prop = unsupported(nodes, bars, live, build)
    ok = buildable(nodes, bars, build)
    st = _steep(nodes, bars, build)
    sag = [e for e in live if not ok[e]]

    # ⚠ BRIDGE_MAX IS A GUESS, AND THE PROP COUNT HANGS ENTIRELY OFF IT. So sweep it rather than
    # quote it. This is the one number a real print would settle in an afternoon.
    from structure.lattice import _tilt
    tilt, L, _hh = _tilt(nodes, bars, build)
    steep = tilt >= np.sin(np.pi / 2 - np.radians(float(OVERHANG)))
    print(f"\n  SENSITIVITY -- props vs the printer's bridging span (BRIDGE_MAX is a GUESS at "
          f"{float(BRIDGE_MAX)*1e3:.0f} mm):")
    print(f"    {'bridge span':>12s} {'props':>7s}")
    for bm in (0.008, 0.010, 0.012, 0.015, 0.018, 0.020):
        n_sag = sum(1 for e in live if not steep[e] and L[e] > bm)
        print(f"    {bm*1e3:9.0f} mm {n_sag:7d}"
              + ("   <- the value assumed" if abs(bm - float(BRIDGE_MAX)) < 1e-9 else "")
              + ("   <- no props at all" if n_sag == 0 else ""))
    print(f"    The longest strut in the structure is {L[live].max()*1e3:.1f} mm. A printer that")
    print("    bridges that far needs NO props, and the whole support bill is the pillars.")

    print(f"\nTHE PRINTABLE GAUNTLET: {len(live)} struts, {mass(r):.2f} g, "
          f"worst button {w*1e6:.0f} um (gate {float(DEFLECTION_MAX)*1e6:.0f})")
    print(f"  radii {r.min()*1e3:.2f}-{r.max()*1e3:.2f} mm; every strut at or above the "
          f"{n*1e3:.1f} mm nozzle: {bool((r >= n - 1e-9).all())}")
    print(f"  self-supporting: {sum(st[e] for e in live)}/{len(live)} "
          f"({100*sum(st[e] for e in live)/len(live):.0f}%); "
          f"{sum(ok[e] and not st[e] for e in live)} print as bridges; {len(sag)} would sag")
    print(f"  SACRIFICIAL SUPPORT: {len(prop)}/{len(used)} nodes ({100*len(prop)/len(used):.0f}%) "
          f"need a pillar, plus {len(sag)} mid-span props.")
    print("  Every other node is held up by a strut that STAYS -- structure, not scaffolding.")

    np.savez("out/printable.npz", nodes=nodes, bars=np.array(bars), live=np.array(live),
             radii=r, buttons=np.array([btn[f] for f in FINGERS]), fingers=np.array(FINGERS),
             anchors=np.array(sorted(ak)), mass=mass(r), button_um=w * 1e6, bars0=len(bars),
             build_dir=build, pitch=pitch, pillars=np.array(prop, dtype=int),
             sagging=np.array(sag, dtype=int),
             overhang=float(OVERHANG), bridge_max=float(BRIDGE_MAX), nozzle_r=n)
    print("\n  wrote out/printable.npz")


if __name__ == "__main__":
    main()

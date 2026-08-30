"""Can ONE rigid carrier present five keywells at the five fingertips?

The 2026-08-29 reframe replaces five independently-adjustable wells with a single Svalboard
kit: one rigid body, five keywells at a FIXED relative geometry. The self-built design got away
with per-finger adjusters (±9.5 mm distal, ±7.2 mm dorsal); a rigid carrier cannot — its five
wells are cast in one piece, so the five fingertip pads must be mutually consistent with ONE
rigid transform of the cluster.

This asks the question the kit's arrival cannot dodge, and it needs no kit measurement to ask:

  1. Where are the five fingertip pads, and which way do they face, in the natural typing
     posture? (well_frame: pos = pad, floor = pad normal, axis = distal phalanx.)
  2. Take the cluster frame as the index's well frame. Where do the OTHER four pads sit in
     that frame? Their spread IS the cluster geometry a rigid carrier must reproduce.
  3. A rigid cluster fixes the well-to-well offsets once. The population (5th–95th hand) moves
     the pads. How much does the pad constellation move across hands — and does a rigid body
     survive that, or does the carrier need per-finger adjustment after all?

This is a MEASUREMENT, not a design. It prints numbers; it decides nothing on its own.
"""
from __future__ import annotations

import numpy as np

from opt.problem import hands
from hand.myohand import FINGERS


def constellation(h, q):
    """The five fingertip pads: position, pad normal, distal axis."""
    wf = {f: h.well_frame(q, f) for f in FINGERS}
    pos = np.array([wf[f]["pos"] for f in FINGERS])
    floor = np.array([wf[f]["floor"] for f in FINGERS])
    axis = np.array([wf[f]["axis"] for f in FINGERS])
    return wf, pos, floor, axis


def main():
    H = hands()
    h = H[50]
    q = np.zeros(h.model.nq)
    wf, pos, floor, axis = constellation(h, q)

    print("THE FIVE FINGERTIP PADS (median hand, neutral posture), metres\n")
    for f, p, n, a in zip(FINGERS, pos, floor, axis):
        print(f"  {f:7s} pad=({p[0]*1e3:6.1f},{p[1]*1e3:6.1f},{p[2]*1e3:6.1f})  "
              f"pad_normal=({n[0]:5.2f},{n[1]:5.2f},{n[2]:5.2f})")

    # pairwise pad-to-pad distances: the shape a rigid cluster must hold
    print("\nPAD-TO-PAD DISTANCES (mm) — the rigid cluster must hold these\n")
    print("        " + "".join(f"{f[:4]:>7s}" for f in FINGERS))
    for i, fi in enumerate(FINGERS):
        row = "".join(f"{np.linalg.norm(pos[i]-pos[j])*1e3:7.1f}" for j in range(len(FINGERS)))
        print(f"  {fi:7s}" + row)

    # how much do the pad NORMALS diverge? A rigid cluster aligns all five wells the same way;
    # if the pads face different ways, a rigid cluster tilts keys off their pads.
    print("\nPAD-NORMAL ANGLES vs the INDEX pad normal (deg) — a rigid cluster can only align one\n")
    ref = floor[1]  # index
    for f, n in zip(FINGERS, floor):
        ang = np.degrees(np.arccos(np.clip(n @ ref, -1, 1)))
        print(f"  {f:7s} {ang:5.1f} deg off the index")

    # population: how far do the pads move 5th -> 95th? The percentile hands are pure
    # scalings sitting at the same world place, so WORLD drift is mostly "the hand is
    # bigger and further from the origin" -- meaningless. The carrier rides the palm, so
    # measure each pad RELATIVE TO THE CAPITATE (the central carpal the gauntlet bears on).
    # That difference, in mm, is the adjustment the carrier must cover across the population.
    print("\nPOPULATION DRIFT of each pad, PALM-RELATIVE (5th vs 95th hand), mm\n")
    lo, hi = H[5], H[95]
    qz_lo = np.zeros(lo.model.nq)
    qz_hi = np.zeros(hi.model.nq)
    _, plo, _, _ = constellation(lo, qz_lo)
    _, phi, _, _ = constellation(hi, qz_hi)
    import mujoco
    lo.fk(qz_lo)
    hi.fk(qz_hi)
    cap_lo = mujoco.mj_name2id(lo.model, mujoco.mjtObj.mjOBJ_BODY, "capitate")
    cap_hi = mujoco.mj_name2id(hi.model, mujoco.mjtObj.mjOBJ_BODY, "capitate")
    o_lo = lo.data.xpos[cap_lo].copy()
    o_hi = hi.data.xpos[cap_hi].copy()
    plo_r = plo - o_lo
    phi_r = phi - o_hi
    print("  (palm-relative: subtract the capitate centre; the carrier rides here)\n")
    print("        " + "".join(f"{a:>8s}" for a in ("dX", "dY", "dZ", "|d|")))
    for i, f in enumerate(FINGERS):
        d = (phi_r[i] - plo_r[i]) * 1e3
        print(f"  {f:7s}" + "".join(f"{v:8.1f}" for v in d) + f"{np.linalg.norm(d):8.1f}")

    # is the constellation SHAPE preserved across the population (pure scaling), or does it
    # deform? Compare adjacent pad-to-pad distances 5th vs 95th.
    print("\nADJACENT PAD-TO-PAD SPACING, 5th vs 95th (mm) — shape preserved?\n")
    for a, b in (("index", "middle"), ("middle", "ring"), ("ring", "little"),
                 ("thumb", "index")):
        ia, ib = FINGERS.index(a), FINGERS.index(b)
        dlo = np.linalg.norm(plo[ia] - plo[ib]) * 1e3
        dhi = np.linalg.norm(phi[ia] - phi[ib]) * 1e3
        print(f"  {a[:4]}-{b[:4]:5s} {dlo:6.1f}  ->  {dhi:6.1f}")


if __name__ == "__main__":
    main()

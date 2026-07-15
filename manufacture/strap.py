"""THE STRAP as a subsystem: the band path, and how it anchors to the gauntlet.

The strap is the COMPLIANT half of the anchor (§8.15f). Flesh can only push the device off the
hand; the strap supplies the entire pull, ~1 N, and without it the structure deflects 18x the gate.
So it is load-bearing, and three things about it have to be right:

  1. THE PATH. A band in tension takes the shortest closed loop around what it wraps, which is the
     convex outline of the UNION of the limb and the gauntlet -- NOT the limb alone. Hull the limb
     only (as viz.strap_loop did) and the band passes straight through the device it is meant to
     hold down. `band_loop` fixes that: it feeds the gauntlet's own tube surfaces into the same
     cross-section tightening, so the band bulges OVER the structure.

  2. THE ANCHOR. Where a soft strap meets a stiff printed part, a bond loaded in PEEL fails. The
     fix is a captured pin -- a watch lug: a printed boss with a through-hole at each band, a pin
     through it, the TPU strap looping the pin in SHEAR. `lug` places one on the gauntlet's own
     anchor feet.

  3. ADJUSTMENT. One device fits the 5th-95th percentile hand, whose wrist circumference spans
     ~1.24x. `adjust_range` measures the band-length spread that the buckle/holes must cover.

⚠ WHAT IS NOT HERE. The bond chemistry (PU adhesive TPU->TPU, vinyl-silane primer to the glass in
glass-nylon) is a materials choice, documented in VISION, not modelled. And a hulled band presses
only on the convex high points -- the bony prominences (§8.15f cc) -- so `crowns_a_prominence`
flags where a pad is needed; sizing the pad is not done.
"""
from __future__ import annotations

import numpy as np

THUMB_BODIES = ("firstmc", "proximal_thumb", "distal_thumb", "trapezium")
NB = 96  # angular bins for the cross-section outline
PIN_R = 0.0011  # m; the lug's through-hole for a ~2 mm spring bar or printed pin. Two 0.4 mm
#                 nozzle widths of wall sit outside it. A GUESS in the sense that no pin is sourced.


def perimeter(loop) -> float:
    """Closed-loop circumference (m). What a buckle/holes must span."""
    return float(np.sum(np.linalg.norm(np.diff(loop, axis=0), axis=1)))


def adjust_range(hands: dict, x: dict) -> dict:
    """Band circumference at the WRIST across the population -- the range the buckle must cover.

    The wrist band is the tightest and is what a watch-style strap buckles on. It is the SKIN
    circumference (the device adds a near-constant bit at every size, so it drops out of the SPREAD),
    measured on each hand's own wrist station.
    """
    from design.vector import posture, tm_of, tp_of
    from hand.myohand import FINGERS
    from structure.anchor import bearing_surface, strap_bands

    perims = {}
    for pct, h in hands.items():
        q = h.compose({f: posture(h, f, tp_of(x, f), tm_of(x, f), float(x.get(f"ab_{f}", 0.0)))
                       for f in FINGERS})
        P, *_ = bearing_surface(h, q)
        st = strap_bands(h, q, P)[0]
        loop = band_loop(h, q, st, device=None)
        perims[pct] = perimeter(loop) if loop is not None else float("nan")
    lo, hi = min(perims.values()), max(perims.values())
    return dict(min=lo, max=hi, spread=hi - lo, per_hand=perims)


def lug_sites(h, q, nodes, anchor_ids, device=None):
    """One watch-lug per anchor foot: (node, band, position, pin axis, hole radius).

    The pin runs along the hand's long axis (e_d), so the CIRCUMFERENTIAL strap loops over it and
    pulls across it -- the pin is loaded in SHEAR, and the soft-to-stiff join is a captured pin,
    not a bond in peel. The lug sits at the anchor node the strap band already pulls on
    (structure.anchor.under_strap), so it feeds the tension straight into the load path.
    """
    from structure.frame import hand_axes
    from structure.anchor import under_strap

    _o, e_d, _e_r, _e_o = hand_axes(h, q)
    band_of = under_strap(h, q, nodes, anchor_ids)
    return [dict(node=int(n), band=int(b), pos=nodes[n], pin_axis=e_d, hole_r=PIN_R)
            for n, b in band_of.items()]


def bridging_fraction(loop, h, q, gap=0.0025):
    """Fraction of the band that stands more than `gap` off the skin -- i.e. bridges a concavity and
    so bears on the convex HIGH POINTS beside it (bony prominences). Those spans want a pad (§8.15f).
    """
    import mujoco
    from hand.flesh import skin
    from scipy.spatial import cKDTree

    V, _, L = skin(h, q, labels=True)
    tid = {mujoco.mj_name2id(h.model, mujoco.mjtObj.mjOBJ_BODY, b) for b in THUMB_BODIES}
    tree = cKDTree(V[~np.isin(L, list(tid))])
    d, _ = tree.query(loop[:-1])
    return float(np.mean(d > gap))


def _strut_points(nodes, bars, live, radii, step=0.002):
    """Points along every live strut's centreline, each tagged with that strut's radius."""
    P, R = [], []
    for k, e in enumerate(live):
        i, j = bars[e]
        a, b = nodes[i], nodes[j]
        L = float(np.linalg.norm(b - a))
        n = max(2, int(L / step) + 1)
        for s in np.linspace(0.0, 1.0, n):
            P.append(a + s * (b - a))
            R.append(float(radii[k]))
    return np.array(P), np.array(R)


def band_loop(h, q, station, device=None, width=0.016, standoff=0.0012, reach=0.020):
    """The band centreline at `station`: the CONVEX HULL of the (skin ∪ device) cross-section.

    A band in tension is the shortest closed loop around what it wraps, and that is the convex hull
    of the union (§8.15f). A convex hull cannot pass through either point set -- every point is on
    it or inside -- so offsetting it outward by `standoff` clears both by construction. (The old
    max-radius-per-angle outline dipped between struts, because a single point per strut undercovers
    the tube's angular width; a hull does not.)

    `device`: optional (nodes, bars, live, radii) of the gauntlet, same frame as the hand.

    ⚠ `reach`: struts reaching OUT to the wells/pillars cross this slab far from the hand, and a
    naive hull would balloon around them -- but they are not something the strap wraps (the fingers
    poke through). So device points more than `reach` beyond the skin at their own angle are dropped.
    That is a heuristic, not a segmentation of body-from-arm; a strut grazing the cutoff can still
    tug the hull. Flagged, not solved.

    Returns the closed loop (M+1, 3), or None if the station has too little to wrap.
    """
    import mujoco
    from scipy.spatial import ConvexHull

    from hand.flesh import skin
    from structure.frame import hand_axes

    V, _, L = skin(h, q, labels=True)
    tid = {mujoco.mj_name2id(h.model, mujoco.mjtObj.mjOBJ_BODY, b) for b in THUMB_BODIES}
    V = V[~np.isin(L, list(tid))]
    o, e_d, e_r, e_o = hand_axes(h, q)

    sel = np.abs((V - o) @ e_d - station) < width / 2
    if sel.sum() < 30:
        return None
    P = V[sel]
    c = P.mean(axis=0)
    su, sv = (P - c) @ e_r, (P - c) @ e_o
    pts = [np.column_stack([su, sv])]
    skin_rmax = float(np.hypot(su, sv).max())

    if device is not None:
        nodes, bars, liv, radii = device
        G, GR = _strut_points(nodes, bars, liv, radii)
        gsel = np.abs((G - o) @ e_d - station) < width / 2
        phi = np.linspace(0, 2 * np.pi, 8, endpoint=False)
        for p, r in zip(G[gsel], GR[gsel]):
            du, dv = float((p - c) @ e_r), float((p - c) @ e_o)
            if np.hypot(du, dv) - r > skin_rmax + reach:      # a well-arm / pillar, not the body
                continue
            pts.append(np.column_stack([du + r * np.cos(phi), dv + r * np.sin(phi)]))

    XY = np.vstack(pts)
    hull = XY[ConvexHull(XY).vertices]                        # ordered, CCW
    ctr = hull.mean(axis=0)
    out = hull + standoff * (hull - ctr) / (np.linalg.norm(hull - ctr, axis=1, keepdims=True) + 1e-12)
    loop = [c + x * e_r + y * e_o for x, y in out]
    loop.append(loop[0])
    return np.array(loop)

"""BONES HAVE NO SHARP EDGES.

THE USER, on the converged structure: "could we do some final optimisation of cubic spline elements
rather than linear? It would be nice to get a smooth solution." And then, when I tried to justify it
with a clearance argument: "MY BONES HAVE NO SHARP EDGES."

That is the whole brief, and it is a better one than mine was.

WHAT IS ACTUALLY WRONG WITH THE STRUCTURE. Every member is a STRAIGHT CHORD between two nodes, so
every load path is a POLYLINE, and a polyline has a KINK AT EVERY NODE. The smooth-min SDF hides
those kinks under a 0.6 mm fillet, so the printed part looks blended -- but the CENTRELINE is still
kinked, which means:

  * the beam model has a MOMENT DISCONTINUITY at every node;
  * a kink in a load path is a STRESS RISER, and this device is supposed to take millions of
    keystrokes, so the kinks are exactly where it will crack;
  * and it reads as "zig-zaggy", which is what the user said about it the first time they saw it
    and which I misdiagnosed as a rendering fault. It was not. The geometry really does zig-zag.

⚠ AND NOTE WHAT THE JUSTIFICATION IS *NOT*. I first tried to argue for curvature on CLEARANCE --
that a straight chord between two nodes sitting `hug` off a convex hand dips toward the flesh in
the middle. Measured: it does, but it does not BIND (0 of 669 members are at the floor). So that is
not the reason, and a reason that does not survive its own measurement is not a reason.

⚠ AND CURVATURE IS NOT FREE. For a member with fixed ends carrying pure AXIAL load, straight is the
stiffest possible path -- curve it and the load becomes eccentric, so it develops bending and the
member goes SOFTER for the same mass. Form-finding drives trusses TOWARD straightness for exactly
this reason. So this cannot be sold as a structural win, and it is not. It is bought, and the price
is measured here.

WHAT THIS DOES. At every node, PAIR UP the members that continue each other -- greedily, straightest
first -- and give each matched pair a SHARED TANGENT (the bisector). A member whose ends are both
matched becomes a cubic Hermite curve that leaves and arrives tangentially, so the load path through
it is C1. Discretise into straight sub-beams and the existing FEM needs no change at all: a curved
beam IS a polyline of straight beams, in the limit.

  1. match the members at each node by straightest continuation   (the load paths, LOCALLY)
  2. shared tangent per matched pair                               (C1 -- the kink goes away)
  3. cubic Hermite per member, discretised into sub-beams          (the FEM is unchanged)
  4. re-size, and MEASURE what the curvature cost

⚠ AND *NOT* WITH `manufacture/wire.py::trails()`, WHICH IS THE OBVIOUS THING AND IS WRONG. A trail
cover must use EVERY EDGE, so when the straight continuations run out it is forced into hairpins:
measured on the real structure, the trails turn a median of 57 deg at each node and up to 180 deg --
a complete reversal. Fitting a spline through THAT gives you a curve that doubles back on itself.
A load path is under no obligation to cover every edge. A wire is. They are different objects.

A member with NO continuation within `MAX_TURN` at one end is at a genuine BRANCH or a free end, and
it simply stays straight there. Five members really do meet at a trabecular node, and only one pair
of them can be tangent -- the rest branch off with a corner, and the SDF fillets it. That is not a
compromise; it is what a branch point actually looks like.

The original node indices are PRESERVED, so the buttons, the anchors and the strap nodes all still
point at the right places; the new interior points are appended.

THE TENSION `tau` IS THE DESIGN VARIABLE, and it is a single scalar:

    tau = 0     -> the tangents vanish and the spline collapses to the straight chords.
                   THE OLD STRUCTURE, EXACTLY. It is the regression case.
    tau = 0.5   -> the standard Catmull-Rom spline.

so the curvature can simply be swept, and the price read off.
"""
from __future__ import annotations

import numpy as np

from design.params import P, Source

TENSION = P("SPLINE_TENSION", 0.5, "-", Source.GUESS,
            "How hard the load paths bow away from their own chords. It is the Catmull-Rom tangent "
            "scale: 0 collapses the spline back onto the straight members (the old structure, "
            "exactly), 0.5 is the standard Catmull-Rom. NOT derived from anything -- it is swept, "
            "and the mass it costs at the deflection gate is measured rather than assumed.")

MAX_TURN = P("MAX_TURN", 75.0, "deg", Source.GUESS,
             "Beyond this angle, two members at a node are NOT a continuing load path -- they are a "
             "BRANCH -- and no tangent is shared between them. Set it too high and the spline tries "
             "to smooth a T-junction into an S-bend; too low and real paths stay kinked.")

# Straight sub-beams per original member. A curved beam IS a polyline of straight beams in the
# limit, so this is a DISCRETISATION -- the error in the curvature, not a claim about the world --
# and it does not belong in the parameter registry. 4 keeps the sub-beam turn small and the FEM
# about 7x the straight one.
SUBDIV = 4


def _hermite(p0, p1, m0, m1, t):
    """The cubic Hermite point at t in [0,1]. With m0 = m1 = 0 this is the straight chord."""
    t2, t3 = t * t, t * t * t
    return ((2 * t3 - 3 * t2 + 1)[:, None] * p0
            + (t3 - 2 * t2 + t)[:, None] * m0
            + (-2 * t3 + 3 * t2)[:, None] * p1
            + (t3 - t2)[:, None] * m1)


def _match(nodes, bars, live):
    """At each node, pair up the members that CONTINUE each other -- greedily, straightest first.

    Returns {(edge, node): tangent} for the matched pairs. Each pair shares ONE tangent line (the
    bisector), which is what makes the path C1 through that node. An edge left unmatched at a node
    is at a BRANCH or a free end, and gets no entry -- it stays straight there.
    """
    nodes = np.asarray(nodes, float)
    adj: dict[int, list[int]] = {}
    for e in live:
        i, j = bars[e]
        adj.setdefault(int(i), []).append(e)
        adj.setdefault(int(j), []).append(e)

    def away(e, n):
        """unit vector pointing AWAY from node n along edge e."""
        i, j = bars[e]
        v = nodes[j if int(i) == n else i] - nodes[n]
        L = np.linalg.norm(v)
        return v / L if L > 1e-12 else np.zeros(3)

    lim = np.cos(np.radians(180.0 - float(MAX_TURN)))     # e and f leave n in ~opposite directions
    tang: dict = {}
    for n, es in adj.items():
        cand = []
        for a in range(len(es)):
            for b in range(a + 1, len(es)):
                ua, ub = away(es[a], n), away(es[b], n)
                c = float(ua @ ub)                        # -1 = dead straight through the node
                if c <= lim:
                    cand.append((c, es[a], es[b]))
        cand.sort()                                       # most-negative = straightest first
        taken: set = set()
        for c, e, f in cand:
            if e in taken or f in taken:
                continue
            taken.add(e)
            taken.add(f)
            # ONE shared tangent line for the pair: the bisector of "out along e" and "back along f"
            t = away(e, n) - away(f, n)
            L = np.linalg.norm(t)
            if L < 1e-9:
                continue
            t /= L
            tang[(e, n)] = t                              # e leaves n along +t
            tang[(f, n)] = -t                             # f leaves n along -t
    return tang


def curves(nodes, bars, live, tension=None, k=None):
    """The straight members, replaced by CUBIC HERMITE curves tangent to their own load paths.

    Returns (nodes2, bars2, owner):
      nodes2  the original nodes, INDICES UNCHANGED (so buttons / anchors / strap nodes still
              resolve), followed by the interior points of every curve
      bars2   the straight sub-beams the FEM actually sees
      owner   owner[j] = the ORIGINAL member index that sub-beam j came from
    """
    nodes = np.asarray(nodes, float)
    tau = float(TENSION) if tension is None else float(tension)
    k = int(float(SUBDIV)) if k is None else int(k)
    tang = _match(nodes, bars, live)

    pts = [nodes]
    n_next = len(nodes)
    bars2: list[tuple[int, int]] = []
    owner: list[int] = []
    t = np.linspace(0.0, 1.0, k + 1)[1:-1]

    for e in live:
        i, j = int(bars[e][0]), int(bars[e][1])
        p0, p1 = nodes[i], nodes[j]
        chord = p1 - p0
        L = float(np.linalg.norm(chord))
        # the tangent at each end: the shared one if this member continues a path there, else the
        # chord itself (straight). Scaled by the chord length, so `tau` is dimensionless.
        m0 = tang.get((e, i), chord / max(L, 1e-12)) * L
        m1 = -tang.get((e, j), -chord / max(L, 1e-12)) * L
        m0 = chord + tau * 2.0 * (m0 - chord)
        m1 = chord + tau * 2.0 * (m1 - chord)

        chain = [i]
        if k > 1:
            interior = _hermite(p0, p1, m0, m1, t)
            pts.append(interior)
            chain += list(range(n_next, n_next + len(interior)))
            n_next += len(interior)
        chain.append(j)
        for a, b in zip(chain[:-1], chain[1:]):
            bars2.append((a, b))
            owner.append(int(e))

    nodes2 = np.vstack(pts) if len(pts) > 1 else nodes
    return nodes2, bars2, np.array(owner, int)


def kink(nodes, bars, live):
    """The TURN ANGLE a load path makes at each node it CONTINUES THROUGH, in degrees.

    This is the number the whole exercise exists to reduce: a kink in a load path is a moment
    discontinuity in the beam model and a stress riser in the part. Measured only over the members
    that actually PAIR UP at a node -- a T-junction is a branch, not a kink, and smoothing it would
    be smoothing away the structure.
    """
    nodes = np.asarray(nodes, float)
    adj: dict[int, list[int]] = {}
    for e in live:
        i, j = bars[e]
        adj.setdefault(int(i), []).append(e)
        adj.setdefault(int(j), []).append(e)

    def away(e, n):
        i, j = bars[e]
        v = nodes[j if int(i) == n else i] - nodes[n]
        L = np.linalg.norm(v)
        return v / L if L > 1e-12 else np.zeros(3)

    lim = np.cos(np.radians(180.0 - float(MAX_TURN)))
    turns = []
    for n, es in adj.items():
        cand = []
        for a in range(len(es)):
            for b in range(a + 1, len(es)):
                c = float(away(es[a], n) @ away(es[b], n))
                if c <= lim:
                    cand.append((c, es[a], es[b]))
        cand.sort()
        taken: set = set()
        for c, e, f in cand:
            if e in taken or f in taken:
                continue
            taken.add(e)
            taken.add(f)
            turns.append(180.0 - np.degrees(np.arccos(np.clip(c, -1.0, 1.0))))
    return np.array(turns)


def push_out(nodes2, n_fixed, bars2, owner, tree, need):
    """Push the curves' INTERIOR points back out until they clear the flesh.

    ⚠ A SPLINE CUTS CORNERS, AND THE CORNER IT CUTS MAY BE THE HAND.

    The straight structure clears the skin BY CONSTRUCTION -- every candidate member was checked
    against the flesh before it was ever offered to the optimiser (`ground()`), and every one that
    passed through it was refused. But the CURVE between the same two nodes is a DIFFERENT OBJECT,
    and it is free to bow the wrong way. Measured, at tau = 0.15: the worst clearance fell from
    2.98 mm to 2.75 mm, straight through a 3.0 mm floor. The smoothing was quietly pressing the
    gauntlet into the hand.

    So every interior point is pushed back out along the local outward normal until the ROD SURFACE
    clears. The NODES themselves never move -- they are where the load is applied, where the anchors
    bear and where the buttons sit, and the whole structure was optimised around them.

    `need[e]` is the centreline clearance member e requires (the floor, plus its own rod radius).
    """
    nodes2 = np.asarray(nodes2, float).copy()
    owner = np.asarray(owner, int)

    # which member does each interior point belong to? (an interior point is used by exactly the
    # sub-beams of one member)
    home = {}
    for j, (a, b) in enumerate(bars2):
        for p in (a, b):
            if p >= n_fixed:
                home[p] = owner[j]
    if not home:
        return nodes2, 0.0

    idx = np.fromiter(home.keys(), int)
    req = np.array([need[home[int(p)]] for p in idx])

    moved = 0.0
    for _ in range(6):
        d, nn = tree.query(nodes2[idx])
        bad = d < req
        if not bad.any():
            break
        # outward = away from the nearest point of the skin
        out = nodes2[idx[bad]] - tree.data[nn[bad]]
        L = np.linalg.norm(out, axis=1, keepdims=True)
        out = np.where(L > 1e-9, out / np.maximum(L, 1e-12), 0.0)
        step = (req[bad] - d[bad])[:, None] * out
        nodes2[idx[bad]] += step
        moved = max(moved, float(np.abs(step).max()))
    return nodes2, moved

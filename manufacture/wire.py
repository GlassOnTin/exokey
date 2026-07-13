"""BUILD IT OUT OF WIRE. Trail-cover the skeleton, so a wire BENDS through a node instead of
being welded at it.

THE USER: "I could use stainless steel wire, and use my laser welder to make the nodal points. The
challenge is forming the shape. I'm imagining 3d printing the skeleton and then using it as a form
for the stainless steel."

That is a better process than printing the structure, and it changes what the model should be
counting. A PRINTED gauntlet has 316 struts and nobody cares, because the printer does not get
tired. A WELDED one has as many joints as you are willing to make by hand, and every one is a
heat-affected zone where annealed 316 yields at ~205 MPa instead of the 500+ of the cold-drawn
wire you started with. So the welds are both the LABOUR and the WEAK POINTS, and the right thing
to minimise is not the number of struts -- it is the number of welds.

AND A WIRE IS NOT A STRUT. A strut is a straight bar between two joints. A wire is CONTINUOUS: it
can run through a node and simply BEND there, no weld at all. So the question is:

    cover every edge of the skeleton with as few continuous trails as possible,
    and prefer, at each node, the continuation that bends LEAST.

That is an Eulerian trail-cover. For each connected component the minimum number of trails is
max(1, odd_degree_nodes / 2) -- a hard bound, not a heuristic -- and the only freedom is WHICH
edges get paired at each node. Pair them by straightest continuation and the wires come out as
long smooth runs rather than as a heap of short zig-zags, which is also the shape a wire wants to
be bent into.

WELDS ARE THEN ONLY WHERE WIRES CROSS: a node of degree 2 that one wire passes through needs
nothing at all.
"""
from __future__ import annotations

import numpy as np


def trails(nodes, bars, live):
    """Cover the skeleton's edges with as few continuous wires as possible.

    Returns (wires, welds): `wires` is a list of node-index paths; `welds` is the set of nodes
    where more than one wire touches, i.e. the joints that actually have to be welded.
    """
    adj: dict[int, list[int]] = {}
    edges = {}
    for e in live:
        i, j = bars[e]
        adj.setdefault(i, []).append(e)
        adj.setdefault(j, []).append(e)
        edges[e] = (i, j)

    def other(e, n):
        i, j = edges[e]
        return j if n == i else i

    def straightness(e_in, e_out, n):
        """cos of the turn a wire makes going e_in -> e_out at node n. +1 = dead straight."""
        a = nodes[n] - nodes[other(e_in, n)]
        b = nodes[other(e_out, n)] - nodes[n]
        na, nb = np.linalg.norm(a), np.linalg.norm(b)
        if na < 1e-12 or nb < 1e-12:
            return -1.0
        return float((a @ b) / (na * nb))

    unused = set(live)
    wires: list[list[int]] = []

    # ⚠ START FROM THE ODD-DEGREE NODES. A trail can only END at an odd-degree node, so if you
    # start anywhere else you close a loop early and need MORE wires than the bound allows.
    odd = [n for n, es in adj.items() if len(es) % 2 == 1]
    starts = odd + [n for n in adj if n not in odd]

    for s in starts:
        while any(e in unused for e in adj.get(s, [])):
            path = [s]
            n = s
            e = next(e for e in adj[n] if e in unused)
            while True:
                unused.discard(e)
                n = other(e, n)
                path.append(n)
                cand = [f for f in adj.get(n, []) if f in unused]
                if not cand:
                    break
                # ...and take the straightest continuation. A wire bent 20 deg is a wire; a wire
                # bent 140 deg is two wires that happen to be touching.
                e = max(cand, key=lambda f: straightness(e, f, n))
            wires.append(path)

    # A NODE NEEDS A WELD only where more than one wire touches it (or a wire ends there).
    touch: dict[int, int] = {}
    for w in wires:
        for k, n in enumerate(w):
            touch[n] = touch.get(n, 0) + (1 if 0 < k < len(w) - 1 else 1)
    counts: dict[int, int] = {}
    for w in wires:
        for n in set(w):
            counts[n] = counts.get(n, 0) + 1
    ends = {w[0] for w in wires} | {w[-1] for w in wires}
    welds = {n for n, c in counts.items() if c > 1} | ends
    return wires, welds


def report(nodes, bars, live, wire_d):
    w, welds = trails(nodes, bars, live)
    L = [sum(float(np.linalg.norm(nodes[p[k + 1]] - nodes[p[k]])) for k in range(len(p) - 1))
         for p in w]
    turns = []
    for p in w:
        for k in range(1, len(p) - 1):
            a = nodes[p[k]] - nodes[p[k - 1]]
            b = nodes[p[k + 1]] - nodes[p[k]]
            c = (a @ b) / (np.linalg.norm(a) * np.linalg.norm(b) + 1e-12)
            turns.append(float(np.degrees(np.arccos(np.clip(c, -1, 1)))))
    nodes_used = {i for e in live for i in bars[e]}
    return dict(wires=w, welds=welds, lengths=L, turns=np.array(turns),
                n_struts=len(live), n_nodes=len(nodes_used), wire_d=wire_d,
                total_mm=sum(L) * 1000)

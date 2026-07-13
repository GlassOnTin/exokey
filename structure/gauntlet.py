"""THE GAUNTLET: the top half of a rigid glove, and let it grow its own bones.

THE USER, and this is the design:

    "the top half of a rigid glove or gauntlet, with the fingers free to move about relative to
     the gauntlet. The optimisation is then of making the rigid form as light and minimal as
     possible but still strong enough to support the button / joystick / cup things. The
     solution will begin looking natural and like an alien bone structure when we get into the
     full structural optimisation."

That is the whole problem, stated properly, and it is the first time in this project the
objective has been the right one:

    MINIMISE MASS
    subject to   the buttons stay steady enough to register a 20 gf press
                 the shell never touches the hand
                 the fingers stay free to move under it

WHY IT WILL LOOK LIKE BONE. Topology optimisation under a stiffness constraint lays material
along the STRESS TRAJECTORIES and deletes everything else. That is exactly what bone does --
Wolff's law: bone remodels along the lines of principal stress. The two processes are solving
the same problem, so they arrive at the same shapes. It will not look designed. It will look
grown.

THE DOMAIN is the dorsal envelope of the hand: a shell standing off the back of the metacarpals
and the back of every finger, by `hug`. Everything the gauntlet is ALLOWED to be. The optimiser
decides what it IS.

⚠ WHAT IS NOT MODELLED. The fingers are free to CURL under a dorsal shell (measured: a fist
still clears by 3.9 mm) but they are NOT free to EXTEND past the posture the shell was built
for (-6.5 mm at full extension). The gauntlet sets an upper limit on extension, not on grip --
which is the right way round for a hand you still want to use, but it IS a limit and it is not
in the objective.
"""
from __future__ import annotations

import mujoco
import numpy as np
from Pynite import FEModel3D

from structure.frame import MATERIALS, hand_axes
from structure.shell import _mat

CHAIN = {"thumb": ("firstmc", "proximal_thumb", "distal_thumb"),
         "index": ("proxph2", "midph2", "distph2"),
         "middle": ("proxph3", "midph3", "distph3"),
         "ring": ("proxph4", "midph4", "distph4"),
         "little": ("proxph5", "midph5", "distph5")}
PALM = ("secondmc", "thirdmc", "fourthmc", "fifthmc")


def _bone_rings(h, q, bone, dors_local, hug, n_arc, n_along=2, wrap=False):
    """Rings of nodes standing off ONE bone's dorsal surface.

    The bone's own axis is authoritative; the dorsal direction is made perpendicular to IT.
    (Doing it the other way round walked the rings off the bone and put the shell 6 mm inside
    the finger -- see scripts/architecture_view.py.)
    """
    m = h.model
    bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, bone)
    R = h.data.xmat[bid].reshape(3, 3)
    dors = R @ dors_local[bone]
    dors /= np.linalg.norm(dors)

    for g in range(m.body_geomadr[bid], m.body_geomadr[bid] + m.body_geomnum[bid]):
        if m.geom_type[g] != mujoco.mjtGeom.mjGEOM_CAPSULE:
            continue
        r = float(m.geom_size[g][0]) + hug
        half = float(m.geom_size[g][1])
        c = h.data.geom_xpos[g].copy()
        ax = h.data.geom_xmat[g].reshape(3, 3)[:, 2]
        dp = dors - (dors @ ax) * ax
        dp /= np.linalg.norm(dp) + 1e-12
        lat = np.cross(dp, ax)

        out = []
        for t in np.linspace(-half, half, n_along):
            out.append((c + t * ax, dp, lat, r))
        if wrap:
            # over the tip and back palmar, to carry the button. A CAP at constant radius from
            # the capsule's distal endpoint -- a fingertip is a hemisphere.
            tip = c + half * ax
            for k in range(1, 6):
                a = k * (np.pi * 0.75) / 5
                dd = np.cos(a) * dp + np.sin(a) * ax
                out.append((tip, dd / np.linalg.norm(dd), lat, r))
        return out
    return []


def domain(h, q, hug: float = 0.004, n_arc: int = 6):
    """Every place the gauntlet is ALLOWED to be. The optimiser decides what it IS.

    Returns (nodes, quads, well_nodes, strap_nodes).
    """
    m = h.model
    h.fk(np.zeros(m.nq))
    _, _, _, e_o0 = hand_axes(h, np.zeros(m.nq))
    dl = {}
    for bn in list(PALM) + [b for bs in CHAIN.values() for b in bs]:
        bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, bn)
        dl[bn] = h.data.xmat[bid].reshape(3, 3).T @ e_o0
    h.fk(q)

    nodes: list[np.ndarray] = []
    quads: list[tuple[int, int, int, int]] = []

    def strip(rings):
        """A quad strip through a list of rings. Returns the node index grid."""
        grid = []
        for (c, dp, lat, r) in rings:
            row = []
            half_arc = np.pi / 2.2                     # ~160 deg: a dorsal half-shell
            for j in range(n_arc + 1):
                s = -half_arc + 2 * half_arc * j / n_arc
                nodes.append(c + r * (np.cos(s) * dp + np.sin(s) * lat))
                row.append(len(nodes) - 1)
            grid.append(row)
        for i in range(len(grid) - 1):
            for j in range(n_arc):
                quads.append((grid[i][j], grid[i][j + 1], grid[i + 1][j + 1], grid[i + 1][j]))
        return grid

    # THE DORSUM: a half-shell over each metacarpal. They are SEPARATE BONES and the shell is
    # corrugated over them -- which is not a compromise, it is stiffer than a flat plate for the
    # same material, for exactly the reason a corrugated roof is.
    palm_grids = [strip(_bone_rings(h, q, b, dl, hug, n_arc, n_along=3)) for b in PALM]

    # THE FINGERS: a rail over each, wrapping the tip to carry the button.
    well_nodes = {}
    fing_grids = {}
    for f, bones in CHAIN.items():
        rings = []
        for k, bn in enumerate(bones):
            rings += _bone_rings(h, q, bn, dl, hug, n_arc,
                                 n_along=2, wrap=(k == len(bones) - 1))
        g = strip(rings)
        fing_grids[f] = g
        well_nodes[f] = g[-1][n_arc // 2]      # the wrap's last ring: the button hangs here

    # ⚠ STITCH RING-TO-RING; DO NOT WELD BLINDLY.
    #
    # Two failures, both instructive. Sewing the strips with extra quads made DEGENERATE
    # elements (repeated nodes; PyNite warned the Jacobian was <= 0) and a singular solve --
    # 3e13 um of deflection, which is a disconnected mesh announcing itself. Then welding by
    # distance with a 6 mm tolerance MERGED NEIGHBOURING NODES OF THE SAME RING, because the
    # mesh spacing is only ~4 mm: 354 quads collapsed to 96. The tolerance must be well BELOW
    # the mesh spacing, not above it.
    #
    # So: weld only truly coincident nodes, and stitch the finger roots to the knuckles
    # RING-TO-RING -- node j of the finger's first ring to node j of the metacarpal's last.
    # Both rings have the same arc ordering, so the quads come out clean by construction.
    nodes_a = np.array(nodes)
    for f, g in fing_grids.items():
        root = nodes_a[g[0]].mean(axis=0)
        best, bg = 1e9, None
        for pg in palm_grids:
            d = float(np.linalg.norm(nodes_a[pg[-1]].mean(axis=0) - root))
            if d < best:
                best, bg = d, pg
        for j in range(n_arc):
            quads.append((bg[-1][j], bg[-1][j + 1], g[0][j + 1], g[0][j]))

    quads = [qd for qd in quads if len(set(qd)) == 4]

    # THE STRAP anchors: the proximal edge of the dorsum -- where the gauntlet is held on.
    strap = sorted({j for pg in palm_grids for j in pg[0]})
    return nodes_a, quads, well_nodes, strap


def solve(nodes, quads, live, well_nodes, anchor_k, t, mat, press_N):
    """(max well deflection, {elem: strain energy}, mass)."""
    model = FEModel3D()
    _mat(model, mat)
    used = set()
    for e in live:
        used |= set(quads[e])
    for i in used:
        model.add_node(f"N{i}", *nodes[i])
    for e in live:
        a, b, c, d = quads[e]
        model.add_quad(f"Q{e}", f"N{a}", f"N{b}", f"N{c}", f"N{d}", t, mat)
    # THE BOUNDARY CONDITIONS (structure/anchor.py). Got right FIRST, then the shape.
    #
    #   * NOT a clamp. Rigid supports absorb the keypress for free and flatter every number.
    #   * NOT a hinge. The supports used to be the proximal RING of the metacarpal shells -- a
    #     LINE of nodes with ZERO extent along the lever. A keypress 121 mm away is a MOMENT,
    #     and a line cannot carry a moment: 55% of the button's movement was the gauntlet
    #     ROCKING, and a thicker shell did nothing about it.
    #   * A DISTRIBUTED PATCH over the metacarpals AND the CARPUS. 92 mm of extent, and the
    #     rocking falls to 0.2 um.
    #   * STIFFNESS FROM MEASURED TISSUE. k = E*A/t, and t is 1.4-3.1 mm over the metacarpals
    #     (bone radius vs flesh capsule). Thin skin over bone is a STIFF anchor -- SOFT_TISSUE_K
    #     was quoted for a PALM patch, a muscle pad ten times thicker.
    #
    # ⚠ THE SPRINGS ARE BIDIRECTIONAL, AND THAT BUNDLES THE STRAP IN. Tissue is COMPRESSION-ONLY
    # -- it can push the gauntlet off the hand but not pull it back -- so the tension side of
    # every one of these springs is the STRAP's job. `strap_tension()` reports what the strap
    # must actually carry, so the assumption is checkable rather than hidden.
    for i, k in anchor_k.items():
        if i in used:
            model.def_support_spring(f"N{i}", "DX", k, None)
            model.def_support_spring(f"N{i}", "DY", k, None)
            model.def_support_spring(f"N{i}", "DZ", k, None)
    fixed = min(anchor_k, key=lambda i: anchor_k[i]) if anchor_k else None
    if fixed is not None and fixed in used:
        model.def_support(f"N{fixed}", support_RX=True, support_RY=True, support_RZ=True)

    for f, i in well_nodes.items():
        if i in used:
            model.add_node_load(f"N{i}", "FZ", -press_N)
    try:
        model.analyze_linear(check_statics=False)
    except Exception:
        return float("inf"), {}, 0.0
    w = 0.0
    for f, i in well_nodes.items():
        if i in used:
            n = model.nodes[f"N{i}"]
            w = max(w, float(np.linalg.norm([n.DX["Combo 1"], n.DY["Combo 1"], n.DZ["Combo 1"]])))
    se, area = {}, 0.0
    for e in live:
        Q = model.quads[f"Q{e}"]
        dv, fv = Q.d("Combo 1"), Q.f("Combo 1")
        se[e] = float(abs(0.5 * (dv.T @ fv)[0, 0]))
        P = [nodes[k] for k in quads[e]]
        area += 0.5 * abs(np.linalg.norm(np.cross(P[1] - P[0], P[3] - P[0])))
        area += 0.5 * abs(np.linalg.norm(np.cross(P[2] - P[1], P[3] - P[1])))
    return float(w), se, float(area * t * MATERIALS[mat]["rho"])


def grow(h, q, t=0.0008, mat="cf_pa12", press_N=0.196, keep=0.30, rate=0.08, gate=0.5e-3,
         on_step=None):
    """Delete the material that carries no load, until only the bones are left.

    THIS IS WHERE THE ALIEN BONE STRUCTURE COMES FROM. Each step removes the elements with the
    LOWEST STRAIN ENERGY -- the ones doing no work -- and re-solves. What survives is the load
    path, and a load path drawn by nothing but the stresses is what a skeleton IS.

    Stops when the target mass fraction is reached, or when the buttons stop being steady
    enough (`gate`, the 0.5 mm key-deflection limit) -- whichever comes first. Removing material
    until the STRUCTURE fails is not optimisation, it is demolition.
    """
    from structure.anchor import bearing_surface

    nodes, quads, wells, strap = domain(h, q)

    # WHICH SHELL NODES ACTUALLY TOUCH THE HAND, and how stiff each contact is. Derived from
    # the bearing surface, not from a list of node indices I chose.
    BP, BN, BK, BT = bearing_surface(h, q)
    anchor_k = {}
    for i, p in enumerate(nodes):
        d = np.linalg.norm(BP - p, axis=1)
        j = int(np.argmin(d))
        if d[j] < 0.006:                      # this shell node sits on the bearing surface
            anchor_k[i] = float(BK[j])
    live = set(range(len(quads)))
    protected = set()
    # ⚠ PROTECT ONLY THE BUTTONS. I first protected every element touching an ANCHOR node too
    # -- and there are 94 anchor nodes spread over the whole dorsum, so most of the shell was
    # frozen and the optimiser could only shave 16% before it ran out of things it was allowed
    # to touch. It had not converged; it had been handcuffed.
    #
    # The anchors are SPRINGS AT NODES, and a node survives as long as ANY element still uses
    # it. So the anchors defend themselves: cut too much of the dorsum and the structure goes
    # singular, and the adaptive halving backs off. The only thing that genuinely must not be
    # orphaned is where the LOAD comes in -- the buttons.
    keepn = set(wells.values())
    for e, qd in enumerate(quads):
        if keepn & set(qd):
            protected.add(e)

    w, se, mass = solve(nodes, quads, live, wells, anchor_k, t, mat, press_N)
    hist = [(len(live), w, mass)]
    if on_step:
        on_step(nodes, quads, sorted(live), w, mass, 0)

    step = 0
    target = max(1, int(keep * len(quads)))
    while len(live) > target:
        cand = sorted((v, e) for e, v in se.items() if e not in protected)
        if not cand:
            break

        # ⚠ ON FAILURE, HALVE THE CUT -- DO NOT GIVE UP.
        #
        # The first version removed a fixed 8% per step and STOPPED the moment a step failed.
        # It got 3% of the mass out and quit. But a failed step does not mean the structure is
        # finished; it means THAT CUT was too greedy -- it punched a hole through the load path
        # and the solve went singular. Cut half as much and try again. Only when a SINGLE
        # element cannot be removed is the structure actually done.
        n_cut = max(1, int(rate * len(live)))
        progressed = False
        while n_cut >= 1:
            trial = live - {e for _, e in cand[:n_cut]}
            w2, se2, m2 = solve(nodes, quads, trial, wells, anchor_k, t, mat, press_N)
            if np.isfinite(w2) and w2 <= gate:
                live, se, w, mass = trial, se2, w2, m2
                progressed = True
                break
            n_cut //= 2
        if not progressed:
            break                       # even one element cannot go. The structure is done.

        step += 1
        hist.append((len(live), w, mass))
        if on_step:
            on_step(nodes, quads, sorted(live), w, mass, step)
    return nodes, quads, sorted(live), wells, anchor_k, hist

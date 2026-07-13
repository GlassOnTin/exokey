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


HALF_ARC = np.pi / 2.2          # ~160 deg of dorsal wrap: open enough to get the finger in


def _skin_radii(W, dirs, cone=0.80):
    """How far the SKIN reaches in each direction, from one ring centre.

    `W` is the skin, already made relative to the centre and pre-filtered to a local ball/slab.
    For each direction `u`, take the skin vertices lying within a cone about `u` and report how
    far out they go.

    ⚠ THIS REPLACES A 92nd-PERCENTILE OF THE *BONE MESH* PLUS A TISSUE CONSTANT. That instrument
    was noisy in a way a shell cannot survive: ADJACENT ARC DIRECTIONS CAME OUT 25 mm APART,
    because a percentile over a slab of bone vertices lurches whenever the slab clips a condyle.
    Elements that long are not a mesh, they are a lie the solver will happily integrate.

    The skin is a smooth closed surface and we now HAVE it, so ask it directly. It also deletes
    the dorsal/palmar tissue interpolation: the skin already IS bone plus tissue.
    """
    d = np.linalg.norm(W, axis=1) + 1e-12
    r = np.full(len(dirs), np.nan)
    for j, u in enumerate(dirs):
        proj = W @ u
        # ⚠ ONLY A TIGHT CONE. A loose fallback (72 deg) admits vertices nearly PERPENDICULAR to
        # `u` and reports their projection as a radius -- which is how the extreme arc directions,
        # where a bone has little skin of its own to offer, came out 30 mm too far. Leave them
        # NaN and let the interpolation below carry the value in from the directions that DO know.
        sel = (proj / d) > cone
        if sel.sum() >= 5:
            r[j] = float(np.percentile(proj[sel], 90))
    ok = ~np.isnan(r)
    if not ok.any():
        return np.full(len(dirs), 0.008)
    r = np.interp(np.arange(len(dirs)), np.flatnonzero(ok), r[ok])
    # smooth around the arc: the skin is smooth, so the shell that follows it must be too
    return np.convolve(np.pad(r, 1, mode="edge"), [0.25, 0.5, 0.25], mode="valid")


def _bone_rings(h, q, bone, dors_local, hug, n_arc, Vs, Ls, n_along=2, wrap=False):
    """Rings of nodes standing off ONE bone's SKIN -- not off a circular capsule.

    THE GAUNTLET FOLLOWS THE HAND, NOT A TUBE. It used to be built at `capsule_radius + hug`,
    which makes every cross-section a CIRCLE. A hand is not circular: the metacarpals are FLAT
    and the fingers are OVAL. A shell fitted to a tube either stands proud of the flats (bulk,
    and it gets in the way) or bites into the sides (it does not fit).

    So the radius is taken PER DIRECTION, straight off the skin surface:

        shell(station, u) = skin_reach(station, u) + hug

    The bone gives only the AXIS and the STATIONS; the skin gives the shape.

    ⚠ ONLY THIS BONE'S OWN SKIN. A cross-section slab through a metacarpal cuts THE OTHER THREE
    METACARPALS TOO, and a cone opened laterally then finds the NEIGHBOUR's skin and reports it
    as this bone's radius -- 18 mm of standoff where 4 mm was asked for, and elements 69 mm long.
    Neighbours are not this ring's business; keeping clear of them is `relax()`'s job, and it
    does it against the whole assembled surface.
    """
    m = h.model
    bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, bone)
    R = h.data.xmat[bid].reshape(3, 3)
    dors = R @ dors_local[bone]
    dors /= np.linalg.norm(dors)

    # the bone's axis: the capsule's if it has one, else its mesh's own long axis
    ax = half = c = None
    for g in range(m.body_geomadr[bid], m.body_geomadr[bid] + m.body_geomnum[bid]):
        if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_CAPSULE:
            c = h.data.geom_xpos[g].copy()
            half = float(m.geom_size[g][1])
            ax = h.data.geom_xmat[g].reshape(3, 3)[:, 2]
            break
    if ax is None:
        V = []
        for g in range(m.body_geomadr[bid], m.body_geomadr[bid] + m.body_geomnum[bid]):
            if m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
                continue
            mid = m.geom_dataid[g]
            va, vn = m.mesh_vertadr[mid], m.mesh_vertnum[mid]
            V.append(m.mesh_vert[va:va + vn] @ h.data.geom_xmat[g].reshape(3, 3).T
                     + h.data.geom_xpos[g])
        if not V:
            return []
        V = np.vstack(V)
        c = V.mean(axis=0)
        ax = np.linalg.svd(V - c, full_matrices=False)[2][0]
        half = float(np.percentile(np.abs((V - c) @ ax), 90))

    # ⚠ THE BONE AXIS MUST POINT DISTALLY, AND A CAPSULE'S LOCAL Z DOES NOT PROMISE THAT.
    # On the fingers MuJoCo's capsule z runs distally; ON THE WHOLE THUMB IT RUNS PROXIMALLY.
    # Taken as given, the thumb's rings came out backwards, so `firstmc`'s LAST ring was stitched
    # to `proximal_thumb`'s FAR end -- a 43 mm leap across the hand, and 66 shell elements with
    # edges up to 93 mm. It also capped the tip WRAP, which is what carries the button, on the
    # WRONG END of the bone.
    #
    # Derive it instead from the model's own kinematic tree: a MuJoCo body's frame sits at its
    # PROXIMAL joint, so the capsule's centre is always distal of the body origin. That fixes the
    # sign for every bone, thumb included, without anyone needing to know which way z points.
    if ax @ (c - h.data.xpos[bid]) < 0:
        ax = -ax

    Vb = Vs[Ls == bid]                  # the skin OVER THIS BONE. See the warning above.
    if not len(Vb):
        return []

    dp = dors - (dors @ ax) * ax
    dp /= np.linalg.norm(dp) + 1e-12
    lat = np.cross(dp, ax)

    angles = [-HALF_ARC + 2 * HALF_ARC * j / n_arc for j in range(n_arc + 1)]
    band = half / max(1, n_along - 1)
    out = []

    for s_ in np.linspace(-half, half, n_along):
        cc = c + s_ * ax
        W = Vb - cc
        near = (np.abs(W @ ax) < band) & (np.linalg.norm(W, axis=1) < 0.035)
        dirs = [np.cos(a) * dp + np.sin(a) * lat for a in angles]
        out.append((cc, dp, lat, _skin_radii(W[near], dirs) + hug))

    if wrap:
        # OVER THE TIP and back palmar, to carry the button. A fingertip is a hemisphere, so the
        # cap rings share the capsule's distal endpoint as their centre and only the direction
        # sweeps -- and each one asks the skin how far the pulp actually reaches that way.
        tip = c + half * ax
        W = Vb - tip
        near = np.linalg.norm(W, axis=1) < 0.030
        for k in range(1, 6):
            a = k * (np.pi * 0.75) / 5
            up = np.cos(a) * dp + np.sin(a) * ax
            up /= np.linalg.norm(up)
            dirs = [np.cos(b) * up + np.sin(b) * lat for b in angles]
            out.append((tip, up, lat, _skin_radii(W[near], dirs) + hug))
    return out


def domain(h, q, hug: float = 0.004, n_arc: int = 12):
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

    from hand.flesh import skin
    Vs, _, Ls = skin(h, q, labels=True)   # the SKIN -- what the gauntlet stands off, not bone

    nodes: list[np.ndarray] = []
    outward: list[np.ndarray] = []      # each node's own outward radial -- see relax()
    quads: list[tuple[int, int, int, int]] = []

    def strip(rings):
        """A quad strip through a list of rings. Returns the node index grid.

        `r` is now a LIST -- one radius per arc direction -- because the hand's cross-section is
        not a circle. That is the whole change."""
        grid = []
        for (c, dp, lat, r) in rings:
            row = []
            for j in range(n_arc + 1):
                s = -HALF_ARC + 2 * HALF_ARC * j / n_arc
                u = np.cos(s) * dp + np.sin(s) * lat
                nodes.append(c + r[j] * u)
                outward.append(u)
                row.append(len(nodes) - 1)
            grid.append(row)
        for i in range(len(grid) - 1):
            for j in range(n_arc):
                quads.append((grid[i][j], grid[i][j + 1], grid[i + 1][j + 1], grid[i + 1][j]))
        return grid

    def relax(nodes, hug, iters=60, cap=2.0):
        """PUSH THE SHELL OUT UNTIL IT REALLY CLEARS THE SKIN -- BUT NOT FOREVER.

        A per-bone ring cannot see a NEIGHBOURING digit. Between two knuckles a node placed a
        clean 4 mm off ITS OWN metacarpal can still be 0.8 mm from the skin of the one next door,
        and 0.8 mm is not a gap, it is a pinch. Only the ASSEMBLED skin knows that, so the
        standoff is enforced against the surface itself.

        ⚠ TWO WAYS THIS GOES WRONG, AND BOTH DID.

        1. Push along the node's own OUTWARD RADIAL, not along (node - nearest skin vertex). In
           the valley between two fingers those point OPPOSITE ways and the second drives the
           node DEEPER INTO the hand -- which is why 8 nodes would not converge at any number of
           iterations.

        2. CAP THE PUSH. Where the outward radial is nearly TANGENT to the skin, each iteration
           buys almost no clearance, so an uncapped loop just keeps going: nodes marched 30+ mm
           out and dragged 36 shell elements up to 63 mm long behind them. Those elements are not
           a mesh, they are a spike the solver integrates without complaint.

        So the push is capped at `cap * hug`, and a node that STILL cannot make the standoff is
        not pushed further -- it is reported. The honest reading of such a node is that THE SHELL
        MAY NOT GO THERE, and the caller drops the elements that touch it.
        """
        from scipy.spatial import cKDTree

        tree = cKDTree(Vs)
        X, U = np.array(nodes), np.array(outward)
        X0 = X.copy()
        for _ in range(iters):
            d, _i = tree.query(X)
            bad = d < hug - 1e-5
            if not bad.any():
                break
            X[bad] += U[bad] * (1.5 * (hug - d[bad]))[:, None]
            over = np.linalg.norm(X - X0, axis=1) > cap * hug     # THE CAP
            if over.any():
                step = X[over] - X0[over]
                X[over] = X0[over] + step / np.linalg.norm(step, axis=1, keepdims=True) * cap * hug
        return X, tree.query(X)[0] < hug - 1e-5

    # THE DORSUM:    # THE DORSUM: a half-shell over each metacarpal. They are SEPARATE BONES and the shell is
    # corrugated over them -- which is not a compromise, it is stiffer than a flat plate for the
    # same material, for exactly the reason a corrugated roof is.
    palm_grids = [strip(_bone_rings(h, q, b, dl, hug, n_arc, Vs, Ls, n_along=5)) for b in PALM]

    # THE FINGERS: a rail over each, wrapping the tip to carry the button.
    well_nodes = {}
    fing_grids = {}
    for f, bones in CHAIN.items():
        rings = []
        for k, bn in enumerate(bones):
            rings += _bone_rings(h, q, bn, dl, hug, n_arc, Vs, Ls,
                                 n_along=4, wrap=(k == len(bones) - 1))
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
    nodes_a, pinched = relax(nodes, hug)
    for f, g in fing_grids.items():
        # ⚠ STITCH TO THE NEAREST PALM RING, NOT TO THE KNUCKLE RING.
        # The four fingers root AT a knuckle, so "the palm strip's last ring" was right for them.
        # THE THUMB DOES NOT: its chain begins at `firstmc`, whose proximal end is back at the
        # CARPUS. Sewing that to the index knuckle drew a 46 mm quad straight across the back of
        # the hand -- the big flat sheet in the render, and 22 elements the solver was integrating
        # as if they were shell.
        root = nodes_a[g[0]].mean(axis=0)
        best, br = 1e9, None
        for pg in palm_grids:
            for row in pg:
                d = float(np.linalg.norm(nodes_a[row].mean(axis=0) - root))
                if d < best:
                    best, br = d, row
        for j in range(n_arc):
            quads.append((br[j], br[j + 1], g[0][j + 1], g[0][j]))

    # A node that could not make the standoff means THE SHELL MAY NOT GO THERE. Drop its
    # elements from the domain rather than let the optimiser build on a node inside the hand.
    quads = [qd for qd in quads
             if len(set(qd)) == 4 and not any(pinched[i] for i in qd)]

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

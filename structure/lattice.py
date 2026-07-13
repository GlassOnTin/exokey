"""A FREE-FORM GROUND STRUCTURE. The gauntlet stops being a grid of rails and becomes bone.

THE USER: "The solution will begin looking natural and like an alien bone structure when we get
into the full structural optimisation."  ...and then: "free-form domain next, let's grow the real
bone structure."

WHAT WAS WRONG WITH THE OLD DOMAIN, and it is a limit of the DOMAIN, not of the optimiser. The
gauntlet's design space was a STRUCTURED QUAD GRID -- half-shells over each metacarpal, rails over
each finger. ESO could only delete quads FROM THAT GRID. It could not put a strut where the grid
had none, could not route a diagonal across the dorsum, could not brace one finger against the
next. Whatever it deleted, the answer was always some subset of five rails and four shells, which
is why the render read as blocky strips rather than as structure. THE OPTIMISER WAS NOT FINDING
RAILS; IT WAS NEVER OFFERED ANYTHING ELSE.

WHAT THIS IS. A ground structure: fill the whole shell of space the gauntlet is allowed to occupy
with a dense lattice of candidate bars, and let ESO delete its way down to the load path. What
survives is chosen by the stresses and by nothing else.

  * TWO SHEETS, not one. Nodes on an inner surface (`hug` off the skin) and an outer one
    (`hug + layer`), cross-braced. A single sheet of bars is a membrane: it has no depth, so it
    has no bending stiffness, and the button would flop. Two sheets separated by `layer` carry a
    moment as a couple -- tension in one sheet, compression in the other. That is what a sandwich
    panel does, and it is what TRABECULAR BONE does, which is why the answer should look like one.

  * EVERY CANDIDATE BAR IS CHECKED AGAINST THE SKIN. This is the topological trap that killed the
    exoskeleton -- you cannot draw a straight line from the palm to a fingertip without crossing a
    finger. Back then it was discovered by rendering four routings and finding all four cut the
    hand. Here it is simply a constraint: a bar that passes through flesh is not a candidate. The
    optimiser is free to find the routes that DO exist, including ones nobody would draw by hand.

  * THE PALM IS NOT IN THE DOMAIN. The user: "having the supporting structure far from the hand is
    a problem because it gets-in-the-way... If the supporting structure hugs the hand and stays
    above the sensors as much as possible it becomes more a natural extension, rather than holding
    a big ball." So the palmar surface of the palm is excluded outright. The DIGITS wrap fully --
    the well has to reach around the fingertip to cup the pad.
"""
from __future__ import annotations

import numpy as np
from Pynite import FEModel3D
from scipy.spatial import cKDTree

from design.params import P, Source
from hand.flesh import CARPUS, METACARPALS, skin
from hand.myohand import FINGERS
from structure.anchor import bearing_surface
from structure.frame import MATERIALS, hand_axes

BAR_R = P("BAR_R", 0.0009, "m", Source.GUESS,
          "Radius of a lattice bar. A round rod ~1.8 mm across -- printable in CF-PA12 by SLS, "
          "which resolves ~0.8 mm features. NOT optimised: the bar section and the topology "
          "trade against each other and only the topology is being solved here.")

LAYER = P("LAYER", 0.006, "m", Source.GUESS,
          "Depth of the lattice -- the gap between the inner and outer node sheets. It is the "
          "lever arm that gives the structure its bending stiffness, so it is the single most "
          "load-bearing number in this file. 6 mm keeps the gauntlet under 10 mm of total "
          "standoff, which is about as thick as a wearable can be before it 'gets in the way'.")

STRAP_K = P("STRAP_K", 3.3e5, "N/m", Source.DERIVED,
            "The strap's total stiffness, holding the gauntlet DOWN onto the hand. Webbing "
            "22 x 1.5 mm, E = 2.0 GPa (structure/frame.py MATERIALS), free length ~ the "
            "circumference of the hand at the palm, 0.20 m:  k = EA/L = 2e9 * 3.3e-5 / 0.20. "
            "It is 20x SOFTER than the tissue patch it works against, so it is the compliant "
            "element in the anchor and it dominates lift-off.")

# The palm's palmar surface is not available: that is where the fingers work.
PALMAR_CUTOFF = -0.35


def _normals(V, F):
    """Vertex normals of the skin, from its own faces."""
    N = np.zeros_like(V)
    tri = V[F]
    fn = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    for k in range(3):
        np.add.at(N, F[:, k], fn)
    return N / np.maximum(np.linalg.norm(N, axis=1, keepdims=True), 1e-12)


def _allowed(h, V, N, L, e_o):
    """Where the gauntlet MAY be. Everything but the working surface of the palm.

    On the DIGITS the wrap is complete, because the well has to reach around the fingertip and cup
    the pad -- the structure must be allowed to get there. On the PALM and CARPUS the palmar
    surface is excluded: a body in the palm is the architecture this project already abandoned.
    """
    import mujoco

    m = h.model
    palm_ids = {mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, b)
                for b in CARPUS + METACARPALS if b != "firstmc"}
    is_palm = np.isin(L, list(palm_ids))
    return ~(is_palm & ((N @ e_o) < PALMAR_CUTOFF))


def ground(h, q, hug=0.004, layer=None, pitch=0.004, reach=2.2, press_N=0.196):
    """The free-form design space: (nodes, bars, buttons, loads, anchor_k, anchor_n).

    `buttons` maps finger -> node index; `loads` maps that node to the force vector the digit
    applies when it presses. `anchor_k` maps node index -> tissue spring stiffness (N/m), and
    `anchor_n` to that node's OUTWARD normal -- which is what tells the solver whether the node is
    pressing INTO the hand (tissue bears) or lifting OFF it (the strap must pull).
    """
    from design.vector import action_dirs, well_channel

    layer = float(LAYER) if layer is None else layer
    V, F, L = skin(h, q, labels=True)
    N = _normals(V, F)
    _o, _e_d, _e_r, e_o = hand_axes(h, q)
    keep = _allowed(h, V, N, L, e_o)
    Vk, Nk = V[keep], N[keep]

    # DECIMATE to a lattice. One seed per `pitch` voxel, carrying that voxel's mean normal.
    cell = {}
    for p, n in zip(Vk, Nk):
        cell.setdefault(tuple(np.round(p / pitch).astype(int)), []).append((p, n))
    seeds, snorm = [], []
    for pts in cell.values():
        seeds.append(np.mean([p for p, _ in pts], axis=0))
        nn = np.mean([n for _, n in pts], axis=0)
        snorm.append(nn / (np.linalg.norm(nn) + 1e-12))
    seeds, snorm = np.array(seeds), np.array(snorm)

    # ⚠ WELD THE NEAR-DUPLICATES. Voxel decimation puts a seed in each cell, so where the skin
    # grazes a cell boundary two seeds land ~0.3 mm apart -- and a 0.3 mm bar of 1.8 mm rod is not
    # a strut, it is an ill-conditioned row in the stiffness matrix.
    order = np.lexsort(seeds.T)
    keepi, taken = [], cKDTree(seeds)
    dead = np.zeros(len(seeds), bool)
    for i in order:
        if dead[i]:
            continue
        keepi.append(i)
        dead[taken.query_ball_point(seeds[i], 0.65 * pitch)] = True
    seeds, snorm = seeds[keepi], snorm[keepi]

    # TWO SHEETS. The depth between them is what carries the moment.
    inner = seeds + snorm * hug
    outer = seeds + snorm * (hug + layer)
    nodes = np.vstack([inner, outer])
    n_in = len(inner)

    skin_tree = cKDTree(V)

    # ⚠ A NODE THAT SITS INSIDE THE HAND IS NOT A DESIGN CHOICE, IT IS A MISTAKE. Offsetting along
    # a vertex normal cannot see the NEIGHBOURING digit, so nodes in the valley between two
    # fingers land in flesh. Drop them; the gauntlet may not be there.
    ok = skin_tree.query(nodes)[0] >= hug - 1e-4
    nodes = nodes[ok]
    remap = -np.ones(len(ok), int)
    remap[np.flatnonzero(ok)] = np.arange(ok.sum())
    n_in = int(remap[:n_in].max()) + 1 if (remap[:n_in] >= 0).any() else 0

    # THE BUTTONS. One per digit, on the back of its well, loaded along the direction that digit
    # actually presses.
    #
    # ⚠ THE OLD SOLVER LOADED EVERY BUTTON ALONG WORLD -Z. World Z is not a direction any finger
    # pushes -- the five digits press five different ways, and the thumb is nearly orthogonal to
    # the fingers. A load applied in the wrong direction grows the wrong structure.
    btn, loads = {}, {}
    extra = []
    for f in FINGERS:
        dist, prox, r = well_channel(h, q, f)
        click = action_dirs(h, q, f)["click"]
        p = 0.5 * (np.asarray(dist) + np.asarray(prox)) + np.asarray(click) * r
        extra.append(p)
        btn[f] = len(nodes) + len(extra) - 1
        loads[btn[f]] = np.asarray(click) * press_N
    nodes = np.vstack([nodes, np.array(extra)])

    # CANDIDATE BARS: every pair close enough to be a strut...
    tree = cKDTree(nodes)
    pairs = np.array(sorted(tree.query_pairs(r=reach * pitch)), dtype=int)
    # ...plus the buttons, which must be able to reach whatever is near them.
    for f, i in btn.items():
        for j in tree.query_ball_point(nodes[i], 1.6 * (hug + layer + pitch)):
            if j < len(nodes) - len(extra):
                pairs = np.vstack([pairs, [i, j]])

    # ⚠ AND NOW THE CHECK THAT KILLED THE EXOSKELETON. A bar is only a candidate if it does not
    # pass THROUGH the hand. Sample along each one and ask the skin.
    a, b = nodes[pairs[:, 0]], nodes[pairs[:, 1]]
    s = np.linspace(0.15, 0.85, 5)
    mid = (a[:, None, :] * (1 - s)[None, :, None] + b[:, None, :] * s[None, :, None])
    clear = skin_tree.query(mid.reshape(-1, 3))[0].reshape(len(pairs), -1).min(axis=1)
    blen = np.linalg.norm(a - b, axis=1)
    bars = [tuple(p) for p, c, ln in zip(pairs, clear, blen)
            if c >= 0.8 * hug and ln >= 0.4 * pitch]

    # THE ANCHOR. The tissue springs (structure/anchor.py) land on their nearest lattice node.
    Pb, Nb, Kb, _Tb = bearing_surface(h, q)
    inner_tree = cKDTree(nodes[:n_in]) if n_in else None
    anchor_k, anchor_n = {}, {}
    if inner_tree is not None:
        for p, nrm, k in zip(Pb, Nb, Kb):
            d, i = inner_tree.query(p)
            if d < 0.012:
                i = int(i)
                anchor_k[i] = anchor_k.get(i, 0.0) + float(k)
                anchor_n[i] = anchor_n.get(i, np.zeros(3)) + np.asarray(nrm)
    for i, v in anchor_n.items():
        anchor_n[i] = v / (np.linalg.norm(v) + 1e-12)
    return nodes, bars, btn, loads, anchor_k, anchor_n


def connected(bars, live, anchor_k, buttons, n_nodes):
    """The live bars that actually have a path to an ANCHOR. Everything else is a floating island.

    ⚠ THIS IS WHY THE LATTICE SOLVED TO NaN AND REPORTED IT AS ZERO. A lattice sampled off a skin
    surface has clusters -- a knot of bars over a fingertip, say -- with no path back to the wrist.
    An island has six rigid-body modes and no restraint, so the stiffness matrix is SINGULAR;
    PyNite's sparse solve returns NaN rather than raising; and `max(0.0, nan)` in Python is 0.0.
    So the model announced "buttons steady at 0 um" -- a perfect score, from a structure that did
    not have a stiffness matrix. It survived halving the bar radius unchanged, which is what gave
    it away: a real structure gets softer when you thin it.

    ESO makes islands constantly (deleting bars is how a component gets cut off), so this is not a
    one-off cleanup -- every solve has to do it.
    """
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components

    if not live:
        return [], False
    ij = np.array([bars[e] for e in live])
    g = coo_matrix((np.ones(len(ij)), (ij[:, 0], ij[:, 1])), shape=(n_nodes, n_nodes))
    _n, lab = connected_components(g, directed=False)
    root = {lab[i] for i in anchor_k}
    keep = [e for e in live if lab[bars[e][0]] in root]
    ok = all(lab[i] in root for i in buttons.values())
    return keep, ok


def solve(nodes, bars, live, buttons, loads, anchor_k, normals, r=None, mat="cf_pa12",
          strap_k=None, iters=12):
    """(worst button deflection, {bar: strain energy density}, mass, strap tension in N).

    ⚠ THE ANCHOR IS BILINEAR, AND GETTING THAT WRONG IS WHAT MAKES A GAUNTLET LOOK GOOD ON PAPER.

    Flesh can PUSH the gauntlet off the hand. It cannot PULL it back on. A keypress at a fingertip
    ~120 mm from the wrist is a MOMENT, so it presses one end of the anchor patch INTO the hand and
    lifts the other end OFF it -- and the lifting end is carried by the STRAP, not by the tissue.

    Modelled as bidirectional springs (which is what the shell gauntlet did, and declared), this
    lattice grew a structure that reported 495 um at the buttons. Re-solved with the pulling
    springs removed and no strap at all, THE SAME STRUCTURE deflects 9178 um -- 18x the gate. Two
    fifths of its anchor reaction was tension that nothing in the world was supplying. The number
    was not slightly optimistic; it was fiction.

    So each anchor node now gets a spring whose stiffness depends on WHICH WAY IT IS MOVING along
    its own outward normal:

        moving IN  (pressing on the hand)  ->  k_tissue,  stiff  (E*dA/t, MRI-measured t)
        moving OUT (lifting off the hand)  ->  k_strap,   soft   (webbing, EA/L)

    That is a nonlinear support, so it is iterated to a fixed active set. It converges in a few
    passes because the sign of the motion at a node rarely flips twice.
    """
    strap_k = float(STRAP_K) if strap_k is None else strap_k
    r = float(BAR_R) if r is None else r
    A = np.pi * r ** 2
    I = np.pi * r ** 4 / 4
    J = np.pi * r ** 4 / 2
    p = MATERIALS[mat]

    live, reachable = connected(bars, live, anchor_k, buttons, len(nodes))
    if not reachable or not live:
        return float("inf"), {}, 0.0, 0.0
    used = {i for e in live for i in bars[e]}
    anch = [i for i in anchor_k if i in used]
    if not anch:
        return float("inf"), {}, 0.0, 0.0

    ktot = sum(anchor_k[i] for i in anch)
    k_strap = {i: strap_k * anchor_k[i] / ktot for i in anch}   # the strap's share of the patch

    lifting = {i: False for i in anch}                  # start assuming everything bears
    for _ in range(iters):
        m = FEModel3D()
        m.add_material(mat, p["E"], p["E"] / 2.6, 0.3, p["rho"])
        m.add_section("bar", A, I, I, J)
        for i in used:
            m.add_node(f"N{i}", *nodes[i])
        for e in live:
            i, j = bars[e]
            m.add_member(f"M{e}", f"N{i}", f"N{j}", mat, "bar")
        for i in anch:
            k = k_strap[i] if lifting[i] else anchor_k[i]
            for d in "XYZ":
                m.def_support_spring(f"N{i}", f"D{d}", k, None)
        m.def_support(f"N{anch[0]}", support_RX=True, support_RY=True, support_RZ=True)
        for i, fvec in loads.items():
            if i in used:
                for ax, comp in zip("XYZ", fvec):
                    m.add_node_load(f"N{i}", f"F{ax}", float(comp))
        try:
            m.analyze_linear(check_statics=False)
        except Exception:
            return float("inf"), {}, 0.0, 0.0

        # who is lifting OFF the hand?  (displacement along its own outward normal)
        nxt, tension = {}, 0.0
        for i in anch:
            n = m.nodes[f"N{i}"]
            D = np.array([n.DX["Combo 1"], n.DY["Combo 1"], n.DZ["Combo 1"]])
            if not np.isfinite(D).all():
                return float("inf"), {}, 0.0, 0.0
            out = float(D @ normals[i])
            nxt[i] = out > 0
            if nxt[i]:
                tension += k_strap[i] * float(np.linalg.norm(D))
        if nxt == lifting:
            break
        lifting = nxt

    w = 0.0
    for f, i in buttons.items():
        if i not in used:
            return float("inf"), {}, 0.0, 0.0
        n = m.nodes[f"N{i}"]
        d = float(np.linalg.norm([n.DX["Combo 1"], n.DY["Combo 1"], n.DZ["Combo 1"]]))
        # ⚠ NEVER `max(w, d)` ON A VALUE THAT MIGHT BE NaN. Python's max returns the FIRST
        # argument when the comparison is False, and every comparison with NaN is False -- so a
        # singular solve came back as 0.0 um and read as a triumph.
        if not np.isfinite(d):
            return float("inf"), {}, 0.0, 0.0
        w = max(w, d)

    # STRAIN ENERGY PER UNIT VOLUME is the ESO criterion, not strain energy. A long bar stores more
    # energy than a short one at the same stress, so ranking on raw energy would delete the short
    # highly-stressed struts first -- exactly backwards.
    se, mass = {}, 0.0
    for e in live:
        M = m.members[f"M{e}"]
        d, fv = M.d("Combo 1"), M.f("Combo 1")
        L = float(np.linalg.norm(nodes[bars[e][0]] - nodes[bars[e][1]]))
        se[e] = float(abs(0.5 * (d.T @ fv)[0, 0])) / max(L, 1e-6)
        mass += A * L * p["rho"]
    return float(w), se, float(mass), float(tension)


def grow(h, q, hug=0.004, pitch=0.004, rate=0.12, gate=0.5e-3, mat="cf_pa12",
         press_N=0.196, on_step=None):
    """WOLFF'S LAW. Delete the bars that carry no load, until the buttons stop being crisp.

    Bone is not designed, it is grown: it lays down material where it is strained and resorbs it
    where it is not. This is the same rule -- remove the lowest strain-energy-density bars and
    re-solve -- and it is why the answer looks like an anatomy rather than like a bracket.
    """
    nodes, bars, btn, loads, ak, an = ground(h, q, hug=hug, pitch=pitch, press_N=press_N)
    live, reachable = connected(bars, list(range(len(bars))), ak, btn, len(nodes))
    if not reachable:
        raise RuntimeError("a button has no load path to the anchor: the domain is disconnected")
    w, se, mass, tens = solve(nodes, bars, live, btn, loads, ak, an, mat=mat)
    hist = [(len(live), w, mass, tens)]
    if on_step:
        on_step(0, live, w, mass, tens)

    cut, step = rate, 0
    while cut > 0.005 and len(live) > 50:
        step += 1
        order = sorted(live, key=lambda e: se.get(e, 0.0))
        drop = set(order[:max(1, int(cut * len(live)))])
        trial = [e for e in live if e not in drop]
        w2, se2, mass2, tens2 = solve(nodes, bars, trial, btn, loads, ak, an, mat=mat)
        if w2 > gate or not np.isfinite(w2):
            cut *= 0.5                      # took too much: back off and try a smaller bite
            continue
        live, w, se, mass, tens = trial, w2, se2, mass2, tens2
        hist.append((len(live), w, mass, tens))
        if on_step:
            on_step(step, live, w, mass, tens)
    return nodes, bars, live, btn, loads, ak, an, hist

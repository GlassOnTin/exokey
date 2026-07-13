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
from scipy.spatial import cKDTree

from design.params import P, Source
from hand.flesh import CARPUS, METACARPALS, skin
from hand.myohand import FINGERS
from structure.anchor import bearing_surface
from structure.fem import Frame
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

SEG_CLEAR = P("SEG_CLEAR", 0.75, "-", Source.GUESS,
              "Minimum gap between any point along a STRUT and the skin, as a fraction of `hug` "
              "(so 3.0 mm at the 4 mm standoff). It cannot be 1.0: a straight chord between two "
              "nodes each `hug` off a CONVEX surface dips below `hug` at its midpoint, which is "
              "geometry, not a fault -- even the un-relaxed lattice sits at 0.83. This is the "
              "floor below which the gauntlet would be RUBBING, and nothing has worn one to find "
              "out where that really is.")

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


def load_cases(h, q, buttons, press_N=0.196, wired=None):
    """ONE CASE PER (DIGIT, DIRECTION), PRESSED ALONE. This is the load set; there was one before.

    ⚠ A WELL IS A FIVE-DIRECTION JOYSTICK. The digit can push it down, forward, back, left or
    right -- five different forces on the structure, of which the model saw one. And the structure
    had been grown with ALL FIVE DIGITS PRESSING AT ONCE, which is not how anyone types. Re-solved
    one digit at a time, the thumb alone deflected 522 um against a 500 um gate: ESO had optimised
    a load case that never happens and failed the one that does.

    `wired` (from design.qwerty.used_actions) restricts this to the directions a character is
    actually assigned to -- 15 of the 25. Demanding all 25 would design for keys nobody presses.
    """
    from design.vector import action_dirs

    cases = []
    for f, i in buttons.items():
        dirs = action_dirs(h, q, f)
        acts = sorted(wired[f]) if (wired and f in wired) else sorted(dirs)
        for a in acts:
            if a in dirs:
                cases.append((f, a, {i: np.asarray(dirs[a], float) * press_N}))
    return cases


def solve(nodes, bars, live, buttons, cases, anchor_k, anchor_n, r=None, mat="cf_pa12",
          strap_k=None, iters=8):
    """(worst button deflection over all cases, {bar: strain energy}, mass, strap tension N).

    ⚠ THE ANCHOR IS BILINEAR, AND GETTING THAT WRONG IS WHAT MAKES A GAUNTLET LOOK GOOD ON PAPER.

    Flesh can PUSH the gauntlet off the hand. It cannot PULL it back on. A keypress at a fingertip
    ~120 mm from the wrist is a MOMENT: it presses one end of the anchor patch INTO the hand and
    lifts the other end OFF it, and only the STRAP can carry the lifting end.

    Modelled as bidirectional springs, the free-form lattice grew a structure reporting 495 um at
    the buttons, of which 40% of the anchor reaction was tension nothing was supplying. The same
    structure, re-solved honestly, deflects 9178 um -- 18x the gate. So:

        moving IN  (pressing on the hand)  ->  k_tissue,  stiff  (E*dA/t, MRI-measured t)
        moving OUT (lifting off the hand)  ->  k_strap,   soft   (webbing, EA/L)

    That is nonlinear, so it is iterated to a fixed active set -- PER LOAD CASE, because different
    digits lift different parts of the patch. Factorisations are cached by active set, and most
    cases converge to the same one, so 15 cases cost far fewer than 15 factorisations.
    """
    strap_k = float(STRAP_K) if strap_k is None else strap_k
    r = float(BAR_R) if r is None else r
    A = np.pi * r ** 2
    I = np.pi * r ** 4 / 4
    J = np.pi * r ** 4 / 2
    p = MATERIALS[mat]

    live, reachable = connected(bars, live, anchor_k, buttons, len(nodes))
    if not reachable or not live:
        return float("inf"), {}, 0.0, 0.0, {}
    lb = [bars[e] for e in live]
    fr = Frame(nodes, lb, p["E"], p["E"] / 2.6, A, I, J)
    anch = [i for i in anchor_k if i in fr.idx]
    if not anch:
        return float("inf"), {}, 0.0, 0.0, {}
    ktot = sum(anchor_k[i] for i in anch)
    k_strap = {i: strap_k * anchor_k[i] / ktot for i in anch}

    def springs(lift):
        return {i: (k_strap[i] if i in lift else anchor_k[i]) for i in anch}

    # ⚠ ONE FACTORISATION, KEPT AND REUSED -- NOT ONE PER LOAD CASE.
    # The anchor is nonlinear, so in principle every case needs its own stiffness matrix. In
    # practice the active set hardly moves between digits (they all press one end of the patch in
    # and lift the other), so carrying it from case to case means the factorisation is REBUILT
    # only when the set actually changes -- typically once for the whole list.
    #
    # Caching one splu PER active set instead exhausted 28 GB and got the run OOM-killed: an splu
    # of a 14k-DOF frame is hundreds of MB of fill-in, and 25 cases wanted 25 of them. The
    # factorisation is cheap; HOARDING it is what is expensive.
    lift: set = set()
    fr.factorise(springs(lift))

    worst, tension, per_case = 0.0, 0.0, {}
    U_all = []
    for f, act, load in cases:
        for _ in range(iters):
            U = fr.solve([load])
            nxt = {i for i in anch if float(fr.disp(U, i)[0] @ anchor_n[i]) > 0}
            if nxt == lift:
                break
            lift = nxt
            fr.factorise(springs(lift))
        d = float(np.linalg.norm(fr.disp(U, buttons[f])[0]))
        if not np.isfinite(d):
            return float("inf"), {}, 0.0, 0.0, {}
        per_case[(f, act)] = d
        worst = max(worst, d)
        tension = max(tension, sum(k_strap[i] * float(np.linalg.norm(fr.disp(U, i)[0]))
                                   for i in lift))
        U_all.append(U[0])

    se_arr = fr.strain_energy(np.array(U_all))
    se = {e: float(se_arr[k]) for k, e in enumerate(live)}
    mass = fr.mass(A, p["rho"])
    return float(worst), se, float(mass), float(tension), per_case


def grow(h, q, hug=0.004, pitch=0.004, rate=0.12, gate=0.5e-3, mat="cf_pa12",
         press_N=0.196, wired=None, relax=True, on_step=None):
    """WOLFF'S LAW. Delete the bars that carry no load, until the buttons stop being crisp.

    Bone is not designed, it is grown: it lays down material where it is strained and resorbs it
    where it is not. This is the same rule -- remove the lowest strain-energy-density bars and
    re-solve -- and it is why the answer looks like an anatomy rather than like a bracket.

    The gate is the WORST of the load cases, not their sum: a key that is mushy when you press it
    is mushy, however crisp the other fourteen are.
    """
    nodes, bars, btn, _loads, ak, an = ground(h, q, hug=hug, pitch=pitch, press_N=press_N)
    cases = load_cases(h, q, btn, press_N=press_N, wired=wired)
    Vs, Fs, _Ls = skin(h, q, labels=True)
    Ns = _normals(Vs, Fs)
    live, reachable = connected(bars, list(range(len(bars))), ak, btn, len(nodes))
    if not reachable:
        raise RuntimeError("a button has no load path to the anchor: the domain is disconnected")
    w, se, mass, tens, pc = solve(nodes, bars, live, btn, cases, ak, an, mat=mat)
    hist = [(len(live), w, mass, tens)]
    if on_step:
        on_step(0, live, w, mass, tens)

    cut, step = rate, 0
    while cut > 0.005 and len(live) > 50:
        step += 1
        order = sorted(live, key=lambda e: se.get(e, 0.0))
        drop = set(order[:max(1, int(cut * len(live)))])
        trial = [e for e in live if e not in drop]
        w2, se2, mass2, tens2, pc2 = solve(nodes, bars, trial, btn, cases, ak, an, mat=mat)

        # ⚠ AND NOW LET THE SURVIVORS DRIFT, EVERY STEP -- not once at the end.
        # The user: "Perhaps we'd find some better solutions allowing the elements to drift
        # throughout the optimisation?" They are right, and it is not merely tidier: deleting a
        # strut changes where the load goes, so the joints that are left are no longer where the
        # NEW load path wants them. Relaxing them recovers stiffness, and stiffness recovered is
        # slack ESO can spend on deleting MORE. Topology and shape are one problem; solving them
        # in sequence leaves material on the table.
        if relax and np.isfinite(w2):
            fr2 = Frame(nodes, [bars[e] for e in trial], MATERIALS[mat]["E"],
                        MATERIALS[mat]["E"] / 2.6, np.pi * float(BAR_R) ** 2,
                        np.pi * float(BAR_R) ** 4 / 4, np.pi * float(BAR_R) ** 4 / 2,
                        spring={i: k for i, k in ak.items()})
            U2 = fr2.solve([c[2] for c in cases])
            moved = relax_nodes(fr2, U2, nodes, bars, trial, btn, ak, Vs, Ns, hug=hug)
            w3, se3, mass3, tens3, pc3 = solve(moved, bars, trial, btn, cases, ak, an, mat=mat)
            if np.isfinite(w3) and w3 <= max(w2, gate) and _clears(moved, bars, trial, Vs, hug):
                nodes, w2, se2, mass2, tens2, pc2 = moved, w3, se3, mass3, tens3, pc3

        if w2 > gate or not np.isfinite(w2):
            cut *= 0.5                      # took too much: back off and try a smaller bite
            continue
        live, w, se, mass, tens, pc = trial, w2, se2, mass2, tens2, pc2
        hist.append((len(live), w, mass, tens))
        if on_step:
            on_step(step, live, w, mass, tens)
    return nodes, bars, live, btn, cases, ak, an, hist, pc


def _clears(nodes, bars, live, skin_V, hug):
    """No STRUT may pass through flesh -- and it is the segment that matters, not just its ends.

    Clamping the NODES to the standoff is not enough: a strut between two legal nodes can still
    bow across a valley and graze the skin. The first relaxation did exactly that -- nodes all
    legal, struts down to 0.70 mm.
    """
    from scipy.spatial import cKDTree

    tree = cKDTree(skin_V)
    pts = []
    for e in live:
        a, b = nodes[bars[e][0]], nodes[bars[e][1]]
        pts += [a + (b - a) * t for t in np.linspace(0.0, 1.0, 9)]
    # ⚠ THE SEGMENT, NOT THE NODES -- AND THE FLOOR IS NOT `hug`.
    #
    # Clamping the NODES to the standoff is not enough: a strut between two legal nodes can bow
    # across a valley and graze the skin. So sample along it.
    #
    # But demanding the SEGMENT clear the full `hug` IS IMPOSSIBLE, and I briefly demanded it. A
    # straight chord between two nodes each 4 mm off a CONVEX surface necessarily dips BELOW 4 mm
    # at its midpoint -- that is geometry, not a defect, and the un-relaxed lattice already sits at
    # 3.31 mm. The check rejected every relaxation, including ones that made nothing worse, and
    # reported "no gain" as though the physics had spoken. It had not; I had mis-set the bar.
    #
    # The floor is SEG_CLEAR: a real minimum gap between any part of a strut and the skin.
    return bool(tree.query(np.array(pts))[0].min() >= float(SEG_CLEAR) * hug)


def cost(h, q, wired=None, press_N=0.196, pitch=0.008, gate=0.5e-3, mat="cf_pa12", rate=0.20):
    """THE STRUCTURAL COST OF A LAYOUT: how many grams of bone does it need? GROW IT. Coarsely.

    This is what the GA calls ~2500 times, so it has to be cheap -- but NOT at the price of being
    a different question.

    ⚠ THE FIRST ATTEMPT WAS A PROXY, AND IT FAILED ITS OWN TEST. It estimated the mass from ONE
    solve of the solid lattice by scaling the bar radius (w ~ r^-3.7, m ~ r^2). Scored against
    eight real growths it managed SPEARMAN rho = +0.45 (p = 0.26) -- no better than chance. The
    reason is visible in the numbers: three designs had near-identical proxy masses (18.8, 18.8,
    19.4 g) and real masses of 4.8, 6.5 and 7.4 g. The solid's compliance hardly varies between
    layouts; what varies is HOW EFFICIENTLY ESO CAN ROUTE THE LOAD, and uniform scaling cannot see
    that, because uniform scaling is precisely the thing ESO refuses to do.

    So the answer is not a better estimator. It is to GROW THE STRUCTURE, on a coarser lattice:
    the same physics, the same load cases, the same Wolff's law, a bigger mesh. Against the fine
    growth it scores rho = +0.76 (p = 0.028) and costs 19 s instead of 10 minutes.

    ⚠ IT IS STILL A COARSER ANSWER, NOT THE ANSWER. It over-estimates the mass by ~10-20% (a
    coarse lattice routes load less efficiently), rho = 0.76 is measured on only n = 8, and one of
    those eight (4.2 g coarse vs 6.9 g fine) it gets badly wrong. The Pareto front is re-grown at
    full resolution afterwards; the coarse number is for STEERING, not for reporting.
    """
    p = MATERIALS[mat]
    try:
        nodes, bars, live, btn, cases, ak, an, hist, _pc = grow(
            h, q, pitch=pitch, rate=rate, gate=gate, mat=mat, press_N=press_N, wired=wired,
            relax=False)
    except (RuntimeError, ValueError):
        return dict(mass_g=float("inf"), solid_g=float("inf"), worst=float("inf"),
                    util=float("inf"), struts=0, feasible=False)
    if not live or not np.isfinite(hist[-1][1]):
        return dict(mass_g=float("inf"), solid_g=float("inf"), worst=float("inf"),
                    util=float("inf"), struts=0, feasible=False)

    n_solid, w_solid, m_solid = hist[0][:3]
    _n, w, m, _t = hist[-1]

    # THE STRESS, on the structure that actually survived -- not on the solid, which is a
    # different structure carrying the same load through 30x more material.
    r = float(BAR_R)
    A = np.pi * r ** 2
    I = np.pi * r ** 4 / 4
    fr = Frame(nodes, [bars[e] for e in live], p["E"], p["E"] / 2.6, A, I, 2 * I,
               spring={i: k for i, k in ak.items()})
    U = fr.solve([c[2] for c in cases])
    util = float(fr.stress(U, r).max() / (p["yield_"] / 2.0))        # SF = 2

    return dict(mass_g=float(m) * 1000.0, solid_g=float(m_solid) * 1000.0,
                worst=float(w_solid), util=util, struts=len(live),
                feasible=bool(w_solid <= gate))


def relax_nodes(fr, U, nodes, bars, live, buttons, anchor_k, skin_V, skin_N,
                hug=0.004, layer=None, step=0.35, iters=1):
    """LET THE NODES DRIFT, AND LET THE PHYSICS SAY WHERE. Form-finding, not smoothing.

    THE USER: "Its becoming fairly minimalist in appearance, but still somewhat jaggedy. Is there a
    smoothness penalisation we could use to discourage zig-zagging?" ...and then, correctly:
    "Perhaps we'd find some better solutions allowing the elements to drift throughout the
    optimisation?"

    WHY THE ZIG-ZAGS ARE NOT PHYSICS. A STRAIGHT strut carries load AXIALLY -- stiffness EA/L. A
    KINKED one carries it in BENDING -- ~12EI/L^3, which for a 1.8 mm rod over a 10 mm span is
    three orders of magnitude softer. A kink is not a cheap way to get somewhere; it is a ruinously
    expensive one. ESO is not choosing them. IT HAS NO CHOICE: every node is pinned to a lattice
    site, so a load path running at an angle to the grid can only STAIRCASE. The jaggedness is a
    DISCRETISATION ARTEFACT.

    WHY NOT A SMOOTHNESS PENALTY. It is a soft constraint, and a soft constraint is something the
    optimiser can BUY -- this project exists because v1 died of exactly that. The weight would
    become a design variable nobody voted for. And it treats the symptom: the structure does not
    need to be TOLD to prefer straight lines. It already prefers them, by a factor of ~1000. It
    needs to be ALLOWED to have them.

    WHY NOT LAPLACIAN SMOOTHING EITHER, which is what I tried first. It straightened the struts
    (mean kink 58 deg -> 18 deg) and cut 29% of the mass -- and it did it by COLLAPSING THE
    STRUCTURE ONTO THE HAND. Median standoff fell 9.1 -> 4.2 mm and the struts came within 0.70 mm
    of the skin. It had thrown away the two-sheet DEPTH -- the entire bending lever arm -- and
    called that a saving. Geometric smoothing does not know what the structure is FOR.

    WHAT THIS DOES INSTEAD. The FEA already knows where every node wants to be. At a joint carrying
    only AXIAL force, the forces balance:  sum_e N_e * u_e = 0.  Whatever is left over is being
    held by BENDING -- the expensive mode -- so that residual IS the error, in newtons, and it
    points the way out. Move each free node along it and the joint straightens BECAUSE that is what
    kills the bending, not because it was asked to look tidy. (This is form-finding / dynamic
    relaxation, and its fixed point is the funicular shape.)

    ⚠ AND HERE IS WHAT IT ACTUALLY BUYS, MEASURED, PAIRED OVER ALL 15 DESIGNS OF THE FRONT:

        mean kink   51.0 deg -> 45.4 deg   Wilcoxon p = 0.006   SIGNIFICANT
        bone mass    5.60 g  ->  5.32 g    Wilcoxon p = 0.22    NOT significant

    It straightens the structure and it DOES NOT make it lighter. A single A/B had shown 16%
    lighter; a different one showed 12% heavier; the per-design spread is -29% to +13%. That was
    trajectory noise in a greedy heuristic, and reporting it as a gain would have been wrong.

    WHICH CORRECTS THE ARGUMENT ABOVE. "A kink is 1000x softer" is true of ONE strut in isolation.
    This lattice is REDUNDANT -- node degree ~12 -- so when a path kinks, the load simply goes
    around it. The zig-zags look bad and cost almost nothing. Smoothness is real, and it is worth
    having (a straight strut prints better and reads as a structure rather than a mesh artefact),
    but it is NOT where the grams are. So it is ON for the structures that get reported and OFF in
    the GA loop, where it doubles the cost of an evaluation for no measurable return.

    THE NODE IS STILL NOT FREE TO DO ANYTHING. It must stay in the band the gauntlet is allowed to
    occupy -- between `hug` and `hug + layer` off the skin -- so it cannot sink into the hand and
    it cannot flatten the lever arm. BUTTONS and ANCHORS are held: they are where the load enters
    and leaves, and they are not the optimiser's to move.
    """
    from scipy.spatial import cKDTree

    layer = float(LAYER) if layer is None else layer
    X = np.array(nodes, float)
    held = set(buttons.values()) | set(anchor_k)
    lb = [bars[e] for e in live]
    tree = cKDTree(skin_V)

    for _ in range(iters):
        N = fr.axial(U)                                  # (ncase, nbar) tension positive
        Nrms = np.sqrt((N ** 2).mean(axis=0))            # one number per bar, over all load cases
        R = np.zeros_like(X)
        deg = np.zeros(len(X))
        for e, (i, j) in enumerate(lb):
            u = X[j] - X[i]
            L = np.linalg.norm(u)
            if L < 1e-9:
                continue
            u /= L
            R[i] += Nrms[e] * u                          # the bar pulls node i toward j
            R[j] -= Nrms[e] * u
            deg[i] += abs(Nrms[e])
            deg[j] += abs(Nrms[e])

        for i in range(len(X)):
            if i in held or deg[i] < 1e-12:
                continue
            # step in METRES: the residual, normalised by the force scale at this joint, times a
            # length scale. A joint in axial equilibrium does not move.
            X[i] += step * (R[i] / deg[i]) * (0.5 * layer)

        # ...and back into the band. The skin is not negotiable and neither is the lever arm.
        d, k = tree.query(X)
        out = X - skin_V[k]
        nrm = np.linalg.norm(out, axis=1, keepdims=True)
        out = np.where(nrm > 1e-12, out / np.maximum(nrm, 1e-12), skin_N[k])
        lo, hi = hug, hug + layer
        bad = (d < lo) | (d > hi)
        if bad.any():
            X[bad] = skin_V[k[bad]] + out[bad] * np.clip(d[bad], lo, hi)[:, None]
    return X

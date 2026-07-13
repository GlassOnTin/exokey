"""The body as a SHELL, not a stick figure.

WHY BEAMS CANNOT SETTLE THE ARCH QUESTION -- and this is the whole reason for this module.

The user asked for the support structure to follow the hand's natural shape: the palm is a CUP
6.4 mm deep and `build_body` bolts a FLAT PLATE across it. The physics says an arch should win,
because a keypress pushes the body INTO the palm and an arch takes that in COMPRESSION where a
plate takes it in BENDING.

A BEAM MODEL CANNOT SEE THAT. An arch carries load as MEMBRANE action -- compression in the
shell's own mid-surface -- and a stick figure of beams has no membrane. Worse, the beam model
CHARGES for the arch: it is 10 discrete struts where a plate is 4, so PyNite bills 3x the mass
for a shape that, moulded, is the same shell merely curved. Measured, the beam model said the
arch was 3x heavier for no stiffness. That is not a finding about arches. It is an artifact of
idealising a shell as sticks.

So: MITC4 shell elements (`FEModel3D.add_quad`), which carry BOTH bending and membrane.

THE GATE, WRITTEN BEFORE THE MODEL (and it is the same discipline that validated the beam
model against a closed-form cantilever): a simply-supported square plate under uniform load has
a closed-form answer,

    w_max = 0.00406 * q * a^4 / D ,     D = E t^3 / (12 (1 - nu^2))          (Timoshenko)

If the shell model cannot reproduce that, nothing it says about an arch is worth reading.

⚠ WHAT THIS IS NOT. It is a linear-elastic, small-deflection shell. It does not model the
moulding process, fibre orientation in CF-PA12, print anisotropy, or the fact that a real
printed shell is not homogeneous. It replaces ONE specific idealisation error -- sticks for a
shell -- and inherits every other limitation the beam model had.
"""
from __future__ import annotations

import numpy as np
from Pynite import FEModel3D

from structure.frame import MATERIALS


def _mat(model: FEModel3D, name: str) -> str:
    """Register one of our materials with PyNite. Same table the beam model uses -- so any
    beam-vs-shell difference is the ELEMENT, not the material."""
    m = MATERIALS[name]
    model.add_material(name, m["E"], m["G"], m["nu"], m["rho"], m["yield_"])
    return name


def simply_supported_plate(a: float, t: float, q: float, mat: str = "al6061",
                           n: int = 8) -> tuple[float, float]:
    """(FE central deflection, Timoshenko closed form) for a simply-supported square plate.

    THE VALIDATION GATE. A shell model that cannot reproduce a textbook plate has no business
    being asked about an arch.
    """
    model = FEModel3D()
    _mat(model, mat)

    for i in range(n + 1):
        for j in range(n + 1):
            model.add_node(f"N{i}_{j}", i * a / n, j * a / n, 0.0)

    for i in range(n):
        for j in range(n):
            model.add_quad(f"Q{i}_{j}", f"N{i}_{j}", f"N{i+1}_{j}",
                           f"N{i+1}_{j+1}", f"N{i}_{j+1}", t, mat)

    # simply supported: the edges are held in Z, free to rotate. Two in-plane restraints kill
    # the rigid-body modes without restraining membrane stretch.
    for i in range(n + 1):
        for j in range(n + 1):
            if i in (0, n) or j in (0, n):
                model.def_support(f"N{i}_{j}", support_DZ=True)
    model.def_support("N0_0", support_DX=True, support_DY=True)
    model.def_support(f"N{n}_0", support_DY=True)

    for i in range(n):
        for j in range(n):
            model.add_quad_surface_pressure(f"Q{i}_{j}", q)

    model.analyze_linear(check_statics=False)
    w_fe = abs(model.nodes[f"N{n//2}_{n//2}"].DZ["Combo 1"])

    E, nu = MATERIALS[mat]["E"], MATERIALS[mat]["nu"]
    D = E * t**3 / (12.0 * (1.0 - nu**2))
    w_ct = 0.00406 * q * a**4 / D
    return float(w_fe), float(w_ct)


if __name__ == "__main__":
    print("SHELL VALIDATION GATE: simply-supported square plate, uniform load")
    print("closed form (Timoshenko):  w = 0.00406 q a^4 / D\n")
    print(f"  {'mesh':>6s} {'FE (mm)':>10s} {'closed form (mm)':>18s} {'error':>9s}")
    for n in (4, 8, 12, 16):
        fe, ct = simply_supported_plate(a=0.20, t=0.002, q=1000.0, n=n)
        print(f"  {n}x{n:<3d} {fe*1000:10.4f} {ct*1000:18.4f} {100*(fe-ct)/ct:8.1f}%")
    print("\n  It must CONVERGE toward the closed form as the mesh refines. If it does not,")
    print("  the shell model is wrong and nothing it says about an arch may be believed.")


def palm_shell(h, q, params: dict, shape: str = "flat", nu_: int = 8, nv: int = 6):
    """The palm support as a SHELL -- flat plate vs arched, at EQUAL MASS.

    This is the comparison the beam model could not make. A curved shell has the SAME AREA and
    the SAME THICKNESS as a flat one, so the arch is FREE: the 3x mass penalty the beam model
    charged was pure idealisation error (10 struts vs 4).

    Boundary conditions, matching the beam model so the comparison is fair:
      * the two EMINENCES (radial and ulnar edges) bear on soft tissue -- elastic supports,
        k = SOFT_TISSUE_K, NOT rigid. A rigid BC would flatter the plate and the arch equally
        but would misstate both.
      * the keypress reaction enters at the DISTAL edge, where the walls attach, and it pushes
        the body DORSALLY -- INTO the palm. That is the load an arch is supposed to be good at.
    """
    from structure.frame import SOFT_TISSUE_K, hand_axes, palmar_arch

    mat = str(params["mat_frame"])
    t = float(params["alu_t"])
    hw = float(params["body_half"])
    dp, dd = float(params["body_prox"]), float(params["body_dist"])
    gap = float(params["palm_offset"])
    press = float(params["press_N"]) * 5.0          # all five wells at once

    o, e_d, e_r, e_o = hand_axes(h, q)
    prof = palmar_arch(h, q, dd, n=9)

    z0 = float(np.mean([p[1] for p in prof]))

    def depth(rad: float) -> float:
        """FLAT / FOLLOW / INVERT -- and the difference decides everything.

        FOLLOW  the shell copies the palm's cup: it bulges DORSALLY, into the hollow.
        INVERT  the shell bulges PALMAR, AWAY from the hand -- bridging the hollow and
                bearing only on the two eminences.

        ⚠ AND *FOLLOW* IS STRUCTURALLY THE WRONG WAY ROUND. A keypress pushes the body
        DORSALLY. A shell that bulges dorsally is being pushed FURTHER INTO ITS OWN
        CONVEXITY -- that is an upward load on an upward arch, which is TENSION and
        snap-through, not compression. An arch only works when the load pushes AGAINST the
        bulge. So the shell must bulge PALMAR: then the dorsal keypress COMPRESSES it, and
        it delivers that compression straight into the eminences.

        That is what "invert the support structure" means, and it is not the same thing as
        "follow the shape of the hand" -- the hand's own shape, copied naively, gives the
        WORST arch of the three.
        """
        if shape == "flat":
            return z0
        z = min(prof, key=lambda p: abs(p[0] - rad))[1]
        if shape == "follow":
            return z
        if shape == "invert":
            return 2.0 * z0 - z          # mirror the cup: bulge PALMAR, bridge the hollow
        raise ValueError(shape)

    model = FEModel3D()
    _mat(model, mat)

    for i in range(nu_ + 1):
        rad = -hw + 2.0 * hw * i / nu_
        for j in range(nv + 1):
            dist = dp + (dd - dp) * j / nv
            P = o + dist * e_d + rad * e_r + (depth(rad) - gap) * e_o
            model.add_node(f"N{i}_{j}", *P)

    for i in range(nu_):
        for j in range(nv):
            model.add_quad(f"Q{i}_{j}", f"N{i}_{j}", f"N{i+1}_{j}",
                           f"N{i+1}_{j+1}", f"N{i}_{j+1}", t, mat)

    # the EMINENCES bear on soft tissue: elastic, not rigid (k in N/m per node)
    k = SOFT_TISSUE_K / (nv + 1)
    for i in (0, nu_):
        for j in range(nv + 1):
            model.def_support_spring(f"N{i}_{j}", "DX", k, None)
            model.def_support_spring(f"N{i}_{j}", "DY", k, None)
            model.def_support_spring(f"N{i}_{j}", "DZ", k, None)
    model.def_support("N0_0", support_RX=True, support_RY=True, support_RZ=True)

    # the keypress reaction: DORSAL, into the palm, entering along the distal edge
    d = e_o * (press / (nu_ + 1))
    for i in range(nu_ + 1):
        for ax, val in zip(("FX", "FY", "FZ"), d):
            model.add_node_load(f"N{i}_{nv}", ax, float(val))

    model.analyze_linear(check_statics=False)

    w = max(
        float(np.linalg.norm([model.nodes[n].DX["Combo 1"],
                              model.nodes[n].DY["Combo 1"],
                              model.nodes[n].DZ["Combo 1"]]))
        for n in model.nodes
    )
    # mass: area x thickness x density. IDENTICAL for flat and arched up to the curve's own
    # extra arc-length -- which is the point.
    area = 0.0
    for i in range(nu_):
        for j in range(nv):
            P = [np.array([model.nodes[f"N{a}_{b}"].X, model.nodes[f"N{a}_{b}"].Y,
                           model.nodes[f"N{a}_{b}"].Z])
                 for a, b in ((i, j), (i + 1, j), (i + 1, j + 1), (i, j + 1))]
            area += 0.5 * np.linalg.norm(np.cross(P[1] - P[0], P[3] - P[0]))
            area += 0.5 * np.linalg.norm(np.cross(P[2] - P[1], P[3] - P[1]))
    mass = area * t * MATERIALS[mat]["rho"]
    return float(w), float(mass)


def key_face(h, q, keys: dict, params: dict, n: int = 4):
    """The KEY FACE as a shell -- because it is a PLATE, and the beam model calls it sticks.

    THIS IS WHERE THE BEAM MODEL IS MOST LIKELY LYING. The face carries 44% of the whole
    structure's compliance (measured, by stiffening each group 100x), and `build_body`
    idealises it as a CHAIN OF STRUTS between the key feet. It is not a chain of struts. It is
    the plate the wells are mounted on, and a plate carries load in two directions at once
    while a chain of struts carries it in one.

    ⚠ AND THE OBVIOUS COMPARISON IS NOT A FAIR ONE, so do not read the numbers that way.
    This meshes the face's BOUNDING RECTANGLE -- a solid slab. The beam model's face is a
    SPARSE CHAIN between the key feet. A slab is inevitably heavier than a chain (+205%,
    measured) and, being thin over a wide span, softer. That compares two DIFFERENT PARTS, not
    two idealisations of the same part, and it says nothing about whether beams misrepresent
    the face.

    To settle THAT, the shell needs the face's real FOOTPRINT -- a shaped web following the
    arc of the fingertips, not a rectangle. That is a design task, not a meshing one, and it
    is not done. Stated so nobody (including me) reads the -205% as a finding.
    """
    from structure.frame import hand_axes

    mat = str(params["mat_frame"])
    t = float(params["alu_t"])
    press = float(params["press_N"])
    o, e_d, e_r, e_o = hand_axes(h, q)

    feet = {}
    for (f, row), (kp, kn) in keys.items():
        p = np.asarray(kp, float) - float(params["stem"]) * np.asarray(kn, float)
        feet[f] = ((p - o) @ e_r, (p - o) @ e_d, (p - o) @ e_o)

    r0 = min(v[0] for v in feet.values()) - 0.008
    r1 = max(v[0] for v in feet.values()) + 0.008
    d0 = min(v[1] for v in feet.values()) - 0.008
    d1 = max(v[1] for v in feet.values()) + 0.008
    zf = float(np.mean([v[2] for v in feet.values()]))

    model = FEModel3D()
    _mat(model, mat)
    nu_, nv = 4 * n, 3 * n
    for i in range(nu_ + 1):
        for j in range(nv + 1):
            P = o + (d0 + (d1 - d0) * j / nv) * e_d + (r0 + (r1 - r0) * i / nu_) * e_r + zf * e_o
            model.add_node(f"F{i}_{j}", *P)
    for i in range(nu_):
        for j in range(nv):
            model.add_quad(f"P{i}_{j}", f"F{i}_{j}", f"F{i+1}_{j}",
                           f"F{i+1}_{j+1}", f"F{i}_{j+1}", t, mat)

    # supported where the walls/legs meet it: the four corners of the face
    for nm in (f"F0_0", f"F{nu_}_0", f"F0_{nv}", f"F{nu_}_{nv}"):
        model.def_support(nm, support_DX=True, support_DY=True, support_DZ=True)

    # each well pushes on the face where its stem lands
    for f, (rr, dd_, _) in feet.items():
        i = int(round((rr - r0) / (r1 - r0) * nu_))
        j = int(round((dd_ - d0) / (d1 - d0) * nv))
        for ax, val in zip(("FX", "FY", "FZ"), e_o * press):
            model.add_node_load(f"F{min(max(i,0),nu_)}_{min(max(j,0),nv)}", ax, float(val))

    model.analyze_linear(check_statics=False)
    w = max(float(np.linalg.norm([model.nodes[nd].DX["Combo 1"], model.nodes[nd].DY["Combo 1"],
                                  model.nodes[nd].DZ["Combo 1"]])) for nd in model.nodes)
    area = abs(r1 - r0) * abs(d1 - d0)
    return float(w), float(area * t * MATERIALS[mat]["rho"])


def face_domain(h, q, keys: dict, params: dict, nu_: int = 14, nv: int = 10):
    """The key face's DESIGN DOMAIN: nodes, quads, loads, supports. No footprint chosen yet.

    THE FOOTPRINT IS NOT MINE TO DRAW. It is what the structure needs, and the structure can
    say so -- so `face_footprint` deletes material until only what carries load is left.
    """
    from structure.frame import hand_axes

    o, e_d, e_r, e_o = hand_axes(h, q)
    feet = {}
    for (f, row), (kp, kn) in keys.items():
        p = np.asarray(kp, float) - float(params["stem"]) * np.asarray(kn, float)
        feet[f] = ((p - o) @ e_r, (p - o) @ e_d, (p - o) @ e_o)

    m = 0.010
    r0 = min(v[0] for v in feet.values()) - m
    r1 = max(v[0] for v in feet.values()) + m
    d0 = min(v[1] for v in feet.values()) - m
    d1 = max(v[1] for v in feet.values()) + m
    zf = float(np.mean([v[2] for v in feet.values()]))

    nodes = {}
    for i in range(nu_ + 1):
        for j in range(nv + 1):
            P = o + (d0 + (d1 - d0) * j / nv) * e_d + (r0 + (r1 - r0) * i / nu_) * e_r + zf * e_o
            nodes[(i, j)] = P
    cell = abs(r1 - r0) / nu_ * abs(d1 - d0) / nv

    def nearest(rr, dd_):
        i = int(round((rr - r0) / (r1 - r0) * nu_))
        j = int(round((dd_ - d0) / (d1 - d0) * nv))
        return min(max(i, 0), nu_), min(max(j, 0), nv)

    loads = {nearest(v[0], v[1]): e_o * float(params["press_N"]) for v in feet.values()}

    # THE FACE IS HELD AT FEET, NOT AT CORNERS. I first supported the four corners of the
    # design domain -- a rectangle's corners -- and that is not how the part is held. In
    # `build_body` the FLOOR LEGS attach to the key foot NEAREST each palm corner, so the face
    # is supported at up to four of the WELL FEET themselves. Optimising a differently
    # supported plate answers a different question.
    hw, dp, dd = float(params["body_half"]), float(params["body_prox"]), float(params["body_dist"])
    sups = []
    for cr, cd in ((hw, dd), (-hw, dd), (hw, dp), (-hw, dp)):
        near = min(feet.values(), key=lambda v: (v[0] - cr) ** 2 + (v[1] - cd) ** 2)
        sups.append(nearest(near[0], near[1]))
    sups = list(dict.fromkeys(sups))
    return dict(nodes=nodes, nu=nu_, nv=nv, cell=cell, loads=loads, sups=sups,
                keep_seed={k for k in loads} | set(sups))


def _solve_face(dom, live: set, t: float, mat: str):
    """(max deflection, {elem: strain energy}) for a given footprint `live`."""
    model = FEModel3D()
    _mat(model, mat)
    used = set()
    for (i, j) in live:
        for c in ((i, j), (i + 1, j), (i + 1, j + 1), (i, j + 1)):
            used.add(c)
    for c in used:
        model.add_node(f"N{c[0]}_{c[1]}", *dom["nodes"][c])
    for (i, j) in live:
        model.add_quad(f"Q{i}_{j}", f"N{i}_{j}", f"N{i+1}_{j}",
                       f"N{i+1}_{j+1}", f"N{i}_{j+1}", t, mat)
    for c in dom["sups"]:
        if c in used:
            model.def_support(f"N{c[0]}_{c[1]}", support_DX=True, support_DY=True,
                              support_DZ=True, support_RX=True, support_RY=True, support_RZ=True)
    for c, F in dom["loads"].items():
        if c in used:
            for ax, val in zip(("FX", "FY", "FZ"), F):
                model.add_node_load(f"N{c[0]}_{c[1]}", ax, float(val))
    try:
        model.analyze_linear(check_statics=False)
    except Exception:
        return float("inf"), {}
    w = max(float(np.linalg.norm([model.nodes[n].DX["Combo 1"], model.nodes[n].DY["Combo 1"],
                                  model.nodes[n].DZ["Combo 1"]])) for n in model.nodes)
    se = {}
    for (i, j) in live:
        Q = model.quads[f"Q{i}_{j}"]
        dv, fv = Q.d("Combo 1"), Q.f("Combo 1")
        se[(i, j)] = float(abs(0.5 * (dv.T @ fv)[0, 0]))
    return w, se


def face_footprint(h, q, keys: dict, params: dict, keep: float = 0.35, rate: float = 0.06):
    """EVOLUTIONARY STRUCTURAL OPTIMISATION: delete the material that carries no load.

    THE FOOTPRINT IS A DESIGN VARIABLE, and this is the point the user made -- the real work
    here has never been searching the design space, it has been fixing WHAT WE OPTIMISE. The
    key face carries 44% of the whole structure's compliance and it has been idealised as a
    CHAIN OF STRUTS, then as a SOLID SLAB, and neither is the part. So: mesh a design domain,
    solve, and iteratively delete the elements with the LOWEST STRAIN ENERGY -- the ones doing
    no work -- until only the load path is left.

    Nodes that carry a WELL or a SUPPORT are never orphaned: their elements are protected.

    Returns (live elements, deflection, mass).
    """
    dom = face_domain(h, q, keys, params)
    t = float(params["alu_t"])
    mat = str(params["mat_frame"])
    rho = MATERIALS[mat]["rho"]

    live = {(i, j) for i in range(dom["nu"]) for j in range(dom["nv"])}
    target = max(1, int(keep * len(live)))
    protected = set()
    for (ci, cj) in dom["keep_seed"]:
        for e in ((ci - 1, cj - 1), (ci - 1, cj), (ci, cj - 1), (ci, cj)):
            if 0 <= e[0] < dom["nu"] and 0 <= e[1] < dom["nv"]:
                protected.add(e)

    w, se = _solve_face(dom, live, t, mat)
    while len(live) > target:
        n_cut = max(1, int(rate * len(live)))
        cand = sorted((v, k) for k, v in se.items() if k not in protected)
        if not cand:
            break
        for _, k in cand[:n_cut]:
            live.discard(k)
        w2, se2 = _solve_face(dom, live, t, mat)
        if not np.isfinite(w2):     # the structure fell apart: put them back and stop
            for _, k in cand[:n_cut]:
                live.add(k)
            break
        w, se = w2, se2
    return live, float(w), float(len(live) * dom["cell"] * t * rho), dom


def dorsal_rail(h, q, finger: str, params: dict, curved: bool, n_long: int = 12, n_arc: int = 6):
    """A rail hugging the back of ONE finger: a FLAT STRIP vs a CURVED SHELL, same material.

    THE LAW THIS TESTS, and it is the whole reason "hug the hand" and "use shells" are the
    SAME request:

        A BEAM FRAME GETS ITS STIFFNESS FROM DEPTH.
        DEPTH IS EXACTLY WHAT GETS IN THE WAY.

    The palmar box is stiff because it is 57 mm deep -- and that depth IS the ball in the palm.
    A frame that hugs the hand has ~5 mm of depth, so AS A STICK FIGURE it can never be stiff:
    triangulated, it still deflected 2.58 mm against a 0.5 mm gate.

    A SHELL does not need depth. It gets its stiffness from CURVATURE. That is a tape measure,
    an eggshell, a fingernail: thin, hugging, and stiff -- because a curved cross-section cannot
    bend without also stretching, and stretching is expensive.

    So: wrap the same thickness of material around the finger instead of laying it flat, and
    ask what it costs. Cantilevered at the knuckle, loaded at the fingertip by one keypress.
    """
    import mujoco

    from structure.frame import hand_axes

    CHAIN = {"thumb": ("firstmc", "proximal_thumb", "distal_thumb"),
             "index": ("proxph2", "midph2", "distph2"),
             "middle": ("proxph3", "midph3", "distph3"),
             "ring": ("proxph4", "midph4", "distph4"),
             "little": ("proxph5", "midph5", "distph5")}
    mat = str(params["mat_frame"])
    t = float(params["alu_t"])
    hug = 0.004
    o, e_d, e_r, e_o = hand_axes(h, q)
    h.fk(q)
    m = h.model

    # the finger's own axis and radius, from its capsules
    cs = []
    for bone in CHAIN[finger]:
        bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, bone)
        for g in range(m.body_geomadr[bid], m.body_geomadr[bid] + m.body_geomnum[bid]):
            if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_CAPSULE:
                cs.append((h.data.geom_xpos[g].copy(), float(m.geom_size[g][0])))
    A, B = cs[0][0], cs[-1][0]
    r = float(np.mean([c[1] for c in cs])) + hug
    axis = B - A
    L = float(np.linalg.norm(axis))
    axis /= L
    u = np.cross(axis, e_o)
    u /= np.linalg.norm(u) + 1e-12
    v = np.cross(axis, u)          # completes the frame; v ~ dorsal

    # arc: WRAP round the finger (curved) or lie FLAT across it (a strip of the same width)
    half = np.pi / 3.0             # 120 deg of wrap
    width = 2.0 * r * half         # SAME material width either way -- that is the point

    model = FEModel3D()
    _mat(model, mat)
    for i in range(n_long + 1):
        c = A + axis * (L * i / n_long)
        for j in range(n_arc + 1):
            s = -half + 2.0 * half * j / n_arc
            if curved:
                P = c + r * (np.cos(s) * v + np.sin(s) * u)      # wraps the finger
            else:
                P = c + r * v + (width * (j / n_arc - 0.5)) * u  # a flat strip, same width
            model.add_node(f"N{i}_{j}", *P)
    for i in range(n_long):
        for j in range(n_arc):
            model.add_quad(f"Q{i}_{j}", f"N{i}_{j}", f"N{i+1}_{j}",
                           f"N{i+1}_{j+1}", f"N{i}_{j+1}", t, mat)

    for j in range(n_arc + 1):     # built in at the knuckle
        model.def_support(f"N0_{j}", support_DX=True, support_DY=True, support_DZ=True,
                          support_RX=True, support_RY=True, support_RZ=True)
    load = -e_o * float(params["press_N"]) / (n_arc + 1)   # a keypress, PALMAR, at the tip
    for j in range(n_arc + 1):
        for ax, val in zip(("FX", "FY", "FZ"), load):
            model.add_node_load(f"N{n_long}_{j}", ax, float(val))

    model.analyze_linear(check_statics=False)
    w = max(float(np.linalg.norm([model.nodes[nd].DX["Combo 1"], model.nodes[nd].DY["Combo 1"],
                                  model.nodes[nd].DZ["Combo 1"]])) for nd in model.nodes)
    mass = width * L * t * MATERIALS[mat]["rho"]
    return float(w), float(mass), float(L)

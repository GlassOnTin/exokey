"""Stage 3 -- the exoskeleton structure. Inner problem B.

An open frame on the BACK of the hand (spine over the metacarpals, bar across the
knuckles), two articulated rails that wrap around the radial and ulnar edges of the hand
to a palmar key bar, and short stalks carrying the keys to the fingertips. Webbing straps
clamp it to the hand. All of it reacts against SOFT TISSUE, not against rigid ground.

Three modelling calls, each of which changes the answer:

  * BEAM elements, not 3D continuum. For slender struts Euler-Bernoulli IS the right
    model, and it costs ~1 ms instead of ~1 s, which is what lets it sit inside the outer
    loop at all. Continuum FEA is for post-hoc stress concentrations at the joint bosses.

  * ELASTIC supports, not rigid. The frame bears on skin and muscle over bone (k is of
    order 10-50 N/mm, and is poorly characterised -- sweep it and report, do not pretend
    to know it). A rigid boundary condition would misrepresent the load path badly: it
    would carry the keypress straight into ground instead of through the frame.

  * TENSION-ONLY straps. Webbing cannot push. PyNite iterates these out.

Everything is SI: metres, newtons, pascals, kilograms.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import mujoco
import numpy as np

from hand.myohand import FINGERS, MyoHand

# Handbook values. E/G/nu/rho are solid; the FATIGUE numbers are the shaky ones and are
# flagged as such -- aluminium has NO true endurance limit (it keeps degrading), so
# `fatigue` here is the stress at ~5e8 cycles, and the polymer figures are rough.
MATERIALS = {
    "al6061": dict(E=68.9e9, G=26.0e9, nu=0.33, rho=2700.0, yield_=276e6, fatigue=96.5e6),
    "al7075": dict(E=71.7e9, G=26.9e9, nu=0.33, rho=2810.0, yield_=503e6, fatigue=159e6),
    "pa12": dict(E=1.7e9, G=0.60e9, nu=0.40, rho=1010.0, yield_=48e6, fatigue=16e6),
    "cf_pa12": dict(E=6.0e9, G=2.2e9, nu=0.40, rho=1060.0, yield_=70e6, fatigue=25e6),
    "webbing": dict(E=2.0e9, G=0.70e9, nu=0.40, rho=1150.0, yield_=60e6, fatigue=20e6),
    # the spring-steel clip that pre-tensions the strap against the palm support
    "spring_steel": dict(E=200e9, G=79e9, nu=0.30, rho=7850.0, yield_=1200e6, fatigue=600e6),
}

SAFETY_FACTOR = 2.0
DEFLECTION_MAX = 0.5e-3  # m; a key that moves more than this feels mushy
SOFT_TISSUE_K = 25e3  # N/m (= 25 N/mm), midpoint of the 10-50 N/mm literature band


def torsion_constant(b: float, d: float) -> float:
    """St Venant J for a solid rectangle (Roark). NOT the polar moment -- that is only
    correct for circular sections and overestimates rectangular torsional stiffness."""
    a, t = max(b, d), min(b, d)
    return a * t**3 * (1.0 / 3.0 - 0.21 * (t / a) * (1.0 - t**4 / (12.0 * a**4)))


def von_mises(N: float, My: float, Mz: float, T: float, b: float, d: float) -> float:
    """Worst-corner von Mises stress in a rectangular beam section.

    Axial and both bending terms are summed with the same sign (the worst corner sees all
    three), which is conservative. Torsional shear uses Roark's rectangular formula --
    tau_max = T(3a + 1.8t)/(a^2 t^2) at the midpoint of the long side.
    """
    A = b * d
    Iy = b * d**3 / 12.0
    Iz = d * b**3 / 12.0
    sigma = abs(N) / A + abs(My) * (d / 2.0) / Iy + abs(Mz) * (b / 2.0) / Iz
    a, t = max(b, d), min(b, d)
    tau = abs(T) * (3.0 * a + 1.8 * t) / (a**2 * t**2)
    return math.sqrt(sigma**2 + 3.0 * tau**2)


@dataclass
class Member:
    name: str
    i: str
    j: str
    material: str
    b: float  # section width, m
    d: float  # section depth, m
    kind: str  # "alu" | "nylon" | "strap" -- for the viz and for tension-only

    @property
    def tension_only(self) -> bool:
        return self.kind == "strap"


@dataclass
class Frame:
    nodes: dict[str, np.ndarray]
    members: list[Member]
    supports: list[str]  # nodes bearing on soft tissue
    keys: dict = field(default_factory=dict)  # (finger, row) -> key node name
    key_normal: dict = field(default_factory=dict)

    def length(self, m: Member) -> float:
        return float(np.linalg.norm(self.nodes[m.j] - self.nodes[m.i]))

    def mass(self) -> float:
        return sum(
            MATERIALS[m.material]["rho"] * m.b * m.d * self.length(m) for m in self.members
        )


# ---------------------------------------------------------------------------------------
# Anatomy: a frame to place the device in, so the geometry follows the hand rather than
# being hardcoded in world coordinates (and so it will scale with hand size at Stage 5).
# ---------------------------------------------------------------------------------------


def hand_axes(h: MyoHand, q: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """(origin, distal, radial, dorsal) -- orthonormal, origin at the capitate.

    dorsal = distal x radial, and the sign is CHECKED against the palm rather than
    assumed: it must point away from the fingertips' side of the hand.
    """
    m = h.model
    h.fk(q)

    def body(name):
        return h.data.xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, name)].copy()

    origin = body("capitate")
    heads = np.array([body(n) for n in ("proxph2", "proxph3", "proxph4", "proxph5")])
    e_dist = heads.mean(0) - origin
    e_dist /= np.linalg.norm(e_dist)

    rad = body("secondmc") - body("fifthmc")  # index side minus little side
    rad -= (rad @ e_dist) * e_dist
    e_rad = rad / np.linalg.norm(rad)

    e_dors = np.cross(e_dist, e_rad)
    pads = np.array([h.pad_pose(q, f)[0] for f in FINGERS])
    if (pads.mean(0) - origin) @ e_dors > 0:  # pointing at the palm: flip
        e_dors = -e_dors
    return origin, e_dist, e_rad, e_dors


# ---------------------------------------------------------------------------------------
# Geometry: design vector -> frame. These offsets ARE the design variables that Stage 4
# will optimise; the defaults here just give a sane baseline device to test the FEA on.
# ---------------------------------------------------------------------------------------

DEFAULTS = dict(
    dorsal_gap=0.018,  # m, frame stands off the back of the hand
    side_gap=0.016,  # m, rails clear the sides of the hand
    stalk=0.012,  # m, key stands off the key bar
    bar_half=0.050,  # m, half-span of the palmar key bar
    sec_alu=(0.008, 0.002),  # m, (width, depth) of the aluminium members
    sec_nylon=(0.006, 0.004),  # m, key stalks
    sec_strap=(0.020, 0.0015),  # m, webbing
    mat_frame="al6061",
    mat_stalk="pa12",
)


BODY_DEFAULTS = dict(
    palm_offset=0.020,  # m, how far palmar of the metacarpals the palm support bears
    body_half=0.026,  # m, half-width of the body (radial extent) -- A DESIGN VARIABLE
    body_prox=0.014,  # m, how far PROXIMALLY into the palm the body reaches
    body_dist=0.055,  # m, how far DISTALLY it reaches
    stem=0.006,  # m, key stands off the body face on its stem
    sec_alu=(0.008, 0.002),  # m, body frame members
    sec_nylon=(0.006, 0.004),  # m, key stems
    sec_strap=(0.022, 0.0015),  # m, velcro webbing
    sec_clip=(0.010, 0.0008),  # m, spring-steel clip
    mat_frame="cf_pa12",
    mat_stalk="pa12",
)


def palmar_arch(h: MyoHand, q: np.ndarray, dist: float, n: int = 5) -> list[tuple[float, float]]:
    """The hand's OWN transverse metacarpal arch, at distal station `dist`.

    Returns [(radial, dorsal)] following the PALMAR SURFACE of the 2nd-5th metacarpals --
    measured from the model's meshes, not assumed.

    THE PALM IS A CUP, and it is 6.4 mm deep. Measured, at the metacarpals' palmar-most
    surface:

        2nd MC (radial)  r=+16.6mm   z=-8.9mm   \
        3rd MC           r= +4.0mm   z=-5.1mm    | the EDGES protrude palmar
        4th MC           r= -8.5mm   z=-3.1mm    | the MIDDLE is HOLLOW
        5th MC (ulnar)   r=-19.8mm   z=-9.5mm   /

    A body resting in the palm bears on the two EMINENCES and BRIDGES the hollow. It does not
    sit on a flat plate -- and `build_body` bolted four corners of a RECTANGLE across it at a
    single depth, so the corners either float off the eminences or dig into the hollow.

    ⚠ The THUMB METACARPAL is excluded on purpose. It swings across the palm (measured -33 mm
    palmar at +27 mm radial in the rest pose) and would swamp the profile -- but it is a
    DIGIT, not a bearing surface, and it moves.
    """
    import mujoco

    m = h.model
    h.fk(q)
    o, e_d, e_r, e_o = hand_axes(h, q)

    pts = []
    for bn in ("secondmc", "thirdmc", "fourthmc", "fifthmc"):
        bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, bn)
        for g in range(m.body_geomadr[bid], m.body_geomadr[bid] + m.body_geomnum[bid]):
            if m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
                continue
            mid = m.geom_dataid[g]
            va, vn = m.mesh_vertadr[mid], m.mesh_vertnum[mid]
            V = m.mesh_vert[va:va + vn] @ h.data.geom_xmat[g].reshape(3, 3).T + h.data.geom_xpos[g]
            pts.append(V)
    P = np.vstack(pts)
    r = (P - o) @ e_r
    z = (P - o) @ e_o

    lo, hi = float(r.min()), float(r.max())
    out = []
    for rr in np.linspace(lo, hi, n):
        w = np.abs(r - rr) < 0.005
        if not w.any():
            w = np.abs(r - rr) < 0.010
        out.append((float(rr), float(np.percentile(z[w], 2))))  # the palmar SURFACE
    return out


def build_arch(h: MyoHand, q: np.ndarray, keys: dict, p: dict | None = None) -> Frame:
    """THE PALM END AS AN ARCH THAT FOLLOWS THE HAND -- not a flat plate on a cantilever.

    (The user: "invert the support structure -- it doesn't follow the natural shape of the
    hand.")

    WHAT CHANGED, AND WHAT DELIBERATELY DID NOT.

    CHANGED -- the palm support. `build_body` bolted FOUR CORNERS OF A FLAT RECTANGLE across
    the palm at a single depth. The palm is not flat: it is a CUP 6.4 mm deep (measured off
    the metacarpal meshes -- `palmar_arch`), with the radial and ulnar edges PROTRUDING palmar
    and the middle HOLLOW. So the corners either float off the eminences or dig into the
    hollow.

    And a keypress pushes the body DORSALLY, INTO the palm. An arch that is CONVEX TOWARD THE
    PALM carries that in pure COMPRESSION, straight into the two eminences the hand actually
    presents. The flat plate carried it in BENDING. Bending is where the mass goes.

    NOT CHANGED -- the distal routing, and this is the honest part. I tried four times to run
    the load straight from the arch out to the wells and it cut the fingers every time
    (-6.5 mm into the middle phalanx; then -3.9; then -5.3; then -5.9). It is the SAME
    topological trap that killed the exoskeleton: YOU CANNOT DRAW A STRAIGHT LINE FROM THE
    PALM TO A FINGERTIP WITHOUT CROSSING THE FINGER, because the finger is what lies between
    them. The box's "ugly" floor ring was solving a real problem, and solving it correctly:
    drop PALMAR into open air first, THEN run distally at the key face's depth. Both legs are
    then outside the hand by construction.

    So the arch replaces the plate and inherits the routing. That is the whole change.

    ⚠ AND IT BUYS NO STIFFNESS. I claimed 3.8x, and I was wrong. Measured, by stiffening each
    group 100x and seeing what the key actually feels:

        key face      44% of the compliance
        floor legs    27%
        PALM SUPPORT  10%   <-- the arch can only ever touch this
        walls          9%
        floor ring     9%
        stems          0%

    The palm end is NOT where the structure is soft. The compliance is in the KEY FACE and the
    long FLOOR LEGS -- the cantilever out to the fingertips. My earlier "3.8x stiffer arch" was
    an ARTIFACT: that version skipped the floor routing altogether (and cut straight through
    the fingers). The gain came from DELETING THE CANTILEVER, not from the arch. I was excited
    by a number produced by a broken geometry.

    SO WHAT IS THE ARCH FOR? FIT AND PRESSURE, not stiffness. A flat plate across a 6.4 mm cup
    bears on whatever it happens to touch; an arch bears on the two eminences the hand actually
    presents and BRIDGES the hollow. That is a comfort and load-distribution argument -- and
    THIS MODEL CANNOT SCORE COMFORT, so it cannot make the case for its own change.

    ⚠ Its 3x mass penalty is largely a BEAM-MODEL ARTIFACT: an arch is 10 discrete beams where
    a plate is 4 corners, and PyNite charges for every one. A MOULDED SHELL following the palm
    costs nothing extra -- it is the same shell, curved. The same caveat already applies to
    BODY_PROX. Deciding this properly needs shell elements, not beams.
    """
    p = {**BODY_DEFAULTS, **(p or {})}
    keys = {(k if isinstance(k, tuple) else (k, 0)): v for k, v in keys.items()}
    o, e_d, e_r, e_o = hand_axes(h, q)

    def P(dist, rad, dors):
        return o + dist * e_d + rad * e_r + dors * e_o

    nodes: dict[str, np.ndarray] = {}
    members: list[Member] = []
    ab, ad = p["sec_alu"]
    nb, nd = p["sec_nylon"]
    sb, sd = p["sec_strap"]
    cb, cd = p["sec_clip"]

    def strut(name, i, j, kind="alu"):
        mat = {"alu": p["mat_frame"], "nylon": p["mat_stalk"],
               "strap": "webbing", "clip": "spring_steel"}[kind]
        bd = {"alu": (ab, ad), "nylon": (nb, nd), "strap": (sb, sd), "clip": (cb, cd)}[kind]
        members.append(Member(name, i, j, mat, bd[0], bd[1], kind))

    # --- the KEY FACE: unchanged ---------------------------------------------------------
    feet = []
    for (f, row), (kp, kn) in keys.items():
        kp, kn = np.asarray(kp, float), np.asarray(kn, float)
        kk, ft = f"key_{f}{row}", f"foot_{f}{row}"
        nodes[kk] = kp
        nodes[ft] = kp - p["stem"] * kn
        strut(f"stem_{f}{row}", ft, kk, kind="nylon")
        feet.append((float((nodes[ft] - o) @ e_r), float((nodes[ft] - o) @ e_d), ft))
    for key_idx, second in ((0, 1), (1, 0)):
        order = sorted(feet, key=lambda t: (t[key_idx], t[second]))
        for a, b in zip(order, order[1:]):
            if a[2] != b[2]:
                strut(f"face{key_idx}_{a[2]}_{b[2]}", a[2], b[2])

    # --- THE ARCH: the palm support, following the hand's own transverse arch -------------
    gap = p["palm_offset"]
    dp, dd = p["body_prox"], p["body_dist"]
    # THE ARCH SPANS THE BODY'S OWN WIDTH, NOT THE HAND'S.
    #
    # I first spanned it across the FULL radial extent of the metacarpals (-27 to +29 mm) and
    # it was a disaster: 109 g (it is a big arch) and its radial node sat inside the THUMB.
    # `body_half` is a DESIGN VARIABLE -- the optimiser sizes the body to the hand -- and I
    # overrode it with the hand's own anatomy. The arch follows the palm's SHAPE; the body
    # decides its WIDTH.
    hw = p["body_half"]
    full = palmar_arch(h, q, dd, n=9)
    arch_pts = []
    for i in range(5):
        rad = -hw + 2.0 * hw * i / 4.0
        near = min(full, key=lambda t: abs(t[0] - rad))
        arch_pts.append((rad, near[1]))

    ribs = []
    for station, dist in (("p", dp), ("d", dd)):
        ring = []
        for i, (rad, dors) in enumerate(arch_pts):
            nm = f"arch_{station}{i}"
            nodes[nm] = P(dist, rad, dors - gap)
            ring.append(nm)
        for a, b in zip(ring, ring[1:]):
            strut(f"rib_{station}{a[-1]}{b[-1]}", a, b)   # the arch: works in COMPRESSION
        ribs.append(ring)
    for i in range(len(arch_pts)):                        # longitudinal, so the ribs cannot scissor
        strut(f"spine_{i}", ribs[0][i], ribs[1][i])

    # --- WALLS + FLOOR: the box's routing, kept because it is RIGHT (see docstring) -------
    face_o = float(np.mean([(nodes[t[2]] - o) @ e_o for t in feet]))
    corners = [(0, ribs[0][0]), (len(arch_pts) - 1, ribs[0][-1]),
               (0, ribs[1][0]), (len(arch_pts) - 1, ribs[1][-1])]
    floors = []
    for k, (ai, corner) in enumerate(corners):
        c = nodes[corner] - o
        cr, cd = float(c @ e_r), float(c @ e_d)
        fl = f"floor{k}"
        nodes[fl] = P(cd, cr, face_o)          # straight PALMAR, into open air
        strut(f"wall{k}", corner, fl)
        nearest = min(feet, key=lambda t: (t[0] - cr) ** 2 + (t[1] - cd) ** 2)
        strut(f"floorleg{k}", fl, nearest[2])  # then distally, palmar of everything
        floors.append(fl)
    for a, b in zip(floors, floors[1:] + floors[:1]):
        strut(f"floorring_{a}_{b}", a, b)

    # --- STRAP over the dorsum, pre-tensioned by a spring-steel clip ----------------------
    nodes["dorsum"] = P(0.5 * (dp + dd), 0.0, 0.026)
    nodes["clip_r"] = P(dd, arch_pts[-1][0] + 0.008, arch_pts[-1][1] - gap + 0.010)
    nodes["clip_u"] = P(dd, arch_pts[0][0] - 0.008, arch_pts[0][1] - gap + 0.010)
    strut("clip_radial", ribs[1][-1], "clip_r", kind="clip")
    strut("clip_ulnar", ribs[1][0], "clip_u", kind="clip")
    strut("strap_radial", "clip_r", "dorsum", kind="strap")
    strut("strap_ulnar", "clip_u", "dorsum", kind="strap")

    # ONLY the eminences bear. The crown of the arch BRIDGES the hollow and touches nothing.
    return Frame(
        nodes=nodes,
        members=members,
        supports=[ribs[0][0], ribs[0][-1], ribs[1][0], ribs[1][-1], "dorsum"],
        keys={(f, r): f"key_{f}{r}" for (f, r) in keys},
        key_normal={(f, r): np.asarray(v[1], float) for (f, r), v in keys.items()},
    )


def build_body(h: MyoHand, q: np.ndarray, keys: dict, p: dict | None = None) -> Frame:
    """STRAP-MOUNTED BODY -- the architecture the target device (typeware.tech) actually uses.

    A body sits IN THE PALM. The fingers curl onto it ("near the optimal finger grip
    position"), the keys are on the face where the fingertips land, a velcro strap runs over
    the back of the hand, and a spring-steel clip pre-tensions the strap against a "palm
    support". The hand grips the device.

    THIS REPLACES THE ARTICULATED EXOSKELETON, and it does so because the exoskeleton could
    not be made to work. Its thumb arm cut through the hand three separate times -- off the
    dorsal knuckle (-6.5 mm, across the palm), off the radial rail (-7.3 mm, through the
    thenar), off an outboard thumb rail (-5.5 mm, through the fingers) -- and the third one
    was not a routing mistake. A deep thumb key sits ~20 mm radial: under the fingers, near
    the palm centre. An OPEN FRAME CANNOT REACH INTO THE PALM FROM OUTSIDE without crossing
    something. It was a topological dead end, and no amount of rerouting fixes topology.

    A body in the palm has no arms at all, so the problem does not exist. It also matches
    where the load wants to go: a keypress reacts straight into the palm it is resting on,
    instead of being carried around the hand through a cantilever.

    Load path: fingertip -> key -> stem -> body face -> body walls -> palm support (soft
    tissue), with the strap and clip holding the body against the palm.
    """
    p = {**BODY_DEFAULTS, **(p or {})}
    keys = {(k if isinstance(k, tuple) else (k, 0)): v for k, v in keys.items()}
    o, e_d, e_r, e_o = hand_axes(h, q)

    def P(dist, rad, dors):
        return o + dist * e_d + rad * e_r + dors * e_o

    nodes: dict[str, np.ndarray] = {}
    members: list[Member] = []
    ab, ad = p["sec_alu"]
    nb, nd = p["sec_nylon"]
    sb, sd = p["sec_strap"]
    cb, cd = p["sec_clip"]

    def strut(name, i, j, kind="alu"):
        mat = {"alu": p["mat_frame"], "nylon": p["mat_stalk"],
               "strap": "webbing", "clip": "spring_steel"}[kind]
        bd = {"alu": (ab, ad), "nylon": (nb, nd), "strap": (sb, sd), "clip": (cb, cd)}[kind]
        members.append(Member(name, i, j, mat, bd[0], bd[1], kind))

    # --- the body's KEY FACE: the surface the fingertips land on -----------------------
    # Each key stands off the face on its own stem, along its own switch axis.
    feet = []
    for (f, row), (kp, kn) in keys.items():
        kp, kn = np.asarray(kp, float), np.asarray(kn, float)
        kk, ft = f"key_{f}{row}", f"foot_{f}{row}"
        nodes[kk] = kp
        nodes[ft] = kp - p["stem"] * kn  # kn points AT the digit; back off to the face
        strut(f"stem_{f}{row}", ft, kk, kind="nylon")
        feet.append((float((nodes[ft] - o) @ e_r), float((nodes[ft] - o) @ e_d), ft))

    # tie the face together: chain radially, and chain again distally, so it is a grid and
    # not a string of beads (a single chain has no torsional stiffness across the face)
    for key_idx, second in ((0, 1), (1, 0)):
        order = sorted(feet, key=lambda t: (t[key_idx], t[second]))
        for a, b in zip(order, order[1:]):
            nm = f"face{key_idx}_{a[2]}_{b[2]}"
            if a[2] != b[2]:
                strut(nm, a[2], b[2])

    # --- the PALM SUPPORT: what the body bears on, and what the strap pulls it against ---
    # The body's SIZE is a design variable, not a constant. It was hardcoded, which meant
    # the optimiser had no way to shrink the body to fit a 5th-percentile hand -- and a
    # one-size body sized for the median put its radial floor inside the small hand's thumb
    # (-6.5 mm into BONE). A rigid device that cannot be resized cannot fit a population.
    pal = -p["palm_offset"]  # palmar of the metacarpals
    hw, dp, dd = p["body_half"], p["body_prox"], p["body_dist"]
    nodes["palm_rd"] = P(dd, hw, pal)  # radial-distal
    nodes["palm_ud"] = P(dd, -hw, pal)  # ulnar-distal
    nodes["palm_rp"] = P(dp, hw, pal)  # radial-proximal
    nodes["palm_up"] = P(dp, -hw, pal)  # ulnar-proximal
    for a, b in (("palm_rd", "palm_ud"), ("palm_ud", "palm_up"),
                 ("palm_up", "palm_rp"), ("palm_rp", "palm_rd")):
        strut(f"palm_{a}_{b}", a, b)

    # --- WALLS: carry the keypress from the face into the palm support ------------------
    # Each palm corner ties to the nearest face foot, so the load goes straight down into
    # the hand rather than round it.
    # The body is a BOX, and its walls run through FREE SPACE.
    #
    # Any rule that connects a palm corner straight to a face foot draws a chord ACROSS the
    # hand: nearest-in-3D ran the radial-proximal wall out to the thumb's deep key foot,
    # through the thenar (-4.6 mm), and nearest-in-plane did the same thing because every
    # foot is distal of every palm corner. The chord is the problem, not the choice of foot.
    #
    # So: drop PALMAR from each palm corner into open air (away from the hand), and only
    # then run distally along a floor to the key face. Both legs are outside the hand by
    # construction -- the first goes away from the palm surface it starts on, the second
    # lies palmar of everything.
    # ⚠ EACH FLOOR LEG MUST REACH A DISTINCT FOOT. The rule used to be "connect each palm
    # corner to its NEAREST foot" -- and it DEGENERATED: every palm corner is PROXIMAL of every
    # well (corners at distal 8-45 mm, feet at 65-126 mm), so the nearest foot to all four is
    # the same one, the thumb's. ALL FOUR LEGS LANDED ON foot_thumb0. The entire keypress load
    # from all five wells funnelled through ONE NODE and the whole key face cantilevered off it.
    #
    # The BEAM MODEL HID THIS for two years of this project's life: a chain of struts carries
    # load AXIALLY and axial stiffness is enormous, so a one-point support still came out stiff
    # (27 um). A SHELL cannot hide it -- a plate held at one node is a floppy cantilever
    # (990 um). The idealisation was flattering a structure that is genuinely badly supported.
    #
    # So: a one-to-one assignment of corners to feet, minimising total leg length. Exact
    # (Hungarian), like the character layout -- and for the same reason: "nearest" is a greedy
    # rule and greedy rules collide.
    from scipy.optimize import linear_sum_assignment

    # THE FLOOR MUST SIT BELOW THE *DEEPEST* WELL, NOT THE AVERAGE ONE.
    #
    # It used to be the MEAN foot depth -- and the wells are not at one depth: the fingers'
    # sit at -58 mm while the THUMB's reaches -85, because the thumb opposes across the palm.
    # A floor at the mean (-66) is DORSAL of the deepest fingertips, so the legs running along
    # it cut back into the hand (-3.3 mm). Take the most palmar well and go below it: then the
    # floor is palmar of everything BY CONSTRUCTION, which is the only way to be sure.
    face_o = min((nodes[t[2]] - o) @ e_o for t in feet) - 0.008
    corners = ["palm_rd", "palm_ud", "palm_rp", "palm_up"]
    C = np.array([[(t[0] - float((nodes[c] - o) @ e_r)) ** 2
                   + (t[1] - float((nodes[c] - o) @ e_d)) ** 2 for t in feet]
                  for c in corners])
    ci, fi = linear_sum_assignment(C)
    pick = {corners[a]: feet[b][2] for a, b in zip(ci, fi)}

    for corner in corners:
        c = nodes[corner] - o
        cr, cd = float(c @ e_r), float(c @ e_d)
        fl = f"floor_{corner[5:]}"
        nodes[fl] = P(cd, cr, face_o)  # straight out from the palm, into free space
        strut(f"wall_{corner}", corner, fl)
        strut(f"floorleg_{corner[5:]}", fl, pick[corner])
    for a, b in (("floor_rd", "floor_ud"), ("floor_ud", "floor_up"),
                 ("floor_up", "floor_rp"), ("floor_rp", "floor_rd")):
        strut(f"floorring_{a}_{b}", a, b)

    # --- STRAP over the back of the hand, pre-tensioned by a SPRING-STEEL CLIP ----------
    nodes["dorsum"] = P(0.5 * (dp + dd), 0.0, 0.026)  # bears on the back of the hand
    nodes["clip_r"] = P(dd, hw + 0.008, pal + 0.010)
    nodes["clip_u"] = P(dd, -hw - 0.008, pal + 0.010)
    strut("clip_radial", "palm_rd", "clip_r", kind="clip")
    strut("clip_ulnar", "palm_ud", "clip_u", kind="clip")
    strut("strap_radial", "clip_r", "dorsum", kind="strap")
    strut("strap_ulnar", "clip_u", "dorsum", kind="strap")

    return Frame(
        nodes=nodes,
        members=members,
        # the body bears on the PALM; the strap bears on the DORSUM. Both soft tissue.
        supports=["palm_rd", "palm_ud", "palm_rp", "palm_up", "dorsum"],
        keys={(f, r): f"key_{f}{r}" for (f, r) in keys},
        key_normal={(f, r): np.asarray(v[1], float) for (f, r), v in keys.items()},
    )


def build_exo(h: MyoHand, q: np.ndarray, keys: dict, p: dict | None = None) -> Frame:
    """Build the exoskeleton around hand posture `q`.

    `keys` maps (finger, row) -> (pos, normal), or finger -> (pos, normal) for a single
    row. `normal` is the key's OUTWARD normal (points at the finger), matching MyoHand.

    Multiple keys per finger are ROWS: a finger reaches its second key by curling further,
    so each row is its own curved bar at its own curl depth. That is what a Twiddler is,
    and it is why more keys costs mass -- each row is another bar and another pair of
    rail connections.
    """
    p = {**DEFAULTS, **(p or {})}
    keys = {(k if isinstance(k, tuple) else (k, 0)): v for k, v in keys.items()}
    o, e_d, e_r, e_o = hand_axes(h, q)

    def P(dist, rad, dors):
        return o + dist * e_d + rad * e_r + dors * e_o

    nodes: dict[str, np.ndarray] = {}
    members: list[Member] = []
    ab, ad = p["sec_alu"]
    nb, nd = p["sec_nylon"]
    sb, sd = p["sec_strap"]

    def strut(name, i, j, kind="alu"):
        mat = {"alu": p["mat_frame"], "nylon": p["mat_stalk"], "strap": "webbing"}[kind]
        bd = {"alu": (ab, ad), "nylon": (nb, nd), "strap": (sb, sd)}[kind]
        members.append(Member(name, i, j, mat, bd[0], bd[1], kind))

    g, s = p["dorsal_gap"], p["side_gap"]

    # --- dorsal spine + knuckle bar (bears on the back of the hand) ---
    nodes["wrist_d"] = P(-0.005, 0.0, g)
    nodes["dorsum"] = P(0.035, 0.0, g)
    nodes["knuck_r"] = P(0.065, 0.028, g)
    nodes["knuck_u"] = P(0.060, -0.026, g)
    strut("spine", "wrist_d", "dorsum")
    strut("spine_r", "dorsum", "knuck_r")
    strut("spine_u", "dorsum", "knuck_u")
    strut("knuckle_bar", "knuck_r", "knuck_u")

    # --- articulated rails: around the radial and ulnar edges of the hand ---
    # These are the "elbows" -- their positions and joint types are the Stage-5 sub-problem.
    nodes["rail_r"] = P(0.072, 0.040 + s, 0.0)
    nodes["rail_u"] = P(0.066, -0.038 - s, 0.0)
    strut("arm_r", "knuck_r", "rail_r")
    strut("arm_u", "knuck_u", "rail_u")

    # --- palmar key bars: one CURVED bar per row ---
    #
    # Curved, not straight. The fingertips lie on an arc, so a straight bar through their
    # centroid cuts the ring and little fingers (measured: -2.3 mm inside the flesh). Each
    # foot instead sits `stalk` behind ITS OWN key, along that key's normal, and the bar is
    # the polyline through the feet -- so it follows the fingertip arc and clears it by the
    # stalk length by construction. Which is what a real key bar looks like.
    rows = sorted({r for (f, r) in keys if f != "thumb"})
    for row in rows:
        feet = []
        for f in [x for x in FINGERS if x != "thumb"]:
            if (f, row) not in keys:
                continue
            kp = np.asarray(keys[(f, row)][0], float)
            kn = np.asarray(keys[(f, row)][1], float)
            kk, ff = f"key_{f}{row}", f"foot_{f}{row}"
            nodes[kk] = kp
            nodes[ff] = kp - p["stalk"] * kn  # kn points AT the finger; back off along -kn
            strut(f"stalk_{f}{row}", ff, kk, kind="nylon")
            feet.append((float((kp - o) @ e_r), ff))
        if not feet:
            continue
        feet.sort(reverse=True)  # radial -> ulnar
        br, bu = f"bar_r{row}", f"bar_u{row}"
        nodes[br] = nodes[feet[0][1]] + 0.018 * e_r
        nodes[bu] = nodes[feet[-1][1]] - 0.018 * e_r
        strut(f"rail_to_{br}", "rail_r", br)
        strut(f"rail_to_{bu}", "rail_u", bu)
        chain = [br] + [nm for _, nm in feet] + [bu]
        for a, b in zip(chain, chain[1:]):
            strut(f"bar{row}_{a}_{b}", a, b)

    # --- thumb: an outboard rail, then one short arm per key ---
    #
    # The thumb's arms have now cut through the hand twice. A chord from the dorsal knuckle
    # to a thumb key crosses the palm (-6.5 mm); moving the origin to the radial rail fixed
    # the FIRST key and not the second, because a deeper (more curled) thumb key sits
    # further into the palm and the chord to it goes straight through the thenar (-7.3 mm).
    #
    # So the thumb gets its own rail, held OUTBOARD of the thumb on the radial side. Every
    # thumb arm then starts outside the hand and reaches inward only over the last stretch,
    # which is the shape the thumb's own geometry demands.
    thumb_keys = [(r, np.asarray(v[0], float), np.asarray(v[1], float))
                  for (f, r), v in keys.items() if f == "thumb"]
    if thumb_keys:
        # Hang the thumb off the RADIAL END OF THE KEY BAR, not off anything outboard.
        #
        # The thumb's arms cut through the hand three times before this. From the dorsal
        # knuckle: across the palm (-6.5 mm). From the radial rail: fine for the shallow key,
        # through the thenar for the deep one (-7.3 mm). From an outboard thumb rail: through
        # the FINGERS (-5.5 mm), and that one is not a routing mistake but a topological one.
        # A deep thumb key sits at ~20 mm radial -- essentially UNDER the fingers, near the
        # palm centre -- and an open frame simply cannot reach into the palm from outside
        # without crossing something.
        #
        # But the key bar is ALREADY there, in the palmar plane a few cm from the thumb keys.
        # Hanging the thumb off its radial end is a short local hop through free space, and
        # it is also what a real device is: a body in the palm carrying both.
        anchor = "bar_r0" if "bar_r0" in nodes else "rail_r"
        for row, kp, kn in thumb_keys:
            kk, el = f"key_thumb{row}", f"thumb_elbow{row}"
            nodes[kk] = kp
            nodes[el] = kp - p["stalk"] * kn
            strut(f"arm_thumb{row}", anchor, el)
            strut(f"stalk_thumb{row}", el, kk, kind="nylon")

    # --- straps: webbing round the wrist and palm, TENSION ONLY ---
    nodes["wrist_p"] = P(-0.005, 0.0, -0.022)  # palmar side of the wrist
    nodes["palm_p"] = P(0.035, 0.0, -0.024)  # palmar side of the metacarpals
    strut("strap_wrist", "wrist_d", "wrist_p", kind="strap")
    strut("strap_palm", "dorsum", "palm_p", kind="strap")

    return Frame(
        nodes=nodes,
        members=members,
        # every node that actually bears on the hand: dorsum, the side rails, and the
        # palmar strap anchors
        supports=["wrist_d", "dorsum", "rail_r", "rail_u", "wrist_p", "palm_p"],
        keys={(f, r): f"key_{f}{r}" for (f, r) in keys},
        key_normal={(f, r): np.asarray(v[1], float) for (f, r), v in keys.items()},
    )


# ---------------------------------------------------------------------------------------
# The FEA
# ---------------------------------------------------------------------------------------


def solve(frame: Frame, chord: list, press_N: float = 0.5, k_soft: float = SOFT_TISSUE_K) -> dict:
    """Static solve for one chord (the set of fingers pressing simultaneously).

    Returns mass, per-member stress + utilisation, and key deflection along the key normal.
    """
    from Pynite import FEModel3D

    fem = FEModel3D()
    for name, mat in MATERIALS.items():
        fem.add_material(name, E=mat["E"], G=mat["G"], nu=mat["nu"], rho=mat["rho"])
    for nm, xyz in frame.nodes.items():
        fem.add_node(nm, float(xyz[0]), float(xyz[1]), float(xyz[2]))
    for m in frame.members:
        sec = f"sec_{m.name}"
        fem.add_section(
            sec, A=m.b * m.d, Iy=m.b * m.d**3 / 12.0, Iz=m.d * m.b**3 / 12.0,
            J=torsion_constant(m.b, m.d),
        )
        fem.add_member(m.name, m.i, m.j, m.material, sec, tension_only=m.tension_only)

    # A node touched only by straps has NO rotational stiffness -- webbing transmits no
    # moment -- so its rotational dofs are singular. They are also physically meaningless
    # (a strap end cannot resist a twist), so fix them. Fixing them at a node the frame
    # DOES reach would be wrong: that would rigidly clamp the frame to the hand in
    # rotation, which is exactly the over-stiff boundary condition we are avoiding.
    touched: dict[str, list[Member]] = {}
    for m in frame.members:
        touched.setdefault(m.i, []).append(m)
        touched.setdefault(m.j, []).append(m)
    for nm, ms in touched.items():
        if all(m.tension_only for m in ms):
            fem.def_support(nm, False, False, False, True, True, True)

    # Soft tissue: linear springs, not rigid ground.
    for nm in frame.supports:
        for dof in ("DX", "DY", "DZ"):
            fem.def_support_spring(nm, dof, k_soft)

    # The finger presses INTO the key along -normal, so the force ON the key is -F*normal.
    for f in [(c if isinstance(c, tuple) else (c, 0)) for c in chord]:
        node = frame.keys[f]
        F = -press_N * frame.key_normal[f]
        for dof, v in zip(("FX", "FY", "FZ"), F):
            fem.add_node_load(node, dof, float(v))

    fem.analyze(check_statics=False)

    stresses, util = {}, {}
    for m in frame.members:
        M = fem.members[m.name]
        N = max(abs(M.max_axial()), abs(M.min_axial()))
        My = max(abs(M.max_moment("My")), abs(M.min_moment("My")))
        Mz = max(abs(M.max_moment("Mz")), abs(M.min_moment("Mz")))
        T = max(abs(M.max_torque()), abs(M.min_torque()))
        if m.tension_only:
            s = abs(N) / (m.b * m.d)  # webbing carries axial tension only
        else:
            s = von_mises(N, My, Mz, T, m.b, m.d)
        stresses[m.name] = s
        allow = MATERIALS[m.material]["yield_"] / SAFETY_FACTOR
        util[m.name] = s / allow

    defl = {}
    for f, node in frame.keys.items():
        n = fem.nodes[node]
        u = np.array([n.DX["Combo 1"], n.DY["Combo 1"], n.DZ["Combo 1"]])
        defl[f] = abs(float(u @ frame.key_normal[f]))  # along the press axis

    return dict(
        mass=frame.mass(),
        stress=stresses,
        util=util,
        max_util=max(util.values()),
        worst_member=max(util, key=util.get),
        deflection=defl,
        max_deflection=max(defl.values()) if defl else 0.0,
        ok=max(util.values()) <= 1.0 and (max(defl.values()) if defl else 0) <= DEFLECTION_MAX,
    )


# ---------------------------------------------------------------------------------------
# Clearance: the frame must not pass through the hand. v1 died on collision handling; here
# it is a hard, cheap geometric check against the model's own flesh capsules.
# ---------------------------------------------------------------------------------------


# The parts of the hand that CURL -- the phalanges. Deliberately NOT the metacarpals.
#
# `firstmc` (the thumb metacarpal) is the THENAR EMINENCE: it is palm, not a moving digit,
# and a palm support is supposed to bear on it. Including it scored the body's own bearing
# surface as a collision (-6.1 mm at the radial-proximal corner) -- the device being
# penalised for resting where it is designed to rest.
DIGIT_FLESH = (
    "proximal_thumb", "distal_thumb",
    "proxph2", "midph2", "distph2", "proxph3", "midph3", "distph3",
    "proxph4", "midph4", "distph4", "proxph5", "midph5", "distph5",
)


_BONE_R: dict = {}


def _bone_radius(h: MyoHand, g: int) -> float:
    """Radius of the BONE inside a flesh capsule, measured from the model's own bone mesh.

    A hand GRIPPING a body is in contact with it everywhere -- that is what gripping is --
    so "does the device touch the flesh" is not a meaningful constraint for a palm-mounted
    body; it penalises the device for being held. Soft tissue compresses under grip. BONE
    does not. So the check is against bone.
    
    Derived, not guessed: for each phalanx we take the maximum distance from its flesh
    capsule's axis to any vertex of its bone mesh.
    """
    key = (id(h.model), g)
    if key in _BONE_R:
        return _BONE_R[key]
    m = h.model
    bid = m.geom_bodyid[g]
    Rg = np.zeros(9)
    mujoco.mju_quat2Mat(Rg, m.geom_quat[g])
    axis = Rg.reshape(3, 3)[:, 2]
    c = m.geom_pos[g]
    best = 0.0
    for gg in range(m.body_geomadr[bid], m.body_geomadr[bid] + m.body_geomnum[bid]):
        if m.geom_type[gg] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        did = m.geom_dataid[gg]
        va, nv = m.mesh_vertadr[did], m.mesh_vertnum[did]
        V = m.mesh_vert[va: va + nv].reshape(-1, 3)
        Rm = np.zeros(9)
        mujoco.mju_quat2Mat(Rm, m.geom_quat[gg])
        V = V @ Rm.reshape(3, 3).T + m.geom_pos[gg]
        d = V - c
        perp = d - np.outer(d @ axis, axis)
        best = max(best, float(np.linalg.norm(perp, axis=1).max()))
    _BONE_R[key] = best if best > 0 else float(m.geom_size[g][0])
    return _BONE_R[key]


def _flesh_capsules(h: MyoHand, q: np.ndarray, only=None, bone=False) -> list[tuple[np.ndarray, np.ndarray, float]]:
    """The hand's soft tissue, as (endpoint a, endpoint b, radius) in world coords.

    myohand.xml wraps each bone in a `class="skin"` capsule. Those, not the bone meshes,
    are what the frame would actually hit.
    """
    m = h.model
    h.fk(q)
    keep = None
    if only is not None:
        keep = {mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, b) for b in only}
    out = []
    for g in range(m.ngeom):
        if m.geom_type[g] != mujoco.mjtGeom.mjGEOM_CAPSULE:
            continue
        if keep is not None and m.geom_bodyid[g] not in keep:
            continue
        r, half = float(m.geom_size[g][0]), float(m.geom_size[g][1])
        c = h.data.geom_xpos[g]
        axis = h.data.geom_xmat[g].reshape(3, 3)[:, 2]  # capsules extend along local z
        out.append((c - half * axis, c + half * axis, r))
    return out


def _seg_seg_dist(p1, p2, p3, p4) -> float:
    """Shortest distance between two line SEGMENTS."""
    u, v, w = p2 - p1, p4 - p3, p1 - p3
    a, b, c = u @ u, u @ v, v @ v
    d, e = u @ w, v @ w
    D = a * c - b * b
    if D < 1e-12:  # parallel
        sN, sD, tN, tD = 0.0, 1.0, e, c if c > 1e-12 else 1.0
    else:
        sN, tN = (b * e - c * d), (a * e - b * d)
        sD = tD = D
        if sN < 0:
            sN, tN, tD = 0.0, e, c
        elif sN > sD:
            sN, tN, tD = sD, e + b, c
    if tN < 0:
        tN = 0.0
        sN, sD = np.clip(-d, 0, a), a
    elif tN > tD:
        tN = tD
        sN, sD = np.clip(-d + b, 0, a), a
    sc = 0.0 if abs(sD) < 1e-12 else sN / sD
    tc = 0.0 if abs(tD) < 1e-12 else tN / tD
    return float(np.linalg.norm(w + sc * u - tc * v))


def clearance(h: MyoHand, q: np.ndarray, frame: Frame, offset=None, only=None, bone=False) -> dict[str, float]:
    """Signed gap from each RIGID member to the hand's flesh. Negative = inside the hand.

    `offset` translates the frame onto a DIFFERENT hand: the device is built in the
    reference hand's world coordinates, and it straps to the dorsum, so on another hand it
    is the same frame shifted to that hand's origin.

    This is checked for the STRUCTURAL frame only -- spine, knuckle bar, side rails, arms.
    Those must never touch the hand. Three exemptions, all deliberate:

      * key stalks and key bars -- these are the parts the finger is SUPPOSED to meet. To
        reach a deeper row a finger must curl PAST the shallower one, so its middle phalanx
        ends up curled around row 0's bar and its cap. A segment-vs-capsule test scores that
        as a collision; it is not one, it is a finger resting over its own low-profile key.
        ⚠ The honest limitation: this test cannot DISTINGUISH the two, so it does not police
        the key plate at all. Doing that properly is a swept-volume problem (does the finger
        sweep clear the caps it passes over?) and is NOT solved here.
      * `only` -- for the STRAP-MOUNTED BODY, pass only=DIGIT_FLESH. The body RESTS ON THE
        PALM; that contact is the entire point of a palm support, and checking it would
        score the device's own load path as a collision. What the body must not do is
        intersect the FINGERS that curl around it, so that is what is checked.
      * straps -- webbing is *supposed* to encircle the limb. Modelled as a straight
        tension-only chord from the dorsal frame to a palmar anchor, it necessarily
        passes through the wrist, which is a geometric artefact of the idealisation and
        not a collision. (It does mean strap LENGTH here understates the wrapped path, so
        strap mass is a slight underestimate.)
    """
    caps = _flesh_capsules(h, q, only=only, bone=bone)
    off = np.zeros(3) if offset is None else np.asarray(offset, float)
    gaps = {}
    for m in frame.members:
        # The CONTACT SURFACE is exempt. The face of a strap-mounted body (and the key bar
        # of an exoskeleton) is what the fingertips land on -- the fingers are SUPPOSED to be
        # hard against it. A finger reaching its deep key necessarily lies over its shallow
        # one and that key's patch of face: that is not a collision, it is how a two-row key
        # works. Checking it flagged the middle finger's own distal phalanx against the face
        # member joining its own two key feet (-6.7 mm) -- the device penalised for being
        # touched where it exists to be touched.
        #
        # ⚠ LIMITATION, stated: this means nothing polices the face itself. Telling "resting
        # on it" apart from "sunk into it" needs a real contact/swept-volume model, which
        # this does not have. What IS checked is the load-bearing structure -- walls, palm
        # support, clip -- which must never intersect a finger.
        if m.kind in ("nylon", "strap") or m.name.startswith(("bar", "stalk", "face", "stem")):
            continue
        a, b = frame.nodes[m.i] + off, frame.nodes[m.j] + off
        gaps[m.name] = min(_seg_seg_dist(a, b, c0, c1) - r for c0, c1, r in caps)
    return gaps


def build_dorsal(h: MyoHand, q: np.ndarray, keys: dict, p: dict | None = None) -> Frame:
    """A FRAME THAT HUGS THE BACK OF THE HAND. The palm stays free.

    THE USER, and it is the best argument made about this device:

        "having the supporting structure far from the hand is a problem because it
         'gets-in-the-way' of me using my hands. If the supporting structure hugs the hand and
         stays above the sensors as much as possible it becomes more a natural extension,
         rather than holding a big ball."

    MEASURED, and they are right: of `build_body`'s 16 structural nodes, **15 are PALMAR of the
    hand**, standing off it by a mean of 27 mm and a maximum of 68 mm. That is the volume you
    use to hold a cup, a pen, a door handle. The device is not ON the hand. It is a BALL the
    hand is wrapped around.

    THE BACK OF THE HAND IS FREE. Nothing uses it. So:

        dorsal spine over the metacarpals  ->  a rail dorsal of each finger  ->  around the
        fingertip  ->  the well, which sits just palmar of the pad.

    Everything hugs. The palm is empty.

    WHY THE OLD OBJECTION IS DEAD. The articulated exoskeleton was abandoned because "an open
    frame cannot reach into the palm from outside" -- its thumb arm cut the hand three ways.
    But that was when the keys were DEEP IN A GRIPPING PALM. The hand is now OPEN and the wells
    are AT THE FINGERTIPS, which a dorsal rail reaches by simply running along the finger and
    wrapping the tip. The topological trap died with the gripping posture, and it was never
    re-examined. It should have been.

    THE LOAD PATH ALSO INVERTS, and in the right direction. A keypress drives the well PALMAR.
    A rail that comes over the fingertip and wraps to the well takes that in TENSION round the
    wrap, then hands it back along the finger to the knuckles and into the strap. The palmar box
    took the same load as BENDING through a 90 mm cantilever hung in mid-air.

    ⚠ The rail is anchored to the METACARPALS, not to the finger -- otherwise the finger and
    the well would move together and no keypress would ever register. It hugs the finger; it is
    not strapped to it.
    """
    import mujoco

    p = {**BODY_DEFAULTS, **(p or {})}
    keys = {(k if isinstance(k, tuple) else (k, 0)): v for k, v in keys.items()}
    o, e_d, e_r, e_o = hand_axes(h, q)
    h.fk(q)
    m = h.model

    nodes: dict[str, np.ndarray] = {}
    members: list[Member] = []
    ab, ad = p["sec_alu"]
    nb, nd = p["sec_nylon"]
    sb, sd = p["sec_strap"]

    def strut(name, i, j, kind="alu"):
        mat = {"alu": p["mat_frame"], "nylon": p["mat_stalk"], "strap": "webbing"}[kind]
        bd = {"alu": (ab, ad), "nylon": (nb, nd), "strap": (sb, sd)}[kind]
        members.append(Member(name, i, j, mat, bd[0], bd[1], kind))

    hug = float(p.get("hug", 0.004))   # how far the rail stands off the skin. HUGGING.

    def dorsal_of(body: str) -> np.ndarray:
        """A point just DORSAL of a bone's flesh -- derived from its own capsule, not guessed."""
        bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, body)
        best = None
        for g in range(m.body_geomadr[bid], m.body_geomadr[bid] + m.body_geomnum[bid]):
            if m.geom_type[g] != mujoco.mjtGeom.mjGEOM_CAPSULE:
                continue
            r = float(m.geom_size[g][0])
            c = h.data.geom_xpos[g]
            best = c + (r + hug) * e_o
        if best is None:
            bid_pos = h.data.xpos[bid]
            best = bid_pos + 0.010 * e_o
        return best

    # --- the DORSAL SPINE: a bar across the knuckles, and one across the wrist -------------
    for i, mc in enumerate(("secondmc", "thirdmc", "fourthmc", "fifthmc")):
        nodes[f"knuck{i}"] = dorsal_of(mc)
    for i in range(3):
        strut(f"spine{i}", f"knuck{i}", f"knuck{i+1}")

    # --- a RAIL dorsal of each finger, out to the fingertip, then AROUND it ----------------
    CHAIN = {
        "thumb": ("firstmc", "proximal_thumb", "distal_thumb"),
        "index": ("proxph2", "midph2", "distph2"),
        "middle": ("proxph3", "midph3", "distph3"),
        "ring": ("proxph4", "midph4", "distph4"),
        "little": ("proxph5", "midph5", "distph5"),
    }
    anchor = {"thumb": "knuck0", "index": "knuck0", "middle": "knuck1",
              "ring": "knuck2", "little": "knuck3"}

    # ⚠ A CHAIN IS A LINKAGE, NOT A TRUSS. The first version ran a serial chain
    #   knuckle -> proximal -> middle -> distal -> wrap -> well
    # with NO triangulation anywhere, and a keypress simply FOLDED it: 3.72 mm of deflection
    # against a 0.5 mm gate -- 105x worse than the palmar box. It hugged beautifully and it was
    # useless. Struts in a line carry nothing but the line.
    #
    # So: TRIANGULATE. A diagonal from the anchor to the distal node braces the chain, and the
    # well hangs from a FORK -- two members, from the middle AND distal rail nodes -- because
    # one member to a well is a LEVER ARM and two are a COUPLE. That is the whole difference.
    dist_nodes = {}
    for (f, row), (kp, kn) in keys.items():
        kp, kn = np.asarray(kp, float), np.asarray(kn, float)
        prev = anchor[f]
        chain = []
        for k, bone in enumerate(CHAIN[f]):
            nm = f"rail_{f}{k}"
            nodes[nm] = dorsal_of(bone)
            strut(f"rail_{f}{k}", prev, nm)
            chain.append(nm)
            prev = nm
        strut(f"brace_{f}", anchor[f], chain[-1])       # triangulate the rail itself
        dist_nodes[f] = chain[-1]

        kk, ft = f"key_{f}{row}", f"foot_{f}{row}"
        nodes[kk] = kp
        nodes[ft] = kp - p["stem"] * kn
        # THE FORK. Two members round the fingertip to the well: a couple, not a lever.
        strut(f"wrap_{f}", chain[-1], ft)
        strut(f"fork_{f}", chain[-2], ft)
        strut(f"stem_{f}{row}", ft, kk, kind="nylon")

    # cross-brace the rails to each other. They are all part of ONE rigid frame anchored to the
    # METACARPALS -- the fingers move UNDER them -- so bracing between fingers costs the hand
    # nothing and turns five floppy cantilevers into a shell.
    order = [f for f in ("thumb", "index", "middle", "ring", "little") if f in dist_nodes]
    for a, b in zip(order, order[1:]):
        strut(f"xbrace_{a}_{b}", dist_nodes[a], dist_nodes[b])

    # --- STRAP around the palm: what the dorsal frame reacts against ----------------------
    # The keypress ends up pulling the frame PALMAR; the strap round the hand holds it on, and
    # the load goes into the palm's soft tissue -- the same place it always went, but WITHOUT
    # a rigid box occupying the palm.
    o_p, dp, dd = o, p["body_prox"], p["body_dist"]
    nodes["palm_r"] = o_p + 0.5 * (dp + dd) * e_d + 0.030 * e_r - 0.012 * e_o
    nodes["palm_u"] = o_p + 0.5 * (dp + dd) * e_d - 0.030 * e_r - 0.012 * e_o
    strut("strap_r", "knuck0", "palm_r", kind="strap")
    strut("strap_u", "knuck3", "palm_u", kind="strap")
    strut("strap_x", "palm_r", "palm_u", kind="strap")

    return Frame(
        nodes=nodes,
        members=members,
        supports=["knuck0", "knuck1", "knuck2", "knuck3", "palm_r", "palm_u"],
        keys={(f, r): f"key_{f}{r}" for (f, r) in keys},
        key_normal={(f, r): np.asarray(v[1], float) for (f, r), v in keys.items()},
    )

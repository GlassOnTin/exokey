"""THE ANCHOR POINTS AND BOUNDARY CONDITIONS. Get these right, then optimise the shape.

THE USER, and it is the correct order of work:

    "The rigid support can still extend over the fingers if needed, but that's what we'll find
     out from the optimisation. What we need is the anchor points and boundary conditions first."

Every mistake in this project has been a boundary condition or an objective, never a search. So:

WHAT THE GAUNTLET BEARS ON, AND HOW HARD

1. IT IS NOT BOLTED TO A WALL. The first gauntlet model used RIGID supports, which absorb the
   keypress for free. Honest soft-tissue anchors made the button deflection 7x worse and it
   failed the gate at every thickness, including 2 mm and 70 g. A rigid clamp flatters every
   number downstream of it.

2. THE ANCHOR WAS A HINGE. The supports were the proximal RING of each metacarpal shell -- a
   LINE of nodes across the hand, with ZERO extent along the lever. A keypress at a fingertip
   121 mm away is a MOMENT, and a line cannot carry a moment. That is why 55% of the button's
   movement was the gauntlet ROCKING, and why a thicker shell did nothing: I was stiffening the
   beam while it pivoted on a pin.

   The fix is not material. It is EXTENT: bear on the CARPUS as well as the metacarpals, which
   takes the anchor patch from ~0 mm of lever to ~70 mm.

3. THE CONTACT IS ONE-WAY. Soft tissue can PUSH the gauntlet off the hand; it cannot PULL it
   back. A keypress drives the button palmar, which LIFTS the proximal end of the gauntlet --
   and nothing in the tissue resists that. Only the STRAP does. So:

       tissue  ->  COMPRESSION-ONLY springs   (it can only push)
       strap   ->  TENSION-ONLY springs       (it can only pull)

   Model them both ways round and the structure is being held on by a force that does not exist.

4. TISSUE STIFFNESS IS NOT ONE NUMBER, AND IT IS NOW MEASURED FROM MRI.

   k = E*A/t, so the thickness matters directly -- and MyoHand could not supply it. Its
   capsules are crude, and over the CARPUS, which is exactly where a gauntlet anchors, it has
   NO FLESH AT ALL. So the wrist tissue was a GUESS (3 mm), and it set the stiffness of the
   structure's main anchor.

   The user: "I think we need to find a flesh model to go with the bones." So we did --
   the PIANO hand-MRI dataset (Apache-2.0, 50 volumes with bone masks; scripts/measure_tissue.py).
   Segment the skin from the raw MRI, take the bone surface from the mask, and RAY-CAST along
   each bone's OWN outward normal until the ray leaves the hand. That distance IS the tissue
   the gauntlet presses through.

       region                     assumed                 MEASURED (MRI)
       dorsal carpus (the anchor)   3.0 mm    (a GUESS)       6.8 mm
       dorsal metacarpals           1.4-3.1 mm (capsules)     4.8 mm
       fingertip pulp (palmar)      --                        4.8 mm  (thicker than the nail
                                                                       bed -- sanity holds)

   THE TISSUE IS ~2x THICKER THAN ASSUMED, so THE ANCHOR WAS ~2x TOO STIFF. The guess was
   flattering the design in the one place the whole structure hangs from.

   ⚠ AND THE DESIGN TURNED OUT NOT TO CARE, which is the point of measuring. With the anchor
   2.75x softer (4,485 kN/m against 12,359), the button deflection moved 361 -> 376 um -- 4%.
   The DISTRIBUTED PATCH made the structure insensitive to the very number it used to hang on.
   Before the patch fix, that guess would have decided everything.
"""
from __future__ import annotations

import mujoco
import numpy as np

from design.params import P, Source
from structure.frame import MATERIALS, SOFT_TISSUE_K, _bone_radius, hand_axes

# The bones a gauntlet may bear on. The FINGERS are NOT here: they move, so they cannot be
# anchors -- they are what the device has to stay clear of.
CARPUS = ("capitate", "hamate", "lunate", "scaphoid", "trapezoid", "triquetrum")
METACARPALS = ("secondmc", "thirdmc", "fourthmc", "fifthmc")
BEARING = CARPUS + METACARPALS

# MEASURED FROM MRI. Not a guess any more.
#
# It WAS a guess -- 3.0 mm -- because MyoHand has no flesh over the carpus at all, and the
# guess set the stiffness of the gauntlet's MAIN ANCHOR. So we went and got a flesh model:
# the PIANO hand-MRI dataset (Apache-2.0), 50 volumes with bone masks. Segment the skin from
# the raw MRI, take the bone surface from the mask, and RAY-CAST along each bone's own outward
# normal until the ray leaves the hand. That distance IS the tissue the gauntlet presses through.
#
#   region                    guessed / MyoHand      MEASURED (MRI)
#   dorsal carpus (the anchor)   3.0 mm  (GUESS)         6.8 mm
#   dorsal metacarpals           1.4-3.1 mm (capsules)   4.8 mm
#   fingertip pulp (palmar)      --                      4.8 mm   (thicker than the nail bed:
#                                                                  the sanity check holds)
#
# THE TISSUE IS ~2x THICKER THAN WE ASSUMED, and k = E*A/t, so THE ANCHOR WAS ~2x TOO STIFF.
# The guess was flattering the design, in the one place the whole structure hangs from.
#
# ⚠ Two things wrong on the way here, both worth keeping:
#   * DISTANCE-TO-AIR IS THE WRONG METRIC. It is the shortest way out in ANY direction, and for
#     a palmar bone that is SIDEWAYS, not through the pad. It read the finger pulp as THINNER
#     than the nail bed, which is anatomically impossible.
#   * CAST ONLY FROM THE FACE YOU MEAN. Ray-casting dorsally from EVERY bone-surface voxel
#     sends the ray from the palmar ones straight THROUGH the bone and out the far side --
#     which read 7 mm of "skin" on the back of a hand that is famously skin over bone.
WRIST_TISSUE = P("WRIST_TISSUE", 0.0068, "m", Source.DERIVED,
                 "Dorsal carpus, MEASURED from the PIANO hand-MRI dataset (Apache-2.0): "
                 "ray-cast from the bone surface along its outward normal to the skin. Was a "
                 "3.0 mm GUESS, which made the gauntlet's main anchor twice as stiff as it is.",
                 describes="wrist anchor")

METACARPAL_TISSUE = P("METACARPAL_TISSUE", 0.0048, "m", Source.DERIVED,
                      "Dorsal metacarpals, MEASURED from the same MRI. MyoHand's crude "
                      "capsules said 1.4-3.1 mm.",
                      describes="dorsum anchor")

# E for compressed soft tissue, BACK-DERIVED from the literature figure the beam model already
# used: SOFT_TISSUE_K = 25 N/mm was quoted for a palm contact patch. Taking that patch as
# ~200 mm^2 over ~15 mm of thenar/hypothenar muscle gives E = k*t/A ~ 1.9 MPa. Consistent with
# published soft-tissue compression moduli, and -- more importantly -- it makes the DORSUM
# stiffer than the PALM by the ratio of their thicknesses, which is the physics.
TISSUE_E = P("TISSUE_E", 1.9e6, "Pa", Source.DERIVED,
             "Back-derived from SOFT_TISSUE_K (25 N/mm on a ~200 mm^2 palm patch over ~15 mm "
             "of muscle): E = k*t/A. Lets stiffness follow the MEASURED tissue thickness "
             "instead of being one number everywhere.",
             describes="soft tissue")


def tissue_thickness(h) -> dict[str, float]:
    """Soft tissue over each bearing bone. MEASURED where the model has flesh; declared where
    it does not."""
    m = h.model
    out = {}
    # MEASURED (MRI), not MyoHand's capsules. Its capsules put the dorsum at 1.4-3.1 mm; the
    # MRI says 4.8 mm, and the difference is a factor of two on the anchor's stiffness.
    for bn in METACARPALS:
        out[bn] = float(METACARPAL_TISSUE)
    for bn in CARPUS:
        out[bn] = float(WRIST_TISSUE)      # ⚠ not measurable here. Declared.
    return out


def bearing_surface(h, q, hug: float = 0.004, n_arc: int = 6, n_along: int = 3):
    """Where the gauntlet TOUCHES the hand, and how stiff each patch is.

    Returns (points, normals, stiffness_per_node, tissue_thickness_per_node).

    The normal points DORSALLY -- out of the hand. So the tissue can only resist motion in the
    -normal direction (the gauntlet pressing IN), which is what makes it compression-only.
    """
    m = h.model
    h.fk(np.zeros(m.nq))
    _, _, _, e_o0 = hand_axes(h, np.zeros(m.nq))
    dl = {}
    for bn in BEARING:
        bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, bn)
        dl[bn] = h.data.xmat[bid].reshape(3, 3).T @ e_o0
    h.fk(q)

    t_map = tissue_thickness(h)
    P_, N_, K_, T_ = [], [], [], []

    for bn in BEARING:
        bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, bn)
        R = h.data.xmat[bid].reshape(3, 3)
        dors = R @ dl[bn]
        dors /= np.linalg.norm(dors)

        cap = None
        for g in range(m.body_geomadr[bid], m.body_geomadr[bid] + m.body_geomnum[bid]):
            if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_CAPSULE:
                cap = g
                break
        if cap is not None:
            r = float(m.geom_size[cap][0])
            half = float(m.geom_size[cap][1])
            c = h.data.geom_xpos[cap].copy()
            ax = h.data.geom_xmat[cap].reshape(3, 3)[:, 2]
        else:
            # the CARPUS has no capsule. Take its bone mesh's extent instead -- crude, and said
            # so: the wrist is where this model is thinnest.
            V = []
            for g in range(m.body_geomadr[bid], m.body_geomadr[bid] + m.body_geomnum[bid]):
                if m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
                    continue
                mid = m.geom_dataid[g]
                va, vn = m.mesh_vertadr[mid], m.mesh_vertnum[mid]
                V.append(m.mesh_vert[va:va + vn] @ h.data.geom_xmat[g].reshape(3, 3).T
                         + h.data.geom_xpos[g])
            if not V:
                continue
            V = np.vstack(V)
            c = V.mean(axis=0)
            r = float(np.percentile(np.linalg.norm(V - c, axis=1), 60))
            half = r
            ax = np.array([0.0, 0.0, 1.0])
            ax = ax - (ax @ dors) * dors
            ax /= np.linalg.norm(ax) + 1e-12

        dp = dors - (dors @ ax) * ax
        dp /= np.linalg.norm(dp) + 1e-12
        lat = np.cross(dp, ax)
        t = t_map.get(bn, float(WRIST_TISSUE))

        # THE PATCH BEARS ON SKIN, NOT ON A CAPSULE. `r` above is MyoHand's flesh capsule (or,
        # for the carpus, a raw bone radius). What the gauntlet actually presses on is the skin:
        # BONE + the MRI-measured tissue. The capsule understates it, so the patch was drawn
        # ~2 mm inside the hand and its area -- hence its stiffness, k = E.dA/t -- with it.
        if cap is not None:
            r = _bone_radius(h, cap) + t
        else:
            r = r + t

        half_arc = np.pi / 3.0                 # bear on the dorsal 120 deg only
        for s_ in np.linspace(-half, half, n_along):
            for j in range(n_arc + 1):
                a = -half_arc + 2 * half_arc * j / n_arc
                nrm = np.cos(a) * dp + np.sin(a) * lat
                P_.append(c + s_ * ax + (r + hug) * nrm)
                N_.append(nrm)
                T_.append(t)
                # the patch each node speaks for
                dA = (2 * half / max(1, n_along - 1)) * ((r + hug) * 2 * half_arc / n_arc)
                K_.append(float(TISSUE_E) * dA / t)
    return np.array(P_), np.array(N_), np.array(K_), np.array(T_)


def report(h, q) -> None:
    P_, N_, K_, T_ = bearing_surface(h, q)
    o, e_d, e_r, e_o = hand_axes(h, q)
    d = (P_ - o) @ e_d
    print(f"  bearing nodes      {len(P_)}")
    print(f"  anchor extent      {(d.max()-d.min())*1000:.0f} mm along the hand "
          f"(a LINE has zero, and a line cannot carry a moment)")
    print(f"  tissue thickness   {T_.min()*1000:.1f} .. {T_.max()*1000:.1f} mm")
    print(f"  total stiffness    {K_.sum()/1000:.0f} kN/m")
    print(f"  stiffest patch     {K_.max()/1000:.1f} kN/m   softest {K_.min()/1000:.1f} kN/m")


STRAP_NODES_MIN = P("STRAP_NODES_MIN", 3.0, "-", Source.GUESS,
                    "Minimum structural nodes each strap band must be held by. A GUARD, not a "
                    "fix: measured, the struts at a strap node run at 4% of allowable stress -- "
                    "LESS than the average strut (7%) -- because the strap carries ~1 N and a "
                    "1.8 mm rod takes 89 N. So this is NOT about strength. It is about ESO being "
                    "free, otherwise, to delete its way down to ONE node holding the entire "
                    "tension side of the anchor: a single weld defect, a single fatigue crack, "
                    "and the gauntlet lifts off. Nothing has tested a joint, so the number is a "
                    "judgement about redundancy, and it is a guess.")

STRAP_W = P("STRAP_W", 0.016, "m", Source.GUESS,
            "Width of a strap band. 16 mm of webbing is a normal glove strap. NOT measured, and "
            "it decides how many anchor nodes the strap can pull on, which decides how much of "
            "the lift-off it can carry.")


def strap_bands(h, q, anchor_pts):
    """WHERE THE STRAP ACTUALLY IS. Two stations along the hand, and they are not free choices.

    ⚠ THE MODEL USED TO SMEAR THE STRAP OVER THE WHOLE BEARING PATCH -- every anchor node could
    pull. A strap cannot do that. It is a BAND: it pulls where it touches and nowhere else. Making
    it honest costs 11% of the button's steadiness (412 -> 459 um against a 500 um gate), and that
    11% was being taken from a strap that did not exist.

    The two stations are DERIVED, not chosen:

      * THE WRIST BAND goes PROXIMAL OF THE THUMB'S CMC. The user, looking at the render: "the
        straps are currently too far down the thumbs and so would restrict or load the thumb."
        Correct, and no loop around the base of the metacarpals can avoid the thumb ray -- it is
        simply THERE. Going proximal of the trapezium is what a watch strap does, and why.
      * THE METACARPAL BAND sits at the distal end of the bearing patch, where it passes through
        the FIRST WEB SPACE, distal of the thumb metacarpal (measured: 28 mm clear).

    And they must be far apart, because the keypress is a MOMENT and what resists a moment is a
    COUPLE. Two bands side by side in the middle of the palm would have almost no lever -- the
    "anchor was a hinge" mistake, which cost 55% of the button's travel when this project made it.

    ONE DEFINITION, used by BOTH the physics (structure/lattice) and the picture (viz/scene). They
    diverged once already: the render drew a strap round the thumb while the model quietly pulled
    on thumb-free nodes, and only the user's eye caught it.
    """
    import mujoco

    from hand.flesh import CARPUS as CARPAL_BONES, skin

    V, _, L = skin(h, q, labels=True)
    o, e_d, _e_r, _e_o = hand_axes(h, q)

    def span(names):
        ids = {mujoco.mj_name2id(h.model, mujoco.mjtObj.mjOBJ_BODY, b) for b in names}
        W = V[np.isin(L, list(ids))]
        if not len(W):
            return None
        d = (W - o) @ e_d
        return float(d.min()), float(d.max())

    # ⚠ THE WINDOW FOR A WRIST BAND IS DERIVED, NOT CHOSEN -- and my first rule was neither.
    #
    # "12 mm proximal of the most proximal thumb skin" sounds safe until you MEASURE where the
    # thumb metacarpal starts: it reaches back to -2 mm, right onto the carpus. So the rule walked
    # the band out to -31 mm -- past the wrist, onto the FOREARM -- where the gauntlet does not go.
    # The user saw it immediately: "the straps no longer appear connected to the elements".
    #
    # A band that the structure cannot reach is not a strap. It is a decoration that the solver is
    # nonetheless leaning on.
    #
    # The real window has two walls and they are both anatomy:
    #     PROXIMAL of the thumb metacarpal   -- or it loads the thumb
    #     ON the carpus                      -- or it is not on the hand
    # A wrist strap lives between them, which is exactly where a watch sits and why.
    thumb_mc = span(["firstmc"])
    carpus = span(CARPAL_BONES)
    d = (np.asarray(anchor_pts, float) - o) @ e_d

    if thumb_mc and carpus:
        lo = carpus[0] + 0.006                      # not right off the proximal edge of the wrist
        hi = min(thumb_mc[0] - 0.004, carpus[1])    # clear of the thumb metacarpal
        wrist = 0.5 * (lo + hi) if hi > lo else lo
    else:
        wrist = float(d.min())
    return [float(wrist), float(np.percentile(d, 88))]


def under_strap(h, q, nodes, anchor_ids, width=None):
    """Which anchor nodes each band touches. Returns {node: band index}.

    Per BAND, not just in total -- because the two bands are a COUPLE and a couple needs both ends.
    Three nodes on the metacarpal band and none on the wrist band is not an anchor; it is a hinge,
    and this project has already paid 55% of the button's travel to learn what a hinge does.
    """
    width = float(STRAP_W) if width is None else width
    o, e_d, _r, _oo = hand_axes(h, q)
    A = np.asarray([nodes[i] for i in anchor_ids], float)
    st = strap_bands(h, q, A)
    d = (A - o) @ e_d
    out = {}
    for i, di in zip(anchor_ids, d):
        for b, s_ in enumerate(st):
            if abs(float(di) - s_) < width / 2:
                out[int(i)] = b
                break
    return out


def strap_grip(strap_n, held):
    """How many structural nodes hold EACH band. The weakest band is the answer."""
    if not strap_n:
        return 0
    n_bands = max(strap_n.values()) + 1
    counts = [sum(1 for i, b in strap_n.items() if b == k and i in held) for k in range(n_bands)]
    return min(counts) if counts else 0

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

4. TISSUE STIFFNESS IS NOT ONE NUMBER. k = E*A/t, and t is MEASURED from the model: the skin
   over the metacarpals is only 2.1 mm thick (bone radius vs flesh capsule, perpendicular to
   the bone axis). Thin tissue is a STIFF anchor. `SOFT_TISSUE_K` = 25 N/mm was derived for a
   PALM patch -- a muscle pad ten times thicker -- and using it on the back of the hand
   understates the anchor badly.

⚠ WHAT IS NOT DERIVABLE FROM THIS MODEL. MyoHand has NO FLESH over the carpus or the forearm --
the carpals and the radius/ulna are bare bones. So the tissue thickness at the WRIST, which is
exactly where a gauntlet wants to anchor, cannot be measured here. It is taken as 3 mm
(anatomical: skin plus extensor tendons over the dorsal wrist) and it is declared a GUESS.
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

# ⚠ GUESS. MyoHand has no flesh over the carpus at all, so this cannot be measured from the
# model. Anatomically the dorsal wrist is skin + extensor tendons over bone: thin.
WRIST_TISSUE = P("WRIST_TISSUE", 0.003, "m", Source.GUESS,
                 "Soft-tissue thickness over the dorsal carpus. NOT derivable: MyoHand's "
                 "carpals have no flesh geoms at all. Anatomically thin (skin + extensor "
                 "tendons). It sets how stiff the gauntlet's main anchor is.",
                 describes="wrist anchor")

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
    for bn in METACARPALS:
        bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, bn)
        for g in range(m.body_geomadr[bid], m.body_geomadr[bid] + m.body_geomnum[bid]):
            if m.geom_type[g] != mujoco.mjtGeom.mjGEOM_CAPSULE:
                continue
            out[bn] = max(0.001, float(m.geom_size[g][0]) - _bone_radius(h, g))
            break
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

"""A FLESH MODEL FOR MYOHAND, from MRI. Bones we had; skin we did not.

THE USER: "I think we need to find a flesh model to go with the bones."  ...and then, seeing
where it led: "That should greatly improve the render quality too!"

Both true. MyoHand ships BONES (meshes) and crude flesh CAPSULES -- and over the CARPUS, which
is exactly where a gauntlet anchors, NO FLESH AT ALL. So we had been:

  * guessing the wrist tissue (3 mm) that set the stiffness of the structure's main anchor,
  * hugging the hand at a fixed 4 mm standoff from CAPSULES rather than from skin,
  * and rendering a skeleton, which is why every geometry bug in this project had to be caught
    by a number rather than by eye.

MEASURED FROM MRI (PIANO dataset, Apache-2.0 -- scripts/measure_tissue.py). Ray-cast from each
bone-surface voxel along its OWN outward normal until it leaves the hand:

    region                       DORSAL     PALMAR
    wrist / carpus               6.8 mm     6.6 mm
    metacarpals                  4.8 mm     3.8 mm
    proximal phalanges           2.8 mm     1.0 mm   <-- see the caveat
    fingertips                   2.8 mm     4.8 mm   <-- pulp thicker than nail bed. Correct.

⚠ THE PROXIMAL-PHALANX PALMAR FIGURE (1.0 mm) IS NOT BELIEVABLE and is not used. The palmar side
of a proximal phalanx carries flexor tendons and fat; 1 mm is too thin. It is most likely the
Otsu hand mask biting into the finger where the digits touch. MyoHand's own capsules give
2-3 mm there and agree with the MRI's DORSAL figures, so the PHALANGES keep their capsules and
only the METACARPALS and CARPUS take MRI values -- which is where MyoHand had nothing anyway.

Say what is measured, say what is kept, and say what is thrown away.
"""
from __future__ import annotations

import mujoco
import numpy as np

from structure.frame import hand_axes

# MEASURED (MRI). Dorsal / palmar, in metres.
TISSUE = {
    "carpus": (0.0068, 0.0066),
    "metacarpal": (0.0048, 0.0038),
}
CARPUS = ("capitate", "hamate", "lunate", "scaphoid", "trapezoid", "triquetrum",
          "trapezium", "pisiform")
METACARPALS = ("firstmc", "secondmc", "thirdmc", "fourthmc", "fifthmc")
PHALANGES = ("proximal_thumb", "distal_thumb",
             "proxph2", "midph2", "distph2", "proxph3", "midph3", "distph3",
             "proxph4", "midph4", "distph4", "proxph5", "midph5", "distph5")


def _capsule_tissue(h, bid):
    """MyoHand's own estimate, for the bones where it HAS one (the phalanges) and where the MRI
    agrees with it."""
    from structure.frame import _bone_radius

    m = h.model
    for g in range(m.body_geomadr[bid], m.body_geomadr[bid] + m.body_geomnum[bid]):
        if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_CAPSULE:
            return max(0.0008, float(m.geom_size[g][0]) - _bone_radius(h, g))
    return None


def skin(h, q, subdiv: int = 1, labels: bool = False):
    """The hand's SKIN, posed. Returns (vertices, faces) -- or (vertices, faces, body_id) if
    `labels`, so a caller can ask for the skin OVER ONE BONE.

    Each bone's own mesh, pushed out along its vertex normals by the tissue that covers it --
    and the tissue is DIRECTION-DEPENDENT, because a hand is not a sausage: the back of the hand
    is skin over bone and the palm is a muscle pad. The offset at a vertex is interpolated
    between the dorsal and palmar figures by how dorsally that vertex faces.
    """
    m = h.model
    h.fk(q)
    _, _, _, e_o = hand_axes(h, q)

    V_all, F_all, L_all = [], [], []
    for b in range(m.nbody):
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b)
        if not name:
            continue
        if name in CARPUS:
            td, tp = TISSUE["carpus"]
        elif name in METACARPALS:
            td, tp = TISSUE["metacarpal"]
        elif name in PHALANGES:
            t = _capsule_tissue(h, b)          # the phalanges keep their capsules: MRI agrees
            if t is None:
                continue
            td = tp = t
        else:
            continue                            # forearm, wrist, world: not part of the hand

        for g in range(m.body_geomadr[b], m.body_geomadr[b] + m.body_geomnum[b]):
            if m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
                continue
            mid = m.geom_dataid[g]
            va, vn = m.mesh_vertadr[mid], m.mesh_vertnum[mid]
            fa, fn = m.mesh_faceadr[mid], m.mesh_facenum[mid]
            Vl = m.mesh_vert[va:va + vn].copy()
            Fl = m.mesh_face[fa:fa + fn].copy()

            R = h.data.geom_xmat[g].reshape(3, 3)
            c = h.data.geom_xpos[g]
            Vw = Vl @ R.T + c

            # vertex normals, from the mesh's own faces
            N = np.zeros_like(Vw)
            tri = Vw[Fl]
            fnrm = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
            for k in range(3):
                np.add.at(N, Fl[:, k], fnrm)
            ln = np.linalg.norm(N, axis=1, keepdims=True)
            N = N / np.maximum(ln, 1e-12)

            # DIRECTION-DEPENDENT OFFSET: dorsal where it faces the back of the hand, palmar
            # where it faces the palm. A hand is not a sausage.
            w = np.clip((N @ e_o + 1.0) * 0.5, 0.0, 1.0)      # 1 = dorsal, 0 = palmar
            t = tp + (td - tp) * w
            V_all.append(Vw + N * t[:, None])
            F_all.append(Fl + sum(len(v) for v in V_all[:-1]))
            L_all.append(np.full(len(Vw), b, int))

    if not V_all:
        z = (np.zeros((0, 3)), np.zeros((0, 3), int))
        return (*z, np.zeros(0, int)) if labels else z
    V, F = np.vstack(V_all), np.vstack(F_all)
    return (V, F, np.concatenate(L_all)) if labels else (V, F)


def clearance_to_skin(h, q, points) -> np.ndarray:
    """Signed distance from each point to the SKIN (positive = outside the hand).

    This is what "hug" should be measured against. It was measured against CAPSULES, which are
    a stand-in for flesh, not flesh."""
    V, _ = skin(h, q)
    if not len(V):
        return np.full(len(points), np.nan)
    return np.array([float(np.min(np.linalg.norm(V - p, axis=1))) for p in np.asarray(points)])

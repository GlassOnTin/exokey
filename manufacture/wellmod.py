"""THE WELL MODULE -- the printed geometry that holds the magnet, the Hall, and the wires.

design.sensor and manufacture.flexure settled the mechanics (a soft TPU dome, k ~= 131 N/m);
manufacture.readout settled the signal (a disc magnet's field the Hall can read). This turns both
into SOLID: per finger, a two-part module oriented by `h.well_frame`.

    THE FRAME   rigid PA, part of the gauntlet solid. A base plate that carries the Hall carrier
                PCB, collar walls that key the drop-in cradle, a boss the truss struts land on, and
                SKIN_R rims. Wire-exit slot + PCB seat are CARVED (manufacture.mesh.carve).
    THE INSERT  a drop-in TPU cradle: the cup the fingertip sits in, on the crown of the dome
                flexure, with the magnet press-fit into a pocket facing the Hall, and a keyed skirt
                that snaps into the frame collar. Printed separately (PA and TPU do not weld -- the
                keying is MECHANICAL) and dropped in.

Everything is world-coordinate primitive lists (boxes / capsules / flat cylinders to ADD, cylinders
/ boxes to CARVE) so a whole hand's worth composes into one `mesh.field` + one `mesh.carve`. The
stack runs along `floor` (palmar): pad -> cup floor -> magnet -> gap -> Hall -> base plate.

⚠ The dome here is a FLAT diaphragm disc of the flexure.dome thickness -- the printable stand-in
for the shallow-cone dome; its snap/buckle is not in this geometry (design.flexure notes it). The
restoring-rate and fatigue claims are design.flexure's; this file only has to be printable and hold
the parts in the right places, which the tests check.
"""
from __future__ import annotations

import numpy as np

from design.params import MAGNET_D, MAGNET_L, REST_GAP, SVALBOARD
from manufacture import mesh
from manufacture.flexure import dome, spring_rate
from manufacture.friendly import SKIN_R
from manufacture.readout import PLUNGE_STOP, TRAVEL
from structure.frame import MATERIALS

# ---- the module's fixed dimensions (SI). Kept here so the tests can assert the design rules. ----
K = spring_rate(float(SVALBOARD.force), float(SVALBOARD.travel))     # ~131 N/m
_TPU = MATERIALS["tpu"]
DOME_A = 0.006                              # dome radius, inside the ~7 mm flesh well (design.flexure)
DOME_T = float(dome(K, DOME_A, _TPU["E"], _TPU["nu"]))              # derived membrane thickness
CUP_WALL = 0.0025                          # TPU cup wall -- stiff relative to the dome (test)
PA_WALL = 0.0018                           # rigid frame wall
BASE_T = 0.0020                            # frame base plate thickness under the PCB
PCB = (0.0064, 0.0064, 0.0018)             # Hall carrier PCB (x along axis, y lateral, z along floor)
MAGNET_POCKET_D = float(MAGNET_D) - 0.1e-3     # 0.1 mm press-fit interference
MAGNET_POCKET_DEPTH = float(MAGNET_L) + 0.2e-3  # 0.2 mm glue relief behind the disc
SKIRT_WALL = 0.0012                        # TPU skirt wall
SKIRT_LIP = 0.0003                         # radial keying interference (snap)
SKIRT_ENGAGE = 0.0010                      # how deep the lip seats
GROOVE_R = 0.0006                          # wire channel radius
GROOVE_BURY = 0.0004                       # channel centre below the strut surface (re-entrant)


def _R(wf):
    """The box orientation matrix: rows = (axis, floor, lateral), the well's own frame."""
    return np.vstack([wf["axis"], wf["floor"], wf["lateral"]])


SKIRT_LEN = 0.003                          # how far the skirt reaches into the collar


def _stack(wf):
    """The offsets ALONG floor (palmar) of each layer, from the pad point `pos` [m].

    Palmar is +floor. The finger bottoms on the cup floor's DORSAL face; the magnet's Hall-facing
    face is flush with the cup floor's PALMAR face, so its pocket carves back INTO the floor slab.
    """
    r = wf["radius"]
    s_cup_dorsal = r                                  # the fingertip bottoms here
    s_cup_palmar = r + CUP_WALL                       # palmar face of the cup floor slab
    s_magface = s_cup_palmar                          # magnet Hall-face flush with the cup floor
    s_hall = s_magface + float(REST_GAP)              # Hall sensing point
    s_pcb_c = s_hall + 0.5 * PCB[2]
    s_base_c = s_hall + PCB[2] + 0.5 * BASE_T
    return dict(cup_dorsal=s_cup_dorsal, cup_palmar=s_cup_palmar, magface=s_magface,
                hall=s_hall, pcb_c=s_pcb_c, base_c=s_base_c, depth=s_base_c + 0.5 * BASE_T)


def module_frame(h, q, finger, *, wire_len=0.010):
    """The rigid PA frame for one well, as primitive lists in world coords.

    Returns dict(boxes, caps, cyls, carve_cyls, carve_boxes):
      boxes/caps/cyls  -- ADD (blend into the gauntlet struts via mesh.field)
      carve_*          -- SUBTRACT (PCB seat, wire-exit slot; via mesh.carve)
    """
    wf = h.well_frame(q, finger)
    pos = np.asarray(wf["pos"], float)
    ax, fl, lat = wf["axis"], wf["floor"], wf["lateral"]
    r, half = wf["radius"], wf["half"]
    R = _R(wf)
    s = _stack(wf)
    cc = pos - 0.5 * half * ax                        # channel centre (behind the pad)

    boxes, caps, cyls, carve_cyls, carve_boxes = [], [], [], [], []

    # BASE PLATE -- carries the Hall PCB, palmar-most.
    boxes.append((cc + s["base_c"] * fl, R,
                  np.array([half + PA_WALL, 0.5 * BASE_T, r + PA_WALL])))

    # COLLAR -- two lateral walls from the cup rim to the base, the fixed guide the skirt keys into.
    # ⚠ These frames are sized per finger INDEPENDENTLY. At the tightest pitch (middle-ring, ~18-20
    # mm) the two sensor tails run nearly parallel and INTERPENETRATE (test_wellmod). Resolving that
    # needs a cluster-level layout -- shared walls or staggered tail depth -- which is future work
    # (VISION 8.15l). Every other pair clears; this one is a named, measured limitation.
    wall_lo, wall_hi = s["cup_dorsal"], s["base_c"]
    wc = 0.5 * (wall_lo + wall_hi)
    wh = 0.5 * (wall_hi - wall_lo)
    for side in (+1.0, -1.0):
        boxes.append((cc + wc * fl + side * (r + 0.5 * PA_WALL) * lat, R,
                      np.array([half, wh, 0.5 * PA_WALL])))
    # DISTAL END WALL (open proximally so the finger enters and wires exit).
    boxes.append((cc + wc * fl + half * ax, R, np.array([0.5 * PA_WALL, wh, r + PA_WALL])))

    # SKIN_R RIM BEADS -- round the two touchable dorsal edges of the collar walls (strut landings).
    for side in (+1.0, -1.0):
        a = cc - half * ax + side * (r + 0.5 * PA_WALL) * lat + s["cup_dorsal"] * fl
        b = cc + half * ax + side * (r + 0.5 * PA_WALL) * lat + s["cup_dorsal"] * fl
        caps.append(((a, b), float(SKIN_R)))

    # CARVE: the PCB seat, opening palmar.
    carve_boxes.append((cc + s["pcb_c"] * fl, R,
                        np.array([0.5 * PCB[0], 0.5 * PCB[2], 0.5 * PCB[1]])))
    # CARVE: the wire-exit slot, from the seat out through the proximal (open) end.
    slot_c = cc + s["hall"] * fl - (0.5 * half + 0.5 * wire_len) * ax
    carve_boxes.append((slot_c, R, np.array([0.5 * wire_len + half, GROOVE_R, GROOVE_R])))

    return dict(boxes=boxes, caps=caps, cyls=cyls,
                carve_cyls=carve_cyls, carve_boxes=carve_boxes, wf=wf, stack=s)


def _insert_primitives(wf, *, nail_hood=True, dome_a=DOME_A, dome_t=DOME_T):
    """The TPU cradle as primitive lists in world coords (its own small part)."""
    pos = np.asarray(wf["pos"], float)
    ax, fl, lat = wf["axis"], wf["floor"], wf["lateral"]
    r, half = wf["radius"], wf["half"]
    R = _R(wf)
    s = _stack(wf)
    cc = pos - 0.5 * half * ax

    boxes, caps, cyls, carve_cyls = [], [], [], []
    # CUP: a CONNECTED U-channel -- floor slab + two flanks + distal end, the walls rising from the
    # floor slab up around the fingertip (floor-range [s_top, cup_palmar]) so it is one solid part.
    s_top = -0.3 * r                                 # the walls rise this far dorsal of the pad
    s_bot = s["cup_palmar"]
    vc, vh = 0.5 * (s_top + s_bot), 0.5 * (s_bot - s_top)
    boxes.append((cc + (r + 0.5 * CUP_WALL) * fl, R, np.array([half, 0.5 * CUP_WALL, r + CUP_WALL])))
    for side in (+1.0, -1.0):
        boxes.append((cc + vc * fl + side * (r + 0.5 * CUP_WALL) * lat, R,
                      np.array([half, vh, 0.5 * CUP_WALL])))
    boxes.append((cc + vc * fl + half * ax, R, np.array([0.5 * CUP_WALL, vh, r + CUP_WALL])))  # end
    if nail_hood:                                   # a dorsal lip over the nail so `back`/lift transmit
        boxes.append((cc + half * ax + s_top * fl, R,
                      np.array([0.5 * CUP_WALL, 0.35 * r, r])))

    # DOME diaphragm: a thin disc (normal = floor) at the cup's palmar face, joining cup to skirt.
    dc = cc + s["cup_palmar"] * fl
    cyls.append((dc, dc + dome_t * fl, dome_a))

    # SKIRT: a tube from the cup palmar face reaching palmar into the frame collar, with a snap lip.
    a_lo = cc + s["cup_palmar"] * fl
    a_hi = cc + (s["cup_palmar"] + SKIRT_LEN) * fl
    cyls.append((a_lo, a_hi, dome_a + SKIRT_WALL))                    # skirt outer
    bore0 = cc + (s["cup_palmar"] + dome_t) * fl                      # start PALMAR of the dome
    carve_cyls.append((bore0, a_hi + 0.001 * fl, dome_a))            # bore hollow -> tube, dome caps it
    lip_c = cc + (s["cup_palmar"] + SKIRT_LEN - SKIRT_ENGAGE) * fl
    cyls.append((lip_c, lip_c + SKIRT_ENGAGE * fl, dome_a + SKIRT_WALL + SKIRT_LIP))  # the snap lip

    # MAGNET POCKET: carved from PALMAR of the dome, THROUGH the dome centre, into the cup floor
    # slab. The magnet is the rigid centre of the dome (the flexure is the surviving annulus) and
    # its face opens into the hollow skirt toward the Hall -- so it can be dropped in, and it is not
    # sealed behind a wall. (A sealed pocket makes an enclosed void: not printable, not fillable.)
    mp_open = cc + (s["cup_palmar"] + dome_t) * fl
    carve_cyls.append((mp_open, mp_open - (dome_t + MAGNET_POCKET_DEPTH) * fl, 0.5 * MAGNET_POCKET_D))

    return dict(boxes=boxes, caps=caps, cyls=cyls, carve_cyls=carve_cyls, R=R)


def insert_mesh(h, q, finger, *, voxel=3e-4, nail_hood=True):
    """The drop-in TPU cradle as a watertight trimesh, printed standalone."""
    p = _insert_primitives(h.well_frame(q, finger), nail_hood=nail_hood)
    f, o, v = mesh.field(p["caps"], p["boxes"], r=[c[1] for c in p["caps"]] or 0.0,
                         voxel=voxel, cyls=p["cyls"])
    mesh.carve(f, o, v, cyls=p["carve_cyls"])
    return mesh.to_mesh(f, o, v)


def frame_mesh(h, q, finger, struts=(), radii=0.0, *, voxel=4e-4):
    """The PA frame for one well (optionally with the truss struts that land on it) -- watertight."""
    m = module_frame(h, q, finger)
    caps = list(struts) + [c[0] for c in m["caps"]]
    rr = ([radii] * len(struts) if np.isscalar(radii) else list(radii)) + [c[1] for c in m["caps"]]
    f, o, v = mesh.field(caps, m["boxes"], r=rr, voxel=voxel, cyls=m["cyls"])
    mesh.carve(f, o, v, cyls=m["carve_cyls"], boxes=m["carve_boxes"])
    return mesh.to_mesh(f, o, v)


def coupon_meshes(dome_t=(0.25e-3, 0.32e-3, 0.40e-3), dome_a=(0.006, 0.007)):
    """Bench coupons: a cup+dome+magnet-pocket TPU insert per (t, a), and one PA seat plate.

    Standalone, axis-aligned (no hand needed) -- the stage-1 bench measures k, mT-vs-mm, and the
    direction confusion matrix on these. Returns {name: trimesh}.
    """
    out = {}
    ax, fl, lat = np.array([1., 0, 0]), np.array([0, 0, -1.]), np.array([0, 1., 0])
    wf0 = dict(pos=np.zeros(3), axis=ax, floor=fl, lateral=lat, half=0.006, radius=0.006)
    for t in dome_t:
        for a in dome_a:
            p = _insert_primitives(wf0, nail_hood=False, dome_a=a, dome_t=t)
            f, o, v = mesh.field(p["caps"], p["boxes"], r=0.0, voxel=3e-4, cyls=p["cyls"])
            mesh.carve(f, o, v, cyls=p["carve_cyls"])
            out[f"insert_t{t*1e3:.2f}_a{a*1e3:.0f}"] = mesh.to_mesh(f, o, v)
    # a flat PA seat plate with a PCB pocket at gap REST_GAP below the magnet rest face
    R = np.vstack([ax, fl, lat])
    seat_c = wf0["radius"] * fl + (float(REST_GAP) + 0.5 * PCB[2]) * fl
    plate = [(seat_c + 0.5 * (PCB[2] + BASE_T) * fl, R, np.array([0.012, 0.5 * BASE_T, 0.012]))]
    f, o, v = mesh.field([], plate, r=0.0, voxel=3e-4)
    mesh.carve(f, o, v, boxes=[(seat_c, R, np.array([0.5 * PCB[0], 0.5 * PCB[2], 0.5 * PCB[1]]))])
    out["seat_plate"] = mesh.to_mesh(f, o, v)
    return out


def harness_grooves(nodes, bars, live, btn, anchors):
    """Wire routes: the shortest path over live struts from each button node to its nearest anchor.

    Returns [route], each a list of NODE INDICES (button -> ... -> anchor) along live struts. The
    caller carves them as re-entrant capsule channels sunk into each strut's surface.
    """
    from scipy.sparse import csr_matrix
    from scipy.sparse.csgraph import dijkstra

    X = np.asarray(nodes, float)
    N = len(X)
    rows, cols, w = [], [], []
    for e in live:
        i, j = bars[e]
        d = float(np.linalg.norm(X[i] - X[j]))
        rows += [i, j]
        cols += [j, i]
        w += [d, d]
    G = csr_matrix((w, (rows, cols)), shape=(N, N))
    anch = list(anchors)
    routes = []
    for b in btn.values():
        dist, pred = dijkstra(G, indices=b, return_predecessors=True)
        reach = [(dist[a], a) for a in anch if np.isfinite(dist[a])]
        if not reach:
            continue
        _, tgt = min(reach)
        path, k = [tgt], tgt
        while k != b and pred[k] >= 0:
            k = int(pred[k])
            path.append(k)
        routes.append(path)
    return routes


def housing(anchor_nodes, *, xiao=(0.021, 0.0178, 0.0035), lipo=(0.020, 0.012, 0.006)):
    """A wrist box for the XIAO nRF52840 + LiPo at the anchor cluster centroid.

    Returns (boxes, carve_boxes): a solid shell to ADD and the two cavities to CARVE.
    """
    C = np.asarray(anchor_nodes, float).mean(axis=0)
    R = np.eye(3)
    wall = 0.0015
    inner = np.array([0.5 * xiao[0], 0.5 * (xiao[1] + lipo[1]), 0.5 * max(xiao[2], lipo[2])])
    shell = [(C, R, inner + wall)]
    cav = [(C + np.array([0, 0.25 * (xiao[1] + lipo[1]), 0]), R, 0.5 * np.array(xiao)),
           (C - np.array([0, 0.25 * (xiao[1] + lipo[1]), 0]), R, 0.5 * np.array(lipo))]
    return shell, cav

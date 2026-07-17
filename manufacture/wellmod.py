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
STALK_R = 0.0025                           # the post that ties a frame to the button node's struts


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


def module_frame(h, q, finger, *, mount=None, wire_len=0.010):
    """The rigid PA frame for one well, as primitive lists in world coords.

    `mount` is the STRUCTURE's button node (nodes[button]) -- where the truss struts actually land,
    which is ~10 mm from the fingertip pad `well_frame` reports (ground() places them differently).
    A stalk ties the frame to it, or the whole module floats free of the gauntlet. Defaults to the
    pad point (for standalone/coupon use, where there is no truss to reach).

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
    LAT = r + CUP_WALL + 0.0008 + 0.5 * PA_WALL       # collar OUTBOARD of the insert cup, so it nests
    s_dorsal = -0.25 * r                              # the frame's dorsal edge (nail side of the pad)

    # BASE PLATE -- carries the Hall PCB, palmar-most; reaches the collar walls so the frame is one.
    boxes.append((cc + s["base_c"] * fl, R,
                  np.array([half + PA_WALL, 0.5 * BASE_T, LAT + 0.5 * PA_WALL])))

    # COLLAR -- two lateral walls OUTBOARD of the insert cup (the insert drops in between them),
    # spanning from the DORSAL edge (where the strut ties in, opposite the palmar magnet) to the
    # base. ⚠ Sized per finger INDEPENDENTLY -- at the tightest pitch (middle-ring) the modules
    # interpenetrate, and nesting the collar outboard widened them, so this is worse until the
    # cluster-level layout (shared walls / staggered depth) that VISION 8.15l flags.
    wall_lo, wall_hi = s_dorsal, s["base_c"]
    wc = 0.5 * (wall_lo + wall_hi)
    wh = 0.5 * (wall_hi - wall_lo)
    for side in (+1.0, -1.0):
        boxes.append((cc + wc * fl + side * LAT * lat, R, np.array([half, wh, 0.5 * PA_WALL])))
    # DISTAL END WALL (open proximally so the finger enters and wires exit).
    boxes.append((cc + wc * fl + half * ax, R, np.array([0.5 * PA_WALL, wh, LAT + 0.5 * PA_WALL])))

    # DORSAL-LATERAL RIM + DISTAL BRACE -- the strut tie-in: the nail-side top edges and the
    # fingertip end, OPPOSITE the palmar magnet and clear of the finger cavity and the proximal
    # entry. The truss lands here, not down by the sensor.
    for side in (+1.0, -1.0):
        a = cc - half * ax + side * LAT * lat + s_dorsal * fl
        b = cc + half * ax + side * LAT * lat + s_dorsal * fl
        caps.append(((a, b), float(SKIN_R)))                       # dorsal-lateral rim rail
    dl = cc + half * ax + s_dorsal * fl
    caps.append(((dl - LAT * lat, dl + LAT * lat), float(SKIN_R)))  # distal brace across the rims

    # STALKS -- tie the button node (where the struts land) UP to the dorsal-lateral rim on both
    # sides, staying outboard so the load path is on the nail side, never across the pad or magnet.
    src = np.asarray(mount, float) if mount is not None else pos
    for side in (+1.0, -1.0):
        tip = cc - 0.3 * half * ax + side * LAT * lat + s_dorsal * fl
        caps.append(((src, tip), STALK_R))

    # CARVE: the PCB seat, opening palmar.
    carve_boxes.append((cc + s["pcb_c"] * fl, R,
                        np.array([0.5 * PCB[0], 0.5 * PCB[2], 0.5 * PCB[1]])))
    # CARVE: the wire-exit slot, from the seat out through the proximal (open) end.
    slot_c = cc + s["hall"] * fl - (0.5 * half + 0.5 * wire_len) * ax
    carve_boxes.append((slot_c, R, np.array([0.5 * wire_len + half, GROOVE_R, GROOVE_R])))

    return dict(boxes=boxes, caps=caps, cyls=cyls,
                carve_cyls=carve_cyls, carve_boxes=carve_boxes, wf=wf, stack=s)


def _unit(v):
    v = np.asarray(v, float)
    return v / (np.linalg.norm(v) + 1e-12)


def cluster_frame(h, q, fingers, mounts, *, wire_len=0.010):
    """ONE PA carrier for a ROW of wells (the long fingers), with SHARED inter-finger walls.

    An independent per-finger frame wide enough to nest its insert is wider than the finger pitch,
    so four of them interpenetrate (§8.15l). The cluster fixes that: the wall BETWEEN two fingers is
    a SINGLE shared wall, not two colliding ones. Per finger it still carries a Hall seat and a cup;
    the cups are the gaps between shared walls, and one continuous base spine + dorsal rim (following
    the fingertip arc) tie the row together and take the struts on the dorsal side.

    `fingers` must be given in ROW ORDER (index..little). `mounts` = {finger: button-node position}.
    Returns the same primitive dict as module_frame.
    """
    wf = {f: h.well_frame(q, f) for f in fingers}
    fl = {f: _unit(wf[f]["floor"]) for f in fingers}
    ax = {f: _unit(wf[f]["axis"]) for f in fingers}
    lt = {f: _unit(wf[f]["lateral"]) for f in fingers}
    r = {f: wf[f]["radius"] for f in fingers}
    half = {f: wf[f]["half"] for f in fingers}
    cc = {f: np.asarray(wf[f]["pos"], float) - 0.5 * half[f] * ax[f] for f in fingers}
    s = {f: _stack(wf[f]) for f in fingers}
    s_dorsal = {f: -0.25 * r[f] for f in fingers}

    boxes, caps, cyls, carve_cyls, carve_boxes = [], [], [], [], []
    for f in fingers:
        R = np.vstack([ax[f], fl[f], lt[f]])
        # BASE PLATE + Hall seat, palmar, PCB-width (the sensor tail is narrow -- it is the collars
        # that collided, and those are now shared).
        pcb_half = 0.5 * PCB[1] + PA_WALL
        boxes.append((cc[f] + s[f]["base_c"] * fl[f], R,
                      np.array([half[f] + PA_WALL, 0.5 * BASE_T, pcb_half])))
        carve_boxes.append((cc[f] + s[f]["pcb_c"] * fl[f], R,
                            np.array([0.5 * PCB[0], 0.5 * PCB[2], 0.5 * PCB[1]])))
        slot_c = cc[f] + s[f]["hall"] * fl[f] - (0.5 * half[f] + 0.5 * wire_len) * ax[f]
        carve_boxes.append((slot_c, R, np.array([0.5 * wire_len + half[f], GROOVE_R, GROOVE_R])))
    # BASE SPINE (palmar) -- runs BELOW the fingers, connecting the Hall seats.
    for a, b in zip(fingers, fingers[1:]):
        caps.append(((cc[a] + s[a]["base_c"] * fl[a], cc[b] + s[b]["base_c"] * fl[b]), 0.5 * BASE_T))

    # WALLS at the 5 lateral positions (2 outer + 3 shared), each spanning the dorsal rim down to the
    # base. ⚠ They sit BETWEEN the fingers -- NEVER over a cup centre -- so each finger drops into its
    # cup freely from the dorsal/proximal side. Each wall's dorsal edge is a RIM NODE; the rim rail
    # and the struts tie in there, on the nail side and opposite the palmar magnet.
    def mid_pt(f):
        return cc[f] + 0.5 * (s_dorsal[f] + s[f]["base_c"]) * fl[f]

    walls = [(fingers[0], mid_pt(fingers[0]) - (r[fingers[0]] + CUP_WALL + 0.0025) * lt[fingers[0]])]
    walls += [(a, 0.5 * (mid_pt(a) + mid_pt(b))) for a, b in zip(fingers, fingers[1:])]
    walls.append((fingers[-1],
                  mid_pt(fingers[-1]) + (r[fingers[-1]] + CUP_WALL + 0.0025) * lt[fingers[-1]]))

    rim_pts = []
    for fref, m in walls:
        u = _unit(fl[fref])
        Rw = np.vstack([_unit(ax[fref]), u, _unit(lt[fref])])
        wh_fl = 0.5 * (s[fref]["base_c"] - s_dorsal[fref])
        boxes.append((m, Rw, np.array([half[fref] + PA_WALL, wh_fl, 0.5 * PA_WALL])))
        rim_pts.append(m - wh_fl * u)                 # the wall's DORSAL edge = a rim node
    # DORSAL RIM rail only along the INTERNAL wall tops -- a rail to an OUTER wall would cross OVER
    # the end finger (measured: it blocked little's entry). The outer walls tie to the base instead.
    internal = rim_pts[1:-1]
    for a, b in zip(internal, internal[1:]):
        caps.append(((a, b), float(SKIN_R)))
    for end in (0, -1):                               # base spine runs only between wells; add the ends
        caps.append(((cc[fingers[end]] + s[fingers[end]]["base_c"] * fl[fingers[end]],
                      walls[end][1]), STALK_R))

    # STRUTS -- each finger's button node ties to the NEAREST internal rim node (a shared-wall top,
    # BESIDE the finger between it and a neighbour), so the load path never crosses the cup or sensor.
    for f in fingers:
        src = np.asarray(mounts[f], float)
        tip = min(internal, key=lambda p: float(np.linalg.norm(p - src)))
        caps.append(((src, tip), STALK_R))

    return dict(boxes=boxes, caps=caps, cyls=cyls,
                carve_cyls=carve_cyls, carve_boxes=carve_boxes)


def cluster_mesh(h, q, fingers, mounts, struts=(), radii=0.0, *, voxel=4e-4):
    """The long-finger cluster frame as a watertight trimesh (optionally with its landing struts)."""
    m = cluster_frame(h, q, fingers, mounts)
    caps = list(struts) + [c[0] for c in m["caps"]]
    rr = ([radii] * len(struts) if np.isscalar(radii) else list(radii)) + [c[1] for c in m["caps"]]
    f, o, v = mesh.field(caps, m["boxes"], r=rr, voxel=voxel, cyls=m["cyls"])
    mesh.carve(f, o, v, cyls=m["carve_cyls"], boxes=m["carve_boxes"])
    out = mesh.to_mesh(f, o, v)
    import trimesh
    bodies = out.split(only_watertight=False)          # drop sub-mm^3 marching-cubes debris shells
    if len(bodies) > 1:
        keep = [b for b in bodies if b.volume > 1e-9]
        out = trimesh.util.concatenate(keep) if len(keep) > 1 else keep[0]
    return out


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


def housing(anchor_nodes, outward, *, xiao=(0.021, 0.0178, 0.0035), lipo=(0.020, 0.012, 0.006),
            clear=0.003, wall=0.0015):
    """A wrist box for the XIAO nRF52840 + LiPo at the anchor cluster, sitting PROUD of the hand.

    ⚠ A world-axis box at the anchor centroid cuts INTO the wrist -- the anchors are only a few mm
    off the skin. So the box is oriented with its THIN axis along `outward` (the dorsal normal) and
    pushed out along it until its inner face clears the skin, and its component pockets open on the
    OUTER face (you drop the boards in from outside, away from the hand). Returns (boxes, carve_boxes).
    """
    A = np.asarray(anchor_nodes, float)
    C = A.mean(axis=0)
    z = np.asarray(outward, float)
    z = z / (np.linalg.norm(z) + 1e-12)
    x = np.cross(z, np.array([0.0, 0.0, 1.0]))
    if np.linalg.norm(x) < 1e-6:
        x = np.cross(z, np.array([0.0, 1.0, 0.0]))
    x = x / (np.linalg.norm(x) + 1e-12)
    y = np.cross(z, x)
    R = np.vstack([x, y, z])                          # rows = box axes; z (thin) = outward
    depth = max(xiao[2], lipo[2])
    half = np.array([0.5 * xiao[0], 0.5 * (xiao[1] + lipo[1]), 0.5 * depth]) + wall
    center = C + (clear + half[2]) * z                # lift the box off the skin along the normal
    boxes = [(center, R, half)]
    # NECKS back to the gauntlet -- capsules from the THREE nearest anchor NODES (real strut
    # endpoints, not the centroid, which floats in space) to the box, or the lifted box detaches.
    order = np.argsort(np.linalg.norm(A - center, axis=1))
    caps = [((A[int(i)], center), STALK_R) for i in order[:3]]
    cav = []                                          # two pockets side by side, opening the +z face
    for comp, sy in ((xiao, +1.0), (lipo, -1.0)):
        ch = 0.5 * np.asarray(comp, float)
        cc = center + sy * 0.25 * (xiao[1] + lipo[1]) * y + (half[2] - ch[2]) * z
        cav.append((cc, R, ch))
    return boxes, caps, cav

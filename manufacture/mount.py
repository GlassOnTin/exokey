"""THE SENSOR MOUNT — rebuilt ENTRY-FIRST, so the finger can actually get into its cup.

The prior mount (withdrawn, VISION §8.15l ppp) was checked only for the finger's static seated
clearance and kept blocking the ENTRY ROUTE. This one is built to the `manufacture.entry` constraint
by construction and validated against it:

    every piece is BESIDE the finger (a flank the phalanx slides between) or BELOW it (the palmar
    sensor stack) or on the DORSAL-LATERAL edge (the strut) -- NOTHING crosses the proximal slide-in.

Per finger the mount is, in the well's own frame (axis = distal, floor = palmar, lateral = across):
  * CUP    two lateral flanks + a palmar floor, OPEN proximally (the phalanx slides in) and dorsally.
  * SENSOR palmar of the floor: a Hall seat (carved PCB pocket) under the magnet gap. Below the finger.
  * STRUT  a post from the structure's button node to the cup's dorsal-lateral edge -- the nail side,
           opposite the palmar magnet, and clear of the entry.

The magnet + dome cradle is the drop-in TPU part (`manufacture.flexure` sizes the dome); this file is
the rigid PA frame the truss carries and the Hall sits in.
"""
from __future__ import annotations

import numpy as np

from design.params import MAGNET_D, MAGNET_L, REST_GAP, SVALBOARD
from manufacture import mesh
from manufacture.flexure import dome, spring_rate
from manufacture.friendly import SKIN_R
from structure.frame import MATERIALS

K = spring_rate(float(SVALBOARD.force), float(SVALBOARD.travel))
DOME_A = 0.006
DOME_T = float(dome(K, DOME_A, MATERIALS["tpu"]["E"], MATERIALS["tpu"]["nu"]))
CUP_WALL = 0.0022                          # flank thickness
FLOOR_T = 0.0022                           # cup-floor thickness
PA_WALL = 0.0016
BASE_T = 0.0018
PCB = (0.0064, 0.0064, 0.0018)             # Hall carrier (x along axis, y lateral, z palmar)
STRUT_R = 0.0025
MAGNET_POCKET_D = float(MAGNET_D) - 0.1e-3
MAGNET_POCKET_DEPTH = float(MAGNET_L) + 0.2e-3


def _frame(wf):
    ax = np.asarray(wf["axis"], float)
    fl = np.asarray(wf["floor"], float)
    lat = np.asarray(wf["lateral"], float)
    return ax, fl, lat, np.vstack([ax, fl, lat]), np.asarray(wf["pos"], float), wf["radius"], wf["half"]


def _stack(r):
    """Offsets along +floor (palmar) from the pad: cup floor, magnet face, Hall, base."""
    s_floor = r                                  # the finger bottoms on the cup floor here
    s_magface = s_floor + FLOOR_T                # magnet Hall-face just palmar of the floor
    s_hall = s_magface + float(REST_GAP)
    s_base = s_hall + 0.5 * PCB[2] + 0.5 * BASE_T
    return dict(floor=s_floor, magface=s_magface, hall=s_hall, base=s_base)


def well_mount(h, q, finger, mount_node, *, wire_len=0.010):
    """The rigid PA frame for one well, ENTRY-FIRST. Returns the mesh-primitive dict."""
    ax, fl, lat, R, pos, r, half = _frame(h.well_frame(q, finger))
    s = _stack(r)
    cc = pos - 0.5 * half * ax                   # channel centre (behind the pad)
    boxes, caps, cyls, carve_cyls, carve_boxes = [], [], [], [], []

    # CUP: two lateral flanks (BESIDE the finger -- guides, not blocks) + a palmar floor. OPEN
    # proximally (finger slides in) and dorsally (nail). No distal end wall over the entry.
    for side in (+1.0, -1.0):
        boxes.append((cc + 0.5 * s["floor"] * fl + side * (r + 0.5 * CUP_WALL) * lat, R,
                      np.array([half, 0.5 * s["floor"] + FLOOR_T, 0.5 * CUP_WALL])))
    boxes.append((cc + (s["floor"] + 0.5 * FLOOR_T) * fl, R,
                  np.array([half, 0.5 * FLOOR_T, r + CUP_WALL])))          # floor (palmar, below finger)

    # SENSOR TAIL: a Hall seat palmar of the floor, PCB-width, below the finger and clear of the entry.
    pcb_half = 0.5 * PCB[1] + PA_WALL
    boxes.append((cc + s["base"] * fl, R, np.array([half, 0.5 * BASE_T, pcb_half])))
    for side in (+1.0, -1.0):                    # two necks BESIDE the PCB pocket, or the carve severs it
        caps.append(((cc + (s["floor"] + FLOOR_T) * fl + side * pcb_half * lat,
                      cc + s["base"] * fl + side * pcb_half * lat), 0.5 * PA_WALL + 0.0006))
    carve_boxes.append((cc + (s["hall"] + 0.5 * PCB[2]) * fl, R,
                        np.array([0.5 * PCB[0], 0.5 * PCB[2], 0.5 * PCB[1]])))               # PCB pocket
    slot = cc + s["hall"] * fl - (0.5 * half + 0.5 * wire_len) * ax
    carve_boxes.append((slot, R, np.array([0.5 * wire_len + half, 0.0006, 0.0006])))         # wire slot

    # STRUT: from the structure's button node to the cup's DORSAL-LATERAL edge (nail side, opposite
    # the palmar magnet), never across the proximal entry.
    edge = cc + (-0.15 * r) * fl + (r + 0.5 * CUP_WALL) * lat        # a dorsal-lateral flank-top point
    caps.append(((np.asarray(mount_node, float), edge), STRUT_R))
    caps.append(((edge, cc + (-0.15 * r) * fl - (r + 0.5 * CUP_WALL) * lat), float(SKIN_R)))  # dorsal rim

    return dict(boxes=boxes, caps=caps, cyls=cyls, carve_cyls=carve_cyls, carve_boxes=carve_boxes)


def _mesh(prims, struts=(), radii=0.0, voxel=4e-4):
    caps = list(struts) + [c[0] for c in prims["caps"]]
    rr = ([radii] * len(struts) if np.isscalar(radii) else list(radii)) + [c[1] for c in prims["caps"]]
    f, o, v = mesh.field(caps, prims["boxes"], r=(rr or 0.0), voxel=voxel, cyls=prims["cyls"])
    mesh.carve(f, o, v, cyls=prims["carve_cyls"], boxes=prims["carve_boxes"])
    out = mesh.to_mesh(f, o, v)
    import trimesh
    bodies = out.split(only_watertight=False)
    if len(bodies) > 1:
        keep = [b for b in bodies if b.volume > 1e-9]
        out = trimesh.util.concatenate(keep) if len(keep) > 1 else keep[0]
    return out


def well_mesh(h, q, finger, mount_node, struts=(), radii=0.0, *, voxel=4e-4):
    return _mesh(well_mount(h, q, finger, mount_node), struts, radii, voxel)


def cluster_mount(h, q, fingers, mount_nodes, *, wire_len=0.010):
    """ONE carrier for a ROW of wells (the long fingers), SHARED flanks, ENTRY-FIRST.

    The flanks between fingers are SHARED and run ALONG the axis (they guide the phalanx in, they do
    not cross the proximal slide-in), a palmar base spine links the Hall seats, and each strut ties
    the button node to a dorsal-lateral flank top. `fingers` in row order; `mount_nodes` = {f: node}.
    """
    wf = {f: h.well_frame(q, f) for f in fingers}
    fr = {f: _frame(wf[f]) for f in fingers}         # (ax, fl, lat, R, pos, r, half)
    s = {f: _stack(fr[f][5]) for f in fingers}
    cc = {f: fr[f][4] - 0.5 * fr[f][6] * fr[f][0] for f in fingers}
    pcb_half = 0.5 * PCB[1] + PA_WALL
    boxes, caps, cyls, carve_cyls, carve_boxes = [], [], [], [], []

    for f in fingers:
        ax, fl, lat, R, pos, r, half = fr[f]
        boxes.append((cc[f] + (s[f]["floor"] + 0.5 * FLOOR_T) * fl, R,
                      np.array([half, 0.5 * FLOOR_T, r + CUP_WALL])))                       # cup floor
        boxes.append((cc[f] + s[f]["base"] * fl, R, np.array([half, 0.5 * BASE_T, pcb_half])))  # Hall seat
        for side in (+1.0, -1.0):
            caps.append(((cc[f] + (s[f]["floor"] + FLOOR_T) * fl + side * pcb_half * lat,
                          cc[f] + s[f]["base"] * fl + side * pcb_half * lat), 0.5 * PA_WALL + 0.0006))
        carve_boxes.append((cc[f] + (s[f]["hall"] + 0.5 * PCB[2]) * fl, R,
                            np.array([0.5 * PCB[0], 0.5 * PCB[2], 0.5 * PCB[1]])))           # PCB pocket
        slot = cc[f] + s[f]["hall"] * fl - (0.5 * half + 0.5 * wire_len) * ax
        carve_boxes.append((slot, R, np.array([0.5 * wire_len + half, 0.0006, 0.0006])))    # wire slot
        edge = cc[f] + (-0.15 * r) * fl + (r + 0.5 * CUP_WALL) * lat
        caps.append(((np.asarray(mount_nodes[f], float), edge), STRUT_R))                   # strut

    # BASE SPINE (palmar) linking the Hall seats.
    for a, b in zip(fingers, fingers[1:]):
        caps.append(((cc[a] + s[a]["base"] * fl, cc[b] + s[b]["base"] * fl), 0.5 * BASE_T))

    # SHARED FLANKS -- guide walls BESIDE the fingers, along the axis (open proximally). One between
    # each adjacent pair + one outboard of each end. Each spans the cup height and ties to both floors.
    def cupc(f):                                     # cup centre at floor height
        return cc[f] + 0.5 * s[f]["floor"] * fl
    fl0 = fr[fingers[0]][1]
    flanks = [(fingers[0], cupc(fingers[0]) - (fr[fingers[0]][5] + CUP_WALL + 0.0015) * fr[fingers[0]][2])]
    flanks += [(a, 0.5 * (cupc(a) + cupc(b))) for a, b in zip(fingers, fingers[1:])]
    flanks.append((fingers[-1],
                   cupc(fingers[-1]) + (fr[fingers[-1]][5] + CUP_WALL + 0.0015) * fr[fingers[-1]][2]))
    for fref, m in flanks:
        ax, fl, lat, R, pos, r, half = fr[fref]
        Rw = np.vstack([ax, fl, lat])
        boxes.append((m, Rw, np.array([half, 0.5 * s[fref]["floor"] + FLOOR_T, 0.5 * CUP_WALL])))
        # tie the flank down to the base spine so it is one piece
        caps.append(((m, cc[fref] + s[fref]["base"] * fl), 0.5 * PA_WALL + 0.0006))

    return dict(boxes=boxes, caps=caps, cyls=cyls, carve_cyls=carve_cyls, carve_boxes=carve_boxes)


def cluster_mesh(h, q, fingers, mount_nodes, struts=(), radii=0.0, *, voxel=4e-4):
    return _mesh(cluster_mount(h, q, fingers, mount_nodes), struts, radii, voxel)


SKIRT_LEN = 0.003


def well_insert(h, q, finger, *, nail_hood=True):
    """The DROP-IN TPU cradle for one well -- the moving part the fingertip sits in and presses. It
    carries the magnet on the §8.15g dome; the finger enters ITS cup, so it gets the same entry check
    as the frame. Cup open proximally (flanks beside, floor below); dome + magnet pocket palmar."""
    ax, fl, lat, R, pos, r, half = _frame(h.well_frame(q, finger))
    s = _stack(r)
    cc = pos - 0.5 * half * ax
    boxes, caps, cyls, carve_cyls = [], [], [], []

    # CUP (TPU): flanks beside the finger + floor below, OPEN proximally and dorsally.
    for side in (+1.0, -1.0):
        boxes.append((cc + 0.5 * s["floor"] * fl + side * (r + 0.5 * CUP_WALL) * lat, R,
                      np.array([half, 0.5 * s["floor"] + FLOOR_T, 0.5 * CUP_WALL])))
    boxes.append((cc + (s["floor"] + 0.5 * FLOOR_T) * fl, R,
                  np.array([half, 0.5 * FLOOR_T, r + CUP_WALL])))
    if nail_hood:                                    # a distal-dorsal lip so lift/contort transmit
        boxes.append((cc + half * ax + (-0.1 * r) * fl, R, np.array([0.5 * CUP_WALL, 0.3 * r, r])))

    # DOME flexure + SKIRT (palmar, below the floor -- out of the entry path).
    dc = cc + (s["floor"] + FLOOR_T) * fl
    cyls.append((dc, dc + DOME_T * fl, DOME_A))                       # the dome membrane
    a_hi = cc + (s["floor"] + FLOOR_T + SKIRT_LEN) * fl
    cyls.append((dc, a_hi, DOME_A + 0.0012))                         # skirt outer
    carve_cyls.append((cc + (s["floor"] + FLOOR_T + DOME_T) * fl, a_hi + 0.001 * fl, DOME_A))  # bore
    mp = cc + (s["floor"] + FLOOR_T + DOME_T) * fl                    # magnet pocket, opening palmar
    carve_cyls.append((mp, mp - (DOME_T + MAGNET_POCKET_DEPTH) * fl, 0.5 * MAGNET_POCKET_D))

    return dict(boxes=boxes, caps=caps, cyls=cyls, carve_cyls=carve_cyls, carve_boxes=[])


def insert_mesh(h, q, finger, *, voxel=3e-4, nail_hood=True):
    return _mesh(well_insert(h, q, finger, nail_hood=nail_hood), voxel=voxel)


def _unit(v):
    v = np.asarray(v, float)
    return v / (np.linalg.norm(v) + 1e-12)


def housing(anchor_nodes, outward, live_nodes, *, xiao=(0.021, 0.0178, 0.0035),
            lipo=(0.020, 0.012, 0.006), clear=0.003, wall=0.0015):
    """The wrist XIAO nRF52840 + LiPo box, sitting PROUD of the wrist (thin axis along the skin
    normal, lifted off), necked to the nearest LIVE-strut nodes so it cannot detach. Far from the
    fingertips -- it does not touch the finger-entry route -- but it must clear the wrist and attach.
    Returns (boxes, caps, carve_boxes)."""
    A = np.asarray(anchor_nodes, float)
    C = A.mean(axis=0)
    z = _unit(outward)
    x = np.cross(z, np.array([0.0, 0.0, 1.0]))
    if np.linalg.norm(x) < 1e-6:
        x = np.cross(z, np.array([0.0, 1.0, 0.0]))
    x = _unit(x)
    y = np.cross(z, x)
    R = np.vstack([x, y, z])
    half = np.array([0.5 * xiao[0], 0.5 * (xiao[1] + lipo[1]), 0.5 * max(xiao[2], lipo[2])]) + wall
    center = C + (clear + half[2]) * z
    boxes = [(center, R, half)]
    L = np.asarray(live_nodes, float)
    near = L[np.argsort(np.linalg.norm(L - center, axis=1))[:3]]
    caps = [((n, center), STRUT_R) for n in near]
    cav = []
    for comp, sy in ((xiao, +1.0), (lipo, -1.0)):
        ch = 0.5 * np.asarray(comp, float)
        cav.append((center + sy * 0.25 * (xiao[1] + lipo[1]) * y + (half[2] - ch[2]) * z, R, ch))
    return boxes, caps, cav


def harness_routes(nodes, bars, live, btn, anchors):
    """Wire routes: the shortest path over live struts from each sensor (button) to its nearest wrist
    anchor. Returns [list of node indices]. The caller sinks each into a re-entrant surface groove."""
    from scipy.sparse import csr_matrix
    from scipy.sparse.csgraph import dijkstra

    X = np.asarray(nodes, float)
    rows, cols, w = [], [], []
    for e in live:
        i, j = bars[e]
        d = float(np.linalg.norm(X[i] - X[j]))
        rows += [i, j]
        cols += [j, i]
        w += [d, d]
    G = csr_matrix((w, (rows, cols)), shape=(len(X), len(X)))
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

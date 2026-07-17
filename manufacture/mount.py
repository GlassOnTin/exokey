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
from hand.flesh import skin
from manufacture import mesh
from manufacture.flexure import dome, spring_rate
from manufacture.friendly import SKIN_R
from structure.frame import MATERIALS

K = spring_rate(float(SVALBOARD.force), float(SVALBOARD.travel))
DOME_A = 0.006
DOME_T = float(dome(K, DOME_A, MATERIALS["tpu"]["E"], MATERIALS["tpu"]["nu"]))
CUP_WALL = 0.0022                          # flank thickness
FLOOR_T = 0.0022                           # cup-floor thickness
SEAT_CLEAR = 0.0004                        # gap between flesh and cup (so the finger slides in, not bites)
PA_WALL = 0.0016
BASE_T = 0.0018
PCB = (0.0064, 0.0064, 0.0018)             # Hall carrier (x along axis, y lateral, z palmar)
STRUT_R = 0.0025
MAGNET_POCKET_D = float(MAGNET_D) - 0.1e-3
MAGNET_POCKET_DEPTH = float(MAGNET_L) + 0.2e-3


def _seat(h, q, finger, cc, fl, lat):
    """WHERE THE FINGER ACTUALLY IS, measured -- the distal-phalanx skin's floor-direction extent in
    the well frame, relative to cc: (v_pad, v_nail, w_half) = palmar-most, dorsal-most, lateral
    half-width (m). `well_frame["pos"]` is the pad SURFACE, not the pulp centre, so the cup must be
    built to the measured skin, not to a `pos + r` guess -- else the finger floats above its cup."""
    V, _F, L = skin(h, q, labels=True)
    bid = h.pad[finger][0]
    tip = np.asarray(V)[np.asarray(L) == bid]
    if len(tip) == 0:                                    # unlabelled: fall back to a nominal seat
        return 0.0, -2.0 * float(wf_radius(h, q, finger)), float(wf_radius(h, q, finger))
    rel = tip - cc
    v, w = rel @ fl, rel @ lat
    return float(v.max()), float(v.min()), float(np.abs(w).max())


def wf_radius(h, q, finger):
    return h.well_frame(q, finger)["radius"]


def _frame(h, q, finger):
    """The well frame PLUS the measured seat. Returns
    (ax, fl, lat, R, pos, cc, r, half, v_pad, v_nail, w_half)."""
    wf = h.well_frame(q, finger)
    ax = np.asarray(wf["axis"], float)
    fl = np.asarray(wf["floor"], float)
    lat = np.asarray(wf["lateral"], float)
    pos = np.asarray(wf["pos"], float)
    r, half = float(wf["radius"]), float(wf["half"])
    cc = pos - 0.5 * half * ax
    v_pad, v_nail, w_half = _seat(h, q, finger, cc, fl, lat)
    return ax, fl, lat, np.vstack([ax, fl, lat]), pos, cc, r, half, v_pad, v_nail, w_half


def _stack(v_pad):
    """Offsets along +floor (palmar) from cc: the pad/floor plane (the MEASURED pad depth), then the
    magnet Hall-face, the Hall, and the carrier base -- each palmar of the pad, under the finger."""
    s_floor = v_pad                              # the pad rests here; the sensor sits palmar of it
    s_magface = s_floor + FLOOR_T                # magnet Hall-face just palmar of the floor plate
    s_hall = s_magface + float(REST_GAP)
    s_base = s_hall + 0.5 * PCB[2] + 0.5 * BASE_T
    return dict(floor=s_floor, magface=s_magface, hall=s_hall, base=s_base)


def _cup(cc, fl, lat, R, half, vf, vd, w_half):
    """Two lateral flanks BESIDE the finger (spanning the dorsal opening vd .. floor vf+FLOOR_T) + a
    palmar floor plate under the pad at vf. OPEN proximally and dorsally. All offset from the flesh by
    SEAT_CLEAR so the finger slides in."""
    lw = w_half + SEAT_CLEAR + 0.5 * CUP_WALL
    mv = 0.5 * (vf + FLOOR_T + vd)               # channel centre
    hv = 0.5 * (vf + FLOOR_T - vd)               # channel half-height (pad -> nail)
    boxes = [(cc + mv * fl + side * lw * lat, R, np.array([half, hv, 0.5 * CUP_WALL]))
             for side in (+1.0, -1.0)]
    boxes.append((cc + (vf + 0.5 * FLOOR_T) * fl, R, np.array([half, 0.5 * FLOOR_T, w_half + CUP_WALL])))
    return boxes


def well_mount(h, q, finger, mount_node, *, wire_len=0.010):
    """The rigid PA frame for one well, ENTRY-FIRST and SEATED-TO-THE-MEASURED-FINGER. Returns the
    mesh-primitive dict."""
    ax, fl, lat, R, pos, cc, r, half, v_pad, v_nail, w_half = _frame(h, q, finger)
    vf, vd, lw = v_pad + SEAT_CLEAR, v_nail - SEAT_CLEAR, w_half + SEAT_CLEAR + 0.5 * CUP_WALL
    s = _stack(vf)
    boxes, caps, cyls, carve_cyls, carve_boxes = [], [], [], [], []
    boxes += _cup(cc, fl, lat, R, half, vf, vd, w_half)          # flanks BESIDE the finger + palmar floor

    # SENSOR TAIL: a Hall seat palmar of the pad, PCB-width, below the finger and clear of the entry.
    pcb_half = 0.5 * PCB[1] + PA_WALL
    boxes.append((cc + s["base"] * fl, R, np.array([half, 0.5 * BASE_T, pcb_half])))
    for side in (+1.0, -1.0):                    # two necks BESIDE the PCB pocket, or the carve severs it
        caps.append(((cc + s["magface"] * fl + side * pcb_half * lat,
                      cc + s["base"] * fl + side * pcb_half * lat), 0.5 * PA_WALL + 0.0006))
    carve_boxes.append((cc + (s["hall"] + 0.5 * PCB[2]) * fl, R,
                        np.array([0.5 * PCB[0], 0.5 * PCB[2], 0.5 * PCB[1]])))               # PCB pocket
    slot = cc + s["hall"] * fl - (0.5 * half + 0.5 * wire_len) * ax
    carve_boxes.append((slot, R, np.array([0.5 * wire_len + half, 0.0006, 0.0006])))         # wire slot

    # STRUT: the button node is PALMAR (it IS the sensor location -- the pad presses down onto it), so
    # tie it to the palmar Hall-seat base. A strut to a dorsal edge would cross straight THROUGH the
    # finger; both node and base sit ~8 mm palmar of the pad, clear of the entry.
    caps.append(((np.asarray(mount_node, float), cc + s["base"] * fl), STRUT_R))

    return dict(boxes=boxes, caps=caps, cyls=cyls, carve_cyls=carve_cyls, carve_boxes=carve_boxes)


def _mesh(prims, struts=(), radii=0.0, voxel=4e-4):
    caps = list(struts) + [c[0] for c in prims["caps"]]
    rr = ([radii] * len(struts) if np.isscalar(radii) else list(radii)) + [c[1] for c in prims["caps"]]
    f, o, v = mesh.field(caps, prims["boxes"], r=(rr or 0.0), voxel=voxel, cyls=prims["cyls"])
    mesh.carve(f, o, v, cyls=prims["carve_cyls"], boxes=prims["carve_boxes"])
    out = mesh.to_mesh(f, o, v)
    import trimesh
    bodies = out.split(only_watertight=False)
    if len(bodies) > 1:                              # drop sub-2 mm^3 marching-cubes / carve debris shells
        keep = [b for b in bodies if b.volume > 2e-9]
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
    fr = {f: _frame(h, q, f) for f in fingers}       # (ax,fl,lat,R,pos,cc,r,half,v_pad,v_nail,w_half)
    cc = {f: fr[f][5] for f in fingers}
    vf = {f: fr[f][8] + SEAT_CLEAR for f in fingers}  # floor plane (v_pad + clear), per finger
    vd = {f: fr[f][9] - SEAT_CLEAR for f in fingers}  # dorsal opening (v_nail - clear), per finger
    s = {f: _stack(vf[f]) for f in fingers}
    pcb_half = 0.5 * PCB[1] + PA_WALL
    boxes, caps, cyls, carve_cyls, carve_boxes = [], [], [], [], []

    for f in fingers:
        ax, fl, lat, R, pos, ccf, r, half, v_pad, v_nail, w_half = fr[f]
        boxes.append((ccf + (vf[f] + 0.5 * FLOOR_T) * fl, R,
                      np.array([half, 0.5 * FLOOR_T, w_half + CUP_WALL])))                  # cup floor
        boxes.append((ccf + s[f]["base"] * fl, R, np.array([half, 0.5 * BASE_T, pcb_half])))  # Hall seat
        for side in (+1.0, -1.0):
            caps.append(((ccf + s[f]["magface"] * fl + side * pcb_half * lat,
                          ccf + s[f]["base"] * fl + side * pcb_half * lat), 0.5 * PA_WALL + 0.0006))
        carve_boxes.append((ccf + (s[f]["hall"] + 0.5 * PCB[2]) * fl, R,
                            np.array([0.5 * PCB[0], 0.5 * PCB[2], 0.5 * PCB[1]])))           # PCB pocket
        slot = ccf + s[f]["hall"] * fl - (0.5 * half + 0.5 * wire_len) * ax
        carve_boxes.append((slot, R, np.array([0.5 * wire_len + half, 0.0006, 0.0006])))    # wire slot
        caps.append(((np.asarray(mount_nodes[f], float), ccf + s[f]["base"] * fl), STRUT_R))  # to palmar base

    # BASE SPINE (palmar) linking the Hall seats.
    for a, b in zip(fingers, fingers[1:]):
        caps.append(((cc[a] + s[a]["base"] * fr[a][1], cc[b] + s[b]["base"] * fr[b][1]), 0.5 * BASE_T))

    # SHARED FLANKS -- guide walls BESIDE the fingers, spanning each finger's channel, centred between
    # adjacent fingers (open proximally). One between each adjacent pair + one outboard of each end.
    def mvp(f):                                      # channel-centre point (floor direction)
        _ax, _fl, _lat, _R, _pos, _cc, _r, _half, _vp, _vn, _wh = fr[f]
        return _cc + 0.5 * (vf[f] + FLOOR_T + vd[f]) * _fl
    off = lambda f: fr[f][10] + SEAT_CLEAR + CUP_WALL + 0.0015                # outboard flank offset
    f0, fN = fingers[0], fingers[-1]
    flanks = [(f0, mvp(f0) - off(f0) * fr[f0][2])]                            # fr[f][2]=lat, [10]=w_half
    flanks += [(a, 0.5 * (mvp(a) + mvp(b))) for a, b in zip(fingers, fingers[1:])]
    flanks.append((fN, mvp(fN) + off(fN) * fr[fN][2]))
    for fref, m in flanks:
        ax, fl, lat, R, pos, ccf, r, half, v_pad, v_nail, w_half = fr[fref]
        hv = 0.5 * (vf[fref] + FLOOR_T - vd[fref])
        boxes.append((m, R, np.array([half, hv, 0.5 * CUP_WALL])))
        caps.append(((m, ccf + s[fref]["base"] * fl), 0.5 * PA_WALL + 0.0006))   # tie flank to base spine

    return dict(boxes=boxes, caps=caps, cyls=cyls, carve_cyls=carve_cyls, carve_boxes=carve_boxes)


def cluster_mesh(h, q, fingers, mount_nodes, struts=(), radii=0.0, *, voxel=4e-4):
    return _mesh(cluster_mount(h, q, fingers, mount_nodes), struts, radii, voxel)


SKIRT_LEN = 0.003


def well_insert(h, q, finger, *, nail_hood=True):
    """The DROP-IN TPU cradle for one well -- the moving part the fingertip sits in and presses. It
    carries the magnet on the §8.15g dome; the finger enters ITS cup, so it gets the same entry check
    as the frame. Cup open proximally (flanks beside, floor below); dome + magnet pocket palmar."""
    ax, fl, lat, R, pos, cc, r, half, v_pad, v_nail, w_half = _frame(h, q, finger)
    vf, vd = v_pad + SEAT_CLEAR, v_nail - SEAT_CLEAR
    boxes, caps, cyls, carve_cyls = [], [], [], []
    boxes += _cup(cc, fl, lat, R, half, vf, vd, w_half)   # flanks beside the finger + floor under the pad
    if nail_hood:                                    # a distal-dorsal lip OVER the nail so lift/contort transmit
        hh = 0.15 * (vf - vd)
        boxes.append((cc + half * ax + (vd - hh) * fl, R, np.array([0.5 * CUP_WALL, hh, w_half])))

    # DOME flexure + SKIRT (palmar, below the floor -- out of the entry path).
    dc = cc + (vf + FLOOR_T) * fl
    cyls.append((dc, dc + DOME_T * fl, DOME_A))                       # the dome membrane
    a_hi = cc + (vf + FLOOR_T + SKIRT_LEN) * fl
    cyls.append((dc, a_hi, DOME_A + 0.0012))                         # skirt outer
    carve_cyls.append((cc + (vf + FLOOR_T + DOME_T) * fl, a_hi + 0.001 * fl, DOME_A))  # bore
    mp = cc + (vf + FLOOR_T + DOME_T) * fl                            # magnet pocket, opening palmar
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

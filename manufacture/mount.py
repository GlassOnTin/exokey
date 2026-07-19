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
            lipo=(0.020, 0.010, 0.004), mux=(0.0114, 0.0086, 0.0016),
            clear=0.003, wall=0.0015, wire_slot=0.002):
    # lipo default = a 401020 cell (4.0 x 10 x 20 mm). Thinner than the old 6 mm cell, so the box
    # stands less proud of the wrist; rated 200 mAh (optimistic for 0.8 cc -- treat as ~100-150 mAh).
    """The wrist box: XIAO nRF52840 + LiPo + the I2C mux breakout (TCA9548A-class), laid side by side,
    sitting PROUD of the wrist (thin axis along the skin normal, lifted off) and necked to the nearest
    LIVE-strut nodes so it cannot detach. A wire-entry slot at the mux end admits the sensor harness
    braid (5 sensors x SDA/SCL + shared VDD/GND -- a ~34 AWG braid) so it drops into the mux bay and the
    caller's groove carries it along the struts. Far from the fingertips -- it does not touch the
    finger-entry route. The mux rides in the DEAD SPACE beside the parts (it is thinner than the LiPo),
    so it costs only ~its own width in y. Returns (boxes, caps, carve_boxes)."""
    A = np.asarray(anchor_nodes, float)
    C = A.mean(axis=0)
    z = _unit(outward)
    x = np.cross(z, np.array([0.0, 0.0, 1.0]))
    if np.linalg.norm(x) < 1e-6:
        x = np.cross(z, np.array([0.0, 1.0, 0.0]))
    x = _unit(x)
    y = np.cross(z, x)
    R = np.vstack([x, y, z])
    comps = [np.asarray(c, float) for c in (xiao, lipo, mux)]        # a row along y: XIAO | LiPo | mux
    tot_y = float(sum(c[1] for c in comps))
    half = np.array([0.5 * max(c[0] for c in comps), 0.5 * tot_y,
                     0.5 * max(c[2] for c in comps)]) + wall
    center = C + (clear + half[2]) * z
    boxes = [(center, R, half)]
    L = np.asarray(live_nodes, float)
    near = L[np.argsort(np.linalg.norm(L - center, axis=1))[:3]]
    caps = [((n, center), STRUT_R) for n in near]
    cav, yo = [], -0.5 * tot_y                                       # carve each part to the outer face
    for c in comps:
        ch = 0.5 * c
        cav.append((center + (yo + ch[1]) * y + (half[2] - ch[2]) * z, R, ch))
        yo += c[1]
    # WIRE ENTRY: a slot through the +y wall at the mux bay, open to the outer face, for the harness braid.
    mch = 0.5 * comps[-1]
    sh = np.array([wire_slot, wall + wire_slot, mch[2]])
    cav.append((center + (half[1] - 0.5 * wall) * y + (half[2] - sh[2]) * z, R, sh))
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


def _steiner_exact(adj, sssp, N, terminals):
    """EXACT minimum Steiner tree in a graph (Dreyfus-Wagner) over `terminals`. `adj[u]` = [(v, w)];
    `sssp[t]` = (dist, pred) from a single-source Dijkstra at terminal t (cached, shared across calls).
    Returns the physical edge set (frozensets, the virtual node N dropped). For the handful of terminals
    here this is cheap and OPTIMAL, where the metric MST is only a 2-approximation."""
    import heapq

    k = len(terminals); V = N + 1
    dp = {1 << i: sssp[terminals[i]][0] for i in range(k)}          # dp[mask][v]: tree spanning mask + v
    merge_val, split, pred_r = {}, {}, {}
    for size in range(2, k + 1):
        for mask in (m for m in range(1, 1 << k) if bin(m).count("1") == size):
            cur = np.full(V, np.inf); sp = np.full(V, -1, dtype=np.int64)
            sub = (mask - 1) & mask
            while sub:                                              # two subtrees meeting at each vertex
                other = mask ^ sub
                if other:
                    cand = dp[sub] + dp[other]; b = cand < cur
                    cur[b] = cand[b]; sp[b] = sub
                sub = (sub - 1) & mask
            merge_val[mask] = cur.copy()
            dist = cur.copy(); pr = np.full(V, -1, dtype=np.int64)  # then grow along graph edges
            pq = [(float(dist[v]), int(v)) for v in np.where(np.isfinite(dist))[0]]
            heapq.heapify(pq)
            while pq:
                d, u = heapq.heappop(pq)
                if d > dist[u]:
                    continue
                for v, wt in adj[u]:
                    nd = d + wt
                    if nd < dist[v]:
                        dist[v] = nd; pr[v] = u; heapq.heappush(pq, (nd, v))
            dp[mask], split[mask], pred_r[mask] = dist, sp, pr

    full = (1 << k) - 1
    edges, stack = set(), [(full, int(np.argmin(dp[full])))]
    while stack:                                                   # backtrack the DP into an edge set
        mask, v = stack.pop()
        if bin(mask).count("1") == 1:                              # a single terminal: its path to v
            pred = sssp[terminals[mask.bit_length() - 1]][1]; x = v
            while pred[x] >= 0:
                p = int(pred[x])
                if x < N and p < N:
                    edges.add(frozenset((x, p)))
                x = p
        elif pred_r[mask][v] >= 0 and dp[mask][v] < merge_val[mask][v] - 1e-12:
            u = int(pred_r[mask][v])                               # reached v along a graph edge
            if v < N and u < N:
                edges.add(frozenset((v, u)))
            stack.append((mask, u))
        else:                                                      # v is where two subtrees merged
            sub = int(split[mask][v])
            stack.append((sub, v)); stack.append((mask ^ sub, v))
    return edges


def harness_bus(nodes, bars, live, btn, anchors, *, max_per_bus=4):
    """MINIMAL-COPPER harness: a SHARED bus over the strut graph, not five point-to-point runs
    (VISION §8.15l qqq-2). The sensors are I2C, so VDD/GND are shared by all and SDA/SCL are a bus:

      * POWER (VDD/GND, 2 conductors) -- one Steiner tree over ALL sensors + the wrist MCU.
      * SIGNAL (SDA/SCL, 2 conductors per I2C bus) -- one tree per bus, over that bus's sensors + MCU,
        the sensors split into <= `max_per_bus` groups (the W2BW address limit) to minimise total length.

    Each tree is the EXACT minimum Steiner-tree-in-a-graph over the live struts (`_steiner_exact`,
    Dreyfus-Wagner), not a metric-MST approximation. Returns `[(i, j, n_wires)]`: the groove segments
    (live-strut node pairs) and how many conductors share each -- 2 where only power runs, up to 6 on
    the trunk where power and both signal buses overlap. The caller sinks each into a groove whose width
    follows `n_wires`. This is the disclosed replacement for the per-sensor `harness_routes`."""
    import itertools

    from scipy.sparse import csr_matrix
    from scipy.sparse.csgraph import dijkstra, minimum_spanning_tree

    X = np.asarray(nodes, float)
    S = list(btn.values())
    N = len(X); MCU = N                              # a virtual MCU node tied to every wrist anchor at ~0
    rows, cols, w = [], [], []
    adj = [[] for _ in range(N + 1)]

    def _add(i, j, d):
        rows.append(i); cols.append(j); w.append(d); adj[i].append((j, d))

    for e in live:
        i, j = bars[e]; d = float(np.linalg.norm(X[i] - X[j])); _add(i, j, d); _add(j, i, d)
    for a in anchors:
        _add(int(a), MCU, 1e-6); _add(MCU, int(a), 1e-6)
    G = csr_matrix((w, (rows, cols)), shape=(N + 1, N + 1))

    sssp = {t: dijkstra(G, indices=t, return_predecessors=True) for t in S + [MCU]}   # cache once
    tree = lambda terms: _steiner_exact(adj, sssp, N, terms)        # exact Steiner tree -> edge set

    def mst_w(terms):                               # cheap metric-MST weight, only to pick the bus split
        D = np.array([[sssp[a][0][b] for b in terms] for a in terms])
        return float(minimum_spanning_tree(D).toarray().sum())

    power = tree(S + [MCU])
    best = None                                     # split sensors into two I2C buses (cheap MST proxy)
    for r in range(1, len(S)):
        for A in itertools.combinations(range(len(S)), r):
            if r > max_per_bus or len(S) - r > max_per_bus:
                continue
            gA = [S[i] for i in A]; gB = [S[i] for i in range(len(S)) if i not in A]
            c = mst_w(gA + [MCU]) + mst_w(gB + [MCU])
            if best is None or c < best[0]:
                best = (c, gA, gB)
    sig = [tree(best[1] + [MCU]), tree(best[2] + [MCU])] if best else [power]

    seg = {fe: 2 for fe in power}                   # 2 power conductors everywhere the power tree runs
    for st in sig:
        for fe in st:
            seg[fe] = seg.get(fe, 0) + 2            # + 2 signal conductors per bus sharing the segment
    return [(*sorted(tuple(fe)), nw) for fe, nw in seg.items() if max(fe) < N]

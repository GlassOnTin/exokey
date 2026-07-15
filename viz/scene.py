"""Browser visualisation. One Plotly scene builder, reused by every stage.

Visualisation is a first-class requirement, not a reporting afterthought: connecting
hand intuition to the numbers is how you catch a wrong pad normal or a mirrored finger
before it silently poisons an 8-hour optimisation. Every stage writes a self-contained
.html -- no server, no build step, just open it.

A live server arrives at Stage 4, where watching NSGA-II converge earns the infra.
"""
from __future__ import annotations

import numpy as np
import plotly.graph_objects as go

from hand.myohand import FINGERS, MyoHand

FINGER_COLOR = {
    "thumb": "#e45756",
    "index": "#4c78a8",
    "middle": "#54a24b",
    "ring": "#f58518",
    "little": "#b279a2",
}


HAND_ROOT = "lunate"  # wrist; everything distal of this is the hand


def _hand_bodies(h: MyoHand) -> set[int]:
    """Bodies from the wrist down.

    myohand.xml ships a whole scene -- a room, walls, furniture and a full human figure
    (mesh geoms on `world` and `full_body`). Rendering every mesh geom draws all of it
    and the hand becomes a speck. Restrict to the descendants of the wrist.
    """
    import mujoco

    m = h.model
    root = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, HAND_ROOT)
    keep = {root}
    for b in range(root + 1, m.nbody):  # children always follow parents in MuJoCo
        if m.body_parentid[b] in keep:
            keep.add(b)
    return keep


def _mesh_traces(h: MyoHand, q: np.ndarray, opacity: float = 0.35) -> list[go.Mesh3d]:
    """Bone meshes, posed by MuJoCo FK and baked into world coords."""
    import mujoco

    m, d = h.model, h.data
    h.fk(q)
    keep = _hand_bodies(h)
    traces = []
    for g in range(m.ngeom):
        if m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        if m.geom_bodyid[g] not in keep:
            continue  # scenery, not anatomy
        did = m.geom_dataid[g]
        va, nv = m.mesh_vertadr[did], m.mesh_vertnum[did]
        fa, nf = m.mesh_faceadr[did], m.mesh_facenum[did]
        V = m.mesh_vert[va : va + nv].reshape(-1, 3)
        F = m.mesh_face[fa : fa + nf].reshape(-1, 3)
        R = d.geom_xmat[g].reshape(3, 3)
        W = V @ R.T + d.geom_xpos[g]  # to world
        traces.append(
            go.Mesh3d(
                x=W[:, 0], y=W[:, 1], z=W[:, 2],
                i=F[:, 0], j=F[:, 1], k=F[:, 2],
                color="#c8ccd4", opacity=opacity, hoverinfo="skip",
                lighting=dict(ambient=0.55, diffuse=0.8), showlegend=False, name="bone",
            )
        )
    return traces


def _pad_traces(h: MyoHand, q: np.ndarray, normal_len: float = 0.012) -> list:
    """Finger pads and their outward normals -- the contact frame the whole model rests on.

    If a pad normal points the wrong way, every effort number downstream is garbage.
    Draw it, and the bug is obvious in one glance.
    """
    traces = []
    for f in FINGERS:
        p, n = h.pad_pose(q, f)
        c = FINGER_COLOR[f]
        traces.append(
            go.Scatter3d(
                x=[p[0]], y=[p[1]], z=[p[2]], mode="markers", name=f"{f} pad",
                marker=dict(size=6, color=c), legendgroup=f,
            )
        )
        tip = p + normal_len * n
        traces.append(
            go.Scatter3d(
                x=[p[0], tip[0]], y=[p[1], tip[1]], z=[p[2], tip[2]],
                mode="lines", line=dict(color=c, width=5), name=f"{f} normal",
                legendgroup=f, showlegend=False,
            )
        )
    return traces


def key_traces(keys: list[dict]) -> list:
    """Keys as {finger, pos, normal, effort?}. Colour by effort when present."""
    traces = []
    for i, k in enumerate(keys):
        p = np.asarray(k["pos"], float)
        n = np.asarray(k["normal"], float)
        eff = k.get("effort")
        label = f"{k.get('finger','key')}[{k.get('idx',i)}]"
        if eff is not None:
            label += f"  a³={eff:.4f}"
        traces.append(
            go.Scatter3d(
                x=[p[0]], y=[p[1]], z=[p[2]], mode="markers", name=label,
                marker=dict(
                    size=9, symbol="square", color=[eff] if eff is not None else "#333",
                    colorscale="Viridis" if eff is not None else None,
                    showscale=False, line=dict(width=1, color="#000"),
                ),
            )
        )
        tail = p - 0.010 * n  # normal points out of the key, toward the finger
        traces.append(
            go.Scatter3d(
                x=[p[0], tail[0]], y=[p[1], tail[1]], z=[p[2], tail[2]],
                mode="lines", line=dict(color="#333", width=3),
                showlegend=False, hoverinfo="skip",
            )
        )
    return traces


def _basis(n: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Two unit vectors spanning the plane perpendicular to n."""
    n = n / (np.linalg.norm(n) + 1e-12)
    a = np.array([0.0, 0.0, 1.0]) if abs(n[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    u = np.cross(n, a)
    u /= np.linalg.norm(u) + 1e-12
    return u, np.cross(n, u)


def well_traces(keys: list[dict]) -> list:
    """Wells drawn as the U-CHANNELS they are, with their five joystick directions.

    THE FINGERTIP BONE GOES INTO THE WELL. It does not rest its pad on the well's opening --
    that was the bug, and it was a real geometric error, not just a drawing one. A well drawn
    as a disc on the pad normal is a device you would have to lower your fingertip into
    vertically, like a piston. A DataHand/Svalboard well is a channel the distal phalanx
    SLIDES INTO along its own axis:

        floor    palmar, under the pad          -> `click` presses into it
        walls    lateral, either side           -> `left` / `right` push against them
        open     proximally (the finger enters) and dorsally (the finger lifts out)

    So the channel's long axis is the DISTAL PHALANX AXIS, and the floor normal is the PAD
    NORMAL. They are perpendicular, and both are needed. `keys` items:
      {pos, axis, floor, lateral, half, radius, finger?, label?, dirs?{name: vec}}
    """
    traces = []
    for k in keys:
        p = np.asarray(k["pos"], float)
        ax = np.asarray(k["axis"], float)
        fl = np.asarray(k["floor"], float)
        lat = np.asarray(k["lateral"], float)
        r = float(k["radius"])
        L = 2.0 * float(k["half"])          # the channel must hold the whole phalanx
        col = FINGER_COLOR.get(k.get("finger", ""), "#444")
        w = r + 0.0015                      # half-width incl. wall

        # the channel runs from the pad (distal) back along the bone (proximal)
        d0 = p + 0.004 * ax                 # a little past the fingertip: the end stop
        d1 = p - L * ax                     # open, proximal end

        def quad(a, b, c, d, color, opacity):
            V = np.array([a, b, c, d])
            F = np.array([(0, 1, 2), (0, 2, 3)])
            return go.Mesh3d(
                x=V[:, 0], y=V[:, 1], z=V[:, 2], i=F[:, 0], j=F[:, 1], k=F[:, 2],
                color=color, opacity=opacity, flatshading=True,
                lighting=dict(ambient=0.5, diffuse=0.8),
                name=k.get("label", "well"), hoverinfo="name", showlegend=False)

        f0 = p + 0.0015 * fl                # floor sits just palmar of the pad
        # FLOOR (what `click` presses into)
        traces.append(quad(d0 + 0.0015 * fl - w * lat, d0 + 0.0015 * fl + w * lat,
                           d1 + 0.0015 * fl + w * lat, d1 + 0.0015 * fl - w * lat, col, 1.0))
        # SIDE WALLS, rising dorsally to about the bone's equator -- that wrap is what lets a
        # sideways nudge register at all, and it is why the rim is ~one flesh-radius tall
        for sgn in (+1.0, -1.0):
            a = d0 + 0.0015 * fl + sgn * w * lat
            b = d1 + 0.0015 * fl + sgn * w * lat
            traces.append(quad(a, b, b - r * fl, a - r * fl, col, 0.40))
        # DISTAL END STOP
        traces.append(quad(d0 + 0.0015 * fl - w * lat, d0 + 0.0015 * fl + w * lat,
                           d0 - r * fl + w * lat, d0 - r * fl - w * lat, col, 0.40))

        for name, dvec in (k.get("dirs") or {}).items():
            dv = np.asarray(dvec, float)
            dv = dv / (np.linalg.norm(dv) + 1e-12)
            a = f0
            b = a + 0.010 * dv
            traces.append(go.Scatter3d(
                x=[a[0], b[0]], y=[a[1], b[1]], z=[a[2], b[2]], mode="lines",
                line=dict(color="#111", width=4),
                name=f"{k.get('finger','')}:{name}", hoverinfo="name", showlegend=False))
    return traces

def frame_traces(nodes: dict[str, np.ndarray], members: list[tuple[str, str, str]]) -> list:
    """Exoskeleton frame: nodes + members. `members` is (i_node, j_node, kind)."""
    style = {
        "alu": dict(color="#7f8c99", width=8),
        "nylon": dict(color="#3d9970", width=6),
        "strap": dict(color="#8b6f47", width=3, dash="dot"),
    }
    traces, seen = [], set()
    for i, j, kind in members:
        a, b = nodes[i], nodes[j]
        traces.append(
            go.Scatter3d(
                x=[a[0], b[0]], y=[a[1], b[1]], z=[a[2], b[2]], mode="lines",
                line=style.get(kind, style["alu"]), name=kind,
                legendgroup=kind, showlegend=kind not in seen, hoverinfo="skip",
            )
        )
        seen.add(kind)
    P = np.array(list(nodes.values()))
    traces.append(
        go.Scatter3d(
            x=P[:, 0], y=P[:, 1], z=P[:, 2], mode="markers",
            marker=dict(size=4, color="#2c3e50"), name="frame nodes",
            text=list(nodes), hoverinfo="text",
        )
    )
    return traces


def show(
    h: MyoHand,
    q: np.ndarray,
    keys: list[dict] | None = None,
    frame: tuple[dict, list] | None = None,
    title: str = "ExoKey",
    path: str = "out/scene.html",
    bones: bool = True,
) -> str:
    import os

    traces = []
    if bones:
        traces += _mesh_traces(h, q)
    traces += _pad_traces(h, q)
    if keys:
        traces += key_traces(keys)
    if frame:
        traces += frame_traces(*frame)

    fig = go.Figure(traces)
    # Equal aspect: a squashed axis makes a bad reach look fine.
    fig.update_layout(
        title=title,
        scene=dict(
            aspectmode="data",
            xaxis_title="x (m)", yaxis_title="y (m)", zaxis_title="z (m)",
        ),
        margin=dict(l=0, r=0, t=40, b=0),
        template="plotly_white",
    )
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    fig.write_html(path, include_plotlyjs="cdn")
    return os.path.abspath(path)


def skin_trace(h, q, opacity: float = 0.35, colour: str = "#e8c4a8"):
    """The hand's SKIN, from the MRI-derived flesh model (hand/flesh.py).

    Every render in this project until now drew BONES and CAPSULES. That is why every geometry
    bug had to be caught by a NUMBER -- the shell 4.3 mm inside a finger, the wrap sweeping
    through a fingertip, the wells overlapping -- when a picture of the actual hand would have
    shown them at a glance. A skeleton is not what the device touches.
    """
    from hand.flesh import skin

    V, F = skin(h, q)
    if not len(V):
        return None
    return go.Mesh3d(
        x=V[:, 0], y=V[:, 1], z=V[:, 2], i=F[:, 0], j=F[:, 1], k=F[:, 2],
        color=colour, opacity=opacity, flatshading=False,
        lighting=dict(ambient=0.55, diffuse=0.85, specular=0.12, roughness=0.9),
        name="skin", hoverinfo="skip", showlegend=False,
    )


def strap_loop(h, q, station, width=0.016, standoff=0.0012, device=None):
    """THE BAND'S ACTUAL CENTRELINE -- ONE definition, shared with the physics.

    Delegates to manufacture.strap.band_loop: the convex hull of the (skin ∪ device) cross-section
    (§8.15f). Pass `device=(nodes,bars,live,radii)` and the band goes OVER the gauntlet, which is
    what a tensioned strap holding the device down actually does; omit it and this is the skin-only
    hull (a slightly rounder version of the old max-radius outline).

    ⚠ ONE DEFINITION. The live view once drew the band as a FIXED 55 mm CIRCLE and the user could
    not see the nodes it pulled on. Two renderers, two geometries, and the picture stopped telling
    the truth. The band path lives in ONE place now, and the render reads it from there.
    """
    from manufacture.strap import band_loop

    return band_loop(h, q, station, device=device, width=width, standoff=standoff)


def strap_traces(h, q, anchor_pts, n_bands=2, width=0.016, standoff=0.0012):
    """THE STRAPS, drawn as the bands they are -- around the WHOLE hand, not just over the top.

    THE USER: "Could you illustrate the straps in the render too?"

    They are not decoration. The gauntlet's anchor is BILINEAR: flesh can PUSH the device off the
    hand but it cannot PULL it back on, and a keypress ~120 mm from the wrist is a MOMENT -- it
    presses one end of the anchor patch INTO the hand and lifts the other end OFF it. THE STRAP
    CARRIES THE LIFTING END. Take it away and the same structure deflects 9178 um instead of 485:
    18x the gate. It is doing about 1 N of work and it is load-bearing.

    So it has to be drawn, and it has to be drawn going ALL THE WAY ROUND. A band drawn only
    across the back of the hand is a band that cannot pull -- and drawing it that way would hide
    exactly the thing the physics depends on.

    Each band is the hand's own cross-section at that station: take the skin in a slab, hull it in
    the plane, and lay the webbing on it.
    """
    import mujoco

    from hand.flesh import skin
    from structure.frame import hand_axes

    # ⚠ THE STRAP MUST NOT WRAP THE THUMB.
    #
    # THE USER: "I see that the straps are currently too far down the thumbs and so would restrict
    # or load the thumb."
    #
    # And they are right, and it is a ROUTING CONSTRAINT, not a drawing error. A band built from
    # the hand's whole cross-section at a station necessarily encircles the THUMB RAY as well --
    # and a strap that crosses the thumb metacarpal loads the thumb on every keystroke and fights
    # its opposition. That is not a strap anyone could fit.
    #
    # A real glove strap goes through the FIRST WEB SPACE: it wraps the four-metacarpal block and
    # the carpus, and passes BETWEEN thumb and index. The anchor already lives there (BEARING is
    # carpus + metacarpals 2-5, never firstmc), so the physics was right and only the picture was
    # wrong -- which is the sort of thing that stays wrong until someone LOOKS at it.
    V, _, L = skin(h, q, labels=True)
    THUMB = ("firstmc", "proximal_thumb", "distal_thumb", "trapezium")
    tid = {mujoco.mj_name2id(h.model, mujoco.mjtObj.mjOBJ_BODY, b) for b in THUMB}
    V_thumb = V[np.isin(L, list(tid))]
    V = V[~np.isin(L, list(tid))]
    o, e_d, e_r, e_o = hand_axes(h, q)
    A = np.asarray(anchor_pts, float)
    if not len(A):
        return []
    # ⚠ THE PHYSICS AND THE PICTURE MUST USE THE SAME DEFINITION, or they drift apart and only an
    # eye catches it. structure.anchor.strap_bands() decides where the bands are -- proximal of the
    # thumb's CMC at the wrist, through the first web space at the metacarpal heads -- and BOTH the
    # solver (which pulls only on the nodes a band touches) and this render read it from there.
    from structure.anchor import strap_bands

    stations = strap_bands(h, q, A) if n_bands > 1 else [float(((A - o) @ e_d).mean())]

    traces = []
    for st in stations:
        loop = strap_loop(h, q, st, width=width, standoff=standoff)
        if loop is None:
            continue

        Vv, Ff = [], []
        for k in range(len(loop) - 1):
            b = len(Vv)
            Vv += [loop[k] - 0.5 * width * e_d, loop[k] + 0.5 * width * e_d,
                   loop[k + 1] + 0.5 * width * e_d, loop[k + 1] - 0.5 * width * e_d]
            Ff += [(b, b + 1, b + 2), (b, b + 2, b + 3)]
        Vv = np.array(Vv)
        Ff = np.array(Ff)
        traces.append(go.Mesh3d(
            x=Vv[:, 0], y=Vv[:, 1], z=Vv[:, 2], i=Ff[:, 0], j=Ff[:, 1], k=Ff[:, 2],
            color="#b03060", opacity=0.30, flatshading=True,
            lighting=dict(ambient=0.7, diffuse=0.5),
            name="strap (carries the lift)", hoverinfo="name", showlegend=False))
    return traces

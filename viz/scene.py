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


def keycap_traces(
    keys: list[dict],
    top: float = 0.0125,
    base: float = 0.0145,
    height: float = 0.0032,
    stem: float = 0.0028,
) -> list:
    """Keys drawn as real low-profile keycaps (Kailh-Choc-ish: ~13 mm cap, ~3 mm tall),
    not as abstract markers.

    Each key is a tapered cap whose TOP FACE is the surface the pad actually lands on, and
    whose axis is the key's switch axis. That makes the geometry legible: you can see at a
    glance whether a cap is presented squarely to the finger or edge-on -- which matters
    here, because the thumb's cap sits at ~80 deg of obliquity to its pad and a square
    marker hides that completely.

    `keys` items: {pos, normal, finger?, label?}. `normal` points OUT of the cap, at the digit.
    """
    traces = []
    for k in keys:
        p = np.asarray(k["pos"], float)
        n = np.asarray(k["normal"], float)
        n = n / (np.linalg.norm(n) + 1e-12)
        u, v = _basis(n)
        col = FINGER_COLOR.get(k.get("finger", ""), "#444")

        def box(c0, c1, s0, s1):
            """Tapered box between face centres c0 (size s0) and c1 (size s1)."""
            corners = [(1, 1), (1, -1), (-1, -1), (-1, 1)]
            A = [c0 + a * s0 / 2 * u + b * s0 / 2 * v for a, b in corners]
            B = [c1 + a * s1 / 2 * u + b * s1 / 2 * v for a, b in corners]
            V = np.array(A + B)
            F = [(0, 1, 2), (0, 2, 3), (4, 6, 5), (4, 7, 6)]
            for i in range(4):
                j = (i + 1) % 4
                F += [(i, j, 4 + j), (i, 4 + j, 4 + i)]
            return V, np.array(F)

        cap_top, cap_base = p, p - height * n
        V, F = box(cap_top, cap_base, top, base)
        traces.append(go.Mesh3d(
            x=V[:, 0], y=V[:, 1], z=V[:, 2], i=F[:, 0], j=F[:, 1], k=F[:, 2],
            color=col, opacity=1.0, flatshading=True,
            lighting=dict(ambient=0.45, diffuse=0.85, specular=0.3),
            name=k.get("label", "key"), hoverinfo="name", showlegend=False,
        ))

        # the switch body under the cap, so the travel direction reads at a glance
        V, F = box(cap_base, cap_base - stem * n, base * 0.55, base * 0.55)
        traces.append(go.Mesh3d(
            x=V[:, 0], y=V[:, 1], z=V[:, 2], i=F[:, 0], j=F[:, 1], k=F[:, 2],
            color="#2b2b2b", opacity=1.0, flatshading=True,
            hoverinfo="skip", showlegend=False,
        ))
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

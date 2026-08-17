"""Render out/gauntlet.stl as a solid Plotly mesh -> out/gauntlet_solid.html.

The old out/gauntlet_solid.html had no generator left in the tree. This replaces it from the
current artifact -- the marching-cubes STL that export_stl.py produces -- so "the printable solid"
is viewable in a browser again. Output path is overridable so a preview can avoid clobbering.

    PYTHONPATH=. .venv/bin/python scripts/solid_view.py [in.stl] [out.html]
"""
from __future__ import annotations

import sys

import numpy as np
import plotly.graph_objects as go


# Optional camera auto-rotate, OFF by default -- the user drives depth by dragging with the mouse.
# The button turns it on; grabbing the scene pauses it. {plot_id} is filled in by plotly write_html.
_SPIN_JS = """
var gd = document.getElementById('{plot_id}');
var spinning = false, t = 0, R = 2.3, Z = 1.05;
function frame() {
  if (spinning) {
    t += 0.005;
    Plotly.relayout(gd, {'scene.camera.eye': {x: R*Math.cos(t), y: R*Math.sin(t), z: Z}});
  }
  requestAnimationFrame(frame);
}
gd.addEventListener('mousedown', function(){ spinning = false; upd(); });
var btn = document.createElement('button');
btn.style.cssText = 'position:fixed;top:10px;right:14px;z-index:9;padding:4px 10px;'
  + 'font:13px sans-serif;border:1px solid #bbb;border-radius:4px;background:#fff;cursor:pointer';
function upd(){ btn.textContent = spinning ? '⏸ pause' : '⟳ spin'; }
btn.onclick = function(){
  spinning = !spinning;
  if (spinning) { var e = gd.layout.scene.camera.eye; t = Math.atan2(e.y, e.x); }
  upd();
};
document.body.appendChild(btn); upd(); requestAnimationFrame(frame);
"""


def build_fig(v, f, title):
    """A lit, height-shaded solid from vertices `v` (N,3) and triangle indices `f` (M,3).

    ⚠ THE LIGHT MUST BE FAR FROM THE MESH. Plotly's `lightposition` is in DATA units; a light at
    (100,200,300) sits INSIDE a ~150 mm gauntlet and lights every face the same -- the part renders
    as a flat silhouette. Put it thousands of mm away so it reads as a direction.

    Depth on a self-overlapping lattice needs more than one still light, so: strong diffuse +
    specular for form, a saturated height ramp for a second cue, and the page auto-rotates
    (see _SPIN_JS) because motion parallax is what actually resolves which strut is in front.
    """
    v = np.asarray(v, float)
    lo, hi = v.min(0), v.max(0)
    span = float(np.linalg.norm(hi - lo))
    fig = go.Figure(go.Mesh3d(
        x=v[:, 0], y=v[:, 1], z=v[:, 2],
        i=f[:, 0], j=f[:, 1], k=f[:, 2],
        intensity=v[:, 2], colorscale=[[0, "#4f4534"], [0.5, "#b09a72"], [1, "#f3e6c8"]],
        showscale=False, flatshading=True,
        lighting=dict(ambient=0.38, diffuse=0.85, specular=0.5, roughness=0.3, fresnel=0.25),
        lightposition=dict(x=lo[0] - 2 * span, y=lo[1] - span, z=hi[2] + 3 * span),
    ))
    fig.update_layout(
        title=title, paper_bgcolor="#eef1f4",
        scene=dict(aspectmode="data", xaxis_visible=False, yaxis_visible=False,
                   zaxis_visible=False,
                   camera=dict(eye=dict(x=1.6, y=-1.5, z=1.05))),
        margin=dict(t=40, l=0, r=0, b=0), template="plotly_white",
    )
    return fig


def write(fig, out):
    """write_html with the auto-rotate script attached."""
    fig.write_html(out, include_plotlyjs="cdn", post_script=_SPIN_JS)


def main():
    import trimesh
    src = sys.argv[1] if len(sys.argv) > 1 else "out/gauntlet.stl"
    out = sys.argv[2] if len(sys.argv) > 2 else "out/gauntlet_solid.html"
    m = trimesh.load(src)
    v, f = np.asarray(m.vertices), np.asarray(m.faces)
    d = m.bounds[1] - m.bounds[0]
    fig = build_fig(v, f, f"{src} — {len(v)} verts, {len(f)} faces, "
                          f"{d[0]:.0f}×{d[1]:.0f}×{d[2]:.0f} mm")
    write(fig, out)
    print(f"wrote {out}  ({len(v)} verts, {len(f)} faces)")


if __name__ == "__main__":
    main()

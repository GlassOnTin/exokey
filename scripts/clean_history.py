"""Strip stale ARM/WORLD points from the anim history 'hand' trace.

    PYTHONPATH=. .venv/bin/python scripts/clean_history.py

out/history/gen_*.json was written by an OLD viz/live.py that drew the WHOLE MuJoCo body chain in the
"hand" trace -- forearm, elbow, shoulder -- not just the hand bones. Those points sit 320-640 mm from
the hand while the hand skeleton itself is within ~84 mm of its centre (a clean, huge gap), and they
drag the anim's autoscale floor-to-shoulder, squashing the hand to a dot.

viz/live.py is now fixed (it names the hand bones -- CARPUS + METACARPALS + PHALANGES -- and requires
both ends of a segment to be hand bones), so a fresh optimisation run writes clean data. This cleans
the already-committed (gh-pages) history in place, by DISTANCE from the hand: robust where the earlier
length/z filters were not, because the arm is a chain and some of its bones are short.
"""
from __future__ import annotations

import glob
import json

import numpy as np

FAR = 0.150      # the hand skeleton spans <= 84 mm from its centre; the arm strays start at 323 mm


def main():
    files = sorted(glob.glob("out/history/gen_*.json"))
    scenes = dropped = 0
    for fn in files:
        d = json.load(open(fn))
        sc = d.get("scene")
        if not sc:                                   # some frames are logged with scene=None
            continue
        scenes += 1
        tr = {t.get("name"): t for t in sc["traces"]}
        g, h = tr.get("gauntlet"), tr.get("hand")
        if not g or not h:
            continue
        G = np.array([(g["x"][i], g["y"][i], g["z"][i])
                      for i in range(len(g["x"])) if g["x"][i] is not None])
        cen = G.mean(0)                              # the gauntlet is unambiguously on the hand
        x, y, z = h["x"], h["y"], h["z"]
        nx, ny, nz = [], [], []
        for k in range(0, len(x), 3):                # segments are triples [p0, p1, None]
            pts = [np.array([x[k + i], y[k + i], z[k + i]], float)
                   for i in (0, 1) if k + i < len(x) and x[k + i] is not None]
            if len(pts) == 2 and max(np.linalg.norm(p - cen) for p in pts) > FAR:
                dropped += 1
                continue
            nx += x[k:k + 3]; ny += y[k:k + 3]; nz += z[k:k + 3]
        h["x"], h["y"], h["z"] = nx, ny, nz
        json.dump(d, open(fn, "w"))
    print(f"{scenes} scenes with geometry; dropped {dropped} non-hand segments "
          f"(> {FAR*1e3:.0f} mm from the hand)")


if __name__ == "__main__":
    main()

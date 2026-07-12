"""Stage 1 -- end-to-end: keys near each fingertip, real muscle effort, in the browser.

Places up to 3 candidate keys per finger, solves the full inner problem (posture +
muscle redundancy under a 0.5 N press) for each, and renders the result coloured by
effort. This is the first thing worth looking at: if a pad normal is flipped or a
finger is mirrored, it is obvious here and invisible in a table of numbers.
"""
from __future__ import annotations

import numpy as np

from hand.myohand import FINGERS, MyoHand
from viz.scene import show

PRESS_N = 0.5
KEYS_PER_FINGER = 3
GAP = 0.004  # m, key sits this far off the pad at rest


def main():
    h = MyoHand()
    q0 = h.q_neutral.copy()

    keys = []
    print(f"{'finger':8s} {'key':4s} {'reach mm':>9s} {'pad deg':>8s} "
          f"{'max act':>8s} {'effort a^3':>11s}  ok")
    print("-" * 60)

    for f in FINGERS:
        p_rest, n_rest = h.pad_pose(q0, f)
        # Fan the candidate keys along the pad's local surface, offset off the pad.
        # A crude spread for now -- Stage 2's effort field replaces this guesswork.
        u = np.cross(n_rest, [0.0, 0.0, 1.0])
        u = u / (np.linalg.norm(u) + 1e-12)
        for k in range(KEYS_PER_FINGER):
            lateral = (k - (KEYS_PER_FINGER - 1) / 2) * 0.008  # -8, 0, +8 mm
            key_pos = p_rest + GAP * n_rest + lateral * u
            key_n = -n_rest  # key faces back at the finger

            post = h.press(f, key_pos, key_n, press_N=PRESS_N, q0=q0)
            keys.append(
                dict(finger=f, idx=k, pos=key_pos, normal=key_n,
                     effort=post.effort, ok=post.ok)
            )
            why = "ok" if post.ok else ("SATURATED" if post.saturated else "reach/angle")
            print(f"{f:8s} {k:<4d} {post.pos_err*1000:9.2f} "
                  f"{np.rad2deg(post.ang_err):8.1f} {post.max_act:8.3f} "
                  f"{post.effort:11.5f}  {why}")

    eff = [k["effort"] for k in keys]
    print("-" * 60)
    print(f"effort range: {min(eff):.5f} .. {max(eff):.5f}   "
          f"({max(eff)/max(min(eff),1e-9):.1f}x spread across key positions)")
    print(f"reachable: {sum(k['ok'] for k in keys)}/{len(keys)}")

    # Render the *rest* posture with all candidate keys, coloured by effort.
    path = show(
        h, q0, keys=keys,
        title=f"ExoKey Stage 1 — {len(keys)} candidate keys, effort = Σa³ @ {PRESS_N} N",
        path="out/stage1.html",
    )
    print(f"\nbrowser view: file://{path}")


if __name__ == "__main__":
    main()

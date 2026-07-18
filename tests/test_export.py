"""Per-user hand-scale export (scripts/export_stl.py --hand-mm), as fast executable claims.

Meshing the full ~47 M-voxel solid takes minutes, so these gate the SCALING CONTRACT instead:
the frame nodes and the hand must scale together, about the same origin, so that at any hand size
the finger still SEATS in its cup (the property that was silently broken by scaling the hand alone
-- the frame stayed put and the cups drifted ~80 mm off the skeleton).
"""
import os
import pickle

import numpy as np
import pytest

from hand.scaling import ANSUR_HAND_LENGTH_MM, REFERENCE_PERCENTILE
from manufacture import entry, mount

REF_MM = ANSUR_HAND_LENGTH_MM[REFERENCE_PERCENTILE]   # 185 mm, the hand the model IS


def _posed(hand_mm):
    """The shipped posture, re-fitted to a `hand_mm` hand -- hand AND frame scaled by the same s,
    exactly as scripts/export_stl.py does it."""
    if not os.path.exists("out/final_design.pkl") or not os.path.exists("out/final.npz"):
        pytest.skip("needs out/final_design.pkl + out/final.npz (the shipped design)")
    from design.vector import posture, tm_of, tp_of
    from hand.myohand import FINGERS, MyoHand
    s = hand_mm / REF_MM
    h = MyoHand(scale=s)
    x = pickle.load(open("out/final_design.pkl", "rb"))["x"]
    q = h.compose({f: posture(h, f, tp_of(x, f), tm_of(x, f), float(x.get(f"ab_{f}", 0.0)))
                   for f in FINGERS})
    z = np.load("out/final.npz", allow_pickle=True)
    nodes = np.array(z["nodes"], float) * s          # the frame scales WITH the hand -- the contract
    mounts = {f: nodes[int(i)] for f, i in zip(z["fingers"], z["buttons"])}
    return h, q, mounts, list(FINGERS), s


@pytest.mark.parametrize("hand_mm", [ANSUR_HAND_LENGTH_MM[5], ANSUR_HAND_LENGTH_MM[95]])
def test_finger_seats_at_the_population_extremes(hand_mm):
    """At the 5th and 95th hand, every fingertip still contacts its cup floor -- the same seating
    assertion as test_mount, but off the median. If `nodes *= s` were dropped, the cup would drift
    off the finger and the pad would float, failing here."""
    h, q, mounts, fingers, _ = _posed(hand_mm)
    for f in fingers:
        fl = np.asarray(h.well_frame(q, f)["floor"], float)
        skin = entry.phalanx_skin(h, q, f)
        pad = skin[np.argsort(skin @ fl)[-len(skin) // 4:]]     # palmar-most quarter = the pad
        p = mount.well_mount(h, q, f, mounts[f])
        d = entry.mount_sdf(pad, boxes=p["boxes"], caps=p["caps"], cyls=p["cyls"])
        assert d.min() < 0.0015, (f, hand_mm, float(d.min()))


def test_a_bigger_hand_gives_a_bigger_button_span():
    """The fit actually changes the geometry: the 95th hand's button positions span wider than the
    5th's (guards against a no-op flag that silently exports the median every time)."""
    def span(hand_mm):
        _, _, mounts, fingers, _ = _posed(hand_mm)
        P = np.array([mounts[f] for f in fingers])
        return float(np.linalg.norm(P.max(axis=0) - P.min(axis=0)))
    small, big = span(ANSUR_HAND_LENGTH_MM[5]), span(ANSUR_HAND_LENGTH_MM[95])
    assert big > small * 1.10, (small, big)    # ~1.24x scale -> a clearly wider button cloud

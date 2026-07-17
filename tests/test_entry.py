"""The finger-entry route, as executable claims.

The whole point: a mount that clears a SEATED finger can still block it from ENTERING. These pin the
model that catches that -- so no future mount can quietly wall off the route the finger comes in by.
The load-bearing pair is the last two: a wall ACROSS the entry must read as a block, and guiding
walls BESIDE the finger (a cup's flanks) must NOT -- that is the distinction the geometry turns on.
"""
import numpy as np
import pytest

from hand.myohand import FINGERS, MyoHand
from manufacture import entry


@pytest.fixture(scope="module")
def hand():
    h = MyoHand()
    return h, h.q_neutral


def _frame(h, q, f):
    wf = h.well_frame(q, f)
    return (np.asarray(wf["pos"], float), np.asarray(wf["axis"], float),
            np.asarray(wf["floor"], float), np.asarray(wf["lateral"], float),
            np.vstack([wf["axis"], wf["floor"], wf["lateral"]]), wf["radius"], wf["half"])


def test_the_phalanx_skin_is_the_fingertip(hand):
    """The route is swept from the distal-phalanx skin -- real geometry, and near the pad."""
    h, q = hand
    tip = entry.phalanx_skin(h, q, "index")
    assert len(tip) > 50
    pad = np.asarray(h.well_frame(q, "index")["pos"], float)
    assert np.linalg.norm(tip.mean(0) - pad) < 0.03      # the cloud sits at the fingertip


def test_an_open_channel_lets_the_finger_in(hand):
    """With only a Hall seat far palmar and nothing in the slide-in path, the finger enters freely."""
    h, q = hand
    pos, ax, fl, lat, R, r, half = _frame(h, q, "index")
    seat = [(pos + 0.013 * fl, R, np.array([half, 0.001, 0.006]))]
    assert entry.entry_clearance(h, q, "index", boxes=seat) > 0
    assert entry.enters_freely(h, q, "index", boxes=seat)


def test_a_wall_across_the_route_blocks_entry(hand):
    """A wall perpendicular to the axis, proximal of the pad, spanning the finger's cross-section,
    is a BLOCK -- the model must report it (this is the failure the whole file exists to catch)."""
    h, q = hand
    pos, ax, fl, lat, R, r, half = _frame(h, q, "index")
    wall = [(pos - 0.010 * ax, R, np.array([0.001, 0.010, 0.010]))]
    assert entry.entry_clearance(h, q, "index", boxes=wall) < -entry.TOUCH_TOL
    assert not entry.enters_freely(h, q, "index", boxes=wall)


def test_guiding_walls_beside_the_finger_are_not_a_block(hand):
    """Flanks either side of the finger GUIDE the phalanx in; the model must not mistake them for a
    block, or every cup would fail. The finger is OUTSIDE them (SDF > 0), so it does not."""
    h, q = hand
    pos, ax, fl, lat, R, r, half = _frame(h, q, "index")
    flanks = [(pos + s * (r + 0.001) * lat, R, np.array([half, 0.008, 0.001])) for s in (+1.0, -1.0)]
    assert entry.enters_freely(h, q, "index", boxes=flanks)

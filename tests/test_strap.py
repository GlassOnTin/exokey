"""The strap subsystem's load-bearing facts, as executable claims."""
import functools
import pickle

import numpy as np

from design.vector import posture, tm_of, tp_of
from hand.myohand import FINGERS
from manufacture.strap import PIN_R, adjust_range, band_loop, lug_sites, perimeter
from opt.problem import hands
from structure.anchor import strap_bands


@functools.lru_cache(maxsize=1)
def _load():
    H = hands((5, 50, 95))
    ref = H[50]
    x = pickle.load(open("out/final_design.pkl", "rb"))["x"]
    q = ref.compose({f: posture(ref, f, tp_of(x, f), tm_of(x, f), float(x.get(f"ab_{f}", 0.0)))
                     for f in FINGERS})
    z = np.load("out/bone.npz", allow_pickle=True)
    nodes = z["nodes"]
    device = (nodes, [tuple(b) for b in z["bars"]], [int(e) for e in z["live"]], z["radii"])
    anchors = [int(i) for i in z["anchors"]]
    st = strap_bands(ref, q, np.array([nodes[i] for i in anchors]))
    return H, ref, x, q, nodes, device, anchors, st


def test_the_band_goes_over_the_device_not_under_it():
    """A tensioned band is the hull of (skin U device), so it bulges OVER the proud gauntlet. The
    old skin-only hull passed UNDER the dorsal structure (it hulled the hand alone) -- the very bug
    of §8.15f. The fix: the (skin U device) band is strictly longer, because it goes over."""
    _H, ref, _x, q, _n, device, _a, st = _load()
    over = band_loop(ref, q, st[0], device=device)      # wrist band
    under = band_loop(ref, q, st[0], device=None)
    assert perimeter(over) > perimeter(under) + 0.005    # bulges >= 5 mm over the structure


def test_one_strap_covers_the_population():
    """The wrist-circumference spread across 5th-95th is small enough that one adjustable strap
    (a watch strap's holes span ~30-40 mm) fits everyone."""
    H, _ref, x, *_ = _load()
    a = adjust_range(H, x)
    assert 0.0 < a["spread"] < 0.040
    assert 1.0 < a["max"] / a["min"] < 1.35


def test_the_lug_prints_and_sits_on_both_bands():
    """A watch-lug per anchor foot: a printable through-hole (>= two nozzle walls), and lugs on
    BOTH bands -- the anchor is a couple, and a couple needs both ends (§8.11)."""
    _H, ref, _x, q, nodes, device, anchors, _st = _load()
    assert PIN_R >= 0.0008
    sites = lug_sites(ref, q, nodes, anchors, device)
    assert len(sites) >= 2
    assert len({s["band"] for s in sites}) == 2

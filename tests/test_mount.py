"""The rebuilt sensor mount, as executable claims -- with the FINGER-ENTRY ROUTE as the constraint.

The prior mount was withdrawn because it blocked the route the finger enters by. Every claim here
runs the `manufacture.entry` swept-clearance: a mount that a finger cannot get into does not pass,
however watertight it is.
"""
import os
import pickle

import numpy as np
import pytest

from manufacture import entry, mount

LONG = ["index", "middle", "ring", "little"]


@pytest.fixture(scope="module")
def posed():
    if not os.path.exists("out/final_design.pkl"):
        pytest.skip("needs out/final_design.pkl (the shipped typing posture)")
    from design.vector import posture, tm_of, tp_of
    from hand.myohand import FINGERS
    from opt.problem import hands
    h = hands()[50]
    x = pickle.load(open("out/final_design.pkl", "rb"))["x"]
    q = h.compose({f: posture(h, f, tp_of(x, f), tm_of(x, f), float(x.get(f"ab_{f}", 0.0)))
                   for f in FINGERS})
    z = np.load("out/final.npz", allow_pickle=True)
    nodes = np.array(z["nodes"], float)
    mounts = {f: nodes[int(i)] for f, i in zip(z["fingers"], z["buttons"])}
    return h, q, mounts, list(FINGERS)


def test_every_well_mount_lets_its_finger_in(posed):
    """The core claim: for every finger, the mount leaves the entry route open (the finger can slide
    into its cup). This is what the withdrawn geometry failed and this file guards."""
    h, q, mounts, fingers = posed
    for f in fingers:
        p = mount.well_mount(h, q, f, mounts[f])
        assert entry.enters_freely(h, q, f, boxes=p["boxes"], caps=p["caps"], cyls=p["cyls"]), f


def test_every_well_mount_is_one_watertight_piece(posed):
    h, q, mounts, fingers = posed
    for f in fingers:
        m = mount.well_mesh(h, q, f, mounts[f])
        assert m.is_watertight and m.body_count == 1, f


def test_the_cluster_lets_every_long_finger_in(posed):
    """The shared cluster keeps every finger's entry open -- shared flanks GUIDE the phalanx in
    (beside it, along the axis), they never cross the slide-in path."""
    h, q, mounts, _ = posed
    p = mount.cluster_mount(h, q, LONG, {f: mounts[f] for f in LONG})
    for f in LONG:
        assert entry.enters_freely(h, q, f, boxes=p["boxes"], caps=p["caps"], cyls=p["cyls"]), f


def test_the_cluster_is_one_watertight_piece(posed):
    h, q, mounts, _ = posed
    m = mount.cluster_mesh(h, q, LONG, {f: mounts[f] for f in LONG})
    assert m.is_watertight
    assert m.body_count == 1

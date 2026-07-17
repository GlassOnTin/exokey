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


def test_the_finger_actually_seats_in_its_cup(posed):
    """THE BUG THE ENTRY TEST MISSED. `enters_freely` passes VACUOUSLY for a cup the finger never
    touches -- the fingertip floated ~7 mm above its cup because the cup was built to `pos + r` and
    `well_frame["pos"]` is the pad SURFACE, not the pulp centre. So check the pad is right against the
    cup floor, not hovering over it. (Regression for the 2026-07-17 'finger above the cup' report.)"""
    h, q, mounts, fingers = posed
    for f in fingers:
        fl = np.asarray(h.well_frame(q, f)["floor"], float)
        skin = entry.phalanx_skin(h, q, f)
        pad = skin[np.argsort(skin @ fl)[-len(skin) // 4:]]   # the palmar-most quarter (the pad region)
        p = mount.well_mount(h, q, f, mounts[f])
        d = entry.mount_sdf(pad, boxes=p["boxes"], caps=p["caps"], cyls=p["cyls"])
        assert d.min() < 0.0015, (f, float(d.min()))          # the pad CONTACTS the cup floor (~SEAT_CLEAR),
        #                                                       not floating ~7 mm above it as it did


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


def test_every_drop_in_insert_lets_its_finger_in(posed):
    """The finger enters the INSERT's cup (it is the cradle it sits in), so the drop-in TPU cradle
    gets the same entry check as the frame -- its cup is open proximally too, nail hood and all."""
    h, q, _, fingers = posed
    for f in fingers:
        p = mount.well_insert(h, q, f)
        assert entry.enters_freely(h, q, f, boxes=p["boxes"], caps=p["caps"], cyls=p["cyls"]), f


def test_every_drop_in_insert_is_one_watertight_piece(posed):
    h, q, _, fingers = posed
    for f in fingers:
        m = mount.insert_mesh(h, q, f)
        assert m.is_watertight and m.body_count == 1, f


def test_the_assembled_frame_and_insert_let_the_finger_in(posed):
    """In the assembled device the finger enters past BOTH the frame and the drop-in cradle; the
    combined geometry must still leave the slide-in route open."""
    h, q, mounts, fingers = posed
    for f in fingers:
        fr, ins = mount.well_mount(h, q, f, mounts[f]), mount.well_insert(h, q, f)
        assert entry.enters_freely(h, q, f, boxes=fr["boxes"] + ins["boxes"],
                                   caps=fr["caps"] + ins["caps"], cyls=fr["cyls"] + ins["cyls"]), f


def _gauntlet_struts():
    z = np.load("out/final.npz", allow_pickle=True)
    nodes = np.array(z["nodes"], float)
    bars = [tuple(b) for b in z["bars"]]
    live = [int(e) for e in z["live"]]
    rr = np.atleast_1d(np.asarray(z["radii"] if "radii" in z.files else 0.0009, float))
    return [((nodes[bars[e][0]], nodes[bars[e][1]]), float(rr[k]) if rr.size > 1 else float(rr[0]))
            for k, e in enumerate(live)]


def test_the_finger_enters_past_the_gauntlet_struts_too(posed):
    """The entry route must clear the GAUNTLET STRUTS as well as the mount -- the truss wraps near the
    fingertips, and a strut across the slide-in would block the finger just as a mount wall would. So
    the check is run against frame + cradle + every live strut, not the mount alone."""
    h, q, mounts, fingers = posed
    struts = _gauntlet_struts()
    for f in fingers:
        fr, ins = mount.well_mount(h, q, f, mounts[f]), mount.well_insert(h, q, f)
        assert entry.enters_freely(h, q, f, boxes=fr["boxes"] + ins["boxes"],
                                   caps=fr["caps"] + ins["caps"] + struts,
                                   cyls=fr["cyls"] + ins["cyls"]), f


def test_the_harness_bus_is_a_shorter_shared_tree(posed):
    """The minimal-copper harness (§8.15l qqq-2): a SHARED bus (Steiner tree over the struts) uses less
    conductor than five point-to-point runs, still reaches every sensor, and lays only in live struts."""
    import collections
    z = np.load("out/final.npz", allow_pickle=True)
    nodes = np.array(z["nodes"], float)
    bars = [tuple(b) for b in z["bars"]]
    live = [int(e) for e in z["live"]]
    btn = {f: int(i) for f, i in zip(z["fingers"], z["buttons"])}
    anchors = [int(a) for a in z["anchors"]]

    bus = mount.harness_bus(nodes, bars, live, btn, anchors)
    uniq = sum(float(np.linalg.norm(nodes[i] - nodes[j])) for i, j, _ in bus)      # shared groove length
    routes = mount.harness_routes(nodes, bars, live, btn, anchors)                 # the baseline
    base = sum(float(np.linalg.norm(nodes[r[k]] - nodes[r[k + 1]]))
               for r in routes for k in range(len(r) - 1))
    assert uniq < 0.85 * base, (uniq, base)                    # the shared bus is materially shorter

    liveset = {frozenset(bars[e]) for e in live}
    assert all(frozenset((i, j)) in liveset for i, j, _ in bus)  # only real struts carry wire
    assert all(nw in (2, 4, 6) for *_, nw in bus)              # 2 power (+2 per signal bus) conductors

    adj = collections.defaultdict(set)                         # every sensor must reach the wrist
    for i, j, _ in bus:
        adj[i].add(j); adj[j].add(i)
    seen, stack = set(anchors), list(anchors)
    while stack:
        k = stack.pop()
        for n in adj[k] - seen:
            seen.add(n); stack.append(n)
    assert all(btn[f] in seen for f in btn), [f for f in btn if btn[f] not in seen]

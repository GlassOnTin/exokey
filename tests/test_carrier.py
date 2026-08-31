"""Gates on the carrier seam (the 2026-08-29 reframe's optimisation path).

The device now CARRIES a purchased Svalboard kit (VISION.md §1). manufacture/carrier.py turns
that into boundary conditions grow() can consume, and it must do so WITHOUT inventing a kit
dimension. These gates hold the two provenances apart:

  * carrier_from_bracket() runs NOW off our disclosed bracket GUESSes, so the optimiser has a
    load to grow against before the kit is measured;
  * carrier_from_kit() refuses until the KIT_* constants are measured — the ship-gate, so a
    design can never be shipped against a made-up payload;
  * the geometry is what the reframe needs: a flat rigid deck on the dorsum (the kit bolts to
    a plane), keywells at the model's OWN fingertips (the probe showed the pad constellation
    rescales ~24% across the population, so wells track the hand, they are not cast fixed),
    and the payload mass pulling the deck down in world -Z.
"""
from __future__ import annotations

import numpy as np
import pytest
import trimesh

from opt.problem import hands
from manufacture.carrier import Carrier, carrier_from_bracket, carrier_from_kit
from hand.myohand import FINGERS


@pytest.fixture(scope="module")
def h50():
    return hands()[50]


@pytest.fixture(scope="module")
def carrier(h50):
    return carrier_from_bracket(h50, np.zeros(h50.model.nq))


def test_bracket_carrier_builds_without_the_kit(carrier):
    """The optimiser must have a load to grow against BEFORE the kit is measured."""
    assert isinstance(carrier, Carrier)
    assert len(carrier.deck) >= 4, "a deck the kit bolts to needs a real footprint"
    assert set(carrier.keywells) == set(FINGERS)
    assert carrier.mass > 0.0


def test_deck_is_a_flat_plane(carrier):
    """The kit bolts to a RIGID plane, so the deck must be planar — it cannot follow the
    curved dorsum (that is why it floats, and why grown struts bridge the gap)."""
    P = np.asarray(carrier.deck)
    c = P.mean(0)
    # best-fit plane normal via SVD; all points must lie in it
    _, s, vt = np.linalg.svd(P - c)
    assert s[2] / (s[0] + 1e-12) < 1e-6, "deck is not planar"


def test_deck_sits_above_the_dorsum(carrier, h50):
    """The mount plane is on the BACK of the hand, clear of the skin."""
    from structure.frame import hand_axes
    from structure.lattice import skin
    from scipy.spatial import cKDTree
    q = np.zeros(h50.model.nq)
    o, e_d, e_r, e_o = hand_axes(h50, q)
    V, _F, _L = skin(h50, q, labels=True)
    hgt = float(np.max((np.asarray(V) - o) @ e_o))
    deck_h = (np.asarray(carrier.deck) - o) @ e_o
    assert np.all(deck_h >= hgt - 1e-6), "deck clips into the dorsum"


def test_keywells_at_the_fingertips(carrier, h50):
    """Each keywell is at its digit's reaction point — the same place the self-built well put
    its button — so the keypress enters where the finger actually pushes."""
    from design.vector import well_channel, action_dirs
    q = np.zeros(h50.model.nq)
    for f in FINGERS:
        dist, prox, r = well_channel(h50, q, f)
        want = 0.5 * (np.asarray(dist) + np.asarray(prox)) + np.asarray(action_dirs(h50, q, f)["click"]) * r
        assert np.allclose(carrier.keywells[f], want, atol=1e-9)
        assert np.allclose(carrier.keywell_dir[f], action_dirs(h50, q, f)["click"], atol=1e-9)


def test_mass_case_pulls_down_in_world_z(carrier):
    """A worn device's mass pulls toward the ground: world -Z, total m*g, spread over the
    deck's bolt pattern (dumping it on one node loads one column and lets ESO delete the
    rest of the plate's support -- the 2026-08-30 floating-deck bug)."""
    case = carrier.mass_case(carrier.deck)
    assert len(case) == len(carrier.deck)
    assert all(f[0] == 0.0 and f[1] == 0.0 for f in case.values())
    assert sum(f[2] for f in case.values()) == pytest.approx(-carrier.mass * 9.81)


def test_carrier_buttons_are_the_deck_bolts(h50, carrier):
    """THE LOAD PATH. The kit's keys are rigid towers bolted to the deck, so the keypress
    enters OUR structure at the deck bolt node -- not at a free fingertip node (nothing
    prints at the fingertip). ground() with a carrier must therefore make the buttons deck
    nodes, load them along the keywell direction, and leave no fingertip node in the domain.
    The deck is a bolted joint standing off the dorsum: it must carry no tissue spring."""
    from structure.lattice import ground
    q = np.zeros(h50.model.nq)
    nodes, bars, btn, loads, ak, an, tris, strap_n, deck, deck_bars = ground(
        h50, q, pitch=0.006, carrier=carrier)
    kdn = carrier.keywell_deck_nodes()
    assert set(btn.values()) <= set(deck), "a button is not a deck bolt"
    for f in FINGERS:
        assert btn[f] == deck[kdn[f]]
        assert np.allclose(loads[btn[f]],
                           np.asarray(carrier.keywell_dir[f]) * carrier.press_N, atol=1e-12)
    assert not (set(deck) & set(ak)), "a tissue spring landed on the mount plate"
    # the plate is a connected grid: every deck node ties to the bolt component
    assert len(deck_bars) >= len(deck) - 1


def test_kit_path_refuses_until_measured(h50):
    """The ship-gate: carrier_from_kit must not run on unmeasured KIT_* — it names what is
    missing rather than optimising against a fabricated payload."""
    with pytest.raises(NotImplementedError) as ei:
        carrier_from_kit(h50, np.zeros(h50.model.nq))
    assert "KIT_MASS" in str(ei.value)


def test_grow_carries_the_payload(h50, carrier):
    """End-to-end: grow() with the carrier as payload produces a structure that passes the
    deflection gate — the grown shell holds the kit's keypresses and its mass. Coarse pitch so
    the test stays quick; final.py's fine grow is the shipping path."""
    from structure.lattice import grow
    from design.params import DEFLECTION_MAX
    q = np.zeros(h50.model.nq)
    *_, hist, _pc, _sh, _ls = grow(h50, q, pitch=0.006, carrier=carrier, plates=False)
    assert np.isfinite(hist[-1][1]), "grow did not converge with the carrier payload"
    assert hist[-1][1] <= float(DEFLECTION_MAX), (
        f"carrier grow fails the deflection gate: {hist[-1][1]*1e6:.0f} um")


def test_carrier_gauntlet_chassis_clears_hand_skin(h50):
    """The biomorphic carrier gauntlet chassis must float clear of the hand skin surface
    everywhere with >= 2.0 mm anatomical tissue standoff (VISION.md §5 gate)."""
    from manufacture.svalboard import build_all_svalboard_units
    from manufacture.carrier_gauntlet import build_organic_carrier_gauntlet
    from structure.lattice import skin
    from scipy.spatial import cKDTree

    q = np.zeros(h50.model.nq)
    V_skin, _F_skin, _L_skin = skin(h50, q, labels=True)
    tree_skin = cKDTree(np.asarray(V_skin))

    units = build_all_svalboard_units(h50, q)
    gauntlet = build_organic_carrier_gauntlet(h50, q, units)
    chassis = gauntlet["chassis"]

    d_chassis, _ = tree_skin.query(chassis.vertices)
    min_standoff = float(np.min(d_chassis))
    assert min_standoff >= 0.0020, (
        f"Carrier gauntlet chassis clips into hand skin: min standoff {min_standoff*1000:.2f} mm < 2.00 mm gate"
    )


def test_svalboard_button_plates_and_cradles_clear_hand_and_each_other(h50):
    """Svalboard 5-way button plates and cradles must not intersect hand skin at rest,
    and adjacent key clusters must have >= 3.0 mm mutual clearance."""
    from manufacture.svalboard import build_all_svalboard_units
    from structure.lattice import skin
    from scipy.spatial import cKDTree

    q = np.zeros(h50.model.nq)
    V_skin, _F_skin, _L_skin = skin(h50, q, labels=True)
    tree_skin = cKDTree(np.asarray(V_skin))

    units = build_all_svalboard_units(h50, q)
    fingers = ["thumb", "index", "middle", "ring", "little"]

    # 1. Check positive clearance to hand skin at rest
    for f in fingers:
        u = units[f]
        d_cradle, _ = tree_skin.query(u["cradle"].vertices)
        assert np.min(d_cradle) > 0.0001, f"{f} cradle intersects hand skin"
        for pname, pmesh in u["paddles"].items():
            d_p, _ = tree_skin.query(pmesh.vertices)
            assert np.min(d_p) > 0.0001, f"{f} [{pname}] paddle intersects hand skin"

    # 2. Check mutual clearance between adjacent finger key clusters
    for i in range(len(fingers)):
        f1 = fingers[i]
        u1 = units[f1]
        m1 = trimesh.util.concatenate([u1["pod"], u1["cradle"]] + list(u1["paddles"].values()))
        tree1 = cKDTree(m1.vertices)
        for j in range(i + 1, len(fingers)):
            f2 = fingers[j]
            u2 = units[f2]
            m2 = trimesh.util.concatenate([u2["pod"], u2["cradle"]] + list(u2["paddles"].values()))
            d_inter, _ = tree1.query(m2.vertices)
            min_inter = float(np.min(d_inter))
            assert min_inter >= 0.0030, (
                f"Svalboard clusters [{f1}] and [{f2}] collide: clearance {min_inter*1000:.2f} mm < 3.00 mm"
            )

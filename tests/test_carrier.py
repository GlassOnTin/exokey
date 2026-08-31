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

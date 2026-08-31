"""The carrier: the printed bracket that presents the Svalboard kit to the hand.

The 2026-08-29 reframe (VISION.md §1) makes the device a CARRIER for a purchased kit. The
optimiser still has to grow a structure, but the load is no longer five self-built wells —
it is one rigid cluster the gauntlet must hold, whose keywells sit at the fingertips and
whose mass hangs off the dorsum. This module turns that into boundary conditions the
existing ESO grow() can consume, WITHOUT inventing a kit dimension:

  * carrier_from_bracket() builds the carrier from OUR bracket GUESSes (CARRIER_* in
    design/params.py) — a mount plane standing off the dorsum, keywell towers at the
    fingertips, a mass budget. It runs NOW, so the optimiser has a load to grow against.
  * carrier_from_kit() builds the same geometry from the MEASURED kit (KIT_*), and stays
    gated behind payload.require_measured() — it refuses until the kit is out of the box.

Both return the same Carrier, so grow() cannot tell which it was given; the difference is
provenance, and it is the ship-gate that keeps them apart.

WHAT THE CARRIER IS, geometrically:

  deck      -- a rigid plane on the dorsum (the surface the kit's body bolts to), at
               CARRIER_STANDOFF above the dorsal skin, spanning the metacarpals. The
               structure must present this plane stiff and flat; the payload mass acts
               through it, so it is the load path's proximal end.
  keywells  -- one node per digit at the fingertip's reaction point (the same place the
               self-built well put its button: well centre + press*r). The keypress load
               enters here. A tower — grown by ESO — carries each back to the deck.
  mass      -- the payload weight at the deck's centre of mass, pulling DOWN in world -Z.
               The deck is cantilevered off the wrist anchor, so this is a moment the
               anchor+strap must carry, not just a force.

The probe (scripts/rigid_cluster_probe.py) established WHY the keywells are placed at the
fingertips rather than cast into one fixed cluster: the five pads' mutual geometry rescales
~24% across the 5th–95th population, so no single rigid well-to-well offset fits every hand.
The kit ships its own per-key adjusters; the carrier's job is to hold that adjuster workspace
over the dorsum, which is why the keywells track the model's OWN fingertips here.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

import design.params as params
from design.vector import action_dirs
from structure.frame import hand_axes
from hand.myohand import FINGERS
from manufacture.payload import require_measured


@dataclass(frozen=True)
class Carrier:
    """The kit-as-carried, in world coordinates, ready for grow().

    deck        (n,3) mount-plane node positions (the kit's bolt-down surface).
    deck_tie    indices into `deck` that must reach the tissue anchor (the load path's
                proximal end — the deck cannot float, it is held by the grown shell).
    keywells    {finger: (3,) reaction point} — the keypress enters here.
    keywell_dir {finger: (3,) unit} — the direction the digit presses (pad normal).
    press_N     the keypress force magnitude (N) applied along each keywell_dir.
    mass        payload mass (kg).
    com         (3,) centre of mass, world coords — where the weight acts.
    """

    deck: np.ndarray
    deck_tie: tuple[int, ...]
    keywells: dict
    keywell_dir: dict
    press_N: float
    mass: float
    com: np.ndarray

    def mass_case(self, g: float = 9.81) -> dict:
        """The payload weight as a {node: force} load, applied at the deck's CoM node.

        World -Z: a worn device's mass pulls toward the ground, and because the deck is
        cantilevered off the wrist anchor this loads the anchor in bending. grow() folds
        this into the ESO ranking like gravity_cases — it nudges growth toward load paths
        that also carry the kit's own weight; it is not the deflection gate."""
        i = self.deck_tie[0] if self.deck_tie else 0
        return {i: np.array([0.0, 0.0, -self.mass * float(g)])}


def _dorsal_deck(h, q, standoff: float, n_across: int = 3, n_along: int = 3):
    """A mount plane on the dorsum: origin at the capitate, spanning the metacarpal region,
    lifted `standoff` above the highest dorsal bearing point."""
    o, e_d, e_r, e_o = hand_axes(h, q)
    from structure.anchor import bearing_surface
    P, N, K, T = bearing_surface(h, q)
    P = np.asarray(P)
    # how high the dorsum reaches along e_o — the deck sits above the tallest bearing point
    hgt = float(np.max((P - o) @ e_o)) + float(standoff)
    # the metacarpal span, projected into the (radial, distal) plane
    mc = P[(P - o) @ e_d > 0.0]                      # distal of the capitate = metacarpals
    if len(mc) < 2:
        mc = P
    r_lo, r_hi = np.min((mc - o) @ e_r), np.max((mc - o) @ e_r)
    d_lo, d_hi = np.min((mc - o) @ e_d), np.max((mc - o) @ e_d)
    rs = np.linspace(r_lo, r_hi, n_across)
    ds = np.linspace(d_lo, d_hi, n_along)
    deck = np.array([o + rr * e_r + dd * e_d + hgt * e_o
                     for dd in ds for rr in rs])
    return deck, (o, e_d, e_r, e_o), (r_lo, r_hi, d_lo, d_hi)


def _carrier(h, q, standoff: float, tower: float, mass: float,
             com_d: float, com_r: float, press_N: float) -> Carrier:
    """Shared construction: deck + fingertip keywells + CoM, from explicit bracket numbers."""
    deck, (o, e_d, e_r, e_o), (r_lo, r_hi, d_lo, d_hi) = _dorsal_deck(h, q, standoff)

    # keywells at each digit's reaction point — the same place the self-built well put its
    # button (well centre + press*r), so the keypress enters where the finger actually pushes.
    from design.vector import well_channel
    keywells, keywell_dir = {}, {}
    for f in FINGERS:
        dist, prox, r = well_channel(h, q, f)
        click = action_dirs(h, q, f)["click"]
        keywells[f] = 0.5 * (np.asarray(dist) + np.asarray(prox)) + np.asarray(click) * r
        keywell_dir[f] = np.asarray(click, float)

    # centre of mass: a fraction along/across the deck footprint, lifted to the deck plane.
    com = (o + (d_lo + com_d * (d_hi - d_lo)) * e_d
              + (r_lo + com_r * (r_hi - r_lo)) * e_r
              + (float(np.max((deck - o) @ e_o))) * e_o)

    # the deck nodes nearest the wrist anchor are the tie-in — the load path's proximal end.
    # Tie the proximal row (smallest distal coordinate) so the deck is held, not floating.
    dcoord = (deck - o) @ e_d
    dmin = np.min(dcoord)
    deck_tie = tuple(int(i) for i in np.flatnonzero(dcoord <= dmin + 1e-9))

    return Carrier(deck=deck, deck_tie=deck_tie, keywells=keywells,
                   keywell_dir=keywell_dir, press_N=float(press_N),
                   mass=float(mass), com=np.asarray(com, float))


def carrier_from_bracket(h, q, press_N: float | None = None) -> Carrier:
    """The carrier built from OUR bracket GUESSes. Runs now — this is the load the optimiser
    grows against before the kit is measured. Every number is a disclosed GUESS (VISION.md),
    and none is a kit dimension: it describes the bracket we print, not the hardware we bought.
    """
    from design.vector import PRESS_N
    press_N = float(PRESS_N) if press_N is None else float(press_N)
    return _carrier(h, q,
                    standoff=float(params.CARRIER_STANDOFF),
                    tower=float(params.CARRIER_TOWER),
                    mass=float(params.CARRIER_MASS),
                    com_d=float(params.CARRIER_COM_D),
                    com_r=float(params.CARRIER_COM_R),
                    press_N=press_N)


def carrier_from_kit(h, q, press_N: float | None = None) -> Carrier:
    """The carrier built from the MEASURED kit. Refuses until KIT_* are measured — the ship-gate.

    When the kit is out of the box and KIT_* are SPEC, this replaces the bracket GUESSes with
    values DERIVED from the real hardware: standoff from KIT_ENV_T, mass from KIT_MASS, the
    deck footprint from KIT_ENV_W/L. Until then require_measured() names what is missing."""
    require_measured("carrier_from_kit")
    # Reached only once the kit is measured. The bracket geometry above is the template; the
    # measured kit supersedes the GUESSes. Deliberately not pre-filled with numbers that would
    # be fabricated until the calipers have been on the real part.
    raise NotImplementedError(
        "the kit params are measured — now derive the carrier geometry from them: standoff "
        "from KIT_ENV_T, mass from KIT_MASS, deck footprint from KIT_ENV_W/L, CoM from the "
        "weighed assembly. carrier_from_bracket() is the structural template; replace its "
        "GUESS arguments with DERIVED values off the measured KIT_*.")
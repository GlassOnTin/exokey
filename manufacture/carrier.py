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
    bolts: dict = None          # {finger: index into deck} -- each key's own bolt node

    def mass_case(self, deck_nodes, g: float = 9.81) -> dict:
        """The payload weight as a {node: force} load, spread over the deck's bolt pattern.

        World -Z: a worn device's mass pulls toward the ground, and because the deck is
        cantilevered off the wrist anchor this loads the anchor in bending. The weight is
        shared across the deck nodes nearest the CoM (the kit bolts down over its footprint,
        so the reaction distributes there) rather than dumped on one node -- otherwise only
        the column under that node carries mass and ESO deletes the rest, leaving the plate
        floating. grow() folds this into the ESO ranking like gravity_cases."""
        nodes = np.asarray(deck_nodes, float)
        w = self.mass * float(g) / len(nodes)
        return {int(i): np.array([0.0, 0.0, -w]) for i in range(len(nodes))}

    def keywell_deck_nodes(self) -> dict:
        """{finger: index into `deck`} -- the bolt point each key's tower bracket lands on.

        The kit's keys are rigid towers bolted to the deck, so a keypress reaction enters
        OUR structure at that key's OWN bolt node, not at the fingertip (nothing prints at
        the fingertip -- the kit's tower reaches up to the finger). This is the load path
        the carrier model must use: key -> tower (kit, rigid) -> deck bolt -> deck -> grown
        shell -> anchor. The tower's own compliance is the kit's contribution to the
        mushiness budget and is NOT modelled here; the deck must be stiff at the bolt.

        ⚠ ONE BOLT PER KEY. Taking the nearest GRID node instead double-books: five keywells
        projected onto a 3x3 grid collapse onto three nodes (measured 2026-08-31), and two
        keys sharing a bolt means one keypress load overwrites the other's -- the grow then
        designs against four keypresses, not five. _carrier() therefore adds each keywell's
        plane projection as a dedicated deck node, and `bolts` records it."""
        if self.bolts is not None:
            return dict(self.bolts)
        out = {}
        for f, p in self.keywells.items():
            d = np.linalg.norm(self.deck - np.asarray(p, float), axis=1)
            out[f] = int(np.argmin(d))
        return out


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

    # THE BOLTS: one dedicated deck node per key. The kit's tower base bolts to the plate
    # somewhere under its key; the bolt is where the keypress enters OUR structure. Taking the
    # nearest GRID node instead double-books -- five keywells onto a 3x3 grid collapse onto
    # three nodes (measured 2026-08-31) and one keypress load overwrites another's, so the
    # grow designs against four keypresses, not five. Project each keywell onto the deck plane
    # along its normal (the plane stays planar) and clamp into the footprint: the plate spans
    # the metacarpals, and a bolt past the distal edge would be a tower base over the fingers
    # themselves, which is where the plate cannot go. The clamp lands the bolts on the distal
    # edge, radially distinct -- the tower rises from the plate's edge toward the fingertip.
    hgt = float(np.max((deck - o) @ e_o))
    bolts = {}
    rows = []
    for f in FINGERS:
        p = np.asarray(keywells[f], float)
        rd = float(np.clip((p - o) @ e_r, r_lo, r_hi))
        dd = float(np.clip((p - o) @ e_d, d_lo, d_hi))
        b = o + rd * e_r + dd * e_d + hgt * e_o
        # a bolt that lands on an existing deck node IS that node -- adding a duplicate would
        # make a zero-length tie bar, and a zero-length bar is a singular stiffness matrix
        # (measured: the index keywell projects exactly onto the distal-radial grid corner).
        hit = np.flatnonzero(np.linalg.norm(deck - b, axis=1) < 1e-9)
        if len(hit):
            bolts[f] = int(hit[0])
        else:
            rows.append(b)
            bolts[f] = len(deck) + len(rows) - 1
    if rows:
        deck = np.vstack([deck, np.array(rows)])

    # centre of mass: a fraction along/across the deck footprint, lifted to the deck plane.
    com = (o + (d_lo + com_d * (d_hi - d_lo)) * e_d
              + (r_lo + com_r * (r_hi - r_lo)) * e_r
              + hgt * e_o)

    # the deck nodes nearest the wrist anchor are the tie-in — the load path's proximal end.
    # Tie the proximal row (smallest distal coordinate) so the deck is held, not floating.
    dcoord = (deck - o) @ e_d
    dmin = np.min(dcoord)
    deck_tie = tuple(int(i) for i in np.flatnonzero(dcoord <= dmin + 1e-9))

    return Carrier(deck=deck, deck_tie=deck_tie, keywells=keywells,
                   keywell_dir=keywell_dir, press_N=float(press_N),
                   mass=float(mass), com=np.asarray(com, float), bolts=bolts)


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
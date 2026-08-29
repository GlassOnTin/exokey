"""The Svalboard kit as a PAYLOAD: one rigid load the gauntlet carries.

The 2026-08-29 reframe (VISION.md §1) retires the self-built five-well cluster in
manufacture/mount.py. What replaces it is an interface to hardware that physically exists
but whose geometry we do not have: the kit's STEP files are customer-gated and no dimension
is published (checked 2026-08-29). Nothing here may invent a number. The kit's constants
live in design/params.py at 0.0 with Source.UNMEASURED, and every consumer of this module
refuses to run until they are measured off the delivered kit — the session laid out in
VISION.md §7 item 9: kitchen scale for mass, calipers for the cluster envelope and the
mount pitch, then the controller board.

The module's job until then is to make the interface VISIBLE and UNRUNNABLE, which is
itself a check on the reframe: if some old path still calls into the well-mount machinery
as if the kit were our own wells, it now fails loudly here rather than quietly optimising
against a payload nobody has.
"""
from __future__ import annotations

from dataclasses import dataclass

import design.params as params


@dataclass(frozen=True)
class Payload:
    """The kit as a rigid body, in its OWN mount-plane coordinates.

    The envelope is the axis-aligned box the carrier must clear around the kit, measured
    with the brackets screwed on (they are the mounting surface, so they belong in the
    payload, not in the gauntlet). Mapping this onto the dorsum — where the mount plane
    sits, in what orientation, with KIT_PITCH's holes landing on real anchor nodes — is
    gauntlet grow()'s job once the numbers exist; this type is what that seam passes.
    """

    mass: float
    envelope: tuple[float, float, float]  # (across digits, along digits, stack depth), m
    pitch: float  # mount-attachment pitch, m


def require_measured(what: str = "") -> None:
    """Refuse while any kit constant is still UNMEASURED (i.e. still 0).

    This is the reframe's enforcement point: until the kit arrives and is measured,
    anything that needs its geometry gets an exception naming exactly what is missing,
    not a placeholder number that would silently optimise against a made-up payload.
    """
    missing = [p.name for p in params.REGISTRY if p.source is params.Source.UNMEASURED]
    if missing:
        raise NotImplementedError(
            "the Svalboard kit has not been measured yet — waiting on: "
            + ", ".join(missing)
            + ". Measure it on arrival (VISION.md §7 item 9): kitchen scale for KIT_MASS, "
            "calipers for KIT_ENV_W/L/T and KIT_PITCH. Once every value is a real number, "
            "re-register it in design/params.py with Source.SPEC (or the honest source)."
            + (f" (called from: {what})" if what else "")
        )


def carrier_envelope() -> Payload:
    """The measured kit as a Payload, ready to be mapped onto the dorsum."""
    require_measured("carrier_envelope")
    raise NotImplementedError(
        "the kit params are measured — now build the Payload from them. This is the "
        "kit-arrival work session, deliberately not pre-written: what the function returns "
        "should be driven by what is actually in the box, not by what was guessed before it."
    )
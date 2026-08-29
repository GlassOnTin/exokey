"""Gates on the kit payload seam (the 2026-08-29 reframe).

The load is now a purchased Svalboard kit whose dimensions are unpublished (STEP files are
customer-gated). These gates hold the line two ways until the kit is measured:

  * nothing in the code may INVENT a kit dimension — every carrier consumer must refuse
    (NotImplementedError) while the KIT_* constants sit at 0 with Source.UNMEASURED;
  * the refusal mechanism itself cannot rot — the UNMEASURED constants stay disclosed in
    VISION.md (the same tripwire the GUESSes get) and the registry cannot quietly drop them.
"""
from __future__ import annotations

import pytest

from design.params import REGISTRY, Source
from manufacture.payload import Payload, carrier_envelope, require_measured


def _kit_params():
    return [p for p in REGISTRY if p.name.startswith("KIT_")]


def test_kit_constants_are_registered_and_still_unmeasured():
    """The payload seam must exist — and must still be honest about being unmeasured."""
    names = {p.name for p in _kit_params()}
    assert {"KIT_MASS", "KIT_ENV_W", "KIT_ENV_L", "KIT_ENV_T", "KIT_PITCH"} <= names
    for p in _kit_params():
        assert p.source is Source.UNMEASURED, (
            f"{p.name} moved source to {p.source.name} — if the kit HAS been measured, "
            "its value must be the real measurement (and this gate updated to pin it), "
            "not a placeholder"
        )
        assert p.value == 0.0, f"{p.name} is unmeasured but no longer 0 — that is fabricated data"


def test_unmeasured_kit_disclosed_in_vision():
    """Mirror of test_no_undeclared_guesses, for the UNMEASURED source."""
    vision = open("VISION.md").read()
    missing = [p.name for p in REGISTRY if p.source is Source.UNMEASURED
               and p.name.lower().replace("_", " ") not in vision.lower()
               and p.name not in vision]
    assert not missing, f"UNMEASURED kit params not disclosed in VISION.md: {missing}"


def test_carrier_refuses_while_the_kit_is_unmeasured():
    with pytest.raises(NotImplementedError) as ei:
        carrier_envelope()
    msg = str(ei.value)
    assert "KIT_MASS" in msg and "KIT_PITCH" in msg, "refusal must name the missing numbers"


def test_require_measured_passes_once_everything_is_spec():
    """The guard's escape hatch is real: with no UNMEASURED params it opens."""
    from design import params

    saved = list(params.REGISTRY)
    try:
        params.REGISTRY[:] = [p for p in params.REGISTRY
                              if p.source is not Source.UNMEASURED]
        require_measured()  # must NOT raise
    finally:
        params.REGISTRY[:] = saved
    # and Payload itself stays constructible from plain measured numbers
    p = Payload(mass=0.1, envelope=(0.1, 0.05, 0.02), pitch=0.015)
    assert p.envelope[0] == 0.1


def test_payload_documented_in_vision_reframe():
    """The reframe text §1 must actually point at the payload seam, or the docs drift."""
    vision = open("VISION.md").read()
    assert "Svalboard kit" in vision
    assert "§7 item 9" in vision or "item 9" in vision
"""Every constant, with its PROVENANCE. And a tripwire for the ones that lie.

WHY THIS EXISTS. Every expensive bug in this project has been the same bug: a constant that
encoded an assumption, with no record of where it came from, no check that it was still
valid, and no alarm when the architecture changed underneath it.

    KEY_PITCH = 12 mm        a KEYCAP pitch, carried silently into the WELL era.
                             Invalidated a whole Pareto front -- the wells overlapped.
    SWITCH_TRAVEL = 3 mm     described a Cherry MX...
    PRESS_N = 0.30 N         ...while this described a dome switch. Two different switches.
    cap radius = PITCH/2     a PACKING number used to answer a CLEARANCE question.
    press_N, body_prox       corner solutions: variables pretending to be decisions.
    common_drive = 0.15      a guess, and it is currently blocking the well layout.

Meanwhile every fix that has NEVER broken came from the same move: stop declaring, start
DERIVING. Flexion direction from the flexor's moment arm. Palmar direction from the tendon
insertions. Bone radius from the mesh. Well radius from the fingertip. A derived quantity
cannot drift away from the thing it describes, because it IS the thing it describes.

So: every constant is tagged, and:

  * DERIVED  -- computed from the model. Preferred. Cannot go stale.
  * SPEC     -- a vendor's published number. Cite it.
  * LITERATURE -- a published figure. Cite it.
  * GUESS    -- we made it up. These are enumerated by test_no_undeclared_guesses and they
                MUST appear in VISION.md's limitations, because a guess that nobody knows is
                a guess is indistinguishable from a fact.

And parameters that describe ONE PHYSICAL THING live together (see Switch, Well), so they
cannot quietly come to describe two different things.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum


class Source(Enum):
    DERIVED = "derived from the model"
    SPEC = "vendor specification"
    LITERATURE = "published figure"
    GUESS = "made up — must be listed in VISION.md limitations"


@dataclass(frozen=True)
class Param:
    name: str
    value: float
    unit: str
    source: Source
    why: str
    describes: str = ""  # the physical thing; params describing one thing must agree

    def __float__(self) -> float:
        return self.value


REGISTRY: list[Param] = []


def P(name, value, unit, source, why, describes="") -> Param:
    p = Param(name, value, unit, source, why, describes)
    REGISTRY.append(p)
    return p


# ---------------------------------------------------------------------------------------
# THE SWITCH. Force and travel describe ONE piece of hardware and are declared together,
# because when they were separate constants they came to describe two different switches
# (a 3 mm Cherry MX travel against a 0.30 N dome force) and that inconsistency alone made
# 2-keys-per-finger look infeasible.
# ---------------------------------------------------------------------------------------
@dataclass(frozen=True)
class Switch:
    force: Param
    travel: Param
    describes: str


SVALBOARD = Switch(
    describes="Svalboard magneto-optical key",
    force=P("PRESS_N", 0.196, "N", Source.SPEC,
            "20 gf, svalboard.com. Light, front-loaded, no spring.",
            describes="Svalboard magneto-optical key"),
    travel=P("SWITCH_TRAVEL", 0.0015, "m", Source.SPEC,
             "'a few mm for any keypress', svalboard.com. Consistent with a 20 gf key.",
             describes="Svalboard magneto-optical key"),
)


# ---------------------------------------------------------------------------------------
# THE WELL. Its radius is DERIVED per finger from that finger's own fingertip -- it is a
# cavity the fingertip sits inside, so it cannot be a constant, and it certainly cannot be
# the 12 mm keycap pitch it was inherited from.
# ---------------------------------------------------------------------------------------
WELL_WALL = P("WELL_WALL", 0.0015, "m", Source.GUESS,
              "wall thickness between adjacent wells. Plausible for a printed shell; not "
              "checked against a print.",
              describes="finger well")

# ---------------------------------------------------------------------------------------
# The remaining GUESSES. Each one is a place this model could be wrong, and each is listed
# in VISION.md section 6 because of it.
# ---------------------------------------------------------------------------------------
COMMON_DRIVE = P(
    "COMMON_DRIVE", 0.15, "fraction of flexion range", Source.GUESS,
    "How differently two neighbouring fingers may curl. A stand-in for ENSLAVEMENT, which "
    "MyoHand does not model at all (its FDP2-FDP5 are strictly independent). The number is "
    "made up. It is currently the binding constraint on the well layout -- a guess is "
    "deciding the design, which is exactly why it must be visible.",
    describes="finger independence")

COLUMN_SHIFT_COST = P(
    "COLUMN_SHIFT_COST", 5e-6, "sum a^3", Source.GUESS,
    "Cost of translating the whole hand to reach the index's second column. Not a finger "
    "action, so it is charged as a flat adder rather than modelled.",
    describes="hand translation")

ADJUSTER_MASS = P(
    "ADJUSTER_MASS", 0.15, "g per mm of travel", Source.GUESS,
    "Mass of a per-finger slide-and-lock adjuster. Not from any real mechanism.",
    describes="well adjuster")

SOFT_TISSUE_K = P(
    "SOFT_TISSUE_K", 25e3, "N/m", Source.LITERATURE,
    "Palm/dorsum contact stiffness, midpoint of a 10-50 N/mm band. Poorly characterised: "
    "swept, and the deflection answer moves 1.40x across the band.",
    describes="soft tissue")

RESIDUAL_MAX = P(
    "RESIDUAL_MAX", 0.05, "fraction of required joint torque", Source.GUESS,
    "How much of the required joint torque the muscles are allowed to FAIL to produce. "
    "Ideally zero: a digit that cannot balance the key reaction cannot press the key. It is "
    "not zero only because a hard equality would be brittle against solver tolerance. "
    "MEASURED at 0.05 by nothing -- it is a tolerance, and the SENSITIVITY to it must be "
    "reported, because the whole action set depends on where this line is drawn.",
    describes="muscle equilibrium")

DEFLECTION_MAX = P(
    "DEFLECTION_MAX", 0.5e-3, "m", Source.GUESS,
    "Above this a key feels mushy. A judgement, not a measurement.",
    describes="key feel")


def guesses() -> list[Param]:
    """Everything we made up. These belong in VISION.md, every one of them."""
    return [p for p in REGISTRY if p.source is Source.GUESS]


def audit() -> str:
    lines = ["parameter provenance:", ""]
    for src in Source:
        ps = [p for p in REGISTRY if p.source is src]
        if not ps:
            continue
        lines.append(f"  {src.name} ({len(ps)}) — {src.value}")
        for p in ps:
            lines.append(f"    {p.name:20s} {p.value:>10.4g} {p.unit:<28s} {p.why[:60]}")
        lines.append("")
    return "\n".join(lines)


def check_coherent(switch: Switch) -> None:
    """Parameters describing ONE physical thing must actually describe the same thing."""
    if switch.force.describes != switch.travel.describes:
        raise ValueError(
            f"{switch.force.name} describes '{switch.force.describes}' but "
            f"{switch.travel.name} describes '{switch.travel.describes}' — "
            "these are two different switches"
        )

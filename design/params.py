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
  * UNMEASURED -- not yet known. A 0 here is a FACT (nothing has been measured), not a guess.
                Added by the 2026-08-29 reframe for the Svalboard kit payload: the kit is
                purchased hardware, its STEP files are customer-gated, and no dimension is
                published anywhere, so its constants can only be measured out of the
                delivered kit. Every consumer must REFUSE to run until then
                (manufacture/payload.require_measured); test_no_undeclared_unmeasured
                applies the same VISION.md disclosure tripwire the GUESSes get.

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
    UNMEASURED = "not yet known (value 0) — measured from the delivered kit; consumers refuse until then"


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
# THE READ-OUT. A disc magnet on the moving cradle over a fixed 3-axis Hall (manufacture.
# readout). The magnet size and rest gap are declared together with the sensor's own
# resolution and range, because the design point is a RELATION between them: the full-travel
# field swing must dwarf the sensor's noise/LSB while the rest field stays inside its range.
# Split them into unrelated constants and that relation can silently break. (SI throughout:
# fields in tesla.)
# ---------------------------------------------------------------------------------------
MAGNET_BR = P("MAGNET_BR", 1.29, "T", Source.SPEC,
              "N42 sintered NdFeB remanence, ~1.28-1.32 T (e.g. supermagnete S-03-01-N). "
              "Grade sets the field; N42 is a common, unremarkable stock disc.",
              describes="cradle magnet")
MAGNET_D = P("MAGNET_D", 0.003, "m", Source.SPEC,
             "Ø3 mm off-the-shelf disc. Diameter chosen (scripts/readout.py sweep) so the "
             "1.5 mm plunge swing clears the Hall noise by >100 LSB while the rest field "
             "stays under the sensor's range.",
             describes="cradle magnet")
MAGNET_L = P("MAGNET_L", 0.001, "m", Source.SPEC,
             "1 mm thick disc (S-03-01-N). Thickness trades field for mass; 1 mm is enough "
             "at a 3.5 mm gap.",
             describes="cradle magnet")

# The Hall. Typical Infineon TLx493D-family figures. The ordered part is the TLV493D-A1B6
# (TLV493DA1B6HTSA2, TSOP-6) -- the W2BW plan died because that part only ships as a
# wafer-level BGA (docs/electronics.md, addressing fork). Same family, same 12-bit 0.098 mT
# LSB and +-130 mT range; its noise sits in the same 0.1-0.4 mT band HALL_NOISE sweeps.
HALL_LSB = P("HALL_LSB", 0.098e-3, "T", Source.SPEC,
             "0.098 mT per LSB, Infineon TLV493D/TLI493D 12-bit. The smallest field step "
             "the sensor can report.",
             describes="Hall sensor")
HALL_RANGE = P("HALL_RANGE", 130e-3, "T", Source.SPEC,
               "+-130 mT full scale, Infineon TLx493D. Beyond this the reading clips, so the "
               "rest AND hard-stop fields must both sit inside it.",
               describes="Hall sensor")
HALL_NOISE = P("HALL_NOISE", 0.2e-3, "T", Source.SPEC,
               "~0.2 mT RMS, Infineon TLx493D (datasheet band 0.1-0.4 depending on mode); "
               "the mid value, swept in tests. The floor each direction's signal must clear.",
               describes="Hall sensor")

REST_GAP = P("REST_GAP", 0.0035, "m", Source.GUESS,
             "Magnet rest face to Hall sensing point. Chosen so the modelled rest field "
             "(~19 mT) sits low-mid range and the full plunge (~61 mT) still clears it by "
             "hundreds of LSB. A frame dimension, not yet confirmed on a print.",
             describes="magnetic read-out")

CRADLE_LEVER = P("CRADLE_LEVER", 0.7, "mm lateral per mm travel", Source.GUESS,
                 "How far the magnet translates sideways per mm of fingertip tilt travel. "
                 "Sets the transverse field a lateral direction presents to the Hall. A "
                 "geometry guess until a stage-1 coupon measures the cradle's real lever.",
                 describes="cradle lever")

EARTH_B = P("EARTH_B", 0.05e-3, "T", Source.LITERATURE,
            "Earth's field magnitude ~50 uT at the surface (NOAA/IGRF). A static per-"
            "orientation offset the baseline tracker removes; sanity bound on the read-out.",
            describes="ambient field")


# ---------------------------------------------------------------------------------------
# THE WELL. Its radius is DERIVED per finger from that finger's own fingertip -- it is a
# cavity the fingertip sits inside, so it cannot be a constant, and it certainly cannot be
# the 12 mm keycap pitch it was inherited from.
# ---------------------------------------------------------------------------------------
WELL_WALL = P("WELL_WALL", 0.0026, "m", Source.DERIVED,
              "half the material the packing constraint must leave between two cups: it is the "
              "actual PRINTED cup wall, manufacture.mount.CUP_WALL (2.2 mm flank) + SEAT_CLEAR "
              "(0.4 mm finger-slide gap) = 2.6 mm. It was a GUESS of 1.5 mm, 'not checked against "
              "a print' -- and it was wrong: key_separation requires cc-gap >= r_a + r_b + "
              "2*WELL_WALL, so 1.5 under-modelled each cup wall by 1.1 mm and the GA packed the "
              "central drop-in cradles into OVERLAP (measured: middle-ring 0.9 mm on the "
              "otherwise-feasible seed-1 winner, 3.6 mm on the old shipped design). The cradles "
              "are independent moving parts, so overlap is interference. Re-derive if CUP_WALL or "
              "SEAT_CLEAR change. Ample room to satisfy it: the central cups can spread to a "
              "52.6 mm gap against the 23.7 mm this now requires.",
              describes="finger well")

# ---------------------------------------------------------------------------------------
# The remaining GUESSES. Each one is a place this model could be wrong, and each is listed
# in VISION.md section 6 because of it.
# ---------------------------------------------------------------------------------------
COMMON_DRIVE = P(
    "COMMON_DRIVE", 0.15, "fraction of flexion range", Source.GUESS,
    "How differently two neighbouring fingers may curl. A stand-in for ENSLAVEMENT, which "
    "MyoHand does not model at all (its FDP2-FDP5 are strictly independent). The number is "
    "made up. Retained only as a loose TOTAL-spread re-check; the binding limit is now the "
    "per-finger INDIVIDUATION below, so this guess no longer decides the design.",
    describes="total finger spread")

# ENSLAVEMENT, GROUNDED PER FINGER. The four long-finger flexors share a muscle belly, so a digit
# cannot be posed independently -- and the RING is the LEAST independent. MyoHand models FDP2-FDP5 as
# INDEPENDENT actuators (no shared belly), so nothing stopped the optimiser posing the ring extended
# while its neighbours flexed: a posture no hand can hold. Measured on the winning design -- it raised
# the ring 9 mm, and it LEANED on it: clamp the ring to the common drive and effort jumps +33% and the
# key layout goes infeasible (keys overlap). The OpenSim hand-and-wrist models get enslavement right by
# driving the FDP's four finger-slips from ONE activation; we express that coupling KINEMATICALLY, per
# digit, by the INDIVIDUATION INDEX -- how independently each finger can move -- so the ring's curl may
# deviate from the common drive far LESS than the index's. This REPLACES the single symmetric
# COMMON_DRIVE/2 = 0.075 that applied to every finger alike.
#
# ⚠ The ORDERING is robust (index & little most independent, ring least -- Hager-Ross & Schieber 2000,
# J Neurosci 20:8542 "Quantifying the independence of human finger movements"). The exact half-ranges
# are that ordering scaled so the INDEX keeps the old 0.075; the ring at ~0.5x it. A measured enslaving
# MATRIX would refine these, and is the next real measurement (VISION §6).
_INDIV = "how far this finger's curl may deviate from the common drive -- its individuation, ring least"
INDIVIDUATION = {
    "index":  P("INDIV_index",  0.075, "curl-fraction half-range", Source.LITERATURE, _INDIV,
                describes="per-finger individuation"),
    "little": P("INDIV_little", 0.060, "curl-fraction half-range", Source.LITERATURE, _INDIV,
                describes="per-finger individuation"),
    "middle": P("INDIV_middle", 0.050, "curl-fraction half-range", Source.LITERATURE, _INDIV,
                describes="per-finger individuation"),
    "ring":   P("INDIV_ring",   0.035, "curl-fraction half-range", Source.LITERATURE, _INDIV,
                describes="per-finger individuation"),
}

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

SPACE_FREQ = P(
    "SPACE_FREQ", 18.0, "keystrokes per 100 letters", Source.LITERATURE,
    "English averages ~4.5 letters per word, so ~18-20 spaces per 100 letters. Standard "
    "figure. Load-bearing: the left hand's 15 QWERTY letters are only 58.7 of those 100, so "
    "SPACE IS ~22% OF THE LEFT HAND'S ENTIRE KEYSTROKE LOAD -- bigger than any letter. It was "
    "not in the objective at all until the thumb could press.",
    describes="keystroke frequency")

SHIFT_FREQ = P(
    "SHIFT_FREQ", 4.0, "keystrokes per 100 letters", Source.GUESS,
    "Capitals and punctuation needing the LEFT shift. Rougher than SPACE_FREQ. It decides "
    "whether a pointer fits: with shift on a well, the mouse costs one slot more than we "
    "have; move it to a hold/chord and the mouse fits.",
    describes="keystroke frequency")

DROOP_MAX = P(
    "DROOP_MAX", 0.010, "m", Source.DERIVED,
    "How far below the index/middle/ring fingertip plane the LITTLE fingertip may sit at the "
    "design posture. MEASURED ON THE USER'S HAND (docs/IMG20260819142422.jpg): comfortably "
    "splayed, every fingertip rests on the table plane -- droop ~0, WITH splay to spare "
    "(adjacent tip gaps ~22/24/35 mm vs the 23.7 the cups need). The seed-1 knee held the "
    "little tip 19.4 mm below the plane -- a posture the model's joint ranges permit and a "
    "real 5th ray does not, which is why the design 'looked like a hand bent 90 deg at the "
    "wrist'. The model cannot represent 0: its own relaxed pose droops 8.8 mm and its floor "
    "at full splay is 7.9 mm (a kinematic offset of MyoHand's 5th ray, not anatomy), so the "
    "bound is that floor plus ~2 mm of representability margin, not the user's true 0. The "
    "model is being held to the best coplanarity it can express; the residual ~8 mm is a "
    "known model artifact to re-examine if a printed device still feels dropped.",
    describes="posture comfort")

DEFLECTION_MAX = P(
    "DEFLECTION_MAX", 0.5e-3, "m", Source.GUESS,
    "Above this a key feels mushy. A judgement, not a measurement.",
    describes="key feel")

DON_CLEAR = P(
    "DON_CLEAR", 0.002, "m", Source.GUESS,
    "The room a finger needs BESIDE it, all the way down its donning corridor, over and above "
    "not intersecting the part. ⚠ THE BUG THIS EXISTS TO KILL (2026-08-22): the entry constraint "
    "demanded only `gap >= -TOUCH_TOL` -- i.e. the gauntlet may not push more than 0.3 mm INTO "
    "the finger. That is a no-interference test, not a fit test, and a mass-minimising optimiser "
    "parks every corridor exactly on it: the shipped design cleared by 0.07-0.72 mm, which the "
    "model called feasible and a hand called jammed. A real donning needs slack for FDM tolerance "
    "(~0.2 mm), for a hand that cannot be perfectly aligned on the way in, and for a knuckle that "
    "is bone, not compressible pulp. 2 mm per side is an estimate, NOT a measurement -- the honest "
    "way to set it is to print a corridor coupon and find where a hand stops binding. Note the "
    "binding element is the CUP WALLS, not the struts (measured: mounts 0.07-0.72 mm vs struts "
    "1.0-2.9 mm), so this constraint is paid for by SPREADING THE WELLS, which is a layout change "
    "and therefore the optimiser's job -- a strut keep-out cannot buy it.",
    describes="donning fit")

CRADLE_CLEAR = P(
    "CRADLE_CLEAR", 0.0015, "m", Source.GUESS,
    "The room the DROP-IN TPU CRADLE needs around it, clear of the grown structure. ⚠ The cradle "
    "is the MOVING part -- the fingertip tilts it over the Hall sensor, and a key that cannot move "
    "cannot be read. Nothing reserved its envelope: the structure was checked against the finger "
    "and against the rigid frame, never against the part that has to travel, and the export carves "
    "only the PCB slot and magnet pocket. Measured on the shipped design, three of five cradles "
    "were compromised -- little BLOCKED by a strut 2.10 mm inside it, thumb and ring clear by only "
    "0.26/0.27 mm, which FDM tolerance (~0.2 mm) closes. 1.5 mm is SWITCH_TRAVEL's 1.5 mm plunge "
    "plus nothing: it is a plausible swing envelope, NOT a measurement of the cradle's real motion "
    "(CRADLE_LEVER, itself a guess, puts the lateral component near 1 mm). Re-derive from a printed "
    "cradle's measured travel.",
    describes="cradle clearance")

DON_LEN = P(
    "DON_LEN", 0.080, "m", Source.GUESS,
    "How far back down the donning path the corridor is checked. ⚠ THE BUG THIS EXISTS TO KILL "
    "(2026-08-22): entry_sweep swept a fixed 20 mm, so the model inspected only the last stretch of "
    "insertion -- where a seated design is clear by construction -- and never looked at the 20-70 mm "
    "stretch the hand actually has to travel through. Measured on the shipped design, that unchecked "
    "region is obstructed for EVERY finger (-0.03 to -1.10 mm) while the checked 20 mm read +0.4 to "
    "+1.7 mm: the model called the device donnable and the hand could not get into it. 80 mm is where "
    "the profile goes clear on this design, NOT a measurement of a real hand's approach -- it is a "
    "geometry guess, and a longer real path would need more. Cost scales with it, so the cloud is "
    "strided (see entry.CORRIDOR_STRIDE).",
    describes="donning fit")

HANDLING_N = P(
    "HANDLING_N", 10.0, "N", Source.GUESS,
    "The lateral force every button mount must SURVIVE (yield/SF2), applied at the button node "
    "in the worst lateral direction. The first print taught this the hard way (2026-08-21): the "
    "whole robustness story was the 0.196 N keypress gate, so ESO pruned the thumb cup's support "
    "to material that was crisp along the press and snapped during the first donning -- a single "
    "1.8 mm CF-PA12 rod yields at ~2-3 N applied at cup distance, and a hand fighting a jammed "
    "corridor delivers far more. 10 N is a firm-handling estimate (with SF 2, ~20 N ultimate), "
    "not a measurement; raise it if the next print still feels fragile, and NEVER let it into "
    "the deflection gate -- a cup may flex under abuse, it may not break.",
    describes="robustness")


THUMB_CMC = P(
    "THUMB_CMC", 0.299, "-", Source.DERIVED,
    "The thumb's CMC flexion, as a fraction of its range: the value MyoHand's own q_neutral sits "
    "at, which is the CLINICAL POSITION OF FUNCTION (derived at Stage 0) -- the posture a resting, "
    "opposed thumb actually holds. Now the DEFAULT when `tp_thumb` is absent from a design dict, "
    "not a fix. "
    "⚠ IT WAS A FIX, AND THE RETIREMENT NO LONGER HOLDS. The evidence was: swept 0.02 to 0.80, "
    "effort/character moves 0.3% and mass is flat below 0.45 (26-27 g, inside ESO's own +-15-30% "
    "trajectory noise), so it is a DEAD VARIABLE. Both halves were measured on a hand with "
    "MyoHand's slim 13 mm thumb well. At the MEASURED 25 mm well the mass half is simply false: "
    "inside the feasible window the same sweep moves the gauntlet 33.6 -> 38.7 g, ~15% of "
    "objective 2, and at (tp .3, tm .2) the cheap end breaks `thumb-opposed`. A live trade, so it "
    "is a variable again (design/vector.REAL_BOUNDS). "
    "The geometric argument for restoring it is the seductive one and it is WRONG -- tp_thumb does "
    "move the newly-binding ('thumb','index') gap by 9.5 mm, but only at mid-curl, which "
    "four-finger packing already excludes; freeing it leaves the pack-and-press window at the same "
    "5 of 81 curl cells. Measured, not assumed, in both directions.")


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


# ---------------------------------------------------------------------------------------
# THE KIT PAYLOAD — the 2026-08-29 reframe (VISION.md §1). This structure's load is now a
# purchased Svalboard kit, and NOTHING about it is published: the cluster STEP files are
# customer-gated and no dimension appears in any public Svalboard material (checked
# 2026-08-29). So these are registered at 0 with source UNMEASURED — a 0 that says "unknown",
# deliberately distinct from a GUESS, which says "we made it up". No value here may be
# invented: they are measured out of the delivered kit (scale, calipers — the session laid
# out in VISION.md §7 item 9), and until then every consumer refuses. See
# manufacture/payload.py, which turns them into the carrier's boundary conditions.
#
# Names follow the VISION.md §7 item 9 measurement list: mass, cluster envelope
# (across digits / along digits / stack depth) and mount pitch. All in SI.
# ---------------------------------------------------------------------------------------
KIT_MASS = P("KIT_MASS", 0.0, "kg", Source.UNMEASURED,
             "Total payload mass carried on the dorsum: printed cluster parts + PCBAs + "
             "magnets + tower brackets, per what the gauntlet actually mounts. To be "
             "measured on a kitchen scale as the assembled carrier-side kit weighs in "
             "(VISION.md §7 item 9). Drives the gauntlet's static load case.",)

KIT_ENV_W = P("KIT_ENV_W", 0.0, "m", Source.UNMEASURED,
              "Keywell-cluster envelope ACROSS the digits (thumb..little). Calipers on the "
              "delivered cluster. Bounds where the carrier plane can sit without fouling "
              "the strap's donning corridor.")

KIT_ENV_L = P("KIT_ENV_L", 0.0, "m", Source.UNMEASURED,
              "Keywell-cluster envelope ALONG the digits (proximal-distal). Calipers on the "
              "delivered cluster. Sets how far the payload lever arm reaches onto the "
              "fingers for the keypress + mass moment cases.")

KIT_ENV_T = P("KIT_ENV_T", 0.0, "m", Source.UNMEASURED,
              "Stack depth of the payload: well block + stainless tower bracket + any "
              "backing plate the carrier must clear. Calipers. Sets the gauntlet's standoff "
              "budget against LAYER's ~10 mm wearable limit (§6).")

KIT_PITCH = P("KIT_PITCH", 0.0, "m", Source.UNMEASURED,
              "Mount-attachment pitch: spacing of the holes/features the gauntlet actually "
              "bolts to on the kit side (tower brackets). Calipers on the delivered "
              "brackets. This is the hard interface: mount point positions must be DERIVED "
              "from KIT_PITCH, never fitted to the structure.")

# ---------------------------------------------------------------------------------------
# THE CARRIER BRACKET — OUR part, so its geometry is a DESIGN CHOICE, not a kit dimension.
#
# The KIT_* numbers above describe the purchased hardware and stay UNMEASURED (a 0 that
# says "unknown"); manufacture/payload.require_measured() refuses to SHIP on them. But the
# optimiser can still run NOW, against a carrier we declare rather than a kit we invent:
# these describe the printed bracket — how high its mount plane stands off the dorsum, how
# tall the keywell towers are, what mass we budget the kit at — and every one is a GUESS,
# disclosed in VISION.md's limitations like every other. The distinction is the whole point:
# "our mount plane stands 12 mm off the dorsum" is a decision we are allowed to make; "the
# kit's cluster is 62 mm wide" would be a fabricated kit dimension. When the kit is measured
# these GUESSes are replaced by DERIVED values computed from the real KIT_* — manufacture/
# carrier.py::carrier_from_kit() is that path, and it stays gated behind require_measured().
# ---------------------------------------------------------------------------------------
CARRIER_STANDOFF = P("CARRIER_STANDOFF", 0.012, "m", Source.GUESS,
                     "Height of the carrier's mount plane above the dorsal skin — the deck "
                     "the kit bolts to. A GUESS: our bracket's choice, not a kit dimension. "
                     "Sits inside the ~10 mm wearable stack budget (§6) once the grown shell "
                     "is counted; replaced by a DERIVED value off the measured KIT_ENV_T.")

CARRIER_TOWER = P("CARRIER_TOWER", 0.010, "m", Source.GUESS,
                  "How far each keywell sits above the mount plane — the tower the key rides "
                  "on, so the fingertip reaches its well from the typing posture. A GUESS at "
                  "our bracket; the kit's own tower bracket height supersedes it on arrival.")

CARRIER_MASS = P("CARRIER_MASS", 0.080, "kg", Source.GUESS,
                 "Mass budget the carrier is sized to carry, for the static load case. A "
                 "GUESS bracket value (80 g — a Svalboard cluster is hand-held light); the "
                 "real number is KIT_MASS off the kitchen scale, and shipping uses that.")

CARRIER_COM_D = P("CARRIER_COM_D", 0.55, "-", Source.GUESS,
                  "Payload centre of mass along the cluster footprint, as a fraction of the "
                  "mount span from its proximal edge (0=proximal, 1=distal). A GUESS: the "
                  "keywells sit toward the distal end of the deck, so the mass moment loads "
                  "the wrist anchor. Replaced by the measured CoM on arrival.")

CARRIER_COM_R = P("CARRIER_COM_R", 0.50, "-", Source.GUESS,
                  "Payload centre of mass across the cluster footprint, as a fraction of the "
                  "mount span from the little-finger edge (0=little, 1=index). A GUESS at "
                  "~centre; the thumb-key mass pulls it index-ward on the real kit.")

# Timestamp — proof of prior-art publication date

This repository contains a **defensive publication** ([VISION.md § 8](VISION.md#8-disclosed-variants-defensive-publication)).
Prior art is only worth anything if its **date is provable to a third party**.

**Git commit dates are worthless for this.** They are set by the committer, trivially forged,
and rewritten by any rebase. So the disclosure is anchored independently.

## What is anchored

The disclosure has been **extended and re-anchored** thirty-nine times. **All forty stamps stand**, and each
one proves what was disclosed *at that moment*. An earlier proof is not invalidated by a later one —
it is a *floor* on the date, and floors do not move.

### Current — THE KEYS ARE PURCHASED NOW

2026-08-29. A Svalboard kit was ordered, and the project is re-scoped: ExoKey stops being a
self-built keyboard and becomes the **support structure that carries Svalboard's hardware** —
the dorsal gauntlet, its tissue-measured anchors, the strap, the impact load case, the
human-factors floor, and the one-printer constraint. The wells, the Hall read-out (§8.15l),
the optimised layout (§3b) and the percentile well travel retire as design obligations; the
fingertip caliper measurements stay, because they now govern strap and anchor fit instead of
well width.

Two existing claims are marked inside their own scope in VISION.md §1 and §3 rather than
deleted, because both were measured: "an exoskeleton cannot reach" was already withdrawn in
§5d for the open hand, and the strap-body-vs-exoskeleton table was measured on the ~60 g
self-built device, whose payload decision a purchased kit supersedes. What the new load
actually needs is not yet measured — kit mass, centre of mass and mount geometry arrive with
the kit. This anchor records the scope change and the retraction boundary the same day it was
made.

**Both files re-stamped** (114 files).

| file | sha256 |
|---|---|
| `VISION.md` (the disclosure) — *re-stamped: records the 2026-08-29 reframe* | `f1ea7eee080cceac0e754d1eb97b2b9806da0931ab8e3876653198ba156d4aff` |
| `MANIFEST.sha256` (hashes of all 114 source + doc files) | `5204377ef3ca7eaa12a4648bfe20506024f076ebf901648ec6c2b79c0623a718` |

Stamped: **2026-08-29T11:11:31Z** (UTC, submission time). Proofs: `MANIFEST.sha256.ots` and
`VISION.md.ots`, both freshly stamped. The outgoing 39th proofs are archived at
`timestamps/MANIFEST.sha256.2026-08-24c.ots` and `timestamps/VISION.md.2026-08-24c.ots` (run
`ots upgrade` once the block mines).

### Thirty-ninth — THE MOVING PART HAS TO BE ABLE TO MOVE

The user, looking at the render: *"I'm concerned that there are some struts dropping down onto the
index and middle cup bases."* Measured, that instinct led straight past what it pointed at to
something worse next door.

Struts fuse **3.6-4.1 mm into all five RIGID cup frames**, and that is intended -- the button node
sits on the mount and its strut has to tie in. But nothing had ever checked the structure against
the **DROP-IN TPU CRADLE**, the part the fingertip tilts over the Hall sensor, and a key that
cannot move cannot be read. The export carves only the PCB slot and the magnet pocket, so the STL
reserved no space for it either. Measured with export-accurate geometry (`cluster_mount` for the
four long fingers, as `export_stl` builds them): thumb **+0.26**, index +1.65, middle +1.84, ring
**+0.27**, little **-2.10 mm**. The little finger's cradle was blocked outright; thumb and ring sat
inside FDM tolerance (~0.2 mm) of fusing. **Three of five keys dead**, on a part that had passed
every check the project had.

`CRADLE_CLEAR` (1.5 mm, GUESS, disclosed) now reserves the cradle's swing envelope in the growth
keep-out and as a sixteenth constraint, so the search prices it rather than discovering it after
export.

⚠ **AND THE FIRST VERSION OF THAT FIX KILLED THE DESIGN SPACE** -- 0 of 31 designs grew at all,
`yield +inf` across the board -- because reserving 3 mm around each cradle also forbids every bar
that reaches the button, and the button *sits on the mount*. That is the third instance in a week
of one shape of error, and it is worth stating as a rule: **the thing the structure must ATTACH to
cannot also be a keep-out.** (The others: a finger's own cup is its destination, not an obstacle,
which made the donning constraint unsatisfiable until it was exempted; and `connected()` keeping
anything that touches the anchor *patch*, so two separate anchored pieces both passed the check
that exists to forbid exactly that.) A bar ENDING at a button is a tie-in and is exempt; a bar
PASSING THROUGH a cradle is a jam and is not.

Re-picked from the existing front rather than re-run: **21 of 31 designs survive**. The new knee
(design 13) carries **+5.00 mm** of cradle clearance against a 1.5 mm bar -- deliberately not the
boundary-hugging choice that cost two rebuilds this week, and it gives up nothing measurable
(effort 5.262e-7 / 32.25 g against 5.242e-7 / 32.03 g). Verified on the geometry that actually
gets printed, both checks together:

| finger | donning (need +2.00) | cradle (need +1.50) |
|---|---|---|
| thumb | +2.50 | +12.39 |
| index | +2.55 | +3.49 |
| middle | +3.61 | +7.88 |
| ring | +2.99 | +3.26 |
| little | +2.83 | +9.93 |

The shipped part: 115 x 159 x 147 mm, one watertight body, 49.9 g of CF-PA12, buttons at 478 um
against the 500 um gate. ⚠ Still open: the search has NOT converged (all three seeds contribute);
`scripts/entry_view.py` still prints the retired 20 mm sweep (+0.4 to +0.7 mm here, which is the
check that passed a gauntlet the user could not put on -- ignore that line); the grab margin is
36 MPa against 35 allowed; and `CRADLE_CLEAR` is a guess awaiting a printed cradle's measured
travel. **Both files re-stamped** (114 files).

| file | sha256 |
|---|---|
| `VISION.md` (the disclosure) — *re-stamped: discloses `CRADLE_CLEAR`* | `08a9035ecf789b01ecfdccd6dc62984d7200724ef54db2578f168056a2c64f24` |
| `MANIFEST.sha256` (hashes of all 114 source + doc files) | `b3e26f423514513b4b4524d1ee38c591704e79783a4a5810f0f4d19b00042c81` |

Stamped: **2026-08-24T16:49:00Z** (UTC, submission time). Proofs: `MANIFEST.sha256.ots` and
`VISION.md.ots`, both freshly stamped. The outgoing 38th proofs are archived at
`timestamps/MANIFEST.sha256.2026-08-24b.ots` and `timestamps/VISION.md.2026-08-24b.ots` (run
`ots upgrade` once the block mines).

### Thirty-eighth — THE MODEL IS THE USER'S HAND, AND THE CHECKS TEST THE PART

The user, holding the printed gauntlet: *"my hand doesn't fit"*; *"too narrow for my hand"*;
*"the last 3 model runs all had that basic bug"*. All three correct. MyoHand had never been the
user's hand, and nothing in the loop ever compared it to a measurement -- so every signal a run
produces (converged fronts, satisfied constraints, rendered previews) was consistent with a
device built for somebody else.

**THE ANATOMY.** Three defects, each invisible to every other check. `_fit_fingertips` grew only
the PADS, leaving the shaft up to **1.77x too thin** -- the model's fingers were narrower than
their own measured breadths, which is impossible, and the PIP that jammed the print lives on that
shaft. Fitting the shafts did not widen the HAND, because flesh cannot: the span is set by the
metacarpals, and the four fingers still covered 66 mm at the knuckles against a measured 104.
Splaying the metacarpals then broke the physics -- the interossei run `thirdmc->fifthmc` and
FDP5/FDS5 run `capitate->fourthmc->fifthmc`, so every path crossing between them lengthened while
MuJoCo still read its force-length curve off the NARROW hand's `actuator_lengthrange`: the little
finger needed **1.000 activation to hold its own rest posture**. Muscles are now scaled with the
bones they span, as OpenSim does when fitting a model to a subject. Result: MCP span 48.0 ->
**81.5 mm**, knuckle width 64.9 -> **102.1 mm** (user: 104), all five digits 0.000 at rest.
And a GATE: `opt.run:preflight()` checks the hand against every measurement before the first
evaluation and REFUSES to start otherwise -- verified to fire by neutering the fits.

**THE DONNING CORRIDOR**, rebuilt after the print jammed on a 25 mm knuckle in a corridor sized
to a 20 mm fingertip. The old check swept the DISTAL phalanx only, 20 mm, along each finger's own
axis -- three fictions. It now sweeps distal + middle phalanx plus caliper-measured PIP bulges
(`PIP_BREADTH`), over the full `DON_LEN` = 80 mm the hand actually travels, along ONE shared
approach with the fingers entering EXTENDED and curling as they seat, demanding real room
(`DON_CLEAR` = 2 mm) rather than mere non-interference. Each finger's OWN cup is exempt from that
room demand -- it is the destination, not an obstacle -- which is what made the constraint
satisfiable at all.

**AND FIVE CHECKS THAT MEASURED THE MODEL INSTEAD OF THE ARTIFACT**, every one found by
measurement after a 5.7-hour seed returned NO FEASIBLE DESIGN with its violation pinned at
0.0019772 from generation 60 to 90: the keep-out and the constraint sampled different
trajectories (6 postures vs 9); `strap-grip` demanded 3 nodes of the 2 the coarse 8 mm lattice
provides, where the 4 mm part holds 9; islands were pruned at the start of growth and never at
the end; `connected()` passed two separate anchored pieces because the anchor is a whole patch;
and -- last and worst -- `printable.py`, which builds the geometry that is actually EXPORTED,
grew it with no keep-out at all, so the shipped STL measured **-1.20 mm into the entering
finger** while every intermediate lattice cleared. A reserved corridor also severs a fine
lattice: at 4 mm pitch the 3.5 mm moat is as wide as the node spacing and 8.8 mm bars cannot
route around it, so the SOLID 13141-bar lattice deflected 8084 um where the coarser 8 mm one gave
48 um -- five times the material, 170x floppier, impossible as stiffness and obvious as
severance.

**HANDLING.** The printed thumb cup snapped at the first donning: the only structural demand
anywhere was the 0.196 N keypress gate, so ESO pruned its support to material crisp along the
press, and a 1.8 mm CF-PA12 rod yields at ~2-3 N at cup distance. `HANDLING_N` (10 N) now enters
the ESO ranking, `cost()` as a fully-stressed-design mass surcharge, and `printable.py`'s radius
sizing as an FSD fixed point that HARD-FAILS the export rather than ship a part that would snap.

The result: a 31-design front (effort 4.516e-7 .. 1.360e-6, mass 28.8 .. 62.6 g) on a hand that
is the user's, with the best effort figure the project has produced. The shipped knee -- 436
struts, buttons 478 um against the 500 um gate, 116 x 159 x 146 mm, 49.9 g of CF-PA12, one
watertight body -- clears every finger's donning corridor **on the geometry that gets printed**:
+2.56 / +3.63 / +3.23 / +3.14 / +2.57 mm. ⚠ The search has NOT converged (all three seeds still
contribute), `scripts/entry_view.py` still reports the retired 20 mm sweep, and the grab margin
is 36 MPa against 35 allowed. **Both files re-stamped** (114 files).

| file | sha256 |
|---|---|
| `VISION.md` (the disclosure) — *re-stamped: discloses `DON_CLEAR`, `DON_LEN`, `HANDLING_N`* | `4221c06e71c73e70653b09a7ef1e33418657641744c49cd591e1c4a6160a5994` |
| `MANIFEST.sha256` (hashes of all 114 source + doc files) | `2f13abdfda9179abae1cc97542e8025c7f38db7c2cabe41449d2d5664257fb59` |

Stamped: **2026-08-24T09:27:00Z** (UTC, submission time). Proofs: `MANIFEST.sha256.ots` and
`VISION.md.ots`, both freshly stamped. The outgoing 37th proofs are archived at
`timestamps/MANIFEST.sha256.2026-08-24a.ots` and `timestamps/VISION.md.2026-08-24a.ots` (run
`ots upgrade` once the block mines).

### Thirty-seventh — THE FIRST PRINT WAS WORN, AND IT FOUND TWO THINGS NO MODEL HAD

`gauntlet_200mm.stl` was printed and put on a hand. It failed twice, and both failures were
model blind spots the constraint set could not see.

**(1) The PIP joint does not fit the corridor.** The entry model swept only the DISTAL phalanx,
so every donning corridor was sized to a fingertip -- and the index PIP is 25 mm against a 20 mm
tip. The hand jammed, then yawed anticlockwise to force the index through, at which point no
other finger met its cup at the right angle. The flesh model was complicit: MyoHand has no
knuckle at all, its middle-phalanx capsule (16 mm) being NARROWER than the distal (17.9 mm). A
third fiction compounded it -- each finger slid along its OWN well axis, five non-parallel
translations no rigid hand can perform. Fixed together: the entering cloud is distal + middle
phalanx skin plus PIP/IP bulge rings at CALIPER-MEASURED breadths (`PIP_BREADTH`,
28/25/25/24/20 mm on the 200 mm hand, reference-scaled like the fingertips); the four fingers
sweep along ONE shared approach axis (the thumb keeps its own -- CMC+MCP+IP really can snake it
in); and the corridor is tested against all five mounts, since a shared approach can cross a
NEIGHBOUR's cup wall. The honest model reproduces the printed jam from geometry alone: index
-0.69 mm, middle -0.22, ring -0.49, little -0.59, thumb +0.20.

**(2) The thumb cup snapped off at the first donning.** Not a topology hole -- min-cut says four
independent strut paths to the anchors -- but a strength blind spot: the ONLY structural demand
anywhere in the loop was the 0.196 N keypress deflection gate, so ESO ranked and the sizer
thinned purely for press stiffness, and a 1.8 mm CF-PA12 rod yields at ~2-3 N applied at cup
distance. The "oddly dropped" thumb elements were ESO correctly deleting everything idle under a
keypress -- which is exactly the bracing a grabbed cup leans on. `HANDLING_N` (10 N, GUESS,
disclosed) now enters the ESO ranking, `cost()` as a fully-stressed-design MASS SURCHARGE
(strength is bought in grams, never a deflection gate -- demanding SF 2 at uniform BAR_R would
read util 4.9 and kill layouts the sizer can build), and `printable.py`'s radius sizing as an
FSD fixed point that HARD-FAILS the export rather than ship a part that would snap. Measured on
the shipped part: 20 grab cases, 5 FSD passes, worst stress 35 MPa against 35 MPa allowable,
**+4.05 g**, and 0/374 struts idle on the nozzle floor.

The re-run merged front: 19 designs, effort 4.963-5.246e-7, mass 28.6-37.8 g. The comfort-polished
knee (effort 4.993e-7, 29.10 g; the polish reclaimed 0.56 g) grows to 560 struts from 12,410
candidates, buttons 489 um against the 500 um gate, and every entry channel now clears on the
SHIPPED geometry under the honest model (+0.4/+0.4/+0.7/+0.5/+0.1 mm) as one printed body. The
two defects cost about 6 g between them -- ~2.1 g of layout in the search, ~4.05 g of radius in
the part -- and bought a device that can be put on and handled. ⚠ The little finger's +0.1 mm is
THIN against FDM tolerance; it is the number to watch on the next print. **`VISION.md` changed**
(HANDLING_N disclosed), so **both files are re-stamped** (113 files).

| file | sha256 |
|---|---|
| `VISION.md` (the disclosure) — *re-stamped this anchor: discloses the `HANDLING_N` grab load* | `27dc2ad402e9082d5e338dc296749eae364a92440d8bff1ff66457c14d6bf7c3` |
| `MANIFEST.sha256` (hashes of all 113 source + doc files) | `8fd67f840a1ad759aad1bc96071aae9a6b6158407e141ba7d400b8fd0a682864` |

Stamped: **2026-08-22T03:53:00Z** (UTC, submission time). Proofs: `MANIFEST.sha256.ots` and
`VISION.md.ots`, both freshly stamped. The outgoing 36th proofs are archived at
`timestamps/MANIFEST.sha256.2026-08-22a.ots` and `timestamps/VISION.md.2026-08-22a.ots` (run
`ots upgrade` once the block mines).

### Thirty-sixth — THE CONSTRAINTS LEARN TO TEST THE PRINTED ARTIFACT, and the SEARCH GETS 2.4× FASTER

A month of the same lesson arriving from four directions: **a constraint that does not share geometry
with the artifact is a wish.** (1) **Box-true cup packing** — the capsule spacing test let a feasible
design ship interpenetrating printed cradles (passed by 4.1 mm as capsules, overlapped 1.8 mm as the
boxes actually printed); the constraint now runs SAT separation on the exact `well_insert` boxes, and
`WELL_WALL` becomes the real printed wall (2.6 mm derived, was a 1.5 mm guess). (2) **The coplanar
little finger** — the model happily bought packing with an 19.4 mm palmar droop of the little tip, a
posture its joint ranges permit and the user's photographed hand does not; `little_droop ≤ 10 mm` is
the new constraint, and a coordinate-descent posture polish (`scripts/comfort.py`) *proves* a winner's
posture constraint-tight rather than asserting it. (3) **The entry route** — rendering the front's knee
caught ESO growing load paths straight through the thumb and ring slide-ins (−0.85 mm into the entering
finger) while every in-loop check passed; the channel bars now leave the growth domain entirely
(keep-out at `BAR_R+FILLET+TOUCH_TOL`), and a 14th constraint re-measures the slide-in against the
struts that actually grew plus the finger's own mount. (4) **One part** — tissue springs on every node
make a disconnected island numerically self-supporting, so connectivity is the 15th constraint, not an
assumption. Also in this anchor: fingertip breadths reference-scaled (measured on the 200 mm hand,
stored at the 185 mm reference — the raw values were silently inflating every cup ~1 mm/side); the XIAO
housing oriented by hand anatomy (`hand_axes`) instead of anchor-centroid luck; the FEM hot path
factored by **CHOLMOD Cholesky** (F bit-identical to splu, 2.4–3.1× per evaluate) with a warm-start
sampling that makes generation 1 feasible; and the Hall addressing fork resolved to the **TCA9548A
mux** with five TSOP-6 `TLV493D-A1B6` (the W2BW address-variant plan died at its BGA-only package).
The merged 15-constraint front: 16 designs, effort 4.980–5.185e-7, mass 26.4–28.5 g; the knee's shipped
STL clears every entry channel (+0.4 mm worst) as one printed body. **`VISION.md` DID change** (the
reference-scaling fix, `WELL_WALL`, and thumb-opposition pricing), so **both `VISION.md` and
`MANIFEST.sha256` are re-stamped** (113 files).

| file | sha256 |
|---|---|
| `VISION.md` (the disclosure) — *re-stamped this anchor: reference-scaled fit, the printed cup wall, opposition priced at the build posture* | `2b38a6188881c6363c221cd545afb4d25ef28262cb4d0318e2e860c6889c5b86` |
| `MANIFEST.sha256` (hashes of all 113 source + doc files) | `e61b77d26e656a34f7964c254bec18b896897bc0842567dd104a943861dab2f5` |

Stamped: **2026-08-20T00:20:00Z** (UTC, submission time). Proofs: `MANIFEST.sha256.ots` and
`VISION.md.ots`, both freshly stamped. The outgoing 35th proofs are archived at
`timestamps/MANIFEST.sha256.2026-08-20a.ots` and `timestamps/VISION.md.2026-08-20a.ots` (both were
pending Bitcoin confirmation when archived; run `ots upgrade` once the block mines).

### Thirty-fifth — THE PRINT-SUPPORT OBJECTIVE WAS BUILT, MEASURED AGAINST A SLICER, AND RETIRED

A negative result, kept because it *is* the finding. A **third NSGA-II objective — sacrificial print
support** — was added to the effort × mass search, scored by a cheap in-loop surrogate: the length of
support column a coarse strut lattice would need, judged by a **hot-bead sag model** (the freshly-laid
FDM bead, still above T_g and soft, cantilevered off the stiff cooled layers below — self-support angle
`arctan(LAYER_H/BEAD_W) ≈ 27°`, sag-limited hot-bridge span `L_max ∝ E_HOT^(1/4) ≈ 15 mm`). In the
*search* it behaved like a real, independent axis (uncorrelated with mass). But **validated against
PrusaSlicer it did not predict real support**: eight designs spanning a 5× surrogate range were grown at
full resolution, oriented wrist-standing, and sliced with organic supports — **pearson −0.06**, real
support flat at ~28 cm³ ± 13%. Real organic support is driven by the **mesh surface** (hollow-tube
undersides, node fillets) and by build **orientation**, neither of which the skeletal line model sees,
so the objective was **removed**: the search stays effort × mass, and print support is handled post-hoc
by orientation (`scripts/orient.py`). The tooling is kept as tested utilities — `support_mm(hot=)`, the
hot-bead constants, `gravity_cases` (finished-part self-weight, measured *not* to lower support so left
unwired), and a **detached cloud-run mode** in `cloud/hetzner.sh` (`run-bg`/`watch`, so a burst survives
an ssh drop or a local reboot). **`VISION.md` DID change this time** — its limitations table now
discloses the two print-sag guesses `E_HOT` (20 MPa) and `SAG_TOL` (0.1 mm) — so **both `VISION.md` and
`MANIFEST.sha256` are re-stamped** (111 files).

| file | sha256 |
|---|---|
| `VISION.md` (the disclosure) — *re-stamped this anchor: adds the `E_HOT`/`SAG_TOL` print-sag guesses* | `c890f3f252c252f47dbfe8dd8e696dc4144e56761f6d9d5747717147127ee7a6` |
| `MANIFEST.sha256` (hashes of all 111 source + doc files) | `636f899e1a8655821fa9480bfc5ee22975f44391dbf1b064525dfafdb9f7e57f` |

Stamped: **2026-07-22T15:26:00Z** (UTC, submission time). Proofs: `MANIFEST.sha256.ots` and
`VISION.md.ots`, both freshly stamped — VISION.md's own proof no longer stands from the 27th anchor,
because its content changed. The outgoing 34th manifest proof is archived at
`timestamps/MANIFEST.sha256.2026-07-22a.ots`, and the outgoing VISION.md proof (which stood from the
27th anchor, Bitcoin block 958567) at `timestamps/VISION.md.2026-07-22a.ots` (both pending Bitcoin
confirmation at stamp time; run `ots upgrade` once the block mines).

### Thirty-fourth — THE HALL SENSOR SLIDES IN FROM THE PROXIMAL END, and the CUP WALLS ARE MADE COHERENT

Three cup fixes surfaced by looking at the sliced cluster. (1) **The Hall PCB now slides in from the
cup's open proximal end.** It had been a buried pocket reachable only from the magnet side — from
outside the print it read as a solid boss. It is now a slide-in slot along the finger axis (PCB
cross-section, wire trailing out), stopped distally so the sensor seats at the read-out gap, and
spanning the magnet gap dorsally so it CONSUMES the fat palmar button strut's reach rather than
severing an isolated sliver (which had split a lone well into two bodies). (2) **Uniform guide-wall
height.** The walls had been built to each finger's measured pad→nail depth and stepped 10.3–14.1 mm
(the index over-reading a noise-sensitive tendon-insertion difference); they are now a single 10 mm,
floor-anchored. (3) **Dedicated per-cup walls.** The four-finger cluster shared guide flanks at the
inter-finger midline; because the per-finger roll is real anatomy (up to ~27°, measured world-frame ==
body-local so it is anatomy not posture, and dominated by the little finger), a shared wall took one
finger's roll and could float palmar of a neighbour's base plate (the 'side plate under the base'
report). Each cup now carries its own two walls, beside its own finger, each meeting its own floor.
The per-finger roll is kept; only the structure is made self-contained. **`VISION.md` is unchanged** —
its 27th-anchor proof (Bitcoin block 958567) stands; only `MANIFEST.sha256` is re-stamped (**111 files**).

| file | sha256 |
|---|---|
| `VISION.md` (the disclosure) — *unchanged; stands from the 27th anchor, Bitcoin block 958567* | `9b10e86724f558c8783ec708235636fc4c2faba93974ecd31987884fa1bfa266` |
| `MANIFEST.sha256` (hashes of all 111 source + doc files) | `91de9e113857ca8a83fbb505ceccbbe1b355e973d1b5bb4eccec94204b95a285` |

Stamped: **2026-07-21T16:09:37Z** (UTC, submission time). Proof: `MANIFEST.sha256.ots` (the
`VISION.md.ots` proof stands unchanged from the 27th anchor). The outgoing 33rd manifest proof is
archived at `timestamps/MANIFEST.sha256.2026-07-21d.ots` (Bitcoin block 959023).

### Thirty-third — THE CUP WALLS NOW MEET THEIR FLOOR, a FILLET-AWARE ENTRY GATE, and a PRINT-ORIENTATION TOOL

Three things on the road to the first good print, all provoked by looking at the sliced `gauntlet.stl`.
(1) **The cluster's shared side walls were detached from the cup floors.** The four long fingers share
guide flanks that sit at the inter-finger midline, but each cup floor reached only its own finger — so
four of the five walls stood 0.7–1.3 mm off the floor (past the 0.6 mm print fillet), rejoined to the
part only by the thin palmar base-spine tie: floating, floppy guide walls. `manufacture.mount` gains
`FLOOR_REACH`, widening the cluster floors to meet the flanks (a continuous base the walls rise from);
all five now merge, guarded by a regression test in `tests/test_mount.py`. (2) **A fillet-aware entry
gate.** The entry check used the hard union of the mount primitives, but the STL is marched from a
*smooth* min (`BLEND` = 0.6 mm) that inflates junctions outward. `manufacture.entry.smin_sdf` evaluates
that same smooth-min field along each finger's slide-in route, so the fillet — not just the centreline —
must clear; it confirms nothing filleted crosses (nearest strut +6.25 mm off the route, +3.56 mm off the
whole finger; the closest thing is each cup's own +0.43 mm seat). (3) **An optimal print orientation.**
`scripts/orient.py` sweeps build directions over the finished mesh (cups, housing, fillets and all) and
minimises downward-facing overhang area; it agrees with the strut-axis `printability.py` that the part
builds along its long axis, and picks a low-profile orientation (9.8 % overhang vs 15.9 % lying flat,
119 mm tall) written to `out/gauntlet_oriented.stl` for PrusaSlicer's organic supports. **`VISION.md`
is unchanged** — its 27th-anchor proof (Bitcoin block 958567) stands; only `MANIFEST.sha256` is
re-stamped (**111 files**).

| file | sha256 |
|---|---|
| `VISION.md` (the disclosure) — *unchanged; stands from the 27th anchor, Bitcoin block 958567* | `9b10e86724f558c8783ec708235636fc4c2faba93974ecd31987884fa1bfa266` |
| `MANIFEST.sha256` (hashes of all 111 source + doc files) | `6d6aad6f537f9256246e7343077711ba74f29389f5176175270d4a8240e8e3bb` |

Stamped: **2026-07-21T15:03:09Z** (UTC, submission time). Proof: `MANIFEST.sha256.ots` (the
`VISION.md.ots` proof stands unchanged from the 27th anchor). The outgoing 32nd manifest proof is
archived at `timestamps/MANIFEST.sha256.2026-07-21c.ots` (pending Bitcoin confirmation at stamp time;
run `ots upgrade` once the block mines).

### Thirty-second — THE PRINTABLE STL NOW EXPORTS IN MILLIMETRES

A usability fix on the road to the first physical print: `scripts/export_stl.py` builds the mesh in SI
(metres), and slicers assume millimetres, so a raw `gauntlet.stl` imported as a sub-millimetre speck —
a footgun nobody had hit because nothing had been printed yet. The export now scales ×1000 at write
time (after every metre-based clearance check), so `gauntlet.stl` and the `make fit` variants slice at
the right size out of the box (94 × 97 × 158 mm), and the "download and slice" path is finally correct.
**`VISION.md` is unchanged** — its 27th-anchor proof (Bitcoin block 958567) stands; only
`MANIFEST.sha256` is re-stamped (**110 files**).

| file | sha256 |
|---|---|
| `VISION.md` (the disclosure) — *unchanged; stands from the 27th anchor, Bitcoin block 958567* | `9b10e86724f558c8783ec708235636fc4c2faba93974ecd31987884fa1bfa266` |
| `MANIFEST.sha256` (hashes of all 110 source + doc files) | `5bbe4dcef50fbf15345b648208ad47e86dd21ed8b50780dfc1b51ab55a319ac3` |

Stamped: **2026-07-21T13:43:18Z** (UTC, submission time). Proof: `MANIFEST.sha256.ots` (the
`VISION.md.ots` proof stands unchanged from the 27th anchor). The outgoing 31st manifest proof is
archived at `timestamps/MANIFEST.sha256.2026-07-21b.ots` (Bitcoin block 958996).

### Thirty-first — THE FIRST FIRMWARE: a bench field-map sketch for the XIAO nRF52840

The read-out is no longer only a prediction on paper -- there is now firmware to *measure* it.
`firmware/bench_fieldmap/bench_fieldmap.ino` (the repo's first firmware) streams the raw 3-axis field
(Bx, By, Bz) from a TLV493D over USB serial as CSV, so a caliper-stepped magnet logs straight into the
COUPON.md T1/T3 tables. It is built on the Infineon **XENSIV TLx493D** library (which also drives the
device's TLI493D-W2BW), with 8-sample averaging and the I²C bus-lock recovery from
`docs/electronics.md`. **Compile-clean** for `Seeeduino:nrf52:xiaonRF52840` (76.5 KB, 9 % of flash);
not yet hardware-tested. `README.md` gains a "Build" section for the coupon/rig prints and the firmware
toolchain (arduino-cli core/lib install, compile, upload), and the manifest rule now anchors `*.ino`.
**`VISION.md` is unchanged** — its 27th-anchor proof (Bitcoin block 958567) stands; only
`MANIFEST.sha256` is re-stamped (**110 files**).

| file | sha256 |
|---|---|
| `VISION.md` (the disclosure) — *unchanged; stands from the 27th anchor, Bitcoin block 958567* | `9b10e86724f558c8783ec708235636fc4c2faba93974ecd31987884fa1bfa266` |
| `MANIFEST.sha256` (hashes of all 110 source + doc files) | `6f5fb4a144715dbd00163d43f98c122dc373546c77fa6bcee7ccb192d7ca40cb` |

Stamped: **2026-07-21T10:16:45Z** (UTC, submission time). Proof: `MANIFEST.sha256.ots` (the
`VISION.md.ots` proof stands unchanged from the 27th anchor). The outgoing 30th manifest proof is
archived at `timestamps/MANIFEST.sha256.2026-07-21a.ots` (Bitcoin block 958690).

### Thirtieth — THE READ-OUT ELECTRONICS, and the wrist housing that carries them (`docs/electronics.md`)

The electronics design the mechanical/magnetic model implies is now written down. `docs/electronics.md`
captures the read-out chain — five bare **TLV493D** Hall sensors (chip + one cap each; pull-ups per
bus) → a **TCA9548A** I²C mux at the wrist → the **XIAO nRF52840** → BLE HID — and the decisions that
shape it: the **mux-vs-W2BW addressing fork** and its copper trade-off (a mux frees the A1B6's 2-address
limit but breaks the minimal-copper shared bus), the **three-layer lock-up recovery**, and the **34 AWG
enamelled braid** that fits the 1.36 mm groove (with an internal-through-the-hollow-tubes routing idea).
`manufacture/mount.housing` now carries the **mux** beside the XIAO and LiPo (in its dead space) and
carves a **wire-entry slot** for the harness; the LiPo default drops to a **401020** cell (4×10×20 mm),
so the box stands less proud — now **24 × 39.4 × 7 mm**, and the full gauntlet re-exports watertight at
42.0 g. It also states plainly what is still **not built**: firmware, custom PCBs, and the read-out
numbers themselves (pending the stage-1 coupon). **`VISION.md` is unchanged** — its 27th-anchor proof
(Bitcoin block 958567) stands; only `MANIFEST.sha256` is re-stamped (**109 files**).

| file | sha256 |
|---|---|
| `VISION.md` (the disclosure) — *unchanged; stands from the 27th anchor, Bitcoin block 958567* | `9b10e86724f558c8783ec708235636fc4c2faba93974ecd31987884fa1bfa266` |
| `MANIFEST.sha256` (hashes of all 109 source + doc files) | `281d2b19d206d179ac09f67a8527b759861494bb7287022f5cf067f9b514841d` |

Stamped: **2026-07-19T01:33:32Z** (UTC, submission time). Proof: `MANIFEST.sha256.ots` (the
`VISION.md.ots` proof stands unchanged from the 27th anchor). The outgoing 29th manifest proof is
archived at `timestamps/MANIFEST.sha256.2026-07-19a.ots` (Bitcoin block 958631).

### Twenty-ninth — A DOME FLEXURE + a printable bench rig to MEASURE the read-out (`COUPON.md`)

The first bench numbers are in: the flat TPU membrane measured **~230 g at 1.5 mm** against a 20 g
target — too stiff, because a flat clamped membrane *stretches* rather than bends at that deflection.
So this anchor adds the softer flexures and the rig to measure the read-out for real: a shallow **dome**
coupon (`scripts/coupon.py` `dome()` — it rolls/snaps instead of stretching), a printed **field-map
fixture** (`scripts/field_fixture.scad`) that fixes the Hall and slides the magnet over it so T1/T3
need only a caliper + micrometer, and a two-part **silicone dome mold** (`scripts/dome_mold.scad`) to
cast the native keypad-rubber flexure — wall set by the mold gap, not print calibration. The manifest
rule now anchors `*.scad` too (the CAD is the source of record; the coupon/rig STLs are generated).
**`VISION.md` is unchanged** — its 27th-anchor proof (Bitcoin block 958567) stands; only
`MANIFEST.sha256` is re-stamped (**108 files**).

| file | sha256 |
|---|---|
| `VISION.md` (the disclosure) — *unchanged; stands from the 27th anchor, Bitcoin block 958567* | `9b10e86724f558c8783ec708235636fc4c2faba93974ecd31987884fa1bfa266` |
| `MANIFEST.sha256` (hashes of all 108 source + doc files) | `b5d687d66875d8b3482af6124a8bb35cb5e5519d1815e8c4b5eb6cbb21cd48d3` |

Stamped: **2026-07-18T23:38:33Z** (UTC, submission time). Proof: `MANIFEST.sha256.ots` (the
`VISION.md.ots` proof stands unchanged from the 27th anchor). The outgoing 28th manifest proof is
archived at `timestamps/MANIFEST.sha256.2026-07-18c.ots` (Bitcoin block 958601).

### Twenty-eighth — STAGE-1 READ-OUT COUPON PROTOCOL, and a clean TPU test article (`COUPON.md`)

The read-out — that a keypress swings the Hall field ~200× above noise, that the five directions sit
≥ 78° apart, that a neighbour leaks below the noise once baselined — is still a numpy **prediction**
(`manufacture/readout.py`). `COUPON.md` is the first *physical* test: **T1–T7**, each with a
**pre-registered** pass threshold that is the matching `tests/test_readout.py` gate restated as a bench
measurement, so no goalpost can move after a result — and it settles the model's two outright guesses,
`REST_GAP` (3.5 mm) and `CRADLE_LEVER` (0.7). `scripts/coupon.py` generates the TPU article: a rigid rim
clamping a flat membrane of thickness `t`, radius `a` — the clamped diaphragm `flexure.dome` sizes, so
its force/travel is a direct check on **k = 130 N/m** — watertight by construction (a solid of
revolution) and exported in mm, replacing the old non-manifold coupon soup. **`VISION.md` is unchanged**
— its 27th-anchor proof (Bitcoin block 958567) stands; only `MANIFEST.sha256` is re-stamped, now
covering the two new files (**106 files**).

| file | sha256 |
|---|---|
| `VISION.md` (the disclosure) — *unchanged; stands from the 27th anchor, Bitcoin block 958567* | `9b10e86724f558c8783ec708235636fc4c2faba93974ecd31987884fa1bfa266` |
| `MANIFEST.sha256` (hashes of all 106 source + doc files) | `4df2685182e3bc3e2d36b65d7e940d2ecced5946613266b8df2f37edf96bd359` |

Stamped: **2026-07-18T17:41:57Z** (UTC, submission time). Proof: `MANIFEST.sha256.ots` (the
`VISION.md.ots` proof stands unchanged from the 27th anchor).

### Twenty-seventh — THE IMPACT BONE HELD OFF THE FINGERS by a flesh-aware relaxation (§8.15k ggg)

The impact-aware structure looked like its struts passed *through* the fingers. Measured, they did
not — every strut cleared the flesh — but the broad knock-bearing grow hugged it at **~1.1 mm**
against the main design's ~3.4 mm, because the grow's clearance floor is checked at the *nominal* rod
radius while the impact sizer fattens struts to R_MAX and node-relaxation then pulls them toward the
skin. The novelty disclosed here is the fix: those hugging struts **carry the 50 N knock** (it lands at
the buttons, near the fingers), so *deleting* them to win clearance fails SF 2 — **2 mm is the most
removal alone survives**. Instead **move** them: make the form-finding node-relaxation **flesh-aware**
— raise each free node's skin-band floor by its own rod radius (`relax_nodes` takes a per-node `hug`),
so the relaxation pushes the fat struts off the finger at **no knock cost**. Result: **95 % of free
struts clear ≥ 3 mm** (median 5.9 mm) with the knock held at 36 MPa (714 struts, 24.2 g, SF 2); only a
few chord-dip residuals and the intrinsic button/mount struts stay closer. Guarded by
`test_the_impact_bone_keeps_its_free_struts_off_the_flesh`. This anchor also packages the project for
build (`BUILD.md`, `BOM.md`, a one-command `Makefile`, pinned `requirements.txt`, and a per-hand STL
fit) — repo hygiene, not disclosure. (Manifest now **104 files**.)

| file | sha256 |
|---|---|
| `VISION.md` (the disclosure) | `9b10e86724f558c8783ec708235636fc4c2faba93974ecd31987884fa1bfa266` |
| `MANIFEST.sha256` (hashes of all 104 source + doc files) | `e5c4ab5b41b30f1b59744e32678e4429e22ad89aaba2a1fbaa67e0502e1bf0de` |

Stamped: **2026-07-18T14:40:50Z** (UTC, submission time). Proofs: `VISION.md.ots`,
`MANIFEST.sha256.ots`. Confirmed in **Bitcoin block 958567** (`VISION.md`) and **958568**
(`MANIFEST.sha256`).

### Twenty-sixth — THE HARNESS BUS IS NOW EXACT (Dreyfus–Wagner), provably minimal (§8.15l qqq-2)

The 25th anchor meshed the minimal-copper harness with a **metric-MST 2-approximation**; this makes it
**exact**. `mount._steiner_exact` (Dreyfus–Wagner with edge recovery) computes the **true minimum
Steiner tree** over the live struts. On the shipped layout the exact tree is **275 mm** against the
approximation's 283 mm — a **further 2.8 %** — so the 2-power + 2-signal bus is **~1100 mm-equivalent of
conductor, −44 %** against the 490 mm of five independent runs. The router is now **provably optimal**,
not merely near-optimal — the branching Steiner tree still beats the daisy-chain (275 vs 373 mm), so it
is *not* the travelling-salesman problem. The export sinks the wire grooves along the exact tree
(258 mm on the finer print bone); one watertight solid, **40.2 g**, `test_the_harness_bus_is_a_shorter_
shared_tree` and 9 other mount tests pass. Also folds in the `harness.html` gallery view added between
anchors. (Manifest now **99 files** — the view scripts joined it.)

| file | sha256 |
|---|---|
| `VISION.md` (the disclosure) | `026748b264f8fe8f2b4967a9fdeecf617f1500e4827cccd807a44252da39b0e3` |
| `MANIFEST.sha256` (hashes of all 99 source + doc files) | `d93b719af895052f4964fdc3d7b9b00247f6a829d198614816963483e39d6d7c` |

Stamped: **2026-07-18T09:32:41Z** (UTC, submission time). Proofs: `VISION.md.ots`,
`MANIFEST.sha256.ots`.

### Twenty-fifth — THE MINIMAL-COPPER HARNESS BUS, meshed into the export (§8.15l qqq-2)

The 24th anchor *disclosed* the minimal-copper harness (a shared bus, not five point-to-point runs);
this **builds** it. `mount.harness_bus` routes the harness as a **shared Steiner tree over the strut
graph** (a metric-MST approximation) — **one power tree** over all five sensors + the MCU, I²C **signal**
per bus, the conductor count folded in per segment — and `scripts/export_stl.py` now sinks the wire
grooves along that bus: **283 mm in 32 segments** (a uniform 4-wire bundle, the two I²C buses taking
separate routes to the wrist) against the **490 mm** of five independent runs — **−42 % copper**, the
groove widening with the conductor count. One watertight solid, **40.2 g**.
`test_the_harness_bus_is_a_shorter_shared_tree` guards it: shorter than the baseline, every sensor
reaches the wrist, and only live struts carry wire. The **branching** Steiner tree beats the single
**daisy-chain** (283 vs 373 mm), so it is *not* quite the travelling-salesman problem — TSP is the
special case with T-junctions forbidden.

| file | sha256 |
|---|---|
| `VISION.md` (the disclosure) | `1cf7f63cd701db816492fc162ef0506f2c24fd0b153ff6defceb8f443e703904` |
| `MANIFEST.sha256` (hashes of all 98 source + doc files) | `d125a45deb5efc2920fc1673572745cb61a68eb250e1c7ee5e1d84f4f6afcbd1` |

Stamped: **2026-07-17T21:06:58Z** (UTC, submission time). Proofs:
`timestamps/VISION.md.2026-07-17g.ots`, `timestamps/MANIFEST.sha256.2026-07-17g.ots`.

### Twenty-fourth — THE FINGERTIP NOW SEATS IN ITS CUP; a minimal-copper harness BUS (§8.15l ppp-2/qqq-2)

Two things. **(1) The finger was floating above its cup.** The entry check (21st–23rd) had a blind
spot a render caught: `enters_freely` only rejects mount material *inside* the swept finger, so a cup
the finger never reaches passes **vacuously** — and this one did (every fingertip hovered ~7–13 mm
*above* its cup, **0–1 % of the skin inside it**). Cause: the cup was built at `well_frame["pos"] + r`
assuming `pos` is the pulp *centre*, but `pad_pose` returns the pad **surface**. `manufacture/mount.py`
now builds the cup to the **measured** pad and nail (the distal-phalanx skin's floor-direction extent):
floor just palmar of the pad, flanks spanning the finger's real depth, strut to the **palmar sensor
base** (the button node *is* the sensor — a strut to a dorsal edge crossed the finger). The pad now
**contacts** the floor (0.4–0.8 mm), ~50–65 % of the skin cradled; one watertight solid, **40.2 g**.
`test_the_finger_actually_seats_in_its_cup` fails if a fingertip floats again — the regression the
vacuous entry check could not be. **(2) A minimal-copper harness bus (analysed).** The wires need not
run point-to-point: the sensors are I²C, so SDA/SCL are a **bus** and VDD/GND are shared, making the
minimum-copper harness a **shared Steiner tree over the strut graph** — one power tree over all five
sensors + the MCU, signal riding the trunk. Measured on `out/final.npz`: **283 mm** shared vs **490 mm**
of independent paths — a 2-power + 2-signal bus is **−42 %** copper. It is *not* quite TSP: the branching
Steiner tree beats the single daisy-chain (**283 vs 373 mm**); TSP is the branch-forbidden special case.
Disclosed as prior art (§8.15l qqq-2); the Steiner-tree router is not yet meshed into the export.

| file | sha256 |
|---|---|
| `VISION.md` (the disclosure) | `5375c98d984e95e7176ce20b7cbbeefb33005ce46ac2696f99fb696037c11d13` |
| `MANIFEST.sha256` (hashes of all 98 source + doc files) | `e56670478ce176e1135a3844ff2873efef57e96c4397471eed38392446680efa` |

Stamped: **2026-07-17T19:30:57Z** (UTC, submission time). Proofs:
`timestamps/VISION.md.2026-07-17f.ots`, `timestamps/MANIFEST.sha256.2026-07-17f.ots`.

### Twenty-third — THE ENTRY CHECK NOW COVERS THE GAUNTLET STRUTS, HOUSING AND WIRES (§8.15l ppp/qqq)

Two gaps closed. **(1)** The finger-entry check ran only against the mount, not the **gauntlet
struts** — but the truss wraps near the fingertips, so a strut across the slide-in would block just as
a mount wall would. Now checked against the struts too
(`test_the_finger_enters_past_the_gauntlet_struts_too`): the nearest strut sits **+3.2 mm** off the
entry sweep, so the mount's guide flanks stay the binding constraint — *verified, not assumed*. And
`out/entry.html` now renders the **whole gauntlet** (struts + mounts + housing) so the path is shown
against what could block it, not the mount in isolation. **(2)** The **wrist MCU housing + wire
routing**, dropped in the entry-first rebuild, are meshed back in (`mount.housing`,
`mount.harness_routes`): the housing sits **proud of the wrist** (clears 1.7 mm, necked to live-strut
nodes), the wires in **dorsal grooves** (264 segments) — both far from the fingertips, neither touches
the entry route. One watertight solid, **41.1 g**. 131 tests pass.

| file | sha256 |
|---|---|
| `VISION.md` (the disclosure) | `cfd4640fa6d4c8eecf35802cd7830df973e9463cb5ce61209b27df0986e32fe3` |
| `MANIFEST.sha256` (hashes of all 98 source + doc files) | `d87fd04bf8c640620421350f69a1cc2c1b953dc66eb0d09169ae857b7013c459` |

Stamped: **2026-07-17T09:43:25Z** (UTC, submission time). Proofs:
`timestamps/VISION.md.2026-07-17e.ots`, `timestamps/MANIFEST.sha256.2026-07-17e.ots`.

⚠ `TIMESTAMP.md` is deliberately **not** in the manifest. It is written *after* the stamp — it holds
the stamp's own hashes and time — so including it would guarantee `sha256sum -c` failed forever.

### Twenty-second — THE DROP-IN CRADLE PASSES THE ENTRY CHECK TOO (§8.15l ppp)

The 21st anchor rebuilt the sensor FRAME entry-first. This adds the **drop-in TPU cradle**
(`mount.well_insert`) — the cup the finger actually presses, carrying the magnet on the §8.15g dome —
and puts it through the *same* `manufacture.entry` check: the finger enters the cradle's cup (open
proximally, flanks beside, floor below, nail hood and all) **freely**, both individually (≥ 1.6 mm) and
**assembled with the frame** (`test_mount.py`). One watertight TPU piece per finger. `out/entry.html`
now shows the nested cradles the slide-in channels pass through. The read-out physics (mmm–ooo) and the
frame (21st) are unchanged.

| file | sha256 |
|---|---|
| `VISION.md` (the disclosure) | `0b9de23253f1d4427a9efe1d4574e63ae28ae4b0b83f78358fcbfa9504f15a5d` |
| `MANIFEST.sha256` (hashes of all 98 source + doc files) | `d947d09cd78d3571e925f2b903e86503cdbc38c66d5d820c2e873594c2b6654b` |

Stamped: **2026-07-17T08:39:04Z** (UTC, submission time). Proofs:
`timestamps/VISION.md.2026-07-17d.ots`, `timestamps/MANIFEST.sha256.2026-07-17d.ots`.

### Twenty-first — THE FINGER-ENTRY ROUTE, and the mount rebuilt to it (§8.15l ppp)

A correction to this session's own work. The sensor mount (17th–20th anchors) was built checking only
the finger's **static seated** clearance and kept blocking the **route the finger enters by** — the
fingertip slides into the cup along the phalanx axis from the proximal-open end, and a strut across
that path or a rim over the cup leaves it nowhere to come in from. A mount can clear a *seated* finger
and still be un-enterable. That geometry is **withdrawn**. The missing step is now a first-class model:
`manufacture/entry.py` sweeps the distal-phalanx skin along the slide-in and tests it against the
mount's exact primitive SDF, distinguishing a **block** (material *inside* the entering finger) from a
**guide** (a flank *beside* it). `manufacture/mount.py` is rebuilt to pass it — cup open proximally,
sensor palmar below the finger, strut on the dorsal-lateral edge, the four long fingers on a shared
cluster — every finger **enters freely** (≥ 3.1 mm; `tests/test_entry.py`, `test_mount.py`), one
watertight solid, **36.0 g**. The read-out physics (mmm–ooo) is unchanged. `out/entry.html` shows each
finger's slide-in channel passing clear of the mount. The 17th–20th anchors stand as dated floors on
the read-out disclosure; the mount *geometry* they carried is superseded here.

| file | sha256 |
|---|---|
| `VISION.md` (the disclosure) | `e048ee3b4d322891a12d6dfd8b1e8e23f6cea85dd093c1999eff0983c05104d3` |
| `MANIFEST.sha256` (hashes of all 98 source + doc files) | `04838528e43a3ddab737ac2c539e2f54d698deede6183f5a96e85dc710a5c5a6` |

Stamped: **2026-07-17T07:38:49Z** (UTC, submission time). Proofs:
`timestamps/VISION.md.2026-07-17c.ots`, `timestamps/MANIFEST.sha256.2026-07-17c.ots`.

### Twentieth — THE FOUR LONG FINGERS SHARE A CLUSTER, with open finger paths (§8.15l sss)

The 19th anchor flagged that four independent sensor modules are wider than the finger pitch and
interpenetrate at every adjacent pair — needing a shared cluster. **Built** (`wellmod.cluster_frame`).
The long fingers share **one carrier** with **shared inter-finger walls** (the wall between two fingers is
one wall, not two), a palmar base spine linking the Hall seats, and a dorsal rim rail + the struts running
along the wall tops **BETWEEN the fingers, never over a cup** — so every finger drops into its cup and
reaches its sensor freely (entries clear **≥ 3.3 mm**). ⚠ A first cut ran the rim over the finger centres
and choked every entry to 0.1 mm — caught **by eye in the render**, now guarded by
`test_the_cluster_leaves_the_finger_entry_open`. It meshes to **one watertight, non-self-colliding
piece**, each cup walled off, and is **lighter** than the four modules were (whole part **42.2 g**, down
from 49.9). The thumb keeps an independent module. The packing `xfail` is retired; **130 tests pass**. ⚠
Open: the drop-in inserts are still per-finger and overlap the shared walls at the tightest pair — they
need matching narrower/webbed cups.

| file | sha256 |
|---|---|
| `VISION.md` (the disclosure) | `96505cbf5bf642442b2ae5701e6a80bb66b83072bffc33dac84ae56671ab4609` |
| `MANIFEST.sha256` (hashes of all 98 source + doc files) | `eba779014fba7bc89817dd6435b67f883048ffb1b9c563537d74298ce2ec8eb1` |

Stamped: **2026-07-17T06:52:11Z** (UTC, submission time). Proofs:
`timestamps/VISION.md.2026-07-17b.ots`, `timestamps/MANIFEST.sha256.2026-07-17b.ots`.

### Nineteenth — THE STRUT TIES IN DORSALLY, and the collar nests the insert (§8.15l rrr/sss)

Two module bugs the one-sensor render exposed. **(1)** The truss tied into the frame **down by the palmar
magnet** — the wrong side; it now lands on a **dorsal-lateral rim + distal brace** (the nail side,
**opposite the magnet**, measured **3.8 mm** clear of the finger). **(2)** The frame collar was **inboard
of the insert cup** (±7.9 vs ±9.5 mm), so the two parts could not nest; the collar now sits **outboard**
and the insert drops in between the walls. Still one watertight solid (component count 1), but **49.9 g** —
proper nesting is heavier. Sizing the collar to nest widened every module past the finger pitch, so **all
three adjacent long-finger pairs** now interpenetrate (index-middle, middle-ring, ring-little), not just
middle-ring: the four long fingers need a **shared cluster**, the next real piece of work
(`test_adjacent_long_fingers_need_cluster_packing`, an `xfail`). A new `scripts/sensor_view.py` renders one
module cut away beside the field-vs-motion signal — which is what caught all of this.

| file | sha256 |
|---|---|
| `VISION.md` (the disclosure) | `dbefe524ccec8f018bb48b96861a1250ef3ceeedc37b1d5a67424cdaa9c20ffd` |
| `MANIFEST.sha256` (hashes of all 98 source + doc files) | `1915ea9d2a35fcbc56f10b14e9ba84b921f4cfce69dcad22c33080e4ef03af78` |

Stamped: **2026-07-17T05:52:34Z** (UTC, submission time). Proofs:
`timestamps/VISION.md.2026-07-17a.ots`, `timestamps/MANIFEST.sha256.2026-07-17a.ots`.

### Eighteenth — CORRECTION: the sensor part was 31 pieces, not one solid; now tied into one (§8.15l rrr)

The 17th anchor claimed the sensor gauntlet meshed as "one watertight solid." It was watertight but in
**31 disconnected pieces**: the five modules and the MCU housing floated **~10 mm off the skeleton**,
because `well_frame`'s fingertip pad is **not** the structure's button node (`ground()` places them
differently), and the housing neck anchored at the anchor *centroid* — empty space. The watertight check
passed; only counting **components** caught it. Fixed: each frame is tied to its button node with
**stalks**, the housing to its nearest **live-strut nodes**, and the sub-mm³ marching-cubes debris is
dropped — now **one connected body, component count 1**, **42.1 g** (the 39.0 g figure was measured off the
broken mesh). The MCU box is also re-oriented along the local skin normal so it sits **proud of the wrist**
instead of cutting in. A new `scripts/gauntlet_solid_view.py` renders the STL back with the magnets and Hall
sensors flagged in contrasting colour, so the mounts are visible.

| file | sha256 |
|---|---|
| `VISION.md` (the disclosure) | `ad483d8c11148d49c234160da6d5fe21ce83a0be9dd19a0bddfb8c887c06a81a` |
| `MANIFEST.sha256` (hashes of all 97 source + doc files) | `d4861889384256b712e9391b64011cd3eb35c02dc1fb5cdb0ab65e1fef1b3533` |

Stamped: **2026-07-16T22:53:48Z** (UTC, submission time). Proofs:
`timestamps/VISION.md.2026-07-16h.ots`, `timestamps/MANIFEST.sha256.2026-07-16h.ots`.

### Seventeenth — THE READ-OUT: the field a moving magnet presents to the Hall, and the printed module (§8.15l)

§8.15g sized the finger-well's restoring spring (a TPU dome) but **deferred the signal** — the field a
moving magnet presents to the Hall. This closes it. A **Ø3×1 mm N42** disc on the cradle over a **3-axis
Hall** reads a keypress at **~430 LSB — ~200× the sensor noise** (`manufacture/readout.py`, an analytic
exact-cylinder + point-dipole model, no new dependency); the five joystick directions sit **≥78° apart**
(0 nearest-template errors in 10⁵ draws at the datasheet noise), and the tightest well pair's crosstalk is
**below the noise floor**, baselined out. The wells become a printable **two-part module**
(`manufacture/wellmod.py`; a `carve()` SDF-subtraction added to `manufacture/mesh.py`): a rigid PA frame
with the Hall seat and re-entrant wire grooves carved in, and a **drop-in keyed TPU cradle** holding a
press-fit magnet over the §8.15g dome. The whole gauntlet — five modules, the harness grooves, a wrist
nRF52840 housing — meshes **one watertight, winding-consistent solid** (39.0 g; **+13.5 g** measured for the
sensors). ⚠ Stated, not hidden: the tightest module pair (**middle–ring**) interpenetrates and needs
cluster-level packing; the read-out is a **model** the stage-1 coupon bench must confirm; the firmware is
**outlined, not built**. 128 tests pass, 1 xfail marking that packing gap.

| file | sha256 |
|---|---|
| `VISION.md` (the disclosure) | `b7d68754afc631c980f8aa8b9c30d268dc33a5a3c06428ac3ff532f427b9029a` |
| `MANIFEST.sha256` (hashes of all 96 source + doc files) | `ae16c7936218aa19408d1b14fc8b460fd462edd7d3359dbba1bae0d57b2f3a0d` |

Stamped: **2026-07-16T22:09:56Z** (UTC, submission time). Proofs:
`timestamps/VISION.md.2026-07-16g.ots`, `timestamps/MANIFEST.sha256.2026-07-16g.ots`.

### Sixteenth — RANK BY STRAIN ENERGY FOR FREE, off the OC's own solve (§8.15k, claim fff)

The prune fix (14th anchor) ranked deletions by strain energy via a second FEM solve, and estimated that
at "~2× the prune time." **Both the mechanism and the estimate are now improved.** `size` reads the strain
energy off the OC's *own* solve — the sizer already computes the displacements and the radius-scaled element
stiffnesses, so the per-member energy density (½·uᵀk u / L) falls out with **no second solve**. And the
"~2×" was pessimistic: **measured, the extra solve was ~20%** (a prune goes **38 → 30 s**), because a prune
step is dominated by the OC's own sizing solves, not the one ranking solve. The truss lands unchanged (253 →
250 members); 110 tests pass.

| file | sha256 |
|---|---|
| `VISION.md` (the disclosure) | `21f5f3ef458a226fb30e2518418873cfc32ec2079fb072f1a62bd6ebe1952829` |
| `MANIFEST.sha256` (hashes of all 90 source + doc files) | `8abf4d692e7278199bc423cf3926f2515e13c047ee7217e986567c53ee9d39bd` |

Stamped: **2026-07-16T17:09:24Z** (UTC, submission time). Proofs:
`timestamps/VISION.md.2026-07-16f.ots`, `timestamps/MANIFEST.sha256.2026-07-16f.ots`.

### Fifteenth — RE-CHARACTERISE THE FRIENDLY COMPARISON UNDER THE FIXED PRUNE (§8.15v)

With the prune fixed (fff), the ergonomic-floor study splits into two regimes, and the split is the finding.
The **device** (grow-based bone) stays **touch-limited — all 408 members sit on the 1.5 mm floor** (was
95%), sized by the hand: solid 20.9 g → **hollow 12.7 g (−39%)**, over-stiff at 172 µm, so the marrow comes
out free. But touch-limited is a property of **density**, not the floor: ask the fixed prune to minimise
mass and it carves a **sparse truss — 61 members, every one at the r_max ceiling (2.5 mm), load-limited,
17.4 g** — which **cannot be hollowed** (at r_max *for stiffness*; removing the core drops the second moment
to ~79% and the well deflection past the 500 µm gate). So touch-limited-dense-and-hollow (**12.7 g**) beats
load-limited-sparse (**17.4 g**): the ergonomic floor plus the marrow cavity is **vindicated by the fix, not
overturned**. README's touch-limited line (now 100%), hollow line (−39%), and trabecular line (400 → 61)
updated to match.

| file | sha256 |
|---|---|
| `VISION.md` (as of 2026-07-16, 16:43Z) | `6019ba7ea36dd5f3055778e810b98ab90c7d2d34abe7fafda8ad22e062b9bfab` |
| `MANIFEST.sha256` (90 files) | `b58cb9305b4fcced0d06a3014e4518de209ed7cd4bbef78f93ffd482bee9ae0d` |

Stamped: **2026-07-16T16:43:22Z** (UTC, submission time). Proofs:
`timestamps/VISION.md.2026-07-16e.ots`, `timestamps/MANIFEST.sha256.2026-07-16e.ots`.

### Fourteenth — FIX THE PRUNE'S MEMBRANE TRAP: rank deletions by strain energy (§8.15k, claim fff)

The membrane was a one-line **ranking** bug. `grow` and `size_and_prune` are both top-down ESO; the only
difference that mattered is the signal each deletes by. `grow` ranks by **strain energy** at a fixed
radius, where an idle member reads as idle whatever the sizer later does; `size_and_prune` ranked by the
**OC-sized radius**, which the OC returns *uniform* on a membrane — no signal, so it deleted ~blindly and
stalled. Measured on the same 8 mm lattice: `grow` carves a **205-strut / 7.2 g** truss (with node
relaxation on *or* off — so it was never relaxation or pitch, only the ranking); the old prune stalled at
**1149 / 41 g**. So `size_and_prune` now ranks deletions by strain energy too, and it carves **253 members
/ 8.9 g** — grow's 205 plus the ~50 FDM support struts the print version keeps. The impact and bone
numbers are unchanged (both were already grow-based); the fix corrects the `printable`/`ergonomic`/
impact-bolt-on prunes. Guarded by `test_the_prune_carves_a_truss_not_a_membrane`, which fails if the prune
ever weighs more than 2.5× the grow again. It costs one extra FEM solve per prune step (~2× the prune
time); reading the strain energy off the OC's own solve would make it free — a noted follow-up.

| file | sha256 |
|---|---|
| `VISION.md` (as of 2026-07-16, 14:42Z) | `865a5bc43b4751b63f37058f401081e50540ebf6a9db4996dc58d5c405b2b310` |
| `MANIFEST.sha256` (90 files) | `fb13ad82e5f81b40e33b72a8f8e6c8e1f1458134251d55cd2765480b0145f8b4` |

Stamped: **2026-07-16T14:42:32Z** (UTC, submission time). Proofs:
`timestamps/VISION.md.2026-07-16d.ots`, `timestamps/MANIFEST.sha256.2026-07-16d.ots`.

### Thirteenth — CORRECTION: the prune's membrane is not enslavement-specific (§8.15k, claim fff)

The twelfth anchor's (fff) said the *pre-enslavement* design "pruned cleanly to 138 members / 8.5 g" and
only the enslavement design "trapped" — and it read the plateau as the build-support rule. **Both were
published without measuring, and both are wrong.** The old design was gitignored and overwritten, so the
138 came from stale doc numbers, not a run. Measured against the archived pre-enslavement front
(`out_archive/pareto_seed1.pkl`), its knee **also prunes to a uniform membrane (754 members, 27.8 g)**, as
does the current design *unconstrained* — no nozzle floor, no support protection (**1799 members,
62.5 g**). The 138 / 8.5 g truss is from an **older design era** and is not reproducible on any recent
design. The real cause: for the recent design family the buttons sit **62–71 mm from the anchors**, so
keypress load fans out across the dorsal skin and every member carries an equal share — a **membrane**.
Uniform strain energy means the sizer parks every radius at 0.90 mm (**p90/p10 = 1.00**) and greedy
top-down deletion has no signal, so the prune dead-ends in a heavy uniform net: a **local optimum** the
grow (bottom-up, free nodes) sidesteps. Enslavement only made it **1.5× heavier** (754 → 1154) by
extending the fingers ~20 mm farther from the anchors. Render-from-grow stands; the reasoning is now
measured, not inferred.

| file | sha256 |
|---|---|
| `VISION.md` (as of 2026-07-16, 12:13Z) | `d7554838166f05418bfdce07440e704c6e520c22d213d5e3bdd670a18ccd6114` |
| `MANIFEST.sha256` (90 files) | `40a7633e049358ff9ef1de1edad39a9eca7c585406b47337dd1832afd0ee5c8e` |

Stamped: **2026-07-16T12:13:49Z** (UTC, submission time). Proofs:
`timestamps/VISION.md.2026-07-16c.ots`, `timestamps/MANIFEST.sha256.2026-07-16c.ots`.

### Twelfth — RENDER THE BONE FROM THE GROW, not a print-time re-prune (§8.15k, claim fff)

⚠ **Corrected by the current anchor above:** the plateau is a membrane *local optimum*, not the
build-support rule, and it is **not** enslavement-specific (the pre-enslavement design membranes too). The
"138-member" comparison below was unmeasured and is false. The render-from-grow fix itself stands.

Regenerating the gauntlet for the enslavement design exposed a **print-pipeline trap**. The keypress
bone had been re-derived for printing by an independent 8 mm re-prune (`size_and_prune`); on this design
its build-support rule (never delete a node's last down-strut) plateaued it at a **dense 1149-member
skin, 33.9 g hollow** — support-limited, not stiffness-limited (the worst well sat at 338 µm, well inside
the 500 µm gate). The **impact** structure settles it: it carries the keypress *and* the 50 N knock at
**23.2 g**, so a keypress-only bone cannot honestly need 33.9 g. So `bone.py` now renders the **grown**
topology directly — the one the objective already form-found off the grid — only **sizing** it to the
ergonomic floor: **7.54 g** beam / **12.7 g** hollow, 410 members, within 6% of the old committed 12.0 g.
Impact re-optimised to **23.2 g** (was 24.2 g), 39% lighter than the 37.7 g bolt-on. Grow it, don't
re-prune it.

| file | sha256 |
|---|---|
| `VISION.md` (as of 2026-07-16, 09:40Z) | `40cc56d91cf9b307e0e283d3aecfbf85fb235ed3c78261985c459e543a619d26` |
| `MANIFEST.sha256` (90 files) | `6bfc868638a875a8361b238d305413de283a097c9eca6a7e4a6ecced8e269c5c` |

Stamped: **2026-07-16T09:40:08Z** (UTC, submission time). Proofs:
`timestamps/VISION.md.2026-07-16b.ots`, `timestamps/MANIFEST.sha256.2026-07-16b.ots`.

### Eleventh — ENSLAVEMENT, grounded per finger, and the layout re-optimised under it (§6)

The winning layout had posed the **ring extended while its neighbours flexed** — raised 9 mm, and the
design leaned on it (+33% effort and key-overlap when clamped). No hand holds that: MyoHand models the
four long-finger flexors as independent actuators, but they share a belly (the OpenSim hand models
drive the FDP's four slips from one activation). We express that coupling **kinematically, per finger**,
by the **individuation index** — `INDIVIDUATION` (Häger-Ross & Schieber 2000, `Source.LITERATURE`):
the ring may deviate ±0.035 from the common curl where the index may ±0.075, replacing the single
symmetric `COMMON_DRIVE` guess. Re-optimised under it, the ring sits **+2.1 mm** (was +8.9), feasible,
effort and mass comparable — the illusory raise cost nothing to remove.

| file | sha256 |
|---|---|
| `VISION.md` (as of 2026-07-16, 07:25Z) | `1b841e6478146f91f529ceb65a6e73bac48706263ee44d9ec1731e8e53add1da` |
| `MANIFEST.sha256` (90 files) | `81c526a05c0c3d4b3d2009ae83dabb84852b7726d1366652215957c539117827` |

Stamped: **2026-07-16T07:25:50Z** (UTC, submission time). Proofs:
`timestamps/VISION.md.2026-07-16a.ots`, `timestamps/MANIFEST.sha256.2026-07-16a.ots`.

### Tenth — SHELL vs LATTICE: the sandwich weighed by a coupled plate FEA (§8.15k, claim eee)

The render looks like a shell, so an explicit one was weighed end to end. A shell for a knock on the
back is pointless (that knock sizes 0.1 g); the discrete tissue anchors ARE a bottleneck, and a
**coupled lattice + finite-stiffness plate FEA** cuts the worst well-knock stress **96 → 52 MPa** — but
that thins the lattice only ~3–4 g while the shell costs ≥ 5 g, so **no shell beats the pure lattice**
(sandwich ≥ 27 g vs **24.2 g**). The density is fundamental; the shell's value is continuous skin
bearing (comfort), already met by the strap (§8.15j). Recorded with its reversals, the FEA the arbiter.

| file | sha256 |
|---|---|
| `VISION.md` (as of 2026-07-15, 21:46Z) | `f1b19f6a015751771b2d67a827937a82d672ef8018d4b4de41c99b037b7afa9a` |
| `MANIFEST.sha256` (90 files) | `ebad7d7b37c804727bc7e645c997d9514a30dfa0712f430a48d9a894f439d891` |

Stamped: **2026-07-15T21:46:13Z** (UTC, submission time). Proofs:
`timestamps/VISION.md.2026-07-15g.ots`, `timestamps/MANIFEST.sha256.2026-07-15g.ots`.

### Ninth — FORM-FINDING BELONGS IN THE WHOLE PIPELINE (§8.15k, generalised)

The shape-convergence pass generalised, and a correction. The **decoupled** pipelines that make the
definitive structures — size-then-prune-then-curve for the keypress bone, grow-then-co-size for the
impact one — curved their load paths but **never moved the nodes**, leaving them staircased on the
grid. `relax_nodes` (form-finding) belongs in *both*, not only in the render: added to the keypress
bone it drops the flagship **11.05 g → 8.51 g (−23%)**, gate still 499 µm (A/B verified: relaxation
off reproduces 11.05 g exactly). This **corrects** the `relax_nodes` note that the pass is "cosmetic,
not where the grams are" — true only of the grow-front designs it was measured on, which are already
relaxed; on a never-relaxed definitive structure it is worth a fifth of the mass.

| file | sha256 |
|---|---|
| `VISION.md` (as of 2026-07-15, 19:44Z) | `9b106711890b9cb4c91b1b2070a493d9ace1517838a3c20c346573bb5cb6fe58` |
| `MANIFEST.sha256` (86 files) | `6003414c039f78efd4b3cdf86229bceb814f80117e7b44ce6ce1069ea8e320d2` |

Stamped: **2026-07-15T19:44:25Z** (UTC, submission time). Proofs:
`timestamps/VISION.md.2026-07-15f.ots`, `timestamps/MANIFEST.sha256.2026-07-15f.ots`.

### Eighth — the IMPACT STRUCTURE, SHAPE-CONVERGED (§8.15k, revised)

The impact re-optimisation, taken to convergence in *shape*, not only topology. The co-sized skeleton
came off the lattice **staircased** — ~8% of its nodes turned a load path past 75° — and never got the
**form-finding** pass. Adding `relax_nodes` after the sizing straightens it (**kinks > 75°: 40 → 11**)
and, because a starved dense lattice had members sized thick to resist *bending* at their kinks, lets
them carry *axially* instead: **29.3 g → 24.2 g**, so in-the-loop is **34% lighter** than the bolt-on.

| file | sha256 |
|---|---|
| `VISION.md` (as of 2026-07-15, 19:23Z) | `4f47c62663c1ba702823088afec7c7c8eb2132a387af54e230daa0fa70c58a7a` |
| `MANIFEST.sha256` (86 files) | `5d233591b2dde6ac48746c70fb97b96dac2239a40d4074c3ae6427af0b967d54` |

Stamped: **2026-07-15T19:23:27Z** (UTC, submission time). Proofs:
`timestamps/VISION.md.2026-07-15e.ots`, `timestamps/MANIFEST.sha256.2026-07-15e.ots`.

### Seventh — the KNOCK RE-SIZES THE BONE (§8.15k, as first disclosed)

Impact is the binding structural load, not the keypress. A 50 N knock breaks the deflection-optimised
bone (**348 MPa** against a 70 MPa yield), while fatigue has a 16× margin. And the knock wants a
*different* skeleton — broad and load-sharing, not the sparse keypress one thickened: grown with the
knock in the load set, the two topologies share only **20% of their members** (Jaccard 0.20). Growing
WITH the knock and co-sizing for the gate AND the stress is **19% lighter** than bolting the impact on
afterward (before the shape-convergence pass above took it to 34%).

| file | sha256 |
|---|---|
| `VISION.md` (as of 2026-07-15, 17:57Z) | `037c9c00e9f3bd4f68e1d06ee1fa05405a1fb20e6fafd45f1a98e5fa1872a215` |
| `MANIFEST.sha256` (84 files) | `391d83a5a57ad04690d6f63e44d1c59ab7ca95e8ad9d24b308ae2d0240db1507` |

Stamped: **2026-07-15T17:57:47Z** (UTC, submission time). Proofs:
`timestamps/VISION.md.2026-07-15d.ots`, `timestamps/MANIFEST.sha256.2026-07-15d.ots`.

### Sixth — the GAUNTLET ON THE OUTSIDE OF THE STRAP (§8.15j)

The design decision that the *strap*, not the gauntlet, is what meets the hand: the gauntlet mounts
on the OUTER face of the soft TPU strap, so the strap is the sole hand interface — cushion, tension
tether, and load-spreader in one part, attached by loops printed into the strap itself. Re-solved:
the 500 µm gate holds (**499 µm**, +0%) with the soft strap in the load path, because TPU is stiffer
in through-thickness compression than the tissue it sits on. This **supersedes the inner bearing
shell (§8.15i)** as the skin interface.

| file | sha256 |
|---|---|
| `VISION.md` (as of 2026-07-15, 14:31Z) | `d92e96496c21acec7568bbbfe53db0aa1fbba4c6caac4fa556554b4f41d0a7b5` |
| `MANIFEST.sha256` (80 files) | `4775e3d98f00977352b0d077cd85b0d867876eff7e164f1db57c29ee071cc0de` |

Stamped: **2026-07-15T14:31:37Z** (UTC, submission time). Proofs:
`timestamps/VISION.md.2026-07-15c.ots`, `timestamps/MANIFEST.sha256.2026-07-15c.ots`.

### Fifth — the SANDWICH GATE RE-SOLVE (§8.15i)

The sandwich inner face added to the per-element solver, and the 500 µm key-deflection gate re-solved
at the bone's real sections: the buttons hold at **485 µm**, so the face does not compromise
key-crispness (its value is the IMPACT, not the gate).

| file | sha256 |
|---|---|
| `VISION.md` (as of 2026-07-15, 13:43Z) | `772124bf2730861e27aa572b66adb057230a8b4362403353904642b1b2bfec0d` |
| `MANIFEST.sha256` (77 files) | `ce4971b13227485c8944c995465a29696cf6361222e5af854892c1a29197bdd2` |

Stamped: **2026-07-15T13:43:52Z** (UTC). Proofs: `timestamps/VISION.md.2026-07-15b.ots`,
`timestamps/MANIFEST.sha256.2026-07-15b.ots`.

### Fourth — the SENSOR, the STRAP ANCHOR, and the BEARING SHELL (§8.15g–i)

Adds the wearable's two practical subsystems and its skin interface: the **contactless-Hall finger
well** — a magnet on a printed **TPU dome** over a 3-axis Hall — with the flexure material chosen by
**σ_fatigue/E** (the maximum recoverable bending strain) and the plunge that must *bend*, not
compress; the measured result that **every well is five-way** (the ulnar "three-way" limit was a
cradle artefact — the interossei are adequate, and the extensor hood a genuine but *non-operative*
MyoHand gap); the **strap anchor** — the band routed as the convex hull of (skin ∪ device) so it
rides *over* the structure it holds down, a **watch-lug** capturing a pin in shear, one adjustable
strap fitting the 5th–95th percentile hand; and the **inner bearing shell** as an **impact
distributor** — a plate on the soft-tissue elastic foundation, sized by the *knock* not the preload —
built as a **sandwich** with the topology-optimised lattice as its core.

| file | sha256 |
|---|---|
| `VISION.md` (as of 2026-07-15, 09:32Z) | `4d663f98528165d61fa3abbe4327db7dc7e64fb934ba447619bb626d85a6b9ad` |
| `MANIFEST.sha256` (75 files) | `3ebcb9b285d300494a746acddc0d23f233083ec204a69b7e2be079b5222d0138` |

Stamped: **2026-07-15T09:32:00Z** (UTC). Proofs: `timestamps/VISION.md.2026-07-15a.ots`,
`timestamps/MANIFEST.sha256.2026-07-15a.ots`.

### Third — HUMAN FACTORS as the organising principle (§5g), and the whole structural stack

Adds: **human factors as the organising principle** (§5g) — nearly every constraint here is a fact
about PEOPLE, only three are facts about a machine, and reproducibility ("one person, one printer")
is a HUMANIST constraint; the **ergonomic floor** `SKIN_R` and the finding that it, not the nozzle,
is what makes a topology-optimised structure **trabecular**; **curved (spline) load paths**;
**oriented elliptical and stadium sections** and the proof that a circle is the worst section for a
member that bends; and the central measured result — **the device is TOUCH-limited, not
load-limited** (95% of its members are as thick as they are because a HAND must bear them), so **the
bone is HOLLOW**.

| file | sha256 |
|---|---|
| `VISION.md` (as of 2026-07-14, 21:40Z) | `15d99f392e9fe34fdec8908cb602bd49e32349c0667c398d005790318296866b` |
| `MANIFEST.sha256` (63 files) | `f0e4fb4db6eb348e8e760464a40b1d58e5eefab1abe560ad1b2a0a3388335d91` |

Stamped: **2026-07-14T21:40:41Z** (UTC). Proofs: `timestamps/VISION.md.2026-07-14b.ots`,
`timestamps/MANIFEST.sha256.2026-07-14b.ots`.

### Second — the dorsal gauntlet, the structure, the anchor, the manufacture (§8.8–8.14)

| file | sha256 |
|---|---|
| `VISION.md` (as of 2026-07-14, 15:24Z) | `a327fa03b832e334dff709dabfccf7fb8dc01ca760da70f380d64b1930cebb14` |
| `MANIFEST.sha256` (53 files) | `e16f938ae7641e1758f625408e21ca4b6269a8b2893b2205da22a168bf0ebf4b` |

Stamped: **2026-07-14T15:24:50Z**. Proofs: `timestamps/VISION.md.2026-07-14a.ots`,
`timestamps/MANIFEST.sha256.2026-07-14a.ots`.

### Original — the palmar body and the layout method (§8.1–8.7)

| file | sha256 |
|---|---|
| `VISION.md` (as of 2026-07-12) | `a1d7c32e743780be7fee98dccf2ef727d4ea26fda8d2b970862b7357f91232be` |
| `MANIFEST.sha256` (27 files) | `4c45f8cdd21e1f5b48e0ad9852ad195cf5c4a07d89d1b46ba3262ef52367c1e4` |

Stamped: **2026-07-12T22:50:22Z**. Proofs: `timestamps/VISION.md.2026-07-12.ots`,
`timestamps/MANIFEST.sha256.2026-07-12.ots`.

⚠ The original proofs cover the *original* file contents. To verify them you need that version of
`VISION.md` — `git show <commit>:VISION.md`. This is why the manifest is hashed separately: the
manifest pins the whole tree at that instant.

## How the proof works

[OpenTimestamps](https://opentimestamps.org/) aggregates the hash into a Merkle tree and
commits the root into the **Bitcoin blockchain**. Once a block confirms it, the proof shows
the file existed *before that block was mined* — a fact anchored in the most expensive
public ledger in existence, verifiable by anyone, forever, with no trusted third party.

The attestation matures in a few hours (it needs a Bitcoin block). Until then `ots verify`
reports a *pending* attestation from the calendar servers; afterwards it reports a
**Bitcoin block height and time**.

## Verify it yourself

```bash
pip install opentimestamps-client

ots verify VISION.md.ots          # -> "Success! Bitcoin block <N> attests existence as of <date>"
ots verify MANIFEST.sha256.ots

# and check the manifest still matches the tree it covers
sha256sum -c MANIFEST.sha256
```

If `ots verify` reports "pending", upgrade the proof once the block is mined:

```bash
ots upgrade VISION.md.ots MANIFEST.sha256.ots
```

## Belt and braces

The Bitcoin anchor is the strong one, but redundancy is cheap:

- **Zenodo DOI** — archival, independently timestamped, and the venue patent examiners and
  courts actually accept. See `CITATION.cff`.
- **Internet Archive** — snapshot the public repository URL.
- **IP.com / Linux Defenders** — purpose-built defensive-publication venues that examiners
  search.

⚠ Not legal advice.

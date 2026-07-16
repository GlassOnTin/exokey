# Timestamp — proof of prior-art publication date

This repository contains a **defensive publication** ([VISION.md § 8](VISION.md#8-disclosed-variants-defensive-publication)).
Prior art is only worth anything if its **date is provable to a third party**.

**Git commit dates are worthless for this.** They are set by the committer, trivially forged,
and rewritten by any rebase. So the disclosure is anchored independently.

## What is anchored

The disclosure has been **extended and re-anchored** sixteen times. **All seventeen stamps stand**, and each
one proves what was disclosed *at that moment*. An earlier proof is not invalidated by a later one —
it is a *floor* on the date, and floors do not move.

### Current — THE READ-OUT: the field a moving magnet presents to the Hall, and the printed module (§8.15l)

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

Stamped: **2026-07-16T22:09:56Z** (UTC, submission time). Proofs: `VISION.md.ots`,
`MANIFEST.sha256.ots`.

⚠ `TIMESTAMP.md` is deliberately **not** in the manifest. It is written *after* the stamp — it holds
the stamp's own hashes and time — so including it would guarantee `sha256sum -c` failed forever.

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

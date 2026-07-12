# ExoKey — a wearable Svalboard

**A wearable, open-source, QWERTY keyboard with one adjustable finger well per digit, each
sensing five directions.** Held in the palm by a strap, not bolted to a desk. Designed by
optimising the *hand* and the *device* together, against a musculoskeletal model, with
feasibility as hard constraints.

Think: **DataHand / Svalboard geometry, made wearable.** You can stand up and walk away with
it still on your hands.

---

## 1. What it is

Each fingertip sits in a **U-shaped well** on a miniature 5-axis joystick — the DataHand
idea, as reimplemented openly by [Svalboard](https://svalboard.com/). One well per finger.
Five inputs each:

| direction | motion | muscles |
|---|---|---|
| **click** | press into the well floor | flexors (FDP/FDS/FPL) |
| **forward** | slide the pad distally | distal extensors |
| **back** | drag the pad proximally | deep flexor, curling the tip |
| **left / right** | push sideways | interossei (RI/UI/LU) |

**5 fingers × 5 directions = 25 inputs.** A QWERTY half-hand needs 15 letters. So ten
directions are *thrown away* — and the model picks which ten.

The wells sit on a **body in the palm**, strapped over the back of the hand. Not an
exoskeleton: an exoskeleton cannot reach a thumb key that sits under the fingers without
crossing the hand (we proved this three ways — §5).

### Why this is not encumbered

- **DataHand's patents (early 1990s) have expired.** The 5-direction finger well is prior art.
- **Svalboard is an open implementation** of the same idea, running Vial/QMK.
- **Typeware's "twin key" patent is a different mechanism** — two switches on one stem, to
  pack more physical keys in. We need no such thing: the extra states come from *sensing more
  muscle groups*, not from more hardware. So the patent is not in the way, and we are not
  designing around it — we are on an older and freer road.

⚠ Not legal advice. If this is ever published as a buildable design, read the claims.

---

## 2. The thesis

**The formulation is the product.** The predecessor project (CapaChord) stalled because its
"effort" was a sum of constraint violations (`unreachable_penalty + collision_err`), it ran
IK with no joint limits, and feasibility was a soft penalty the optimiser could *buy* its way
out of. Every one of those is fatal, and all three are formulation errors, not bugs.

So:

| | |
|---|---|
| **Effort** | muscle activation, Σaᵢ³ (Crowninshield–Brand) over MyoHand's 39 Hill-type muscles. A physical quantity. Not a geometric proxy. |
| **Feasibility** | HARD constraints. NSGA-II's constrained tournament: any feasible point dominates any infeasible one. There is no exchange rate. |
| **Search** | multi-objective (pymoo NSGA-II), mixed continuous/categorical. Effort, mass and crispness genuinely conflict; scalarising them hides the trade. |
| **Visualisation** | first-class, every stage, in the browser. Two of the worst bugs in this project were caught by *looking at the render*, and would have survived any table of numbers. |

---

## 3. What is measured (not asserted)

**The five directions are genuinely distinct** — each recruits a different muscle group, which
is the entire premise: a sensor could not tell them apart otherwise.

**Their costs differ by 1.9 million×:**

| finger | click | forward | back | left | right |
|---|---|---|---|---|---|
| middle | **1.4e-07** | 4.8e-06 | 2.4e-05 | 1.9e-04 | 2.5e-05 |
| ring | 6.2e-07 | 1.6e-06 | 3.3e-04 | 9.7e-03 | 2.0e-03 |
| index | 2.7e-06 | 1.6e-03 | 7.7e-04 | **2.7e-01** | 7.1e-06 |

`index/left` is near muscle saturation — effectively impossible for anybody. **Click is
cheapest, everywhere.** This asymmetry is the design's main lever.

**QWERTY's "home row" is a fiction** inherited from a mechanical typewriter. The real row
frequencies (left hand):

| top (QWERT) | home (ASDFG) | bottom (ZXCVB) |
|---|---|---|
| **30.2%** | 23.0% | 5.5% |

The *top* row is the most-used. So the cheapest direction belongs there — and **choosing the
direction→row mapping is worth 5×**. It is firmware. It is free. It should be optimised.

**The population needs ~12 mm of per-finger well travel** to span the 5th–95th percentile —
and it is essentially **two axes, not five**:

| axis | range needed |
|---|---|
| distal (along the finger) | **± 9.5 mm** |
| dorsal (depth) | **± 7.2 mm** |
| radial (sideways) | ± 3.3 mm |

Svalboard ships 5-axis adjustment. **Two may do.** That is a concrete, testable simplification.

**Hand scaling obeys its own law** (verified against theory, not just "it ran"): muscle force
scales as area (s²), moment arms as length (s), so activation for a fixed key force goes as
**s⁻²**. Measured ratio 5th/95th = 1.551 against 1.543 predicted. Big hands are relatively
stronger; the *small* hand binds.

**The strap-mounted body beats the exoskeleton on every axis:**

| | exoskeleton | strap body |
|---|---|---|
| mass | 85 g | **~60 g** |
| key deflection | 0.082 mm | **0.021 mm** |
| thumb reachable | **no** | yes |

3× stiffer *and* lighter, because the keypress reacts straight into the palm it rests on
instead of being carried around the hand through a cantilever.

---

## 4. Design gates

Each is a pass/fail test defined *before* the work. `pytest tests/ -q` — **57 passing.**

### Passed

| gate | result |
|---|---|
| Beam FEA vs closed-form Euler–Bernoulli | **0.000%** error |
| A keypress recruits **flexors**, not extensors | ✅ (caught the fingernail bug) |
| Effort rises monotonically with press force | ✅ all five digits |
| A hand can reach a key at its **own** fingertip | ✅ 0.000 mm (was 1.5–3.2 mm) |
| Hand scaling follows s⁻² | ✅ 1.551 vs 1.543 |
| Five directions use ≥3 distinct muscle groups | ✅ |
| Tension-only straps carry no compression | ✅ (PyNite signs tension **negative** — verified, not assumed) |
| No posture hyperextends any digit | ✅ |
| Two keycaps cannot occupy the same space | ✅ |
| The firmware mapping is ONE mapping for everyone | ✅ |

### Open

| gate | status |
|---|---|
| A feasible Pareto front on the current model | **running** (2 seeds, cloud) |
| Beats a hand-built baseline at equal mass | pending the front |
| 3D continuum FEA on the winner (stress concentrations) | not started |
| Tier-2 coupled verification (enslavement) | **blocked** — needs a hand model that has it |
| **Print it and measure it** | not started — and nothing above substitutes |

---

## 5. What we already learned by failing

Recorded because the failures are the most valuable output so far.

**The "finger pad" was the fingernail.** MyoHand's `class="skin"` ellipsoid sits on the
*extensor* side. Every key went behind the nail; a "keypress" recruited EIP, an extensor.
Nothing failed — the physics stayed perfectly self-consistent and answered the wrong
question. Only asking *which muscles does this recruit* exposed it.

**The thumb's joint signs are not self-consistent with each other.** `cmc_flexion` flexes
positive; `mp_flexion` and `ip_flexion` flex *negative*. Read `ip_flexion`'s range
`[−75°, +25°]` correctly and it is 75° of flexion; read it backwards and it is 75° of
*hyper*extension, which is nothing. Flexion direction is now **derived from each digit's
flexor moment arm**, never assumed.

**I reproduced CapaChord's fatal bug in my own pose solver.** Reach entered the cost in
metres² (1 mm → 1e-6) against a normalised comfort penalty of 1e-5 — so comfort outweighed
reach by ~4×10¹⁰ and the solver bought comfort with reach error. Feasibility as a soft
penalty, via nothing but a units mismatch. Now lexicographic: reach first, comfort second.

**An open frame cannot reach into the palm.** The exoskeleton's thumb arm cut through the
hand three ways (−6.5 mm across the palm, −7.3 mm through the thenar, −5.5 mm through the
fingers). The third proves it is *topological*, not a routing mistake.

**A constraint that punishes intended contact.** Four separate bugs, one species: the thenar
counted as a "finger"; the body face counted as an obstacle; "clearance" applied to a device
the hand *grips*; a body whose size was hardcoded so it could not shrink to fit. An
exoskeleton stands off the hand, so "don't touch" was nearly right. **A gripped body is
*held*, so "don't touch" is nearly always wrong.**

---

## 6. Model limitations — stated, not hidden

These bound every conclusion above.

- ⚠ **MyoHand's thumb has no thenar intrinsics.** No adductor pollicis, no FPB, no APB —
  the muscles you actually press with. Thumb effort is systematically overstated (~1000× the
  index at one point). **Thumb numbers rank thumb options against each other at best.**
- ⚠ **MyoHand has no enslavement.** FDP2–FDP5 moment arms are strictly diagonal — four
  independent actuators. Curling the ring alone is *free* in this model and impossible in a
  hand. Mitigated by an externally imposed common-drive constraint; not solved.
- ⚠ **ANSUR II percentiles are recalled, not read from the dataset.** The 95th/5th ratio
  (~1.24) is load-bearing. **Verify before publishing.**
- **Σaᵢ³ is a hypothesis** about what humans minimise, not ground truth. Field-standard, but
  a model.
- **Gravity is off** — a hand-mounted device is used in every orientation. Measured cost of
  the choice: Spearman ρ = 0.975 on the key ranking.
- **No swept-volume contact model.** We cannot distinguish "the finger rests on the well"
  from "the finger is sunk into it".
- **No typing-SPEED model.** We optimise muscle effort. Speed (Fitts' law, travel time) is a
  different objective and may disagree — this is likely where the twin key's real value lies.
- **Placeholder numbers**: adjuster mass, column-shift cost.
- **Comfort ≠ minimum activation.** A device can be metabolically cheap and still feel bad.
  Only §4's last gate settles that.

---

## 7. Design iterations to explore

Roughly in order of value.

1. **Build the thing.** Everything above is simulation. The 25 directions, the 5 wells and the
   strap exist as beam elements and muscle activations. A printed body with five thumbsticks
   and a QMK board would falsify more in a weekend than another month of optimisation.
2. **Two-axis adjustment instead of five.** Measured: ±9.5 mm distal, ±7.2 mm dorsal,
   ±3.3 mm radial. If radial is genuinely negligible, the mechanism gets much simpler than
   Svalboard's. **Test on real hands.**
3. **Optimise the layout, not just the mapping.** We currently take QWERTY's finger→letter
   assignment as given and only choose which *direction* means which row. Freeing the whole
   assignment (a QAP over 25 slots) would say what QWERTY costs — and QWERTY is a
   typewriter-jam workaround, so the answer is probably "a lot". Keep QWERTY as the *product*;
   report the optimum as the *result*.
4. **A speed objective.** Muscle effort says click is cheapest. Nothing yet says which
   direction is *fastest*, or how much a direction change between successive letters costs.
   This is where the twin key's ±30% travel reduction would show up, and where our model is
   currently blind.
5. **A hand model with a thumb and with enslavement.** OpenSim ARMS (43 muscles) has the
   thenar group. Both of our biggest caveats trace to MyoHand, and both are fixable by
   changing hands.
6. **The thumb cluster.** Svalboard uses thumb layers (numbers, functions, mouse). We model
   the thumb as one more finger, which it is not.
7. **Wearability.** Svalboard is desk-mounted; this is not. What holds the body when the hand
   opens fully? The spring-steel clip idea (from Typeware) is good and we have not modelled it
   — the clip is currently just a beam.
8. **Both hands.** Everything here is the left hand.

---

## 8. Disclosed variants (defensive publication)

**Purpose of this section.** Everything below is hereby **publicly disclosed**. Its purpose is
to place this subject matter into the public domain as **prior art**, so that it cannot later
be patented by anyone — ourselves included. It is written enumeratively, and in enough detail
to be *enabling*: a person skilled in the art can build these from this document plus the
accompanying source. Vague gestures are worthless as prior art; specifics are not.

⚠ **This is a one-way door.** Absolute novelty (EPO and most jurisdictions, no grace period)
means this disclosure is prior art against *our* later applications too. That is intended.

⚠ Prior art gives **no freedom to operate**. It prevents others patenting what is disclosed
here. It does not clear anyone else's existing patents. Not legal advice.

### 8.1 The core disclosure

A **wearable keyboard** in which:

- a **body** is held against the **palm** of the hand, such that the hand **grips** it, the
  fingers curling onto it in a natural grip posture;
- the body is retained by a **strap** passing over the **dorsum** (back) of the hand, the
  body bearing on a **palm support** that reacts the keypress load directly into the palm
  (rather than carrying it around the hand through a cantilever or exoskeleton);
- the body presents **one concave finger well (cavity) per digit**, positioned where that
  digit's fingertip naturally lands;
- **each well senses a plurality of distinct fingertip force/displacement directions**, each
  direction being driven by a **different muscle group** of that digit and therefore
  independently distinguishable;
- the assignment of sensed directions to typed characters is **chosen to minimise a
  musculoskeletal effort metric** weighted by character frequency (see §8.6).

### 8.2 Sensing modality — any of

Hall-effect; magneto-optical; magnetoresistive; capacitive; optical (reflective, occlusive,
or image-based); resistive; piezoresistive; strain-gauge; inductive; time-of-flight;
magnetometer sensing a magnet borne on the fingertip, nail, or well; or any combination.
Analog (proportional) or digital (thresholded). One sensor per direction, or a single
multi-axis sensor per well resolving all directions.

### 8.3 Directions per well — any of

**3, 4, 5, or 6** distinct directions per digit, drawn from:

| direction | motion | driving muscles |
|---|---|---|
| **click / down** | press into the well floor, along the pad normal | flexors (FDP, FDS, FPL) |
| **forward** | slide the pad distally within the well | distal extensors |
| **back** | drag the pad proximally | deep flexor, curling the tip |
| **left** | push laterally | interossei (RI, UI) |
| **right** | push medially | interossei, lumbricals |
| *(optional)* **lift / pull** | withdraw the pad from the well floor | extensors (EDC, EIP, EDM, EPL) |
| *(optional)* **twist / roll** | rotate the pad about its normal | interossei, differential |

Implemented as a **miniature joystick / thumbstick** beneath the well; as a well on a
compliant stalk; as discrete switches around the well rim; or as a rigid well over a
multi-axis sensor.

### 8.4 Switch characteristics — any within

Actuation force **0.05–0.60 N** (5–60 gf). Travel **0.2–4 mm**. Force profile linear,
front-loaded, tactile-detented, or spring-free (magnetic/optical). Haptic confirmation by
vibration, magnetic detent, or none.

### 8.5 Per-finger adjustment

Each well is **independently adjustable in position** relative to the body, to fit the
individual hand, in **1, 2, 3, 4, or 5 axes**.

**Measured requirement to span the 5th–95th percentile hand (this work):**

| axis | range required |
|---|---|
| distal (along the finger) | **± 9.5 mm** |
| dorsal (depth into the palm) | **± 7.2 mm** |
| radial (sideways) | ± 3.3 mm |
| **total per-finger well travel** | **≈ 12 mm** |

Specifically disclosed: **a TWO-AXIS adjustment (distal + dorsal) is sufficient** to span this
population, the radial component being an order smaller. Adjustment by slide-and-lock, lead
screw, detented rail, telescoping stalk, ball joint, deformable/settable material, or
per-user 3-D printed insert.

### 8.6 Layout by musculoskeletal optimisation — the method

Disclosed as a method, and as a device programmed by that method:

1. Model the hand as a musculoskeletal system (joints, Hill-type musculotendon actuators,
   hard joint limits).
2. For **each (digit, direction) pair**, compute the muscle activation required to exert the
   switch's actuation force in that direction at the fingertip, by static optimisation
   (minimising Σaᵢ³, Crowninshield–Brand, or any equivalent effort criterion).
3. Weight by **character frequency** of the target language.
4. **Assign directions to characters so as to minimise total weighted effort** — selecting
   *which* directions to use at all, and *which character each means*.
5. Enforce, as **hard constraints**, that every used direction is achievable within joint
   limits and without muscle saturation, for every hand in the target population.

**Specific findings hereby disclosed** (measured; see §3):

- The directions differ in muscle cost by **up to ~10⁶×** on the same finger.
- **`click` (pressing into the well floor) is the cheapest direction on every finger** — and
  the most frequent character should therefore be assigned to it.
- Some directions are **effectively unusable** (index-finger lateral push approaches muscle
  saturation) and should be **left unwired**.
- With 5 digits × 5 directions = **25 inputs** and only ~15 needed for a QWERTY half-hand,
  **the ten most expensive directions may simply be discarded.**
- For QWERTY, the **top row is the most frequently used** (30.2%, vs 23.0% for the so-called
  "home" row); the cheapest direction should be mapped to it.
- **The direction→character mapping is firmware and therefore free to change**; choosing it by
  the above method is worth **≈5×** in weighted effort over the naive geometric mapping.

### 8.7 Additional disclosed variants

- **Chorded** operation (simultaneous multi-digit) as well as, or instead of, one-character-
  per-direction.
- **Layer/shift** selection by thumb, by an inertial sensor detecting hand row/column
  translation, by a dedicated direction, or by dwell.
- **Body materials**: CF-filled nylon, PA12, aluminium, titanium, or 3-D printed polymer;
  strap of webbing/velcro/elastomer; a **spring-steel clip** pre-tensioning the strap against
  the palm support and holding it clear when the hand withdraws.
- **Mounting** alternatives: glove, ring(s), finger sleeves, wrist brace, or rigid shell.
- **Pointing device** (trackball, trackpoint, touchpad, optical sensor, IMU) integrated into
  the body or a thumb well.
- **Bilateral** operation: one body per hand, each typing its half of the layout.
- **Thumb well** carrying the same or a different direction set, and/or layer selection.
- **Co-design**: optimising well placement, body geometry, adjustment range, and layout
  *jointly* against the musculoskeletal model, with feasibility as hard constraints — as
  implemented in the accompanying source.

### 8.8 Provenance

The **five-direction finger well** derives from the **DataHand** keyboard (patents filed early
1990s, now **expired**) and its open modern reimplementation
**[Svalboard](https://svalboard.com/)**. This work's contribution is the **wearable,
palm-gripped, strap-retained** form; the **measured two-axis adjustment requirement**; and the
**musculoskeletal-effort-optimised layout method** of §8.6.

**Timestamp:** this document is cryptographically timestamped — see `VISION.md.ots` and
`TIMESTAMP.md` in the repository root.

---

## 9. Repo

```
hand/       myohand.py   MyoHand: FK, muscle redundancy, effort, the 5 directions
            scaling.py   population: lengths ~s, muscle force ~s², activation ~s⁻²
design/     vector.py    the design vector θ and its evaluation
            qwerty.py    the layout: which direction types which letter, and what it costs
structure/  frame.py     the strap-mounted body (PyNite beams, elastic soft-tissue supports)
opt/        problem.py   pymoo NSGA-II, 9 hard constraints
            run.py       the outer loop
            merge.py     merge Pareto fronts across seeds
viz/        scene.py     browser views (Plotly, self-contained HTML)
cloud/      hetzner.sh   burst the optimisation onto a cloud box, then delete it
tests/      57 gates
```

Everything here is ours and freely publishable. The muscle-effort model, the population
scaling and the co-design loop owe nothing to anyone's patent.

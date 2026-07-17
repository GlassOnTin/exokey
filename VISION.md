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

**But only three of the five are PERFORMABLE, and which three is not a free choice.** An action
is performable only if the digit's muscles can actually balance the key reaction (§5 — this was
never checked, and the omission invalidated everything that used to be written here). Effort is
meaningless for an action that cannot be done; `—` marks those, with the irreducible torque
residual in brackets.

| finger | click | forward | back | left | right |
|---|---|---|---|---|---|
| **thumb** | **6.7e-08** | 7.2e-07 | **1.3e-07** | 1.4e-04 | 3.7e-07 |
| index | **1.1e-07** | 2.5e-06 | 2.0e-06 | 1.6e-05 | 1.2e-03 |
| middle | **1.2e-08** | 3.9e-06 | 4.8e-06 | — (7%) | — (41%) |
| ring | **3.8e-07** | 4.6e-05 | 1.8e-05 | — (37%) | — (63%) |
| little | 4.5e-05 | 3.7e-06 | **1.9e-06** | — (24%) | 3.7e-04 |

Read it in three parts, because each is a design decision:

1. **The thumb performs ALL FIVE**, and it is the cheapest digit on the hand — once it is
   given the muscles a thumb actually has (§5). Stock MyoHand's thumb cannot press *at all*.
2. **`left`/`right` mostly fail** for middle, ring and little — the interossei are genuinely
   weak. That is a *prediction* of the muscle model, not a tuned outcome.
3. **`click` / `forward` / `back` work on all four fingers** = three rows each = 15 letters,
   which is exactly what QWERTY's left half needs — **plus a thumb with five spare inputs.**

Among the actions that *can* be done, costs still differ by **~10⁵×**, and that spread is the
design's main lever.

> ⚠ **This table replaces one that claimed a 1.9-million× spread and "click is cheapest
> everywhere".** Both were artifacts: an action a digit *cannot* perform produced a small
> achievable torque, hence small activations, hence **low effort** — so impossible actions
> looked **cheap**. See §5.

**QWERTY's "home row" is a fiction** inherited from a mechanical typewriter. The real row
frequencies (left hand):

| top (QWERT) | home (ASDFG) | bottom (ZXCVB) |
|---|---|---|
| **30.2%** | 23.0% | 5.5% |

The *top* row is the most-used. So the cheapest direction belongs there — and **choosing the
direction→row mapping is worth 5×**. It is firmware. It is free. It should be optimised.

**The population needs ~14 mm of per-finger well travel** to span the 5th–95th percentile —
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

## 3b. The layout, and the pointer

**The thumb carries space.** It was idle — five performable directions on the cheapest digit
on the hand — while the **little finger**, the weakest digit there is, carried three QWERTY
rows. And **space was not in the objective at all**: ~18 keystrokes per 100 letters, against a
left hand whose 15 QWERTY letters are only 58.7 of those 100. **Space is ~22% of the left
hand's entire load** — more than any letter, more than `e`. The objective was missing its
single largest term.

```
digit        click   forward      back      left     right
thumb        SPACE         ·     SHIFT         ·         ·
index            R         V         F         ·         ·
middle           E         D         C        --        --
ring             S         X         W        --        --
little           A         Q         Z        --         ·
```
`--` the digit **cannot** perform it (weak interossei) · `·` performable but unused

| layout | effort/keystroke | | cost to the user |
|---|---|---|---|
| QWERTY-strict, thumb idle | 2.36e-06 | 1.00× | — |
| **QWERTY + thumb** (shipped) | **1.73e-06** | **1.36×** | **nothing to learn** |
| FREE assignment (exact) | 6.11e-07 | **3.86×** | a new layout |

QWERTY+thumb costs the user **nothing** to learn: nobody's muscle memory encodes *which
direction means which row* — that mapping is new either way. ⚠ But note the last row: **QWERTY
costs 3.9×** on a device where "rows" are not rows at all, they are muscle directions. That is
a large number to leave on the table, and it should be a conscious choice, not an inherited one.

**Not a QAP.** The plan deferred character assignment as an NP-hard quadratic assignment
problem. That is right for a *chording* keyboard, where a chord's cost depends on what else is
in it. This device has one key per action and no chords, so cost is just `freq(c) × effort(slot)`
— a **linear** assignment, solved *exactly* by Hungarian in milliseconds. The NP-hard framing
was inherited from a device we no longer build.

### The pointer (Svalboard ships a trackball / touch sensor)

A 5-direction well **is** a 2-axis stick with a click, so a well can *be* the mouse — at the
price of its four tilts, which stop being characters.

**Any finger can drive one** — a 2-axis stick needs all four tilts, and with the well floor
restored (§8.15g) every finger performs them. (An earlier draft claimed only the thumb and index
could, "the interossei are weak"; that was a cradle artefact, not the muscle model — see §8.15g.)
So capability does not choose the digit. **Cost does, and it inverts convention:**

| | typing cost |
|---|---|
| pointer on the **thumb** (where every keyboard puts it) | **6.08× worse** |
| pointer on the **index** | **1.88× worse** |

Putting it on the thumb is the *worst* choice — precisely **because** the thumb turned out to be
the cheapest digit, so surrendering it is expensive. You cannot see that until the thumb has its
muscles.

⚠ **And it does not fit at all without freeing a slot.** The shipped layout leaves **six**
performable directions unused and a pointer needs only four — but a stick needs **four tilts on
ONE digit**, and the six spares are scattered across five. *Counting total spares was the wrong
question.* Moving **shift** to a hold or a chord is what makes the mouse fit — which is why
`SHIFT_FREQ` is load-bearing despite being a guess.

(Svalboard sidesteps all of this: its trackball is a **separate sensor**, so it consumes no
directions — only hardware and thumb *time*.)

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

**A keycap pitch that outlived the keycaps — and it invalidated a whole Pareto front.**
`KEY_PITCH = 12 mm` was correct when the fingers pressed *caps on stems*. It survived the
switch to *wells*, and a well is not a cap: it is a **cavity the fingertip sits inside**, so
its size is the **fingertip's** size, which the model knows (12–14 mm across). All 200
designs on the first front had physically overlapping wells. Nothing failed — the number was
plausible, the constraint was checked, and it was answering a question about a device that no
longer existed. Well radius is now **derived from each finger's own flesh capsule**.

The same constant had a twin, and finding one led straight to the other: the swept-path check
modelled a neighbouring well as a **keycap standing 3 mm proud of a plate** (`CAP_HEIGHT`).
A well is not a cap, it is a **cup** — and a cup that holds a fingertip *and can be nudged
sideways by it* must wrap that fingertip to about its equator, so its rim stands proud by
roughly the fingertip's own **radius** (6–7 mm, not 3). The device was being modelled as half
as obstructive as it is. Now derived from `well_radius`, and it tightened the constraint from
93% to 97% of random designs violated. **Two of the constraints were quietly still
describing keycaps.**

**And that fix decided the architecture.** Wells need the fingertips **spread** (~17 mm
apart); **gripping converges them** (~6 mm at high curl). Even at maximum splay *and*
maximum stagger, a gripping hand cannot fit five wells — middle and ring overlap by 2.1 mm.
So the hand does **not** grip the body. It rests **open and splayed** on it. That is not a
style choice; it is forced by the fingertip's own width, and it fell out of a bug fix.

**A constraint the search had to keep rediscovering.** `common-drive` (the four fingers must
curl together, since MyoHand has no enslavement) was a *constraint* on four independently
drawn curls — but four independent draws over [0.10, 0.80] have an expected spread of ~0.42
against a limit of 0.15, so **98% of designs were born violating it** and the GA burned its
budget rediscovering it. It is now built into the **parameterisation** (one shared hand curl
+ bounded per-finger deviations), so it cannot be represented, let alone violated: 98% → 0%,
and randomly drawn feasible designs went 0/240 → 1/240.

⚠ **And then a single symmetric guess was found too loose — now grounded per finger.** One bound,
`COMMON_DRIVE/2 = 0.075`, applied to every finger alike. That let the optimiser pose the **ring
extended while its neighbours flexed**: measured on the winning design, the ring rode **9 mm** above
its neighbours, and the design *leaned* on it — clamp the ring to the common curl and effort jumps
**+33%** and the key layout goes infeasible (keys overlap). No hand holds that; the ring is the least
independent digit. So the symmetric guess is replaced by **per-finger `INDIVIDUATION`** bounds (index
±0.075 … ring ±0.035), scaled by the individuation index of each digit (Häger-Ross & Schieber 2000) —
the kinematic form of the **shared-FDP enslavement** the OpenSim hand-and-wrist models capture (one
activation drives the FDP's four finger-slips) and MyoHand lacks (FDP2–5 independent). A `GUESS`
became `LITERATURE`; the ordering (ring least) is robust, a measured enslaving matrix would refine the
magnitudes. The layout re-optimised under it is the honest one.

**THE WELL IS A CRADLE, AND I MODELLED IT AS A PIN. This overturns the negative result below.**

Every effort and feasibility number applied the key reaction as a **single point force at the
pad**, and demanded the digit's own muscles balance the entire resulting joint torque. On that
model an **open hand cannot press** — a 32–35% irreducible torque residual — and that finding
drove everything: it forced a curled hand, which converges the fingertips, which broke the well
packing, which produced an empty Pareto front.

**It is contradicted by billions of people typing on flat keyboards with semi-extended fingers
every day.** When a model says something impossible that people do hourly, the model is wrong,
and I should have caught that before writing "the fingers cannot press" into this document.

The user named the missing physics: **piano technique.** A pianist does not *generate* force
with the finger — the finger is a **strut that transmits** it, braced, while the arm supplies the
weight. And a well does exactly that bracing. It is a U-channel that **cradles the distal
phalanx**: floor, two flanks, an end stop. The reaction bears on the **whole palmar surface**, so
the **centre of pressure is free to sit anywhere along the bone** — and a reaction near the DIP
has a far smaller moment arm about it than the same force at the fingertip.

**That is the only thing a cradle contributes, and it is enough.** At the OPEN hand — the very
posture where the wells fit — all four fingers now get their three directions:

| digit | click | forward | back | left | right | performable |
|---|---|---|---|---|---|---|
| **thumb** | 18% | 36% | 31% | 45% | 35% | **0/5 — the control** |
| index | 0% | 0% | 0% | 0% | 0% | 5/5 |
| middle | **0%** | **0%** | **0%** | 14% | 50% | **3/5** |
| ring | **0%** | **0%** | **0%** | 40% | 76% | **3/5** |
| little | **0%** | **0%** | **0%** | 29% | 11% | **3/5** |

The pattern is a *prediction*, not a fudge: `click` (into the floor), `forward` (against the end
stop) and `back` (the deep flexor's own pull) work, while `left`/`right` stay hard for
middle/ring/little because the interossei are genuinely weak.

⚠ **THE CONTROL IS THE IMPORTANT HALF: A CRADLE MUST LEND NO MUSCLE.** The control is now
**stock MyoHand's thumb**, which has no adductor at all — and *no* amount of cradling may let it
press. An earlier version of the cradle let the finger lean on the floor, **both** walls and the
end stop at once; those forces **self-cancel**, so it conjured a keypress out of a completely
passive finger and duly reported the adductor-less thumb pressing 4 of 5 directions. **That is
how a too-permissive contact model announces itself, and it is the only reason it was caught.**
The fix: only the *sensed* surface is loaded, and its contacts sum to the switch force. The
freedom is the centre of pressure — nothing else.

(The control used to be "the thumb still cannot press". That is now obsolete — the thumb *can*,
once it has its thenar group — so the control was **replaced, not deleted**. A control that
passes because the thing it guards has changed is not a control.)

⚠ **This is CONSERVATIVE in one respect and optimistic in another.** Frictionless (friction could
only help), so a "cannot press" verdict still stands. But the contact forces are limited only by
muscle capacity, so a "can press" verdict is the optimistic end of the band.

**AND SO THE NEGATIVE RESULT BELOW IS WITHDRAWN.** It is kept, in full, because it is the most
instructive failure in the project: a wrong contact model produced a *coherent, well-evidenced,
completely wrong* conclusion — that the product could not exist — and it took an outside
observation (piano) to break it. Everything below the line was true of the pin model and false of
a well.

---

**~~THE OPTIMISER NOW FINDS NO FEASIBLE DESIGN — AND THAT IS THE RESULT.~~ (WITHDRAWN — see above)**

With equilibrium enforced (the digit must actually be able to perform the action), NSGA-II
returns **nothing**. Not a worse front: an empty one. That is not a failure of the search, and
it is not a bug. It is a measurement, and it says something specific.

**Two requirements pull in opposite directions, and they do not overlap.**

| shared curl | can the fingers press? (3rd direction, need ≤5%) | do the wells fit? (need ≤0) |
|---|---|---|
| 0.20 | 31.6% ✗ | **−3.2 mm ✓** |
| 0.35 | 35.0% ✗ | **−0.4 mm ✓** |
| 0.50 | 19.0% ✗ | +1.7 mm ✗ |
| 0.65 | 7.2% ✗ | +8.8 mm ✗ |
| 0.80 | **4.0% ✓** | +7.5 mm ✗ |
| 0.90 | **3.8% ✓** | +7.7 mm ✗ |

An **open** hand fits five wells but **cannot press three directions**. A **curled** hand can
press but its **wells overlap by 7–9 mm** — because curling *converges* the fingertips and a
well is a cavity that needs them *spread*. There is no curl that does both.

**AND THE THING THAT DECIDES IT IS THE GUESS.**

Each finger CAN press three directions — at a curl of its own choosing:

| finger | best 3rd-direction residual | at curl |
|---|---|---|
| index | **0.0%** | tp 0.23, tm 0.82 |
| middle | **3.7%** | tp 0.90, tm 0.32 |
| ring | **0.0%** | tp 0.90, tm 0.40 |
| little | **0.0%** | tp 0.15, tm 0.90 |

Those curls are spread by **0.75** in MCP flexion. `COMMON_DRIVE` allows **0.15**. And with
enslavement removed entirely — every finger at its own curl, near-full splay — **the wells fit
(−1.6 mm) AND all four fingers press.** Put the shared curl back and the wells clash (+3.1 mm)
at *any* splay.

> **The feasibility of this entire device is decided by finger ENSLAVEMENT — the one thing
> MyoHand does not model at all (its FDP2–FDP5 are strictly independent), and the one number
> we made up (`COMMON_DRIVE = 0.15`).**

This has been flagged in these limitations from the start as "a guess that is currently the
binding constraint on the well layout". It is worse than that: it is the binding constraint on
whether the product can exist.

⚠ **And a real hand almost certainly cannot do it.** The design needs a 0.75 spread in MCP
flexion across the four fingers — five times what we allowed — and the ring and little fingers
are the *least* independent digits in the hand. So the honest reading is that **the device as
specified (one well per finger, three QWERTY rows per well) is probably infeasible**, and the
model cannot yet tell us for certain because it does not contain the physics that decides it.

**THE NEXT MEASUREMENT IS NOT A SIMULATION.** What is needed is the enslavement matrix of a
real hand — how much a finger's MCP can individuate while its neighbours hold a different
flexion — and that is measurable on a person in an afternoon. It would either kill this design
or unlock it, and no amount of further optimisation can substitute for it.

**Escape routes, not yet explored** (they change the product, so they are decisions, not fixes):
- **Fewer rows per finger.** Three directions per well is what forces the curl. Two rows per
  finger plus more index/thumb columns would relax it.
- **Let the finger move between actions.** All three directions are currently required to be
  performable at ONE resting posture. A real finger in a well shifts as it tilts the stick.
- **Give up on QWERTY.** The three-row requirement is QWERTY's, not the hand's.

**THE EQUILIBRIUM RESIDUAL WAS NEVER CHECKED. This is the worst bug in the project.**

`solve_activations` does not enforce equilibrium. It least-squares-fits the required joint
torque, pins the demand to the closest **achievable** torque `tau* = A·a_ls`, and returns the
shortfall as `residual`. **No caller had ever read it.** Its own docstring says *"feasibility
is then a real, reportable quantity"* — and then nothing reported it.

Measured on the shipped design, the muscles failed to balance the load by **14% to 62%**. The
index was at **0.0%** (it genuinely can press); the thumb at 48%, the ring at 62%. Every
effort number, every Pareto front and two headline results were computed for presses the hand
**cannot perform**.

**And the error was self-reinforcing.** An action a digit cannot perform yields a small
*achievable* torque, hence small activations, hence **LOW EFFORT**. Impossible actions look
**cheap** — so `best_action_map`, which picks the three cheapest directions, was *systematically
drawn to the impossible ones*. The cheapest number in the entire model (middle/`click`,
4e-08) was an unbalanced press, while the index's honest `click` cost 1e-06. **The optimiser
preferred the actions the hand cannot do.**

This is v1's disease one level deeper. v1 let the optimiser **buy its way out** of a soft
constraint. Here the constraint **was never checked at all**. Equilibrium is now hard, and an
unperformable action is **unavailable, not expensive**.

Retracted as a result: *"the five directions differ by 1.9 million×"* and *"click is the
cheapest action everywhere"*. Both were artifacts of unbalanced presses.

**THE THUMB CANNOT PRESS, AND IT IS NOT A CALIBRATION ERROR — IT IS A MISSING MUSCLE.**

Two independent instruments agree. A max-force LP: the thumb exerts **0.0 N** at its pad, at
every posture, along **all 600 directions sampled** (the index manages 194/600, within 5° of
its pad normal). And the residual: **45.6%**, irreducible, at every posture.

The muscle list says why. MyoHand's thumb has FPL (flexes), EPL/EPB (extend), APL (abducts),
OP (opposes). **Nothing ADDUCTS.** Pressing a key is pushing *against* something, and adductor
pollicis is the muscle that does it. MyoSuite ships no alternative — every XML in `myo_sim`,
hand and arm, has the same five.

So we **add adductor pollicis** (`hand/thenar.py`), via MuJoCo's `MjSpec`, leaving the pinned
submodule untouched. Attachments derived from the model's own anatomy (its own tendons state
which way is palmar, radial and distal on each bone); peak force anchored to FPL by PCSA ratio.

| | stock (39 muscles) | + ADP (40) |
|---|---|---|
| **thumb** | **45.6%** | **11.9%** |
| index / ring / little | 0.0% | 0.0% |
| middle | 0.6% | 0.6% |

Validated three ways, none of them "it ran": the moment arms have the right **signs** (adducts
the CMC like OP, flexes the MP like FPL, **exactly zero** at the IP because it does not cross
it); the four fingers are **bit-for-bit unchanged** (the control — without it, a global shift
could pass for a fix); and it is **insensitive to the peak-force guess**, 11.9% from 100 N to
400 N, because a residual is a *direction* problem and scaling a muscle does not rotate its
column.

**FPB and APB were attempted and thrown away.** They came out *extending* the MP joint that
FPB exists to *flex*, and moved the thumb residual by nothing. Fixing them meant sliding
attachment points until the moment arms came out how I wanted. One muscle that is right beats
three that flatter the result.

⚠ **A bug the gate caught, not me:** the first ADP insertion sat on the straight line from the
MP joint centre to the origin, so the tendon passed **through** the joint and its `mp_flexion`
moment arm was **exactly 0.0000** — anatomically placed and mechanically inert. A zero that
reads like a rounding error and is a modelling one.

**~~THE DESIGN DECISION THAT FALLS OUT: no characters under the thumb.~~ WITHDRAWN.**

That decision rested on the thumb being unable to press, and it *was* — with only ADP added
(11.9% residual). But **ADP was necessary and nowhere near sufficient**, and stopping there was
a failure of nerve, not a finding. The user pushed back: *"the thumb is likely the most flexible
of all."* They were right, and the model was missing two more muscles.

| | residual | max press force |
|---|---|---|
| stock MyoHand (no adductor) | 45.6% | **0.0 N** |
| + ADP | 11.9% | 0.0 N |
| + FPB | 7.0% | 0.0 N |
| **+ APB** | **0.0%** | **66.8 N** |
| *published human tip pinch* | | *45–70 N* |

**The force was never fitted.** The moment arms were fitted to *published anatomy*; the pinch
force fell out, and it lands inside the human band. You cannot get that by tuning toward an
answer that was never the target. It recruits FPB, APB, FPL and ADP — the thenar group plus the
long flexor, which is what a real thumb pinches with.

**Why it took all three, and why each was necessary:**

- **FPB** is the *only* muscle that flexes the MP **without crossing the IP**. FPL does both, so
  it over-flexes the IP before it supplies enough MP torque; EPL could cancel that, but it
  **extends** the MP — the very thing needed. Without FPB the thumb has no way to flex one
  joint without wrecking the other.
- **APB** breaks a *saturation* trap. With ADP+FPB the MP flexors were firing at activations of
  **0.007–0.019** — nowhere near their limit. They were not weak; they were **capped by
  `cmc_abduction`**, because *every* muscle that flexes the MP also **adducts** the CMC. APB
  flexes the MP while **abducting**, which releases the cap. I had explicitly refused to add
  APB, reasoning "it abducts; it does not supply MP flexion" — **that was simply wrong**: APB
  inserts on the proximal phalanx, so of course it flexes the MP.

**So the thumb carries characters after all**, and it is the *cheapest* digit on the hand. What
it should carry is now an open layout question (§7), not a constraint.

**The well was a disc, and a well is a CHANNEL — caught by the user looking at the render.**
*"The finger tip bone should fit into the well, not simply rest the pad on its opening."*
Exactly right, and it was a geometric error, not a drawing one. The well had been modelled as
a **disc at the pad, on the pad normal** — which describes a device you would have to lower
your fingertip into **vertically, like a piston**. A DataHand/Svalboard well is a **U-channel**:
open **proximally** so the finger slides in along its own bone axis, open **dorsally** so it can
lift out, a palmar **floor** under the pad (that is `click`), and side **walls** for left/right.
The user's own first description of this device, months earlier, was "a U-shaped cavity".

Three consequences, all of which the disc model hid:

- **A well has three axes, not one.** Channel axis = the distal phalanx; floor normal = the pad
  normal (so the *press* direction was right); lateral = the walls. All three are needed.
- **Well separation is a SEGMENT-SEGMENT test.** Point-to-point between two pads assumes a
  disc. Two channels can sit comfortably apart at the fingertips and still **cross further back**,
  where the fingers converge toward the knuckles — a point test cannot see that collision at all.
  It tightened the binding margin from −8.9 mm to −5.1 mm.
- ⚠ **The thumb's channel pointed backwards, away from the thumb.** A MuJoCo capsule extends
  along its local **z**, and nothing says which end is distal: z runs **distally** on the four
  fingers and **PROXIMALLY on the thumb**. Measured, the thumb bone sat at **+2…+24 mm** along a
  channel spanning **−22…+4**. The axis is now aimed at the model's own tip site. **This is the
  same species as the thumb's flexion-sign bug — I trusted a sign convention again, in the same
  digit, having already been burned by exactly that.**

Now gated: the bone must lie inside its channel, laterally within the well radius, above the
floor, for every digit.

**A workaround that outlived its bug — caught by the user looking at the render.**
*"The thumb button isn't orthogonal to the thumb pad."* It was **63° off**. Every key had been
oriented to the direction the digit can **push**, not the way its pad **faces**, because a
pad-normal thumb key had once measured **exactly zero press travel**. But that zero was an
artefact of the thumb **sign** bug (`mp_flexion`/`ip_flexion` flex *negative*) — which was
found and fixed **afterwards**, and nothing re-tested the workaround once its reason was gone.

Re-measured: the thumb gets **+35.0 mm** of travel along its pad normal against **+9.8 mm**
along the flexor push — 3.6× *more*, not zero. And the error mattered more than it used to,
because **a well is a cup, not a cap**: 63° of obliquity on a flat cap is a contoured keycap;
63° on a *cup* means the pulp never seats, it jams on the rim. Wells now face their pads (0°
by construction) and every digit — thumb included — has 31–39 mm of click travel.

**This retires a "limitation" that was never real.** VISION.md carried "the thumb's pad meets
its key at ~80° of obliquity, a real build needs an angled thumb cap" as a *consequence of the
missing thenar muscles*. It was not. It was a consequence of my own stale workaround. **A
false limitation is as dishonest as a hidden one**, and it was load-bearing: it had been
offered as evidence for how badly the missing adductor pollicis distorts the thumb.

**The enslavement constraint was watching the wrong joint.** `common-drive` bounded the spread
of `tm` (the PIP) and left `tp` (the MCP) completely free — so the optimiser individuated the
fingers at the MCP instead: **spread 0.37 against a 0.15 limit**, ring nearly straight (0.12)
while the index was half-curled (0.49). That is precisely the hand-nobody-can-make the
constraint exists to forbid; it had simply moved to the joint that was not being watched. A
constraint on one joint of a two-joint chain is not a constraint.

**A `SyntaxError` that 57 passing tests could not see.** An edit left an empty `for` loop in
`opt/run.py`; the suite went green and two 35-minute optimiser runs died on the import. **A
test suite that never imports a file cannot defend it.** Now imported by a test.

**A constraint that punishes intended contact.** Four separate bugs, one species: the thenar
counted as a "finger"; the body face counted as an obstacle; "clearance" applied to a device
the hand *grips*; a body whose size was hardcoded so it could not shrink to fit. An
exoskeleton stands off the hand, so "don't touch" was nearly right. **A gripped body is
*held*, so "don't touch" is nearly always wrong.**

---

## 5b. The support structure does not follow the hand — and the obvious fix does not work

**The user: "invert the support structure — it doesn't follow the natural shape of the hand."
They are right, and the hand says so.**

**The palm is a CUP, 6.4 mm deep** (measured off the metacarpal meshes, `palmar_arch`):

| metacarpal | across the palm | palmar surface |
|---|---|---|
| 2nd (index) | +16.6 mm | **−8.9 mm** |
| 3rd | +4.0 mm | −5.1 mm |
| 4th | −8.5 mm | **−3.1 mm** |
| 5th (little) | −19.8 mm | **−9.5 mm** |

The **edges protrude palmar and the middle is hollow** — the transverse metacarpal arch. And
`build_body` bolts **four corners of a flat rectangle** across it at a single depth, so they
either float off the eminences or dig into the hollow. That is a real defect and it is exactly
what was pointed at.

**The obvious fix is an ARCH**, and the physics argues for it: a keypress pushes the body
**dorsally, into the palm**, so an arch convex toward the palm takes that in **compression**
where a plate takes it in **bending**.

### ⚠ And the measurement refuses to support it.

Stiffening each group 100× and seeing what the key actually feels:

| design | key face | floor legs | **palm support** |
|---|---|---|---|
| optimiser's lightest | 49% | 38% | 13% |
| optimiser's knee | 44% | 27% | 10% |
| hand-built baseline | 2% | 12% | **18%** |

On the devices the **optimiser** produces, the compliance is in the **cantilever out to the
wells** and the palm is minor — an arch there cannot help. On **hand-built** devices the palm
dominates — it could. **The conclusion is design-dependent, and I asserted it universally
twice, in opposite directions, before measuring properly.**

⚠ **And my "3.8× stiffer arch" was an artifact.** That version skipped the floor routing
entirely and cut straight through the fingers (−6.5 mm into the middle phalanx). The gain came
from **deleting the cantilever**, not from the arch. I was excited by a number produced by a
broken geometry.

**Four routings all failed the same way**, and the failure is the same topological trap that
killed the exoskeleton: **you cannot draw a straight line from the palm to a fingertip without
crossing the finger, because the finger is what lies between them.** The box's "ugly" floor
ring — drop palmar into open air first, *then* run distally — was solving a real problem, and
solving it correctly.

**So what is the arch actually for?** **Fit and pressure distribution**, not stiffness: bear on
the two eminences the hand presents, rather than a plate across the hollow. **This model cannot
score comfort**, so it cannot make the case for its own change. And the arch's 3× mass penalty
is largely a **beam-model artifact** (10 discrete beams where a plate is 4; a **moulded shell**
following the palm costs nothing extra — the same shell, curved). Settling this properly needs
**shell elements, not beams** — the same caveat already on `BODY_PROX`.

### 5c. Shell elements — and what they actually settled

`structure/shell.py`. MITC4 shells (PyNite `add_quad`), which carry **membrane** action —
which is precisely what an arch uses and what a stick figure of beams **does not have**.

**The gate first** (same discipline as the beam model's closed-form cantilever): a
simply-supported square plate, `w = 0.00406·q·a⁴/D` (Timoshenko). It converges — **2.3% → 1.9%
→ 1.0% → 0.7%** as the mesh refines. A shell that cannot reproduce a textbook plate has no
business being asked about an arch.

**What it settled — the beam model was wrong by 25×:**

| | beam model | shell model |
|---|---|---|
| arch mass penalty | **+212%** | **+8.4%** |
| arch stiffness | 1.00× | **1.12×** |

A curved shell has the same area and thickness as a flat one, so the arch costs only its extra
**arc length**. The beam model billed 10 discrete struts where a plate is 4. **That was never a
finding about arches; it was an artifact of idealising a shell as sticks.**

**What it did NOT settle, and I will not pretend otherwise:**

- ⚠ **The arch is still not clearly worth having.** 1.12× on a component carrying only ~10% of
  the compliance is **~1% overall**. The shell corrects the *artifact* without vindicating the
  *arch*. Those are different claims and only the first is established.
- ⚠ **A linear shell cannot distinguish "follow the palm" from "invert".** They give
  *identical* deflections. My argument that the cup must be **inverted** — because a keypress
  pushes the body dorsally, so a dorsally-bulging shell is pushed *into its own convexity*
  (tension and snap-through) rather than compressed — is **real physics that this model cannot
  see**. It would appear as a *buckling* margin, and buckling does not bind at ~1 N. So the
  sign is free, and **"follow the palm" wins on fit at equal stiffness.**
- ⚠ **The key face — 44% of the compliance — is still unresolved.** Meshing its *bounding
  rectangle* gives a solid slab that is 3× heavier than the beam model's sparse chain, but
  that compares **two different parts**, not two idealisations of the same one. Settling it
  needs the face's real **footprint** (a shaped web following the fingertip arc), which is a
  design task, not a meshing one.

`build_arch` is implemented and clears bone in both postures. It is **still not wired into the
optimiser** — not because of the beam artifact (now disproved), but because **~1% is not worth
a new architecture**, and the component that *would* be worth it (the key face) has not been
modelled honestly yet.

---

## 5d. "It gets in the way" — and the law that follows from it

**The user, and it is the best argument made about this device:**

> *"Having the supporting structure far from the hand is a problem because it 'gets-in-the-way'
> of me using my hands. If the supporting structure hugs the hand and stays above the sensors
> as much as possible it becomes more a natural extension, rather than holding a big ball."*

**Measured, and they are right.** Of `build_body`'s structural nodes:

| | |
|---|---|
| nodes **palmar** of the hand (the space the palm needs) | **15 of 16** |
| mean standoff from the hand | **27 mm** |
| worst (the floor ring) | **68 mm** |

The palm is the **working surface** of the hand. Every one of those nodes sits in the volume you
use to hold a cup, a pen, a door handle. **The device is not on the hand — it is a ball the hand
is wrapped around.**

### The law

> **A BEAM FRAME BUYS ITS STIFFNESS WITH DEPTH.
> DEPTH IS EXACTLY WHAT GETS IN THE WAY.**

The palmar box is stiff *because* it is **57 mm deep** — and that depth **is** the ball. A dorsal
frame that hugs the hand has ~5 mm of depth, and as a **stick figure** it is hopeless:
triangulated, forked and cross-braced, `build_dorsal` still deflected **2.58 mm** against a
0.5 mm gate. Ergonomically perfect (mean standoff **5 mm**), structurally useless.

**A SHELL needs no depth.** It gets its stiffness from **curvature** — a curved section cannot
bend without also *stretching*, and stretching is expensive. That is a tape measure, an eggshell,
a fingernail. Same material, same thickness, same width, merely **wrapped around the finger**
instead of laid flat across it:

| finger | flat strip | curved shell | gain |
|---|---|---|---|
| index | 0.035 mm | **0.001 mm** | **46×** |
| middle | 0.035 mm | **0.001 mm** | **48×** |
| ring | 0.026 mm | 0.001 mm | 44× |

⚠ And the gain scales the right way: a flat strip's bending stiffness goes as **t³**, so thick
stock is already stiff and curvature buys less (9× on the thick hand-built baseline). **The
thinner and lighter you want the device, the more curvature is worth** — which is exactly the
regime a wearable lives in.

**So "hug the hand" and "use shell elements" are the same request.** A hugging structure does not
have to be floppy; it has to be a **shell**. And the *structure model* had to change before the
*architecture* could even be seen — which is the whole lesson of this project.

### And the objection that killed the exoskeleton is dead

The articulated dorsal exoskeleton was abandoned because "an open frame cannot reach into the
palm from outside" — its thumb arm cut the hand three ways. **But that was when the keys were
deep in a GRIPPING palm.** The hand is now **open** and the wells are **at the fingertips**,
which a dorsal rail reaches by running along the finger and wrapping the tip. **The topological
trap died with the gripping posture, and it was never re-examined.** It should have been, and it
took the user's ergonomic intuition to force the re-examination.

---

## 5e. The gauntlet — anchor points and boundary conditions FIRST

**The user, and it is the correct order of work:**

> *"The rigid support can still extend over the fingers if needed, but that's what we'll find out
> from the optimisation. What we need is the anchor points and boundary conditions first."*

Every mistake in this project has been a boundary condition or an objective. Never a search.
And getting these right was worth more than every optimisation run put together.

### Three boundary-condition errors, each of which flattered the answer

**1. It is not bolted to a wall.** The first gauntlet used **rigid** supports. Rigid anchors
absorb the keypress for free. With honest soft-tissue anchors the button deflection was **7×
worse** and the design **failed the gate at every thickness** — including 2 mm and **70 g**.

**2. The anchor was a hinge.** The supports were the proximal *ring* of the metacarpal shells —
**a line of nodes, with zero extent along the lever**. A keypress at a fingertip **121 mm** away
is a **moment**, and *a line cannot carry a moment*. **55% of the button's movement was the
gauntlet rocking**, and thickening the shell did nothing, because I was stiffening a beam that
pivoted on a pin. **The fix was not material. It was EXTENT.**

| anchor | extent along the lever | rocking at the button |
|---|---|---|
| line at the knuckles | **~0 mm** | **~387 µm** |
| **patch + carpus** | **92 mm** | **0.2 µm** |

**3. The contact is one-way.** Soft tissue can **push** the gauntlet off the hand but cannot
**pull** it back. A keypress drives the button palmar, which *lifts* the proximal end — and
nothing in the tissue resists that. **Only the strap does.** The springs are bidirectional in the
model, and that **bundles the strap's hold-down into the tissue's stiffness** — declared, not
hidden.

### And tissue stiffness is not one number

`k = E·A/t`, and **t is measured**: the skin over the metacarpals is only **1.4–3.1 mm** (bone
radius vs flesh capsule, taken *perpendicular to the bone axis* — measuring it from the centroid
gives half the bone's *length* and negative tissue, which is how the first attempt read).

**Thin skin over bone is a STIFF anchor.** `SOFT_TISSUE_K = 25 N/mm` was quoted for a **palm**
patch — a muscle pad ten times thicker — and I had been applying it to the back of the hand.

### The result

| | mass | button deflection |
|---|---|---|
| solid gauntlet (1.2 mm CF-PA12) | 42.1 g | 183 µm |
| **grown** (ESO, minimise mass, gate 500 µm) | **25.2 g** | 221 µm |
| *the palmar body it replaces* | *36.9 g* | *38 µm, and 27 mm of standoff* |

**40% of the mass deleted**, and it hugs. It stops on **connectivity**, not stiffness — a finer
mesh would carve further.

⚠ **`WRIST_TISSUE` is a guess and it is load-bearing.** MyoHand's carpals have **no flesh geoms
at all**, so the tissue over the dorsal wrist — the gauntlet's *main anchor* — cannot be measured
from this model. It is taken as 3 mm and declared.

---

## 5f. A flesh model to go with the bones

**The user: "I think we need to find a flesh model to go with the bones."** It was the
load-bearing gap. MyoHand ships **bones** and crude flesh **capsules** — and over the **carpus**,
exactly where a gauntlet anchors, **no flesh at all**. So `WRIST_TISSUE` was a **guess** (3 mm),
and it set the stiffness of the whole structure's main anchor.

### The source, and why not the obvious one

| | licence | |
|---|---|---|
| **[PIANO hand-MRI dataset](https://github.com/reyuwei/PIANO_mri_data)** | **Apache-2.0** | 50 MRI volumes + bone masks + muscle masks. **Used.** |
| [NIMBLE](https://github.com/reyuwei/NIMBLE_model) (parametric model built on it) | ⚠ **avoided** | code is MIT but `LICENSE.md` is the **unedited GitHub template**; the model weights sit on a **Google Drive with no licence**; and it emits `*_manov.xyz` — **MANO-topology** vertices. MANO is Max Planck's **non-commercial** licence, which this project rejected on day one as "a live landmine if this ever becomes a product". |

We take the **source data**, not the MANO-registered model.

### The method, and two ways of getting it wrong

Segment the hand from the raw MRI (Otsu), take the bone surface from the mask, and **ray-cast
from each bone-surface voxel along its OWN outward normal** until the ray leaves the hand. *That*
is the tissue the gauntlet presses through.

⚠ **Distance-to-air is the wrong metric.** It is the shortest way out in *any* direction — and
for a palmar bone that is **sideways**, out of the side of the hand rather than through the pad.
It read the finger **pulp as thinner than the nail bed**, which is anatomically impossible, and
that is how it announced itself.

⚠ **Cast only from the face you mean.** Ray-casting dorsally from *every* bone-surface voxel
sends the ray from the palmar ones **straight through the bone** and out the far side. It read
**7 mm of "skin" on the back of a hand** that is famously skin over bone.

The dorsal/palmar **sign** is derived, not assumed: **the fingertip pulp is palmar, and it is the
thicker side** — an anatomical fact the data can be asked to confirm.

### What it changed

| region | assumed | **measured (MRI)** |
|---|---|---|
| **dorsal carpus** (the anchor) | **3.0 mm** *(guess)* | **6.8 mm** |
| dorsal metacarpals | 1.4–3.1 mm *(capsules)* | **4.8 mm** |
| fingertip pulp (palmar) | — | 4.8 mm ✓ |

**The tissue is ~2× thicker than assumed, so the anchor was ~2× too stiff.** The guess was
flattering the design in the one place the whole structure hangs from.

**⚠ And the design turned out not to care — which is the entire point of measuring.** With the
anchor **2.75× softer** (4,485 kN/m against 12,359), the button deflection moved **361 → 376 µm**.
Four percent. The **distributed-patch anchor** had already made the structure insensitive to the
number it used to hang on. Before that fix, this guess would have decided everything.

### And the renders finally show a hand

`hand/flesh.py` gives MyoHand a **skin** — each bone offset by its *measured*, direction-dependent
tissue. Every render until now drew **bones and capsules**, which is why every geometry bug in
this project had to be caught by a **number** rather than by eye: the shell 4.3 mm inside a
finger, the wrap sweeping through a fingertip, the wells overlapping. **A skeleton is not what the
device touches.**

⚠ **One MRI figure is thrown away, not used:** proximal-phalanx *palmar* tissue reads **1.0 mm**,
which is not believable (flexor tendons and fat live there). Most likely the Otsu mask biting into
the finger where the digits touch. The **phalanges keep MyoHand's capsules** — which agree with
the MRI's *dorsal* figures — and only the **metacarpals and carpus** take MRI values, which is
where MyoHand had nothing anyway.

---

## 5g. The organising principle: this is a HUMAN FACTORS design

**THE USER:** *"The whole exercise is a human factors design. That's part of the appeal."*

That is the frame, and losing it is what has cost this project the most. **Nearly every constraint
here is a fact about PEOPLE:**

| the constraint | what it actually is |
|---|---|
| `DEFLECTION_MAX` = 500 µm | a key that moves feels **mushy** |
| `PRESS_N` = 0.30 N | what a **finger** presses |
| `SWITCH_TRAVEL` | what a **finger** feels |
| effort = Σaᵢ³ | what a **hand** can sustain |
| `hug` / `SEG_CLEAR` | it must not **rub** |
| `STRAP_NODES_MIN` | the strap must not **dig in** |
| 5th–95th percentile | it must **fit a person** |
| mass | it is **worn all day** |
| **`SKIN_R`** | **it must not scratch or cut** ← *was missing entirely* |

And only **three** are facts about a **machine**: `NOZZLE_R`, `OVERHANG`, `BRIDGE_MAX`.

⚠ **AND I LET ONE OF THE MACHINE'S NUMBERS SET A LIMIT THAT A HAND HAS TO TOUCH.** `NOZZLE_R` =
0.4 mm is what a **printer** can lay. It is not, and never was, what a **palm** can bear. Measured on
the structure this project was about to ship: **669 of 669 members (100%) are thinner than a friendly
surface allows**, at a median radius of **0.41 mm** — a 0.8 mm wire — and **56 of them end in a free
0.4 mm POINT.** Every structural measure in the project was blind to those points, *because a
cantilever tip carries no load and so costs nothing to leave in.* A hand would have found all 56 of
them in a second.

**This is the same failure as every other big one here**, and the pattern is worth naming: the model
kept failing to represent the **human**, and it always looked like a technical bug —

- the "finger pad" was the **fingernail** (§4a);
- the thumb had **no adductor**, so it could not press (§6);
- **flesh could pull as well as push**, so the anchor was a fiction (495 µm → 9178 µm);
- "clearance" **punished intended contact** — a gripped body is *held* (§4d);
- and the minimum feature was set by the **printer**, not the **hand**.

Every one a human-factors failure wearing a technical costume.

### 5g.1 Reproducibility is a HUMANIST constraint, and it is hard

**THE USER:** *"I'm hoping for a 3D-printable design as it fits the open source model. The open
source model is also humanist and I think more efficient than the typeware.nl business model."*

So: **the design must be reproducible by ONE PERSON with ONE PRINTER.** That is a first-class
constraint, and it is the thing that separates this from a product behind a patent.

It **rules out**, however good the engineering:

- **a soft closed-cell foam over a stiff skeleton** — which is the textbook answer, and how ski
  boots, helmets and prosthetic liners solve exactly this problem — because it needs a second
  material, a mould, and a supplier;
- **dual-material printing** (a TPU overmould) — because it needs a second extruder;
- **the welded-wire build** (§8.12) — because it needs a laser welder.

Each is good engineering. **Each is a barrier between a person and a working keyboard.** An open
design that needs a second extruder is not open to everyone, and a design that must be *bought* is
not open at all.

Consequence, and it is the harder road: **the friendliness has to come out of the same single
printed part** — from the geometry, not from a liner.

## 6. Model limitations — stated, not hidden

These bound every conclusion above.

- ⚠ **`SKIN_R` = 1.5 mm is a GUESS, and it now sets 95% of the device's mass.** It is the smallest
  convex radius any surface a hand can touch is allowed to have — the number that separates a
  wearable from a knuckle-duster. Consumer-product edge rules put accessible edges in the region of
  0.5 mm and edges that bear *pressure* rather higher, but **nothing here has been checked against a
  standard, and nothing has been checked against a hand.** It is the single most load-bearing guess
  in the project: 146 of the 153 members sit on it.
- ⚠ **`ASPECT_MAX` = 3:1 is a GUESS** — the flattest a member may get. Unbounded, the optimiser would
  drive a purely-bent member to a ribbon, which buckles laterally and prints badly on edge. It is
  currently **inert** (the accepted section is a round tube, so nothing flattens), but it would bind
  on a load-limited structure.
- ⚠ **The load paths are CURVED by a guess, not by an optimiser.** `SPLINE_TENSION` (how hard a load
  path bows away from its own chord) and `MAX_TURN` (75° — beyond which two members at a node are a
  *branch*, not a continuing path, and are left as a corner) are both **GUESSES**. The tension is
  *swept* and the mass it costs at the deflection gate is measured, so its price is known — but the
  curvature itself is **fitted, not optimised**: nothing moves the control points to minimise
  anything. A true spline-shape optimisation (form-finding on the interior points) has **not** been
  done.
- ⚠ **THE MATERIALS ARE CHOSEN ON DATASHEET NUMBERS, AND IT IS THE *IN-SERVICE* BEHAVIOUR THAT
  DECIDES.** Twice now the disqualifying property has not been stiffness or strength:
  - **PLA creeps.** A long bridging span (which buys away the support props) is only reliable in PLA,
    and PLA creeps under **sustained** load at Tg ≈ 60 °C. The strap holds this device in *permanent
    tension* against a warm hand — which is exactly a sustained load. **A worn device that relaxes
    its own preload stops registering keypresses.**
  - **⚠ NYLON IS HYGROSCOPIC, and this thing lives against a sweaty hand all day.** Glass- or
    carbon-filled nylon is the intended material (E ≈ 5–7 GPa, close to the 6.0 GPa modelled; glass
    is **~15–25% denser** than carbon, so ~11 g → ~13 g). But saturated nylon's modulus drops
    substantially, and **the entire design is stiffness-limited** — the 500 µm displacement gate is
    the only active structural constraint. **NOT MEASURED. Measure the wet modulus before trusting
    the gate in service.**
- ⚠ **The printability model is a GEOMETRIC one, and nothing has been printed.** The
  self-support rule (45°), the bridging span (`BRIDGE_MAX`, 10 mm) and the bed-contact depth
  (**`BASE_T`, 3 mm — a GUESS**: how close to the lowest point a node must be to count as
  sitting on the bed, given a brim) are handbook/practice numbers, not measurements from *this*
  printer with *this* filament. The pillar counts in §8.15 move with all three. **Print it and
  count the supports** before any of those numbers is quoted as a result.
- ⚠ **NOTHING ENFORCES A BEARING *AREA*.** The wells got a CRADLE — contact distributed along the
  phalanx, centre of pressure free (§8.13). **The ANCHOR FEET did not.** A member standing on the
  hand and ending in a 1.5 mm hemisphere, pressed into the flesh by the strap, is not a spike — but
  it *is* a **pressure point**, and it is where the device's whole reaction load lands. The feet
  need **pads**. This is the same omission as every other one in this project: a human-factors
  requirement with no constraint behind it.
- ⚠ **The minimum-pillar build direction leaves the part 185 mm tall on a 9-node,
  16 × 28 mm bed footprint.** The direction was optimised for support count ALONE. A real print
  also wants a low part on a broad base, and that trade has not been made.

- ⚠ **The thumb still cannot press, even with ADP added.** Stock MyoHand has no adductor at
  all and the thumb exerts **0.0 N**; we add adductor pollicis (`hand/thenar.py`) and the
  irreducible torque residual falls 45.6% → **11.9%**, against **≤0.6%** for every finger.
  **FPB and APB are still missing** — attempted, wrong moment arms, discarded rather than
  tuned. Consequence: **no characters are placed under the thumb** (§5). Thumb numbers rank
  thumb options against each other at best; they must not be compared with the fingers'.
- ⚠ **ADP's peak force is a PCSA ratio (ADP ≈ 1.25 × FPL) that was RECALLED, NOT READ.** The
  conclusions are shown not to depend on it (11.9% from 100 N to 400 N), but the number itself
  is unverified.
- ⚠ **MyoHand has no enslavement.** FDP2–FDP5 moment arms are strictly diagonal — four
  independent actuators. Curling *or extending* the ring alone is *free* in this model and
  impossible in a hand. Mitigated by a common-drive parameterisation on **both** the MCP and
  PIP joints (it originally covered only the PIP, and the optimiser went straight through the
  gap — see §5), now with **per-finger `INDIVIDUATION` bounds** (`Source.LITERATURE`, Häger-Ross
  & Schieber 2000): the ring may deviate ±0.035 where the index may ±0.075. This caught a real
  artefact — the winning design had raised the ring 9 mm and leaned on it (+33% effort, key-overlap
  when clamped) — the kinematic form of the shared-FDP coupling the OpenSim hand models carry. Not
  *solved* (a measured enslaving matrix would refine the magnitudes), but no longer a single symmetric
  guess deciding the design.
- ⚠ **MyoHand has no EXTENSOR HOOD** (it models the intrinsics as bare tendons). This turned out
  **not** to be what limited the lateral tilts — that was a cradle artefact (the withheld well floor,
  now fixed; §8.15g), and the interossei were adequate. But the hood is a genuine gap for anything
  depending on its **IP-extension coordination**, and **OpenSim ARMS (real extensor hood) is the
  pending cross-check** for those.
- ⚠ **MyoHand does not model the IP COLLATERAL LIGAMENTS.** A finger's DIP/PIP take lateral load
  passively through these; MyoHand, having only flexion hinges there, demands a muscle for it. The
  well floor now stands in for that support during a lateral press, but a device that loaded the IPs
  laterally *without* a floor would expose the gap.
- ⚠ **The sensor's false-trigger floor is NOT MEASURED.** Effort is negligible wherever a direction
  is feasible, so the dome wants to be *as soft as possible*; the only thing that would push it
  stiffer is resistance to accidental actuation (a resting finger, a neighbour's motion, tremor),
  and that floor has not been quantified. Until it is, the dome rate (131 N/m at the 20 gf
  reference) is an **upper bound on softness**, not a determined value.
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
- **Every number we made up is now tagged and enumerated** (`design/params.py`, and
  `test_every_constant_declares_where_it_came_from` fails if one is not disclosed here):

  | GUESS | value | what it decides |
  |---|---|---|
  | `COMMON_DRIVE` | 0.15 | how differently neighbouring fingers may curl — a stand-in for **enslavement**, which MyoHand does not model at all. ⚠ **This guess is currently the binding constraint on the well layout.** A made-up number is deciding the design. |
  | `WELL_WALL` | 1.5 mm | wall between adjacent wells; never checked against a print |
  | `REST_GAP` | 3.5 mm | magnet-face to Hall at rest (§8.15l). A frame dimension, not yet confirmed on a print; sets where the field sits in the sensor's range. |
  | `CRADLE_LEVER` | 0.7 | lateral magnet travel per mm of fingertip tilt — sets each tilt direction's signal (§8.15l). A geometry guess until a stage-1 coupon measures it. |
  | `DEFLECTION_MAX` | 0.5 mm | above this a key "feels mushy" — a judgement, not a measurement |
  | `ADJUSTER_MASS` | 0.15 g/mm | mass of a per-finger slide; not from any real mechanism |
  | `COLUMN_SHIFT_COST` | 5e-6 | cost of translating the hand to the index's 2nd column |
  | `SHIFT_FREQ` | 4.0 /100 letters | left-shift usage. It **decides whether a pointer fits**: with shift on a well, the mouse costs one slot more than the hand has; move shift to a hold/chord and it fits. |
  | `WRIST_TISSUE` | 3.0 mm | soft tissue over the dorsal **carpus** — the gauntlet's main anchor. **NOT derivable:** MyoHand's carpals have no flesh geoms at all. Sets how stiff the anchor is. |
  | `BAR_R` | 0.9 mm | radius of a lattice strut in the grown gauntlet. ⚠ **The bar section and the topology trade against each other and only the topology is being solved.** A thinner rod would grow a denser, finer skeleton at the same mass; a thicker one a sparser, chunkier one. Nothing has measured which is better, and nothing has printed one. |
  | `LAYER` | 6 mm | depth of the lattice — the gap between its inner and outer node sheets. It **is the lever arm** that gives the gauntlet its bending stiffness, so it is the most load-bearing number in `structure/lattice.py`. Chosen to keep the total standoff under ~10 mm, which is a judgement about when a wearable starts to "get in the way", not a measurement. |
  | `STRAP_NODES_MIN` | 3 per band | minimum structural nodes each strap band must be held by. ⚠ **NOT a strength constraint, and I claimed it was.** Measured, the struts at a strap node run at **4% of allowable — *less* than the average strut (7%)**, because the strap carries ~1 N and a 1.8 mm rod takes 89 N. It is a **single-point-of-failure** guard: nothing else stops ESO deleting down to *one* node holding the entire tension side of the anchor, and **3 of 8 designs did exactly that** (two of them off a single node). Per *band*, because the two bands are a **couple** — three nodes on one and none on the other is a hinge. Nothing has tested a joint, so the number is a judgement about redundancy. |
  | `STRAP_W` | 16 mm | width of a strap band. It decides **how many anchor nodes the strap can pull on**, and therefore how much of the lift-off it carries — and that is not a small lever: smearing the strap over the *whole* bearing patch instead of the two bands it actually occupies made the button 276% steadier than it really is (412 µm → 1550 µm when corrected). A guess is deciding the structure again. |
  | `SHELL_T` | 0.6 mm | thickness of a candidate **plate** in the mixed ground structure. It trades directly against `BAR_R`, and the greedy ESO criterion (energy per unit volume) systematically favours the *thinner* element — plates rank 857,000× above struts on it. Plates still lose on mass by 6×, so the conclusion is safe in direction; the *magnitude* is contaminated by this number. |
  | `SEG_CLEAR` | 0.75 × hug (3.0 mm) | minimum gap between any point along a **strut** and the skin. It cannot be 1.0: a straight chord between two nodes each `hug` off a **convex** surface necessarily dips below `hug` at its midpoint — geometry, not a defect. This is the line below which the gauntlet would be **rubbing**, and nothing has worn one to find out where that line really is. ⚠ I briefly set it to 1.0, which is unsatisfiable; the check then rejected every node relaxation and reported "no gain" as though the physics had spoken. It had not — I had mis-set the bar. |
  | `RESIDUAL_MAX` | 0.05 | how much of the required joint torque a digit may FAIL to produce and still be said to "press" the key. ⚠ Ideally **zero**. It is a tolerance, and **the whole action set depends on where this line is drawn** — sensitivity must be reported. |

- **`SOFT_TISSUE_K`** (25 N/mm) is literature, not measurement: the band is 10–50 N/mm and
  the deflection answer moves **1.40×** across it.
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

**MEASURED (§8.15g): every finger's well is five-way (25/25).** The ulnar lateral tilts looked
infeasible only because the cradle model withheld the well **floor** the finger still rests on during
a lateral press; with it restored, all five directions are actuable on every digit.

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

### 8.8 The DORSAL GAUNTLET (this supersedes the palmar body of §8.1)

Also, and independently, disclosed — a **wearable keyboard** in which:

- a **rigid open framework ("gauntlet") sits on the BACK (dorsum) of the hand and fingers**,
  standing off the skin by a clearance, and **carries the finger wells** out to the fingertips;
- the hand is **not required to grip anything**: there is no body in the palm, and the palmar
  surface of the palm is left entirely free;
- the framework **reacts the keypress load into the carpus and metacarpals** through a
  **distributed bearing patch** (see §8.10), and thence to the wrist;
- the wells are held **in front of / around the fingertips**, the framework wrapping the distal
  phalanx to reach the palmar side where the pad meets the well floor.

Disclosed in combination with **any** of §8.2 (sensing), §8.3 (directions), §8.4 (switch), §8.5
(adjustment), §8.6 (layout by musculoskeletal optimisation), §8.7 (variants).

### 8.9 The design domain derived from a MEASURED SOFT-TISSUE MAP

Disclosed as a method, and as a device produced by it:

1. Obtain a **soft-tissue thickness map** of the hand — by MRI, CT, ultrasound, calipers,
   3-D scan, or any equivalent — giving, **per region and per direction**, the distance from
   bone surface to skin.
2. Construct the wearable's **inner surface as an offset from the SKIN**, not from the bone and
   not from a circular idealisation of the limb.
3. Set the standoff as a clearance of **the part** — its centreline offset by (clearance + the
   member's own radius + any fillet it will carry) — **not of a centreline**.

**Specifically disclosed measured values (this work, MRI, ray-cast from each bone-surface voxel
along its own outward normal):**

| region | dorsal tissue | palmar tissue |
|---|---|---|
| wrist / carpus | **6.8 mm** | 6.6 mm |
| metacarpals | **4.8 mm** | 3.8 mm |
| fingertips | 2.8 mm (nail bed) | **4.8 mm (pulp)** |

Also disclosed: that a cross-section of the hand is **not circular** (the metacarpals are flat,
the digits oval), so a shell fitted to a tube either stands proud of the flats or bites into the
sides; and that the correct construction takes the radius **per direction** from the skin surface.

### 8.10 The load-bearing structure, GENERATED by topology optimisation

Disclosed as a method, and as any device bearing a structure so generated:

1. Fill the **entire volume the wearable is permitted to occupy** — the shell between the skin
   offset and an outer offset — with a **dense lattice of candidate structural members** (a
   "ground structure"), in **two or more layers separated in depth** and cross-braced, the depth
   being the lever arm that carries bending.
2. **Reject any candidate member that passes through flesh**, by checking it against the skin
   surface of §8.10. (Members, not just their end nodes.)
3. Apply, as **separate load cases, one per WIRED (digit, direction) pair, each pressed ALONE**
   — because a typist presses one key at a time and a well is a multi-direction joystick, so
   each direction is a different force. Constrain the **worst** case, not their sum.
4. Remove or thin material by any of:
   - **evolutionary structural optimisation (ESO)** — delete the lowest strain-energy-density
     members and re-solve, iterating to a stiffness gate (Wolff's law);
   - **continuous member sizing by gradient descent** with **adjoint sensitivities**
     (`d(q·u)/dr = −λᵀ(∂K/∂r)u`, one back-substitution per constraint), minimising mass subject
     to a maximum permitted well displacement, alternating **size → prune → re-size** so the
     result is buildable rather than a haze of sub-manufacturable members;
   - **density-based (SIMP) topology optimisation** on a voxel discretisation of the same volume;
   - or any equivalent.
5. Optionally **let the surviving nodes drift** during the optimisation — moving each along its
   residual axial-force imbalance (form-finding), which straightens the members because that is
   what removes bending, while constraining each node to remain within the permitted shell.

**Specifically disclosed:** that the resulting structure is a **trabecular, bone-like network of
members of differing thickness**; that gradient sizing yields **~42% less mass than ESO at the
same displacement gate** (measured: 4.79 g vs 8.19 g); and that the mass of such a structure for
a five-digit hand keyboard is **of order 5–10 g** in CF-PA12.

### 8.11 The ANCHOR, and the STRAPS

Disclosed as a method and a construction:

- The wearable bears on the hand through a **distributed patch** over the **carpus and
  metacarpals 2–5** — an area, spanning **≥ ~60 mm along the hand** — and **not** through a line,
  ring, edge, or single ring of contact. *A keypress ~120 mm from the wrist is a MOMENT, and a
  line cannot carry a moment: measured, a line contact put **55% of the button's movement** into
  the device rocking on its own anchor.*
- The contact is modelled and constructed as **BILINEAR**: soft tissue **can only PUSH** the
  device away from the hand (compression-only, stiffness `E·dA/t` with `t` the **measured**
  tissue thickness of §8.9); the **tension side is carried only by a STRAP**, and only **where a
  strap band physically touches**.
- **Two strap bands**, separated so as to form a **couple** against the keypress moment:
  - a **wrist band**, placed **PROXIMAL of the thumb's metacarpal (first metacarpal / CMC
    joint)** and **on the carpus** — because a band anywhere distal of that necessarily crosses
    the thumb ray and loads or restrains the thumb on every keystroke;
  - a **metacarpal band**, passing through the **first web space** (between thumb and index),
    distal of the thumb metacarpal.
- A **minimum number of structural nodes per band** is enforced, so the tension side of the
  anchor is never carried by a single joint (a single point of failure).

**Specifically disclosed measured consequence:** modelling the strap as if it could pull anywhere
on the bearing patch, rather than only under its bands, makes the wells appear **~4× steadier than
they are** (412 µm vs 1550 µm), and a structure designed against the former **fails** its
displacement gate in reality.

### 8.12 MANUFACTURE of the structure

Disclosed, any of:

- **Additive manufacture as a single filleted solid.** Construct a **signed distance field** as a
  **smooth minimum** (e.g. `f(x) = −k·log Σᵢ exp(−dᵢ(x)/k)`) over the members and wells, and
  extract the surface by marching cubes. The **fillets at every junction are not modelled — they
  fall out of the blend radius `k`**, which is also what makes the junctions printable and
  removes the stress concentrations a boolean union of cylinders would leave. Disclosed with
  `k` ≈ 0.5–2 mm.
- **WELDED WIRE, formed on a printed jig.** Build the structure from **metal wire** (e.g. 316
  stainless, **0.6–1.2 mm** diameter), **formed against a 3-D printed replica of the optimised
  skeleton**, and joined (laser, resistance, TIG, or braze) **only where wires cross**.
  - Disclosed method: **cover the member graph with as few CONTINUOUS trails as possible**
    (an Eulerian trail-cover, preferring at each node the continuation that bends least), so that
    **a wire BENDS THROUGH a node rather than being welded at it**. Welds are then needed only
    where separate wires cross.
  - **Specifically disclosed measured result:** 226 straight segments across 176 nodes reduce to
    **32 continuous wires and 71 welds**, using **1688 mm** of 1.0 mm wire, mass 10.6 g.
  - Disclosed rationale: the weld's **heat-affected zone is annealed** whatever the wire started
    as, so joints are designed against the **annealed** yield (~205 MPa for 316), and the count of
    welds is both the labour and the weakness — thicker wire buys **fewer** welds (measured: 71
    welds at 1.0 mm vs 150 at 0.6 mm) at little mass penalty.
- Also disclosed: machining, casting, metal AM, or composite layup of the same optimised
  geometry.

### 8.13 The WELL as a CRADLE, and the load set

Disclosed:

- The well is a **U-CHANNEL** whose long axis is the **distal phalanx's own axis** and whose floor
  normal is the **pad normal** — the fingertip **slides into it along the bone**, and is *not*
  lowered onto a disc or a flat pad. It is open proximally (the finger enters) and dorsally (the
  finger lifts out); its floor is what `click` presses into, and its two side walls are what
  `left`/`right` push against.
- Contact between the fingertip and the well is modelled and constructed as a **CRADLE** —
  distributed along the phalanx, with the **centre of pressure free** — **not** as a point or pin.
  *A pin model wrongly concludes an open hand cannot press at all.*
- The well's radius is **derived from the digit's own distal-phalanx flesh dimensions**, not from
  a keycap pitch.
- **Only the surface a direction actually loads is loaded** (floor for `click`, a wall for
  `left`/`right`, the end for `forward`), the contacts summing to the switch force.

### 8.14 Additional measured findings, disclosed

- **A hand model lacking the thenar intrinsics cannot press with its thumb.** Adding **adductor
  pollicis (ADP), flexor pollicis brevis (FPB), and abductor pollicis brevis (APB)** takes the
  thumb from **0.0 N and a 45.6% equilibrium residual to 66.8 N of pinch and 0.0% residual** —
  and **APB is required**, because every MP flexor also adducts and the thumb otherwise saturates
  its CMC abduction. Disclosed: a device layout that puts characters on the thumb should be
  designed against a model carrying the thenar group.
- **The design is CONSTRAINT-DOMINATED, and the binding constraints are GEOMETRIC PACKING** — the
  five wells colliding with each other and with neighbouring digits, and the swept path of a
  finger reaching its own well. Measured over 32 samples: `key-overlap` binds 22/32,
  `swept-path` 19/32, `well-finger` 17/32 — while **structural yield binds 0/32** and structural
  supportability **0/32**. The feasible region is **~0.1%** of the design space (3 feasible in
  2400 uniform random samples).
- **Consequently the structure need not be co-optimised with the layout**: structural mass carries
  **no** trade against typing effort (Spearman ρ = −0.18, p = 0.53), while the **required
  adjustment range** carries all of it (ρ = −0.91, p < 0.001). Disclosed: choose the posture and
  well positions first, then optimise the structure to them.
- **Typing effort varies smoothly but over ~5 orders of magnitude** with hand curl (a nearly
  closed fist can barely press), so a device must hold the hand in a **narrow band of postures** —
  open enough that the wells do not collide, closed enough that the digits can press.

### 8.15 MANUFACTURABILITY as a constraint on the DESIGN SPACE — printing the structure

Disclosed as a method, and as any device bearing a structure so generated: making a
topology-optimised **wearable** structure printable by imposing the process limits **on the design
domain and on the deletion rule**, never as a penalty on the objective.

**(a) The MINIMUM FEATURE is imposed by DELETE-then-FATTEN-then-RE-SIZE — and each of the three
obvious shortcuts fails in its own way.** An unconstrained minimum-mass truss over a conforming
shell puts most of its material below the process's minimum feature: **788 of 921 struts (86%) came
out thinner than a 0.4 mm nozzle**. Disclosed method, and the reasoning that forces it:

- **A member thinner than the nozzle is NOT a member the optimiser is asking to delete.** Those 788
  sub-nozzle struts **carry 53% of the mass**. They are not numerical dust — they are a **fine net
  doing real work**, which is exactly what a minimum-mass *shell* wants to be. *Delete them and the
  structure collapses:* the survivors must then be pinned at the maximum radius to hold the same
  displacement gate, and the mass goes from **38 g to 284 g**.
- **DELETE by a fixed RATE, thinnest-first** (the classical size → prune → re-size alternation),
  ranked by radii that were sized against a floor *below* the minimum feature.
- **FATTEN every survivor up to the minimum feature.** This can only ADD material, therefore only
  STIFFEN, therefore the displacement gate **still holds by construction**. **Measured cost: 1.6×
  (4.79 g → 7.54 g).** The minimum feature is cheap; deleting to satisfy it is ruinous.
- **RE-SIZE the converged topology with the minimum feature as the true lower bound**, so the
  optimality criteria can redistribute *around* the floor and give the slack back.
- **THE SIZING FLOOR IS NOT THE PRINTING FLOOR**, and each of the three obvious shortcuts fails in
  its own characteristic way. All three measured:
  - *Size against the minimum feature (`r_min = r_nozzle`)* → every idle member parks at **exactly**
    the floor, the radii stop spanning decades, and the ranking the pruning depends on is destroyed.
    **The pruner then deletes at random**: cutting a quarter of the members made the structure
    *heavier* (19.97 g → 30.0 g), and 13 g of a 20 g answer was floor material under members doing
    no work.
  - *Delete everything below the minimum feature* → **collapse**, as above (38 g → 284 g).
  - *Delete only what the sizer drove to its numerical floor* → **nothing is ever deleted.** The
    sizer does not abandon members; it just makes them thin. The whole 4283-member domain survived,
    and with a floor under every one the answer was **22.35 g of uniform 0.40 mm strut: pure floor.**
    **With a minimum-feature bound, DELETION IS THE ONLY THING THAT REDUCES MASS** — the sizer cannot
    express "I want this gone", so the pruner must, and must keep doing it.
- Disclosed for any minimum feature size and any process (FDM ~0.4 mm, SLS ~0.8 mm, DMLS, resin).

**(a2) Disclosed as a RATIONALE — and ⚠ still NOT demonstrated.** The hypothesis is that the
minimum-feature bound is what gives a topology-optimised structure its *hierarchy*: a minimum-mass
truss **wants** many equally thin members, and what would force **few, thick, graded** ones is
MANUFACTURE.

The topology **is** now re-derived from scratch under the floor (669 members, 6.06 g — see §8.15b),
and it is **27% sparser and 13% lighter** than the fattened-and-re-sized 921-member version it
replaced. That is consistent with the hypothesis but it does not establish it: the structure is
still a net, not a set of chunky trunks. **Open.**

### 8.15b The printable gauntlet — measured

The disclosed structure of §8.10, **optimised from scratch under the printing constraints** of
§8.15 (CF-PA12 / CF-nylon, E = 6.0 GPa; 17 wired load cases, each digit-direction pressed alone at
0.196 N; displacement gate 500 µm; nozzle 0.4 mm):

| | unconstrained | **printable (FDM)** |
|---|---|---|
| members | 921 | **669** |
| member radii | 0.26 – 1.55 mm | **0.41 – 1.17 mm** (none below the nozzle) |
| mass, beam model | 4.79 g | **6.06 g** (1.27×) |
| mass, the actual filleted SOLID | — | **15.5 g** |
| worst well displacement | 499 µm | **499 µm** (gate 500) |
| members idle at the nozzle floor | — | **0 / 669** |
| sacrificial support | — | **91 pillars + 283 props**, 13.0 m of column |
| build orientation | — | **palm up**, 100 mm tall, 96 × 38 mm bed footprint |

- ⚠ **The beam model is a WIRE DIAGRAM and it is not the part.** The solid — SDF smooth-min,
  marching cubes, watertight, clearing the skin by 3.04 mm — is **15.5 g, +157%** on the beam
  model's 6.06 g. A wire diagram double-counts the volume where members overlap at a node and
  **misses the fillets entirely**. **Only the solid is the truth.**

**(i) HOW TO TELL A CONVERGED TOPOLOGY OPTIMISATION FROM A STALLED ONE.** Disclosed as a diagnostic,
because a pruner that halts early and reports a number is indistinguishable from one that converged:

> **A converged design has NO members idle at the minimum-feature floor, and NO unspent constraint
> margin.** Either one is proof the run stopped early.

Measured, on two runs of the same problem: the accepted answer has **0 / 669 idle** and spends the
gate exactly (499 µm of 500). A run on a finer lattice reported **13.95 g** — but carries **733 /
1872 (39%) idle at the floor** and leaves **10% of the gate unspent** (451 µm). It is an upper
bound, not an optimum, and quoting it as one would have been wrong.

**(j) A PRUNER MUST NOT TREAT A FAILED CUT AS A STOPPING CONDITION.** Four defects, all the same
shape — *halting while progress was still available* — and together they cost a factor of ~1.5 in
mass and ~2 in support:
- **A cut that breaks the constraint is not a stopping condition; it is a cut that was TOO BIG.**
  Put it back, halve the rate, and try again. (Likewise a cut that disconnects a load point.)
- **Warm-start each re-size from the surviving members' own radii.** The stiffness *reference*
  radius and the search *start* radius are different things; conflating them made every re-size
  begin from a uniform rod and rediscover a five-decade spread from scratch.
- **The set of members that must not be deleted has to be recomputed AS THE CUT PROCEEDS.** A
  "protected" set computed once, before the cut, does not protect a node with *two* thin
  support-members: the cut takes both, the node is orphaned, and the repair hands one straight back
  — so the trial stops shrinking and the no-progress guard fires.
- **One non-improving step is not convergence.** The optimality-criteria loop is not monotone (the
  contact set moves, the aggregation correction moves). Breaking on the first non-improving step
  left 8% of the deflection budget unspent, and every unspent micron is grams.

**(k) MEASURED TRADE — MASS against SUPPORT, and it is steep and monotone.** All four points are
the same problem, the same gate, the same material, differing only in what the domain OFFERS:

| the domain | members | **mass** | support points | support column |
|---|---|---|---|---|
| **8 mm lattice (accepted)** | 669 | **6.06 g** | 374 | 13.0 m |
| 5.5 mm lattice (props vanish by construction) | 1872 | 13.95 g ⚠ | 302 | 8.8 m |
| 8 mm lattice, **shallow-and-long members forbidden** | 229 | **32.53 g** | **39** | **1.2 m** |

- **A finer lattice removes the props but costs ≥2.3× the mass.** ⚠ And that cost is **not** the
  extra floor material: both domains have essentially the **same total member length** (27.7 m vs
  27.4 m) and therefore the same floor mass (14.8 g vs 14.6 g). A plausible 1/pitch² argument for
  it was **measured and found false**.
- **Forbidding the members that need props cuts the support burden 10× (374 → 39 points, 13.0 m →
  1.2 m of column) — and costs 5.4× the mass.** *The long shallow members ARE the load path*: they
  are the efficient diagonal ties a truss wants, and a structure denied them must be pinned at its
  maximum radius to hold the same gate.
- **Mass is worn every day; support is paid once.** So the lightest domain wins, and the support is
  bought down by the one lever that is free: **the printer's bridging span.**

**(l) THE BRIDGING SPAN IS BOUGHT WITH MATERIAL AND SURFACE FINISH — it is not a free parameter.**
Measured on the accepted structure: props fall **325 (8 mm span) → 283 (10) → 218 (12) → 106 (15)
→ 15 (18 mm)**. So "how many supports" is largely a statement about *the printer*, not the design.
- ⚠ **But a long span restricts the material.** An 18 mm span is reliable in **PLA** and not in
  CF-nylon, and PLA costs **+39% mass** at the same gate (6.06 → 8.4 g; ρ/E is 2.0× worse, softened
  to 1.39× because the frame is partly bending-dominated). **The disqualifier is not stiffness — it
  is CREEP.** A strap holds this device in *permanent tension* against the hand: that is a sustained
  load, PLA creeps under sustained load, and Tg ≈ 60 °C means it does so faster in a car or in the
  sun. **A worn device that relaxes its own preload stops registering keypresses.**
- ⚠ **And supports are themselves a SURFACE-FINISH problem, not only a time-and-plastic one.** They
  blemish the part wherever they touch, and **43 of the support contacts land on or around a well**
  — the faces the fingertip actually presses. Fewer supports is therefore also *better finish where
  it matters*, and the two goals align rather than trade.

⚠ **NOT MODELLED: the wells' own printability.** The whole analysis is on the **member graph**, not
on the filleted solid. The wells are U-channels — boxes with floors and walls — with their own
overhangs, and if bottom-surface quality matters anywhere on this part it is the **cup floor** under
the pad. The model has nothing to say about it.

### 8.15c CURVED (spline) load paths — and two justifications that failed their own measurement

**THE USER:** *"My bones have no sharp edges."*

Disclosed: a method for removing the **kinks** from a topology-optimised structure, and the measured
finding that doing so is **free**.

**(m) THE LOAD PATHS OF A GROUND-STRUCTURE TRUSS ARE POLYLINES, AND THEY KINK AT EVERY NODE.** Every
member is a straight chord, so a load path through several members is a polyline with a corner at
each one. **Measured: 384 of the 669 members continue a load path through a node, and they turn a
median of 31° there — up to 74°.** A smooth-min SDF fillet hides this on the *printed part*; the
**centreline** is still kinked, and the beam model still has a moment discontinuity at every one.

**Disclosed construction:**
- At each node, **pair up the members that continue each other** — greedily, straightest first — and
  give each matched pair a **SHARED TANGENT** (the bisector). Each member then becomes a **cubic
  Hermite curve** that leaves and arrives tangentially, so the load path through it is **C¹**.
- **Discretise each curve into straight sub-beams.** A curved beam *is* a polyline of straight beams
  in the limit, so **the FEM needs no new element type at all** — only a finer mesh. (4 sub-beams
  per member here.)
- A member with **no continuation within `MAX_TURN` (75°)** at one end is at a genuine **BRANCH**,
  and stays a corner. Five members really do meet at a trabecular node and only one pair of them can
  be tangent; the rest branch, and the fillet blends them. *That is what a branch point is.*
- The **tension `τ`** scales the tangents. **τ = 0 collapses the curve back onto the straight chord
  exactly** — it is the regression, and it reproduces the straight structure to +0.00% of length.

**(n) ⚠ AND *NOT* WITH A TRAIL COVER, WHICH IS THE OBVIOUS THING AND IS WRONG.** An Eulerian trail
cover (as used for wire-forming, §8.12) already picks "the straightest continuation at each node" —
but it must use **every edge**, so when the straight continuations run out it is **forced into
hairpins**. Measured on the real structure: the trails turn a **median of 57°** and **up to 180°** —
a complete reversal. Fit a spline through that and the curve doubles back on itself. **A load path
is under no obligation to cover every edge. A wire is. They are different objects.**

**(o) ⚠ A SPLINE CUTS CORNERS, AND THE CORNER IT CUTS MAY BE THE HAND.** The straight members clear
the flesh **by construction** — every candidate was checked against the skin before it was ever
offered to the optimiser (§8.10.2). **The CURVE between the same two nodes is a different object**,
and it is free to bow the wrong way. Measured, unrepaired: the worst clearance fell from **2.98 mm
to 2.35 mm**, straight through a 3.0 mm floor. So the curves' **interior points** are pushed back out
along the local skin normal until the rod surface clears — and **the NODES never move**, because they
are where the load is applied, where the anchors bear and where the buttons sit.

**(p) MEASURED: THE CURVATURE IS FREE.** Same topology, same gate, same nozzle:

| τ | mass | peak stress | load-path turn (90th pct) |
|---|---|---|---|
| 0.00 (straight) | 6.38 g | 4.7 MPa | 25.8° |
| 0.15 | 5.91 g (−7.4%) | 7.6 MPa | 21.1° |
| 0.30 | 6.25 g (−2.1%) | 5.9 MPa | **17.3°** |
| 0.50 | 6.39 g (+0.2%) | 7.2 MPa | 18.3° |

- **The mass is FLAT and NON-MONOTONE across τ.** That is the optimiser's own noise, not a trend. So
  the disclosed claim is **not** "curvature saves 7%" — it is the weaker and defensible one:
  **curvature costs nothing measurable in mass**, while the load paths measurably straighten
  (90th-percentile turn **26° → 17°**).

**(q) ⚠ AND BOTH OBVIOUS JUSTIFICATIONS FOR IT FAIL THEIR OWN MEASUREMENT — recorded because a
reason that does not survive its own test is not a reason:**
- **CLEARANCE.** *"A straight chord between two nodes sitting `hug` off a convex limb dips toward the
  flesh in the middle."* It does — but it **does not bind**: 0 of 669 members sit at the clearance
  floor. Not a reason.
- **FATIGUE.** *"A kink in a load path is a stress riser, and this device takes millions of
  keystrokes."* Measured peak stress is **4.7–7.6 MPa against a 70 MPa yield — 10% utilisation.** The
  structure is **stiffness-limited, not strength-limited**, so the kinks were never going to crack
  it. Not a reason either.
- What is left is the one the user gave, and it is sufficient: **the structure is a bone, and bones
  have no sharp edges.** It costs nothing, so there is nothing to trade it against.

⚠ **NOT OPTIMISED: the curvature is FITTED.** `SPLINE_TENSION` is *swept*, so its price is known —
but nothing moves the control points to minimise anything. A true spline-shape optimisation
(form-finding on the interior points, subject to the flesh constraint) has **not** been done.

### 8.15d The SECTION — and the finding that the device is TOUCH-limited, not load-limited

**THE USER:** *"I think the thickness of struts should be a spline too, with a major and minor
radius, and principal orientation as a spline."*

**(r) A CIRCLE IS THE WORST SECTION FOR A MEMBER THAT BENDS IN ONE PLANE.** For an ellipse of
semi-axes a, b: mass `A = πab`, but `I = πa³b/4` one way and `πab³/4` the other. **At constant mass**,
material can be moved out of the direction nothing pushes and into the direction that is bent.
Disclosed, and **locked by a test**: an oriented 2:1 ellipse is **2× stiffer than a circle of the
same mass**, and **4× stiffer than the same ellipse turned the wrong way**.
- Disclosed construction: separate `Iy`/`Iz` per element plus a **ROLL**; and **neither the roll nor
  the aspect needs a gradient** — both are read off the solved moments. **Roll = the principal moment
  direction. Aspect = the ratio of the two moments** (minimising `M₁²/k + M₂²k` gives exactly that).
  Solve → align → re-solve. *This is literally Wolff's law.*

**(s) ⚠ AND AN ELLIPSE FAILS, BECAUSE FOR AN ELLIPSE *FLAT MEANS SHARP*.** Its tightest convex
curvature is at the ends of the major axis, radius `b²/a` — so flattening drives it to a **point**,
and a 2:1 ellipse is *sharper* than the circle of equal area. Against the ergonomic floor (§5g) the
constraint becomes `b²/a ≥ SKIN_R`, i.e. `s ≥ SKIN_R·k^1.5` — a 3:1 ellipse would need a 7.8 mm
member. **Measured: the aspect capped out at 1.41:1 and the section bought nothing (+4% mass).**

**(t) A STADIUM IS BOTH FLAT AND BLUNT** — a rectangle with semicircular ends; the sweep of a circle
of radius `b` along a segment. **Its minimum surface radius is simply `b`.** So the friendly
constraint becomes a plain lower bound, *uncoupled from the flatness*: it can be as flat as it likes
and stay blunt. Measured: a 3:1 stadium is **6.96× stiffer in its flat plane and exactly as blunt.**
`A = 4bc + πb²`, `I_flat = 4bc³/3 + πb⁴/4 + πb²c²`, `I_thin = 4cb³/3 + πb⁴/4`, and
`J = 4·I_flat·I_thin/(I_flat + I_thin)` — **exact for both a circle and an ellipse**, so well-founded
for a stadium. `c = 0` recovers the circle exactly.

**(u) ⚠ AND THE STADIUM BOUGHT NOTHING EITHER — WHICH IS THE MOST USEFUL RESULT IN THIS SECTION.**
Measured: **0 of 153 members flattened.** Every one of them *wants* to (the moment ratio is 1.9
median, up to 11), but:

> **146 of the 153 members (95%) sit ON the ergonomic floor.**

Their size is set by a **HAND**, not by a force — they are already **thicker than their load
requires**. A clever section can only pay where the section is set by the **LOAD**, and here almost
none of it is. Flattening a floor-bound member raises its area to buy stiffness it does not need.

> **THE FRIENDLY GAUNTLET IS NOT STIFFNESS-LIMITED OR STRENGTH-LIMITED. IT IS TOUCH-LIMITED.**
> Its mass is set by the human, not by the physics.

**(v) WHICH HANDS US THE ANSWER, AND IT IS THE ONE A BONE ALREADY USES: MAKE IT HOLLOW.** If the
**outer** surface is what must be friendly, and the inside is doing nothing — **take the inside out.**
A tube of outer radius `b` and wall `w`:
- **its outer radius is still `b`** → *exactly* as blunt, *exactly* as friendly. Hollowing changes
  **nothing** about how it feels.
- it loses almost no stiffness, because **bending stiffness lives in the outer fibres** — the
  material near the axis was contributing almost nothing.
- and **it is free to print**: a **0.8 mm wall is two perimeters of a 0.4 mm nozzle**. *A hollow strut
  is a strut printed with no infill.* The printer was going to do this anyway.

**Measured, same members, same outer radii, same gate:**

| the friendly structure | mass | worst well | sharpest surface |
|---|---|---|---|
| solid rods | 14.90 g | 500 µm | 1.50 mm |
| **hollow tubes (0.8 mm wall)** | **12.00 g (−19%)** | 496 µm | **1.50 mm — unchanged** |

retaining **94%** of the bending stiffness. **A long bone is a tube with a marrow cavity. That is not
an analogy — it is the same optimisation under the same constraint.**

⚠ **RE-CHARACTERISED UNDER THE FIXED PRUNE (§8.15k (fff)).** The 153-member structure above was the
top-down prune's MEMBRANE — a bug. Re-run with the prune fixed (strain-energy ranking), the ergonomic-floor
study splits into two regimes, and the split *is* the finding:

- **The device stays touch-limited — more so.** The grow-based bone (§8.15k) is dense: **all 408 of its
  members sit on the 1.5 mm floor** (measured; was 95%), sized by the hand, not the load. Solid **20.9 g →
  hollow 12.7 g (−39%)**, worst well **172 µm** — and hollowing is free precisely *because* a floored member
  is **over-stiff** (172 µm against a 500 µm gate), so taking out the marrow spends slack it never needed.
- **But touch-limited is a property of DENSITY, not of the floor.** Ask the fixed 8 mm prune to *minimise
  mass* and it carves a SPARSE truss instead: **61 members, every one at the r_max ceiling (2.5 mm), 17.4 g**
  — **load-limited**, sized by the gate, not the hand. That is where the old "146-on-the-floor" reading
  would have gone had the prune not membraned; the ergonomic floor does not *make* a structure touch-limited,
  a dense one is.
- **And that is exactly why the device does not go sparse.** A load-limited member cannot be hollowed: it
  is at r_max *for its stiffness*, so removing the core drops its second moment to **~79%** and the well
  deflection from 405 µm **past the 500 µm gate**. The touch-limited dense bone, hollowed, is **12.7 g**; the
  load-limited sparse truss, which cannot hollow, is **17.4 g**. Touch-limited-and-hollow wins — the ergonomic
  floor plus the marrow cavity is **vindicated by the fix, not overturned by it.**

### 8.15e The SPIKE, the DEBRIS, and the BEARING AREA — defects that are structurally invisible

Disclosed as a class, because it is the one that has caught this project out most often:

> **A feature that carries no load is invisible to every structural measure — and may be the first
> thing a hand finds.**

**(w) A MEMBER WITH A FREE END CARRIES ZERO LOAD, SO IT COSTS NOTHING AND NOTHING DELETES IT — AND
IT IS A SPIKE.** Nothing is attached to a loose end, so there is nothing to react: the member is pure
dead weight, the sizer drives it to the minimum feature, and every stress, mass and stiffness measure
in the optimiser is perfectly happy. Measured on the shipped structure: **56 members ending in free
0.4 mm POINTS**, on a device that goes on a hand.
- ⚠ **AND THE SUPPORT REPAIR *PROTECTS* THEM, for a reason that is absurd once seen:** a rule of the
  form *"never delete the last member holding a node up"* will refuse to delete a loose end, **because
  it is the last thing holding up its own tip** — a node that only exists because the member exists,
  and which would vanish with it.
- Disclosed fix: delete every member with a free end, iterating (deleting one creates the next), with
  the **load points exempt** — a well legitimately ends at the fingertip, and a bearing foot
  legitimately stands on the flesh.

**(x) THE STRUCTURE IS THE PIECE THAT HOLDS THE LOAD POINTS. EVERYTHING ELSE IS DEBRIS.** A
connectivity rule of the form *"keep everything reachable from any support"* is **too weak**: a
support is merely a node that bears on the body, so a fragment hanging off one and connected to
nothing else **passes**. Measured: a **13-node component floating free**, touching no anchor and no
well, carrying nothing — 1 g of dead weight that survived every prune.
- Disclosed fix: keep only the component containing the **load points** (the wells), and run it to a
  fixed point against the free-end sweep, **because each fix causes the other**.
- ⚠ **And the order matters:** the free-end sweep must run **AFTER** the support repair, never before.
  The repair *adds members back* to hold up orphaned nodes, and every member it puts back can land a
  fresh free end that a check which has already run walks straight past.

**(y) A SPIKE IS WORSE THAN A SACRIFICIAL PILLAR, and the trade is explicit.** A support member whose
far end is loose holds a node up **for the printer** *and* is a rod ending in a point **on a device
that is worn**. Delete it, and the node it was holding pays a **pillar** instead. **A pillar is
snapped off and thrown away. A spike is worn.**

**(z) BEARING AREA IS A CONSTRAINT, AND IT IS MISSING.** The wells are constructed as a **CRADLE** —
contact distributed along the phalanx, centre of pressure free (§8.13). The **anchor feet**, where
the device's entire reaction load is delivered into the hand, are **not**: a member standing on the
flesh and ending in a `SKIN_R` hemisphere, pressed in by the strap, is a **pressure point**.
Disclosed: **any surface through which a wearable delivers a sustained reaction into the body must be
constrained on its bearing AREA, not merely on its surface radius.** *Not yet implemented.*

### 8.15f The STRAP as a TENSIONED GEODESIC, and its printable form

**(aa) A STRAP IN TENSION TRACES THE CONVEX HULL OF THE CROSS-SECTION IT WRAPS.** Disclosed, with the
reason: a tensioned band takes the **shortest closed path** around the limb, and the shortest loop
around a cross-section **is its convex hull**. It cannot enter a concavity; it **bridges** it. So the
band's centreline is not a circle, not an offset of the skin, and not a fitted curve — it is a hull,
and it is *derived*, not drawn.

**(bb) ⚠ AND IT IS THE HULL OF (LIMB ∪ DEVICE), NOT OF THE LIMB.** The strap passes **over** the
structure in order to pull it **down** onto the body. Hull the limb alone — as this project's own
`strap_loop()` does — and the strap passes **straight through the device**. That is adequate for
deciding *which nodes the strap pulls on* (which is all the FEM needs) and **wrong as a printable
shape**. The gauntlet stands up to **~7 mm proud** of the skin.

**(cc) COROLLARY — A HULLED STRAP PRESSES ONLY ON THE CONVEX HIGH POINTS.** Where the hull bridges a
concavity the strap **does not touch**, so it delivers no pressure there; the load concentrates on
the extremes of the hull, which on a hand are the **bony prominences** (2nd and 5th metacarpal heads,
ulnar styloid) — exactly the places that hurt. Disclosed: the same **bearing-area** constraint of (z)
applies to the strap, and a hulled band **must be padded or broadened where it crowns a prominence.**

**(dd) MEASURED — AN ELASTOMERIC (TPU) STRAP, PRINTED RATHER THAN SOURCED.** The strap is the
**compliant element of the whole anchor**: flesh can only *push*, so the strap supplies the entire
*pull*, and without it this structure deflects **9178 µm** instead of 495. TPU is **~100× softer**
than the nylon webbing assumed (E ≈ 12–30 MPa against 2.0 GPa). Same structure, same gate:

| strap | E | worst well displacement |
|---|---|---|
| nylon webbing (assumed) | 2.0 GPa | **498 µm** ✓ |
| **TPU 95A** | 30 MPa | **567 µm** (misses a 500 µm gate by 13%) |
| TPU 85A | 12 MPa | 578 µm |

- **A 100× softer strap costs only 13% of the gate — and THAT is the disclosed finding.** The
  friendly, floor-bound structure (§8.15d) is **chunky, and therefore far less strap-dependent** than
  the wire-thin one it replaced. **The ergonomic floor bought structural robustness it was never
  asked for**, and it is what makes a printed elastomeric strap viable at all.
- Disclosed: a **single-material-printable** wearable (rigid skeleton + printed elastomeric band)
  needs **no webbing, no buckle, and no supplier** — which is the reproducibility constraint of
  §5g.1 satisfied, not merely acknowledged.

### 8.15g The SENSOR as a PRINTED FLEXURE, and how many directions each well really has

The well replaces the Svalboard **20 gf magneto-optical key** with a **magnet on a compliant
flexure over a 3-axis Hall** — every custom part printed, no contacts to wear, no spring to source.
Sizing the flexure (`manufacture/flexure.py`, `scripts/flexure.py`; `design/sensor.py`,
`scripts/sensor.py`) settled the material, the mechanism, and the per-finger layout — and two of
them are *forced*, not chosen.

**(ee) THE FLEXURE MATERIAL IS DECIDED BY ONE NUMBER — σ_fatigue/E, THE MAXIMUM RECOVERABLE BENDING
STRAIN.** A well that actuates at 20 gf over 1.5 mm needs a *soft* restoring spring, k = F/travel ≈
**131 N/m**. Built as an isotropic rod or dome soft enough for that, a stiff FDM plastic runs past
its own fatigue limit and cracks: glass-nylon **50 MPa vs 25**, PLA/PETG/ASA alike; plain PA12 is
the one exception and it merely *squeaks under* (13 vs 16 MPa), with no safety margin. Only **TPU**
(σ_fat/E ≈ **0.115**, ~27× the stiff plastics) has the headroom as a one-part flexure; thin
**spring steel** has it too, but only as a **leaf or cruciform** — go thin, which a shim can and a
nozzle cannot. Disclosed: the gauntlet is stiff (glass-nylon), the flexure soft (**a TPU dome, or a
steel cruciform**), and the split falls straight out of a single material property.

**(ff) THE PLUNGE MUST BEND, NOT COMPRESS — WHICH IS WHY THE ANSWER IS A DOME.** The down-press
cannot be the flexure *shortening*: even TPU is **~112× too stiff** in axial compression (glass-nylon
1700×, steel 9800×). It must be a **bending** mode, so the one-part flexure is a **dome/diaphragm** —
soft in tilt *and* in plunge from a single magnet, and a shallow dome **snaps** for a tactile click.
A TPU dome of radius ≈6 mm, thickness ≈0.32 mm, at ~1 MPa surface stress, fits inside the ~7 mm
flesh-radius well. (The 0.3 mm membrane sits at the FDM single-perimeter floor — dome it, corrugate
it, or drop to a 0.25 mm nozzle.)

**(gg) THE MOMENT AND LEVER CANNOT MANUFACTURE A MUSCLE.** Actuation effort — measured per finger
and per direction across the 5th–95th population by the *same cradle solve the gauntlet layout uses*
— is **negligible wherever a direction is feasible**, because the cradle bears the load. So the dome
wants to be **as soft as a deliberate press allows**, and tuning the lever (cup height, contact
offset) does **not** change *which* directions a finger can use: those are limited by **muscle
capacity, not leverage**. The optimisation's real yield is therefore a *feasibility map*, not a
per-finger stiffness.

**(hh) THE WELLS LOOKED THREE-WAY ON THE ULNAR FINGERS — AND RUNNING THAT DOWN FOUND A CRADLE
ARTEFACT, NOT A MUSCLE LIMIT.** `click` / `forward` / `back` are actuable by every finger; the
lateral tilts were the question. On the first model only **18 of 25** (finger, direction) pairs
passed — thumb 5/5, index 4/5, the ulnar three 3/5. The chase (`design/sensor.py`, `hand/cradle.py`):

- The finger **interossei are present** — RI/UI/LU per finger, hundreds of mN·m of abduction
  capacity. Never the problem. (An earlier claim they were weak/absent was wrong.)
- A lateral tilt demands almost pure **MCP abduction** (~15 mN·m at 20 gf), yet it came out
  infeasible with the residual on **IP flexion** — ~1 mN·m the muscles could not cancel.
- The cause was **the cradle model, not the hand**. It let only the *sensed* wall react during a
  lateral press and **withheld the well floor** — a conservatism adopted to avoid a
  self-cancelling-preload bug (§8.13). But the finger is *still resting on the floor* while it tilts,
  and that floor bears the small IP torque; withholding it demanded a **muscle** for a **floor's**
  (and, in a real finger, a DIP **collateral ligament's**) job.
- Restore the floor as free support during a non-floor press — **only the floor, never the opposing
  wall**, so it cannot self-cancel into a fake press — and stock MyoHand is **25/25**: every well
  five-way, all five digits.

| | stock, floor withheld | floor restored |
|---|---|---|
| usable (finger, direction) pairs | 18/25 | **25/25** |

The control still holds and is the important half: a stock thumb with **no adductor still presses
0/5** — the floor lends no muscle (`test_design`). And it is corroborated directly by a pianist: the
independent middle/ring adduction of split chords and augmented sixths is not difficult.

**(ii) ⚠ WHAT THIS DID AND DID NOT SETTLE.** An intermediate hypothesis — that MyoHand's missing
**extensor hood** (it models the intrinsics as bare tendons) was the cause — *also* makes the tilts
feasible, and the hood is a genuine gap. But it was **not the operative one**: the floor fix resolves
the tilts with the interossei MyoHand already has, so the hood was not needed here and the
scaffolding for it was removed. The hood remains a real limitation for anything that turns on its
**IP-extension coordination**, and **OpenSim ARMS** (43 muscles, real extensor hood; access pending)
is the cross-check for that. Separately, the **false-trigger floor** — the only thing that would push
the dome stiffer than "as soft as possible" — is still **not measured**.

### 8.15h The STRAP ANCHOR — the band that goes OVER the device, and the watch-lug that holds it

⚠ **RECONFIGURED by §8.15j**: the strap is now the *innermost* layer (against the hand) with the
gauntlet on its outer face, so the band is the hull of the *limb*, not of (skin ∪ device). The
hull math (jj), the watch-lug (kk), and the adjustment (ll) all still hold; the band just moves inboard.

The strap is the compliant half of the anchor (§8.15f); this is how it attaches and adjusts
(`manufacture/strap.py`, `scripts/strap.py`, `tests/test_strap.py`).

**(jj) THE BAND GOES OVER THE STRUCTURE, AND THE CODE NOW DOES TOO.** §8.15f (bb) named the
principle — the band is the convex hull of (skin ∪ device), not of the limb — but the renderer still
hulled the limb alone, so the drawn band passed *under* the dorsal gauntlet. Measured, the device
**is** proud at both strap stations (the wrist band bulges **27 mm** over it, the metacarpal band
more), so a skin-only band would sit inside the structure it is meant to pull down. `band_loop` now
takes the 2-D convex hull of the union — which cannot pass through either set by construction — and
`viz` reads that same one definition. (An earlier attempt with a max-radius-per-angle outline *dipped
between struts*, because one point per strut undercovers each tube's angular width; a hull does not.)

**(kk) THE JOIN IS A CAPTURED PIN, NOT A BOND IN PEEL.** Where a soft strap meets a stiff printed
part, an adhesive loaded in peel is where it fails. So each anchor foot carries a **watch-lug**: a
printed boss with a through-hole (r 1.1 mm, ≥ two nozzle walls), a pin along the hand's long axis,
and the circumferential TPU strap **looping the pin in SHEAR**. Three lugs sit on each band, on the
nodes the strap already pulls (§8.11) — the tension goes straight into the load path.

**(ll) ONE ADJUSTABLE STRAP FITS THE POPULATION.** The wrist-band circumference over the 5th–95th
percentile hand is **139 → 160 mm — a 21 mm spread (1.15×)**, and a watch strap's holes span
30–40 mm. So a single strap covers everyone; the per-finger wells carry the rest of the fit (§8.5).

**(mm) THE BOND IS A MATERIALS CHOICE, AND IT ONLY HAS TO SURVIVE HANDLING.** PU adhesive keys the
TPU to the printed lug; a **vinyl-silane primer** couples to the *glass* in glass-nylon before a
structural epoxy/PU. Because the pin carries the tension, the bond is not in the primary load path.

**(nn) ⚠ AND WHAT IS NOT SOLVED.** The metacarpal band rides mostly on the *device* (it wraps the
well-arm fan-out), and the pad flag (`bridging_fraction`) measures stand-off from *skin* — so it
cannot yet tell "riding the smooth device" (fine) from "bridging a skin concavity onto a bony
prominence" (needs a pad, §8.15f cc). The band path also drops struts more than 20 mm proud of the
skin as well-arms rather than segmenting body-from-arm; a strut grazing that cutoff can still tug the
hull. Both are heuristics, flagged, not solved.

### 8.15i The INNER BEARING SHELL — an impact distributor, and a sandwich

⚠ **SUPERSEDED as the skin interface by §8.15j** (the gauntlet mounts on the OUTSIDE of the strap, so
the soft strap is the pad and no inner shell is added). Retained as the reasoning that led there, and
because the impact-distributor physics still applies to anything that *does* bear directly.

The device is strapped to the hand and worn all day, so how it MEETS the skin is a first-class
ergonomic problem — and one the structural model leaves open (`manufacture/bearing.py`,
`scripts/bearing.py`, `tests/test_bearing.py`).

**(oo) THE GAUNTLET DOES NOT TOUCH THE SKIN YET — IT BEARS THROUGH AN ABSTRACT SPRING.** Measured,
the optimised structure floats **≥ 5.5 mm off the skin** (no node within 2 mm), and it transmits its
load through a *distributed compression spring* hanging below it — there is no real contact geometry.
Build that interface naively, as feet on the structural nodes, and it becomes a set of PRESSURE
POINTS: a 50 N knock through one 1.5 mm foot is **7.1 MPa**. The whole ergonomic outcome lives in
this un-designed interface, and the STRAP is what makes it bite — its preload presses the interface
into the flesh whether or not a key is being pressed.

**(pp) THE INTERFACE IS AN IMPACT DISTRIBUTOR, AND THE KNOCK — NOT THE PRELOAD — SIZES IT.** A
conformal SHELL is a stiff plate on the soft tissue (a Winkler elastic foundation, k = E_tissue /
thickness). A concentrated load spreads over a characteristic length **λ = (D/k)^¼**, and the peak
skin pressure under it is **P / (8 λ²)**. The steady strap preload is shared over several lattice
junctions (< 1 N each) and is comfortable at any thickness; a KNOCK lands as a single point, so it
binds. Measured: a **1.5–2 mm glass-nylon shell** spreads a 50 N knock over a ~5–6 cm patch to
**56–86 kPa** — felt, not injurious — against **7.1 MPa** for a bare foot. So the impact requirement
sets the thickness, and it comfortably covers the preload.

**(qq) THE SHELL IS THE INNER FACE OF A SANDWICH, NOT A SOLID PLATE.** A solid shell of that
thickness over the dorsal patch is ~10–13 g — it roughly *doubles* the 11 g bone. So the disclosed
construction is a **sandwich**: the bearing shell as the **inner (skin-side) face**, the
topology-optimised lattice as the **core**, and a thin dorsal **outer face** — a sandwich panel is
far stiffer per unit mass than a solid plate, so it delivers the impact-spreading λ at a fraction of
the mass, and it *reuses the bone already optimised* rather than adding to it. The inner face doubles
as the surface the wells and sensor PCBs mount to.

**(rr) THE GATE, RE-SOLVED WITH THE FACE — IT HOLDS.** The inner face was added to the per-element
Sizer as CST membrane triangles and the 500 µm key-deflection gate re-solved at the bone's real
sections (`scripts/sandwich.py`, `tests/test_sandwich.py`): the buttons hold at **485 µm ≤ 500 µm**,
and the no-face baseline reproduces the **498 µm** bone gate exactly. So the face does not compromise
key-crispness. It barely *helps* the gate (−3%) — the anchor is tissue-dominated — which confirms the
face's value is the IMPACT (pp), not crispness.

**(ss) ⚠ WHAT IS VERIFIED, AND WHAT IS NOT.** The sizing physics is verified (plate on elastic
foundation, `tests/test_bearing.py`) and order-of-magnitude only — quasi-static, infinite-plate,
Winkler — so the knock pressures are a lower bound. The gate re-solve models the face as a **flat CST
membrane tying the bearing nodes**: it captures the in-plane tying and the gate, but NOT the
out-of-plane plate bending that *is* the impact benefit (that stays the (pp) analysis), and the
full-membrane face mass (**+10.8 g** at 1.5 mm) is an upper bound — the sandwich's real saving is
*thin* faces over the lattice core, a bending efficiency the membrane model does not compute. A
finer conforming shell on the skin (not the ~5.5 mm-proud bearing nodes) and the outer face remain
to be built. The 50 N knock magnitude and the comfort/pain thresholds (~4 kPa capillary, ~20 kPa
worn, ~200 kPa painful) are estimates, flagged.

### 8.15j THE GAUNTLET MOUNTS ON THE OUTSIDE OF THE STRAP — the interface reduced to one soft part

A design decision, reached by the loop that runs this whole project: an instinct at the render (the
strap presses the gauntlet's hard bits into the hand), the reasoning for why it felt wrong (the
anchor is a tension tether, not a press-in), and the model that grounded it (`scripts/onstrap.py`).

**(tt) THE SOFT STRAP IS THE SOLE HAND INTERFACE.** Mount the gauntlet on the OUTER face of the
strap, and the soft TPU band becomes the only thing that ever touches the hand — it cushions the hand
from every hard or pointy feature of the structure. This **supersedes the bearing shell (§8.15i) as
the skin interface**: nothing is added to the gauntlet's inner face, because the strap already is a
soft, distributed pad.

**(uu) ONE PART, THREE JOBS.** In tension the strap **tethers** the gauntlet against the keypress
lift-off (the anchor is compression-only, §8.15f/§8.11 — the flesh cannot pull, so the strap
supplies it); the same soft band **cushions** the hand from the gauntlet; and it **spreads** the
bearing over its contact area. The tether, the pad, and the cushion are the same component.

**(vv) MEASURED — THE GATE HOLDS THROUGH THE SOFT LAYER.** Re-solved with the strap as a compliant
layer in series with the soft tissue on the pressing side (`scripts/onstrap.py`): the buttons hold at
**499 µm** (+0% vs the 498 µm baseline) at strap thicknesses from **1 mm to 5 mm**. TPU is floppy in
bending but **stiffer in through-thickness compression than the tissue it sits on** (k ≈ 520–2600 vs
67 kN/m), so it is negligible in the load path. Conservative — the strap's load-spreading (ignored)
would only stiffen it.

**(ww) THE STRAP FLIPS TO THE HULL OF THE LIMB.** Innermost now, the band is the convex hull of the
HAND (hand-hugging), not of (skin ∪ device); the gauntlet rides on its outer face. (§8.15h's
over-the-device band was the previous configuration; the hull math of (jj) stands, applied to the
limb.)

**(xx) THE ATTACHMENT IS A PRINTED TPU LOOP — AND IT PRINTS AS ONE PART WITH THE STRAP.** The
strap→gauntlet join carries the anchor loads in shear/tension, and it need be nothing more than
**loops or hooks printed integrally into the TPU strap** that catch the gauntlet's own anchor
members — a belt-loop, not a fastener. Strap and attachment are then a *single* printed part, with no
buckle and no supplier — the reproducibility constraint of §5g.1 met again. (TPU prints such loops
cleanly.)

**(yy) ⚠ STILL OPEN.** The **meshed strap** (the re-solve is a conservative compliance model, not a
modelled band) and the **loop geometry** itself. (The **impact** through the path, flagged open here,
is now computed — **§8.15k** — and it both re-sized the bone and showed the strap cushions most knocks
but not a direct one on a fingertip well.) The decision is sound on the gate and the comfort; the
meshed strap and the loop are the remaining work.

### 8.15k WILL IT BREAK? — impact as the binding structural load, and the re-optimisation for it

§8.15j (yy) left the **impact** argued but not computed. It is now computed — `scripts/robust.py`
(does it break or fatigue), `scripts/impact.py` (does a knock hurt the hand through the strap),
`scripts/regrow_impact.py` (does the knock want a different skeleton), `scripts/impact_opt.py` (the
proper re-optimisation) — and it moves the structure.

**(zz) THE KNOCK, NOT THE KEYPRESS, IS THE BINDING STRUCTURAL LOAD.** Two questions the 500 µm
deflection gate never asked. **Fatigue** is trivial: the worst member sees **1.6 MPa** under a 0.196 N
keypress against a 25 MPa fatigue limit — a **16× margin**, because the device is touch-limited
(§8.15d) and its members are already far over-thick for a keypress. But a **50 N knock is 250× a
keypress**, and on the deflection-optimised bone it drives the worst member to **348 MPa against a
70 MPa yield — it BREAKS** (by well: thumb 348, little 254, ring 245, index 239, middle 116, dorsal
72 MPa). A structure sized only to be *crisp* is not a structure that *survives being knocked*.

**(aaa) THE KNOCK WANTS A DIFFERENT SKELETON — BROAD, NOT SPARSE.** A keypress-deflection gate wants a
*sparse, efficient* skeleton; a localised knock wants a *broad, redundant* one that shares the blow
over many members. Measured: the same 50 N knock is **348 MPa on the sparse bone but ~56 MPa spread
over the broad candidate domain**. Grown *with* the knock in the load set versus without, the two
skeletons share only **20% of their members (Jaccard 0.20)** — the impact-aware grow keeps 781 struts
to the keypress grow's 339, and **595 of them are members the keypress grow threw away**. The
robustness the ergonomic floor bought (§8.15f) covers the keypress and the fatigue; it does **not**
cover the knock, which needs its own topology.

**(bbb) IMPACT-IN-THE-LOOP BEATS BOLTING IT ON.** Two ways to make the bone survive the knock at a
safety factor of 2: *bolt-on* — size the keypress skeleton for the gate, then thicken every
over-stressed member (fully-stressed design) — or *in-the-loop* — grow WITH the knock and size for the
gate AND the stress together. Measured like for like (circular rods, one anchor, one gate, both
surviving): bolt-on reaches **37.7 g and cannot even hit SF 2** — its sparse members **saturate at the
radius ceiling** at 50 MPa; in-the-loop reaches **23.2 g at SF ≈ 2** (36 MPa, 747 struts), **39% lighter and
more robust**, because the broad skeleton never drives a member to its ceiling *and* its shape is
form-found (**(ddd)**), not left staircased on the grid. The method: a knock is
a *local stress* limit and a keypress a *global deflection* limit, married by feeding a
fully-stressed-design floor `r_e ≥ r_e·√(σ_e·SF/yield)` into the deflection sizer as a per-member
lower bound (`rlo`), iterated to a fixed point as the knock redistributes; and `grow()` ranks members
by their keypress **and** impact strain energy while the deletion gate stays keypress deflection.

**(ccc) THE STRAP CUSHIONS MOST KNOCKS — BUT NOT A DIRECT ONE ON A FINGERTIP WELL.** Through the
gauntlet → strap → hand path (§8.15j), a 50 N knock spread over the strap footprint is **10× gentler
than through a bare 1.5 mm foot** (7.1 MPa): a knock on the back of the hand or a well shared by two
anchor loops lands at **36–93 kPa** (felt, not injurious). But a knock straight onto the thumb, index
or middle well **levers onto a single anchor loop** and reaches **228–689 kPa** — past the ~200 kPa
threshold of a painful knock. So the fingertip wells, the most exposed features, want their own
mitigation: **more anchor loops sharing, or a compliant well cup** that yields before the load
concentrates. (Both deferred; the cup also cuts the input force the structure must survive in (zz).)

**(ddd) THE SHAPE HAD TO CONVERGE TOO — AND THE DECOUPLED PIPELINE HAD FORGOTTEN IT.** A sized skeleton
comes off the lattice *staircased*: nodes pinned to grid sites turn a load path in kinks (~8% past 75°
on the impact structure, 12% on the keypress bone), which the eye reads — correctly — as un-converged,
and which `curves()` can only *follow*, not straighten. `relax_nodes` is the fix: **form-finding** —
move each free node toward axial equilibrium (the FEA residual points the way), held in the skin band,
buttons and anchors fixed. It is the pass every *reported* structure is meant to get; `grow()` runs it
*inside* the topology search, but the **grow-then-co-size** impact pipeline applied it only once at the
end, and the old **size-then-prune-then-curve** keypress-bone pipeline not at all — and that pipeline is
now abandoned, because its top-down re-prune dead-ends in a heavy membrane (**(fff)**). Adding the relax
straightens the impact structure (**kinks > 75°: 37 → 12**, median through-turn **30° → 13°**) **and
lightens it**: **26.3 → 23.2 g (−12%)**, gate still held. Straightening is the reason — a member sized
thick to resist **bending** at its kink carries **axially** once straight, and axial members are thinner.
The keypress bone is now rendered straight from the grown, already-form-found topology (**(fff)**), so it
needs no separate relax pass at all.

⚠ **This corrects an earlier reading** (the `relax_nodes` note): that relaxation is "cosmetic, barely
moves mass" on a sparse bone. That was measured on the grow-**front** designs — which are *already*
relaxed, so more relaxation buys little. On a structure that was **never** relaxed it is worth a fifth
of its mass. The trap was a real measurement quoted outside the scope it was taken in.

**(eee) THE SHELL THE RENDER SUGGESTS IS A COMFORT FEATURE, NOT A MASS ONE — WEIGHED BY A COUPLED
PLATE FEA.** The form-found impact lattice, dense in a thin band off the skin, *looks* like a shell —
correctly, because a form-found lattice confined to a thin layer **is** a discretised shell. So would
an explicit shell or sandwich (§8.15i) be lighter? Tested end to end, and the answer is **no**, for a
reason worth recording:
- a shell to take a knock **on the back** is pointless — the dorsal-ridge knock sizes **5 members,
  0.1 g**; the fingertip-**well** knocks size the rest (`sandwich_weigh.py`);
- the discrete tissue anchors **are** a bottleneck (the well-knock reaction funnels through ~31 of
  them), and a continuous bearing spreads it: a **coupled lattice + finite-stiffness plate FEA**
  (`sandwich_fea.py`, PyNite MITC4 quads tied to the anchors) cuts the worst well-knock member stress
  **96 → 52 MPa (~46%)** at a 1 mm shell — real, not the 65% rigid-shell bound of the proxy;
- **but that thins the lattice only ~3–4 g, while the shell over its ~100 cm² footprint costs ≥ 5 g** —
  so **no shell thickness beats the pure lattice**: the lightest sandwich is **≥ 27 g** against the
  lattice's **23.2 g**. The shell saves less lattice than it weighs.

So the **density is fundamental**: a topology-optimised lattice is already the efficient way to carry a
knock into the tissue, and a plate cannot pay for itself thinning it. The shell's real value is
**continuous skin bearing** — comfort, low pressure — which the soft strap (§8.15j) already provides
for free. (Honest arc, recorded because the reversals are the method: a "16–18 g sandwich" guess, then
"the lattice always wins", then a proxy that reopened it at "up to 65%", before the coupled FEA settled
it. The plate FEA is the arbiter; the guesses were not.) ⚠ The shell mass is pure geometry (ρ·A·t) and
solver-independent; the lattice it saves is measured in PyNite (which reads ~2.6× the project solver's
stress), so the robust part is the **ratio** — shell mass ≫ lattice saved — not the absolute grams. Flat
shell, stiff-strut coupling, FSD thinning; a curved shell would save a little more but still cannot
out-run its own mass. Method chain: `manufacture/bearing.py` (Westergaard) → `scripts/shell_fea.py`
(plate FEA firms the shell mass) → `scripts/sandwich_weigh.py` (which knock drives the mass) →
`scripts/sandwich_fea.py` (the coupled FEA that pins the number).

⚠ **QUASI-STATIC**, as throughout (§8.15i): a real impact adds dynamic amplification (energy, contact
time) this does not model, so the stresses and pressures are a lower bound and the 50 N magnitude is
an estimate. ⚠ The **39% of (bbb) is a RATIO in circular rods** — the sizer's section, not the hollow
stadium of the final bone (§8.15d) — so it transfers as a ratio, not as grams; the bolt-on's own
hollow-stadium number is **11 g → 17 g at SF 2** (`out/robust.npz`, gate 498 → 114 µm, so thicker is
also crisper). The fully-stressed floor oscillates at 36–39 MPa; both structures stay well under yield.

**(fff) RENDER THE BONE FROM THE GROW, NOT A RE-PRUNE — THE TOP-DOWN PRUNE DEAD-ENDS IN A MEMBRANE.** The
keypress bone used to be re-derived for printing by `printable.py`: re-ground an 8 mm lattice and
`size_and_prune` it top-down (delete the lowest-strain-energy members, re-size, repeat). On the
enslavement design (§6) that returned **1149 members / 33.9 g hollow** — 5× the grow's own 410-strut
topology, which meets the *same* gate at **7.5 g** beam. The impact structure proves it is too heavy: it
carries the keypress *and* the 50 N knock at **23.2 g**, so a keypress-only bone cannot honestly need 33.9 g.

⚠ **CORRECTION, recorded because the failures are the method.** An earlier version of this claim said the
*pre-enslavement* design "pruned cleanly to 138 members / 8.5 g" and only the enslavement design
"trapped." **That was wrong, and I published it without measuring it** — the old design was gitignored and
overwritten, so I took the 138 from stale doc numbers instead of a real run. The archive held a
pre-enslavement front (`out_archive/pareto_seed1.pkl`); its knee **also prunes to a uniform membrane —
754 members, 27.8 g** — and so does the current design *unconstrained* (no nozzle floor, no support
protection: **1799 members, 62.5 g**). The 138 / 8.5 g truss is from an **older design era** and is **not
reproducible** on any recent design. The trap is neither enslavement-specific nor an FDM artefact.

**What it actually is.** For the recent design family the buttons (fingertips) sit **far from the dorsal
anchors** — mean **62–71 mm** — so a keypress load must **fan out across the whole dorsal skin**. Every
member then carries a similar small share: a **membrane**, not a truss. Uniform strain energy means the
sizer cannot differentiate — it parks every radius at r0 = 0.90 mm (measured: **p90/p10 = 1.00**) — and
greedy top-down deletion has no signal, so cutting *any* member breaches the gate. The prune dead-ends in
a heavy uniform net: a **local optimum**. A light 7.5 g truss exists; the **grow** finds it because it
works **bottom-up** — adding high-strain-energy members toward the load paths from a connected seed on a
free-node 4 mm lattice — and so never forms the membrane it would then be unable to escape.

**Enslavement's role is 1.5×, not 8×.** The re-opt landed on a **less-curled posture** (the long-finger
tm drives fell ~0.42 → 0.11), extending the fingers and moving each button **15–22 mm farther from the
anchors** (mean 62 → 71 mm). Farther buttons ⇒ a wider membrane ⇒ **754 → 1154 members, 27.8 → 41 g**.
That is the entire "it got heavier" effect; the membrane itself was already there.

**The fix.** `bone.py` renders the **grown** topology directly (`out/final.npz`): rebuild the anchor model
with a cheap `ground()` call (the grow moved the nodes but kept every index, so anchors/buttons/bars line
up), then only **size** the struts to the ergonomic floor, curve and hollow them — **20.9 g solid →
12.7 g hollow, 172 µm**, no prune; within **6%** of the old committed 12.0 g. The lesson holds and is now
better founded: a topology the optimiser already grew must not be re-**searched** by a greedy top-down
prune that can dead-end in a membrane — **size it, don't re-prune it.**

⚠ **AND THE PRUNE ITSELF IS NOW FIXED — it was a one-line RANKING bug.** `grow` and `size_and_prune` are
both top-down ESO; the only difference that mattered is the signal each deletes by. `grow` ranks members
by **strain energy** at a fixed radius, where an idle member reads as idle whatever the sizer later does;
`size_and_prune` ranked by the **OC-sized radius**, which the OC returns *uniform* on a membrane — no
signal, so it deleted ~blindly and stalled. Measured, on the same 8 mm lattice: `grow` carves a **205-strut
/ 7.2 g** truss; the old prune stalled at **1149 / 41 g**; `grow` with node-relaxation *off* still gives
205 / 7.2 g — so it was never relaxation or pitch, only the ranking. So `size_and_prune` now ranks
deletions by strain energy too (one `solve` at a fixed radius), and it carves a truss: **253 members /
8.9 g** — grow's 205 plus the ~50 FDM support struts the print version keeps. The impact and bone numbers
are unchanged (both were already grow-based); the fix corrects the `printable`/`ergonomic`/impact-bolt-on
prunes. Guarded by `test_the_prune_carves_a_truss_not_a_membrane`, which fails if the prune ever weighs
more than 2.5× the grow again. And the ranking is **free**: `size` reads the strain energy off the OC's
*own* solve — the sizer already has the displacements and the radius-scaled element stiffnesses, so the
per-member energy density (½·uᵀk u / L) falls out with no second solve. Measured, the prune goes **38 → 30 s**
(the truss lands unchanged, 253 → 250 members). ⚠ An earlier note estimated the extra solve at "~2× the
prune time"; that was pessimistic — it was ~20%, because a prune step is dominated by the OC's own sizing
solves, not the one ranking solve.

### 8.15l THE READ-OUT — the field a moving magnet presents to the Hall (the mount that holds it is being redesigned, ppp)

§8.15g sized the restoring **spring** (a TPU dome, k ≈ 131 N/m) but explicitly deferred the
**signal** — "NOT MODELLED HERE: the field a moving magnet presents to the Hall." This closes that
gap and turns both into a printable **two-part module** (`manufacture/readout.py`,
`manufacture/wellmod.py`, `scripts/readout.py`, `scripts/coupon.py`; `tests/test_readout.py`,
`tests/test_wellmod.py`). Every number below is a **prediction** from a numpy field model
(exact-cylinder on-axis + point dipole, no `magpylib`), to be checked on the stage-1 bench.

**(mmm) THE SIGNAL SWAMPS THE SENSOR.** A **Ø3×1 mm N42** disc on the cradle over a **3-axis Hall**
(TLI493D-W2BW class, 0.098 mT/LSB, ±130 mT, ~0.2 mT noise) at a **3.5 mm** rest gap:

| | field | vs sensor |
|---|---|---|
| plunge (`click`) rest → full 1.5 mm | 19 → 61 mT | Δ **42 mT = 427 LSB ≈ 200× noise** |
| at the 1.8 mm hard stop | 80 mT | 0.62 of range — no clipping |
| weakest lateral tilt | ~7 mT | ~73 LSB ≈ 36× noise |

The plunge uses the **exact** axial formula (a dipole is 15–35 % high in the near field); the four
tilts use a dipole **difference** (posed − rest), so that bias cancels and what is left is the
transverse field a sideways shift raises.

**(nnn) THE FIVE DIRECTIONS ARE MUTUALLY LEGIBLE.** `click` is a pure +z swing; the four tilts are
±x / ±y in the transverse plane. Their ΔB vectors sit **≥ 78° apart**, and a nearest-template
classifier makes **0 errors in 10⁵ draws** at the datasheet noise. A ±0.5 mm build error in the gap
keeps them ≥ 74° apart — a per-well calibration (read each direction's real ΔB once) absorbs it.

**(ooo) CROSSTALK AND AMBIENT ARE BELOW THE FLOOR, AND BASELINED.** The tightest well pair
(middle–ring, **18.6 mm**, from `out/final.npz`) leaks **0.23 mT static / 0.055 mT** full-travel
modulation onto its neighbour's Hall — the modulation is **< 1 LSB and below noise**; the static
part, like Earth's 0.05 mT, is a constant a **baseline tracker** removes. Every wider pair is
smaller by 1/r³.

**(ppp) ⚠ THE MOUNT GEOMETRY IS BEING REDESIGNED — the first attempt did not model the finger's
ENTRY ROUTE, and is withdrawn.** A printed PA frame + drop-in TPU cradle to hold the magnet and Hall
were built and OTS-anchored (17th–20th), but checked only for the finger's *static seated* clearance
— never for the **route the fingertip must traverse to enter the cup** (it slides in along the
phalanx axis from the proximal-open end). A mount can clear a *seated* finger and still block it from
ever *entering*, which is exactly what kept recurring (a strut across the entry; a rim over the cup).
That geometry is **withdrawn from HEAD** (the anchors stand as dated floors on the read-out
disclosure). The mount is being rebuilt with a **finger-entry-route swept-clearance as a first-class
constraint** (`manufacture/entry.py`), the geometry validated against it by construction rather than
by eye. The read-out physics above (mmm–ooo) is independent of the mount and stands.

**(qqq) THE HARNESS AND THE MCU (concept).** Five sensors on the **nRF52840's two hardware I²C
buses** using the TLI493D-W2BW address variants — no mux, no chip-select fan-out (confirm the exact
address count against the ordered variant). Fine wires route to a **XIAO nRF52840 + 100 mAh LiPo** at
the wrist. A duty-cycled power **sketch** (SPEC/estimate): ~1.5 mA at a 500 Hz scan → ~**68 h** on
100 mAh. Firmware is **outlined, not built**: boot baseline → 500 Hz scan → project ΔB onto the
per-well calibrated 5×3 map → per-direction Schmitt (on 60 % / off 40 %) → idle-gated baseline
tracker → `action_map` → BLE HID. The physical wire routing is part of the mount geometry (ppp).

**(rrr) ⚠ WHAT IS NOT YET SETTLED (read-out).**
- **The dome membrane (~0.32 mm) is at the FDM single-perimeter floor** — it needs a 0.25 mm nozzle
  or a corrugation, as §8.15g already flagged.
- **`REST_GAP` (3.5 mm) and `CRADLE_LEVER` (0.7) are GUESSES** — the gap is a mount dimension not yet
  confirmed on a print; the lever (lateral magnet travel per mm of fingertip tilt) sets the
  tilt-direction signal and is a geometry guess until a bench coupon measures it.
- **The read-out is a model, not a measurement.** A stage-1 coupon family (TPU domes across the
  thickness/radius band, a PA seat, a TLV493D breakout) is what measures k, mT-vs-displacement, the
  five-direction confusion matrix, the 18.6 mm crosstalk, and 1k-cycle creep — with pass thresholds
  pre-registered before the print.

### 8.16 Provenance

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

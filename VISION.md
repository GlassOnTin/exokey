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

**Only the thumb and the index can drive one at all.** A 2-axis stick needs all four tilts, and
the middle and ring cannot perform `left`/`right` — they could only ever be a *one*-axis mouse.
That falls out of the muscle model; it is not an opinion.

**And the answer inverts convention:**

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
and randomly drawn feasible designs went 0/240 → 1/240. ⚠ Note what this means: a **guess**
(`COMMON_DRIVE = 0.15`) is now *structural*. That makes it more load-bearing, not less.

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

## 6. Model limitations — stated, not hidden

These bound every conclusion above.

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
  independent actuators. Curling the ring alone is *free* in this model and impossible in a
  hand. Mitigated by an externally imposed common-drive constraint on **both** the MCP and
  PIP joints (it originally covered only the PIP, and the optimiser went straight through the
  gap — see §5); not solved. `COMMON_DRIVE = 0.15` is a **guess**, and it is now built into
  the parameterisation, which makes it *more* load-bearing, not less.
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

### 8.15 Provenance

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

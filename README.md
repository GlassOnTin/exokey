# ExoKey

**A wearable Svalboard.** One adjustable finger well per digit, each a 5-direction joystick,
on a body strapped into the palm. Types QWERTY. Designed by optimising the device and the
hand *together* against a musculoskeletal model.

DataHand / Svalboard geometry, made wearable — you can stand up and walk away with it on.

> **Status: research. Nothing has been built.** Everything here is simulation.
>
> The current model says the device is **feasible**, but only after a correction that
> overturned an earlier negative result. We had modelled the finger well as a **pin** — a point
> force at the pad, with the finger's own muscles required to balance the whole joint torque —
> and concluded that an open hand *cannot press*. That is contradicted by everyone who types on
> a flat keyboard. A well is a **cradle**: it supports the distal phalanx along its length, so
> the finger acts as a **strut** (piano technique names this exactly). Fixing it dissolved the
> contradiction. See [VISION.md §5](VISION.md#5-what-we-already-learned-by-failing).
>
> **Stock MyoHand's thumb cannot press at all** (0.0 N, every direction, every posture) — it
> has *no adductor*, and pressing a key is pushing *against* something. We add the thenar group
> (`hand/thenar.py`: ADP, FPB, APB) and the thumb reaches a **66.8 N pinch — inside the
> published human band of 45–70 N**, a number that was *never fitted*. It is now the cheapest
> digit on the hand.

---

## What it does

Given a musculoskeletal hand (MyoSuite **MyoHand**: 23 DOF, 39 Hill-type muscles), it
co-designs a wearable keyboard by multi-objective optimisation:

- **Effort** = muscle activation, Σaᵢ³ (Crowninshield–Brand). A physical quantity, not a
  geometric proxy.
- **Feasibility** = hard constraints (11 of them, including *"the digit must actually be able to perform the action"*). NSGA-II's constrained tournament means the
  optimiser cannot *buy* an unreachable key by paying a penalty.
- **Objectives** = effort per character, device mass, key deflection. They genuinely conflict.
- **Population** = the 5th–95th percentile hand, with the finger wells *adjusting* to fit.

### Some things it found

| | |
|---|---|
| **MyoHand's thumb cannot press a key.** Not "weakly" — **at all** | **0.0 N**, at every posture, in all 600 directions sampled |
| ...because it has **no adductor**. FPL flexes, EPL/EPB extend, APL abducts, OP opposes | pressing a key is pushing *against* something |
| Adding the thenar group ([`hand/thenar.py`](hand/thenar.py)) took **all three** muscles | ADP 45.6%→11.9%, FPB →7.0%, **APB →0.0%** |
| The resulting pinch force **was never fitted** — the moment arms were fitted to anatomy | **66.8 N** vs a published human **45–70 N** |
| `click`/`forward`/`back` work on every finger = 3 rows × 4 fingers | = 15 letters, exactly QWERTY's left half |
| **Space** — 22% of the left hand's keystrokes — was not in the objective at all | it now sits on `thumb/click`, the cheapest slot on the hand |
| Handing the thumb the modifiers is **free to learn** | **1.36×** cheaper (nobody's muscle memory encodes *direction*→*row*) |
| What QWERTY itself costs, vs. a free (exact, Hungarian) assignment | **3.86×** |
| A pointer can only go on the **thumb or index** (only they manage 4 tilts) | and it belongs on the **index**: 1.9× vs the thumb's **6.1×** |
| A well is a **cup**, not a cap: the fingertip *bone* slides in along its own axis | so the wells need a **spread, open** hand — gripping converges the fingertips |
| QWERTY's *top* row is the most-used, not "home" | 30.2% vs 23.0% |
| A palm-strapped body vs an articulated exoskeleton | **3× stiffer, 35% lighter** |

### And some things it got wrong, and had to retract

Two headline numbers that used to be in this table are **withdrawn**. They were artifacts.

> ~~The five finger directions differ in muscle cost by **1.9 million×**~~
> ~~`click` is the cheapest, everywhere, on every finger~~

`solve_activations` never enforced equilibrium: it least-squares-fitted the required joint
torque, settled for the closest **achievable** one, and returned the shortfall as `residual`
— **which no caller ever read**. So an action a digit *cannot perform* produces a small
achievable torque, hence small activations, hence **low effort**. Impossible actions looked
*cheap*, and the optimiser was systematically drawn to them. The cheapest number in the whole
model (middle/`click`, 4e-08) was an **unbalanced press**.

That is [v1's disease](#a-note-on-how-this-was-built) one level deeper: v1 let the optimiser
*buy its way out* of a soft constraint; here the constraint **was never checked at all**.
Equilibrium is now a hard constraint, and an unperformable action is **unavailable**, not
expensive.

Full numbers, gates and caveats: **[VISION.md](VISION.md)**.

---

## Quickstart

```bash
git clone --recurse-submodules https://github.com/GlassOnTin/exokey
cd exokey
python3 -m venv .venv
.venv/bin/pip install mujoco numpy scipy pymoo PyNiteFEA plotly pytest

# the gates. 64 of them. each one caught a real bug.
PYTHONPATH=. OMP_NUM_THREADS=1 .venv/bin/python -m pytest tests/ -q

# the effort landscape over a finger's reachable workspace -> out/stage2_field.html
PYTHONPATH=. .venv/bin/python scripts/stage2_field.py

# the device on the hand, loaded, members coloured by stress -> out/stage3.html
PYTHONPATH=. .venv/bin/python scripts/stage3_view.py

# the optimiser (this is the slow one)
PYTHONPATH=. OMP_NUM_THREADS=1 .venv/bin/python -m opt.run --pop 200 --gen 150
PYTHONPATH=. .venv/bin/python scripts/stage4_view.py --pick knee   # -> out/stage4.html
```

`OMP_NUM_THREADS=1` is **not optional** for the optimiser: N worker processes × N BLAS
threads oversubscribes the machine and the "parallel" run comes out slower than serial.

Every stage writes a self-contained browser view. That is deliberate — two of the worst bugs
in this project were caught by *looking at the render*, and would have survived any table of
numbers.

### Bursting it onto a cloud box

```bash
echo "$HCLOUD_TOKEN" > ~/hetzner.api && chmod 600 ~/hetzner.api
./cloud/hetzner.sh price          # real prices, from the API
./cloud/hetzner.sh up cpx62       # create + install + run the test suite on it
./cloud/hetzner.sh burn --pop 200 --gen 150   # run, fetch results, DELETE the box
```

⚠ **Hetzner bills for a server while it *exists*.** Powering it off changes nothing — only
deleting stops the meter. There is deliberately no `stop` subcommand, because a "stop" that
keeps charging you is a trap. Use `burn`: it tears the box down when the run ends, and its
trap fires on crash and interrupt too, so a failed run cannot leave a machine billing.

Take a **dedicated-vCPU** tier only if you have quota for a big one. Measured on this
workload: a 16-vCPU *shared* box (`cpx62`) beat an 8-vCPU *dedicated* one (`ccx33`) by
**2.3×** for the same money — shared cores deliver ~77% of a dedicated core, but you get
twice as many.

---

## Prior art, and the patent question

**We are not designing around anyone's patent. We are on an older road.**

- The 5-direction finger well is **DataHand**, whose patents (early 1990s) have **expired**.
- [**Svalboard**](https://svalboard.com/) is an open, modern implementation of that idea
  (Vial/QMK), and the direct inspiration here. This project is a *wearable* form of it.
- [**Typeware**](https://typeware.tech/) build a wearable keyboard with a **patented "twin
  key"** — two switches on one stem, to pack more physical keys in. **We need no such thing.**
  The extra states come from *sensing more muscle groups*, not from more hardware.

⚠ None of this is legal advice. A patent is not copyright: writing your own code and CAD does
not help, and an open-source licence grants nothing against one. If you intend to *build and
sell*, read the claims.

---

## Licence

**[AGPL-3.0](LICENSE).** If you run a modified version as a network service, you owe your
users the source.

Third-party components, all permissively licensed and AGPL-compatible:

| | |
|---|---|
| [MyoSuite](https://github.com/MyoHub/myo_sim) `myo_sim` hand model (`data/`, git submodule, version-pinned) | Apache-2.0 |
| [MuJoCo](https://mujoco.org/) | Apache-2.0 |
| [pymoo](https://pymoo.org/) | Apache-2.0 |
| [PyNite](https://github.com/JWock82/PyNite) | MIT |
| NumPy / SciPy / Plotly | BSD / BSD / MIT |

The muscle-effort model, the population scaling and the co-design loop are ours.

---

## A note on how this was built

The commit history is mostly **failures**, and they are the most valuable thing in the repo.
The "finger pad" was the fingernail for a week — every key sat behind the nail, a "keypress"
recruited an *extensor*, and nothing failed, because the physics stayed perfectly
self-consistent while answering the wrong question. The thumb's joint signs are not
self-consistent with each other. And a units mismatch (metres² against a normalised penalty)
faithfully reproduced the exact bug that killed the predecessor project.

Each of those is now a test. See [VISION.md §5](VISION.md#5-what-we-already-learned-by-failing).

# ExoKey

**A wearable Svalboard.** One adjustable finger well per digit — each a five-direction joystick —
carried on a **topology-optimised gauntlet over the back of the hand**, held on by a soft strap.
It types QWERTY, and it is designed by optimising the device and the hand *together* against a
musculoskeletal model. DataHand / Svalboard geometry, made wearable: you can stand up and walk
away with it on.

> **Status — research. Nothing has been built; everything here is simulation.** The model says the
> device is feasible, and its numbers come with their gates and caveats in **[VISION.md](VISION.md)**.
> The disclosure is Bitcoin-anchored — see **[TIMESTAMP.md](TIMESTAMP.md)**.

**See it move → the live gallery: [glassontin.github.io/exokey](https://glassontin.github.io/exokey/out/)**
— the hand posed into every keypress, the gauntlet drawn at the radius the physics chose for each
member, the strap, and the optimiser converging.

---

## Why it looks like this

Nearly every constraint in this project is a fact about **people**, not machines: a key that moves
more than **500 µm** feels mushy; a finger can comfortably press about **0.30 N**; effort is muscle
activation (**Σaᵢ³**); a palm can bear a feature down to about **1.5 mm** before it hurts; the device
must fit the **5th–95th percentile** hand and be light enough to wear all day. Only three constraints
are about the printer — nozzle, overhang, bridging. That balance *is* the thesis, and every large
error in the project has been the model failing to represent the human while wearing a technical
costume.

The result is grown, not drawn — Wolff's law on a free-form lattice — so it comes out looking like an
anatomy rather than a bracket:

| the gauntlet, optimised under every constraint at once | |
|---|---|
| members | **410** (drawn as 1632 curved sub-beams) |
| mass | **7.54 g** (beam model); **12.7 g** printed as hollow tubes |
| section | **hollow tubes**, 0.8 mm wall — two perimeters of a 0.4 mm nozzle |
| worst well displacement | **481 µm** against a 500 µm gate |
| sharpest surface anywhere | **1.50 mm** — no spikes, no loose ends, one piece |
| survives a **50 N knock** | re-sized for impact, not just crispness (§8.15k) |
| support to print it | 1021 pillars, **0 props** |

---

## What the optimisation found

A musculoskeletal hand (MyoSuite **MyoHand**: 23 DOF, 39 Hill-type muscles) drives a multi-objective
search (pymoo **NSGA-II**): **effort** (Σaᵢ³) and **mass** and **key deflection** as objectives that
genuinely conflict, **feasibility** as hard constraints the optimiser cannot buy its way out of. Some
of what fell out:

| | |
|---|---|
| **Stock MyoHand's thumb cannot press a key** — 0.0 N, every posture, every direction | it has **no adductor**; pressing a key is pushing *against* something |
| Adding the thenar group ([`hand/thenar.py`](hand/thenar.py): ADP, FPB, APB) | thumb reaches a **66.8 N pinch — inside the human 45–70 N band, never fitted** |
| **The device is TOUCH-limited, not load-limited** | **100%** of the grown bone's members sit on the 1.5 mm floor — thick because a *hand* must bear them, not a force (§8.15v) |
| ...so the bone is **hollow**, exactly as a real one is | **−39%** mass, identical to the touch — free *because* it is touch-limited (over-stiff), which a load-limited truss is not (§8.15v) |
| The **ergonomic** minimum feature (1.5 mm), not the nozzle's (0.4 mm), makes it trabecular | the fixed prune coarsens **400 → 61 members** as the floor rises, and the rendered bone is **spike-free** (§8.15v) |
| A palm-strapped body beats an articulated exoskeleton — then *it* was dropped too | the load path wants to go over the **back** of the hand, not around it |
| Every well is a five-way joystick, on every finger | the "ulnar can't tilt" limit was a modelling artefact, not muscle |
| A magnet on a printed TPU flexure over a 3-axis Hall **reads a keypress at ~430 LSB** (200× the sensor noise); the five directions sit **≥78° apart**, 0 classifier errors in 10⁵ | the read-out §8.15g deferred, now modelled — the wells become a **watertight, one-piece** print (component count 1) with the sensor seats and wire grooves carved in (**49.9 g** solid, +24.4 g for the sensors; the strut ties in dorsally, opposite the magnet). ⚠ the four long fingers' modules are wider than the finger pitch — they need a shared cluster (§8.15l) |
| A **50 N knock**, not the keypress, is what sizes the structure | and it wants a *broad* skeleton — growing with it in the loop (**23.2 g**) is 39% lighter than bolting it on (**37.7 g**, §8.15k) |
| **Grow it, don't re-prune it.** A top-down re-prune dead-ends in a heavy uniform *membrane* — a local optimum it can't escape (1149 members, 33.9 g hollow) | the *grow* builds a light truss bottom-up instead; rendering that topology holds the bone at **7.5 g** beam / **12.7 g** hollow (§8.15k) |

Full findings, the retracted ones included (a headline "1.9-million×" number that turned out to be an
unbalanced-press artefact, and two design justifications that failed their own measurement), are in
**[VISION.md](VISION.md)** — the failures are the most valuable thing in the repo, and each is now a
regression test.

---

## Quickstart

```bash
git clone --recurse-submodules https://github.com/GlassOnTin/exokey
cd exokey
python3 -m venv .venv
.venv/bin/pip install mujoco numpy scipy pymoo PyNiteFEA plotly pytest

# the gates. 109 of them. each one caught a real bug.
PYTHONPATH=. OMP_NUM_THREADS=1 .venv/bin/python -m pytest -q

# the bone, in the order it is built:
PYTHONPATH=. OMP_NUM_THREADS=1 .venv/bin/python scripts/printable.py   # nozzle floor  -> out/printable.npz
PYTHONPATH=. OMP_NUM_THREADS=1 .venv/bin/python scripts/ergonomic.py   # 1.5 mm floor  -> out/friendly.npz
PYTHONPATH=. OMP_NUM_THREADS=1 .venv/bin/python scripts/bone.py        # curved+hollow -> out/bone.npz
PYTHONPATH=. .venv/bin/python scripts/typing_view.py                   # watch it type -> out/typing.html
PYTHONPATH=. .venv/bin/python scripts/export_stl.py                    # printable solid -> out/gauntlet.stl
```

`OMP_NUM_THREADS=1` is **not optional** for the optimiser: N worker processes × N BLAS threads
oversubscribes the machine, and the "parallel" run comes out slower than serial. Every stage writes a
self-contained browser view on purpose — two of the worst bugs here were caught by *looking at the
render*, and would have survived any table of numbers.

To burst the optimisation onto a cloud box and tear it down when done, see
[`cloud/hetzner.sh`](cloud/hetzner.sh) (`burn` deletes the box on exit, crash, and interrupt — because
a "stop" that keeps billing is a trap).

---

## Prior art, and the patent question

**We are not designing around anyone's patent — we are on an older road.**

- The five-direction finger well is **DataHand**, whose patents (early 1990s) have **expired**.
- [**Svalboard**](https://svalboard.com/) is an open, modern implementation of that idea, and the
  direct inspiration; this is a *wearable* form of it.
- [**Typeware**](https://typeware.tech/) ship a wearable keyboard with a patented "twin key" (two
  switches on one stem). We need no such thing — the extra states come from *sensing more muscle
  groups*, not more hardware.

⚠ Not legal advice. A patent is not copyright: writing your own code and CAD does not help, and an
open-source licence grants nothing against one. If you intend to build and sell, read the claims.

---

## Licence

**[AGPL-3.0](LICENSE).** Run a modified version as a network service and you owe your users the source.

Third-party components, all permissively licensed and AGPL-compatible: **MyoSuite** `myo_sim` hand
model (Apache-2.0, git submodule, version-pinned), **MuJoCo** (Apache-2.0), **pymoo** (Apache-2.0),
**PyNite** (MIT), NumPy / SciPy / Plotly (BSD / BSD / MIT). The muscle-effort model, the population
scaling, and the co-design loop are ours.

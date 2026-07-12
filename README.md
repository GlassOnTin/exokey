# ExoKey

**A wearable Svalboard.** One adjustable finger well per digit, each a 5-direction joystick,
on a body strapped into the palm. Types QWERTY. Designed by optimising the device and the
hand *together* against a musculoskeletal model.

DataHand / Svalboard geometry, made wearable — you can stand up and walk away with it on.

> **Status: research. Nothing has been built.** Everything here is simulation, and its
> limitations are stated plainly in [VISION.md §6](VISION.md#6-model-limitations--stated-not-hidden).
> The single most valuable next step is to *print one and measure it*.

---

## What it does

Given a musculoskeletal hand (MyoSuite **MyoHand**: 23 DOF, 39 Hill-type muscles), it
co-designs a wearable keyboard by multi-objective optimisation:

- **Effort** = muscle activation, Σaᵢ³ (Crowninshield–Brand). A physical quantity, not a
  geometric proxy.
- **Feasibility** = hard constraints (9 of them). NSGA-II's constrained tournament means the
  optimiser cannot *buy* an unreachable key by paying a penalty.
- **Objectives** = effort per character, device mass, key deflection. They genuinely conflict.
- **Population** = the 5th–95th percentile hand, with the finger wells *adjusting* to fit.

### Some things it found

| | |
|---|---|
| The five finger directions differ in muscle cost by | **1.9 million×** |
| `click` (press into the well) is the cheapest | everywhere, on every finger |
| QWERTY's *top* row is the most-used, not "home" | 30.2% vs 23.0% |
| Choosing which direction means which row is worth | **5×** — and it's firmware, so it's free |
| Per-finger adjustment needed to span 5th–95th | **~12 mm**, and essentially **2 axes**, not Svalboard's 5 |
| A palm-strapped body vs an articulated exoskeleton | **3× stiffer, 35% lighter** |

Full numbers, gates and caveats: **[VISION.md](VISION.md)**.

---

## Quickstart

```bash
git clone --recurse-submodules https://github.com/GlassOnTin/exokey
cd exokey
python3 -m venv .venv
.venv/bin/pip install mujoco numpy scipy pymoo PyNiteFEA plotly pytest

# the gates. 57 of them. each one caught a real bug.
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

# Building an ExoKey

> **Status: nothing here has been built.** ExoKey is a research project that designs a
> wearable keyboard by simulation. The printed *structure* is real and printable today; the
> *working device* is not — it has no firmware and no proven electronics yet. This page is the
> honest path from "I own a 3D printer" to "I have the parts in hand," and it is clear about
> where the road currently ends. For the science and the full record, see
> [VISION.md](VISION.md); for the parts, [BOM.md](BOM.md).

## What it is

A keyboard you wear on the back of one hand. Instead of pressing keys on a desk, each
**fingertip sits in a little cup** and can push in one of **five directions** — like a tiny
joystick per finger. Five fingers × five directions is enough to type QWERTY by chords. The
cups are carried on a lightweight printed lattice — the **gauntlet** — that arches over the
back of your hand and is held on by a soft **strap**. You can stand up and walk away wearing
it.

The unusual thing about the design is *how* it was made: the shape was **grown by an
optimiser** that co-simulates the device together with a model of the hand's muscles, so the
structure comes out looking like a bone rather than a bracket — thick where a hand must bear
it, thin everywhere else, hollow inside.

## How it works

- **The key.** Each fingertip cup holds a small **disc magnet** on a springy printed
  [flexure](#glossary). Under the cup sits a fixed **3-axis Hall sensor** that measures the
  magnetic field in x, y, z. Push the fingertip and the magnet moves; the field shifts in a
  direction the firmware can tell apart from the other four. No switches, no contacts — just a
  magnet moving over a sensor.
- **The structure.** The [gauntlet](#glossary) is a lattice of hollow printed tubes. It is
  [touch-limited, not load-limited](#glossary): the tubes are as thick as they are because a
  *hand* has to be able to bear against them without a sharp edge (a 1.5 mm minimum radius),
  not because the forces are large. Being over-stiff for its load is exactly why it can be
  hollow for free.
- **The strap.** A webbing band pulls the gauntlet down onto the hand with about 1 newton.
  It is not optional — without it the frame springs away from the hand many times too far.
- **The read-out (firmware).** Boot, learn the resting field, then scan ~500 times a second,
  compare each sensor's field to a calibrated map of its five directions, debounce, and send
  the resulting keystrokes over Bluetooth. **This firmware is designed on paper but not
  written** — see [What is not done yet](#what-is-not-done-yet).

## Print it

**Easiest path — just print the structure:** download `gauntlet.stl` from the project's
**[GitHub Releases](https://github.com/GlassOnTin/exokey/releases)** and slice it. You do not
need Python, the simulator, or any of this repo to print that file.

**Print settings** (these are baked into the geometry — the part is *designed* to print this
way):

| Setting | Value | Why |
|---|---|---|
| Nozzle | 0.4 mm | The tube walls are two perimeters of a 0.4 mm nozzle. |
| Perimeters / walls | **2** | |
| Infill | **0%** | The tubes are meant to be **hollow** — 2 walls + 0% infill is what makes them so (~40 g instead of solid). |
| Orientation | **standing on the wrist, fingers pointing up** | Chosen by the optimiser to need no prop supports. |
| Supports | **none** | Overhangs stay under 45° and bridges under 10 mm by construction. |
| Material | CF-PA12 (SLS) or a stiff FDM filament | The mass/strength figures assume CF-PA12; any stiff filament prints. |

Print the **five TPU cradles** separately in flexible TPU (they are the moving keys). Their
~0.32 mm spring membrane is at the limit of a 0.4 mm nozzle — a **0.25 mm nozzle** helps.

**Fit it to your hand:** the shipped `gauntlet.stl` is sized to the *median* (185 mm) hand. To
print your size, install the tools (below), download the design files (`final_design.pkl` and
`bone.npz`) from the Release, and run:

```bash
make fit MM=192        # your hand length, wrist crease to middle-fingertip, in millimetres
# -> out/gauntlet_192mm.stl
```

This re-fits the finger cups and the frame to your hand. It is a **first-order fit**: the cups,
sensor seats, and frame all scale together, but the component pockets (Hall sensor, magnet,
XIAO) stay at true size, and the *topology* of the skeleton is still the one optimised across
the whole population — a fully re-optimised skeleton for your exact hand means re-running the
search (`make optimise`, hours). The strap buckle and the wells' adjustment cover the rest.

## Assemble it

Assembly is **modelled geometrically but never performed.** What the model guarantees: every
finger can **slide into** its cup along a clear entry route, and its pad **seats** on the cup
floor (both are gated by tests). What does not exist as instructions: a step-by-step build, an
exploded view, or wiring. In outline: press a magnet into each TPU cradle; drop each cradle
into its well; seat a Hall breakout in the pocket under each well; route the 4-wire bus
(power + I²C) along the dorsal grooves to the XIAO + battery in the wrist housing; thread the
strap through the lugs. **Do not treat this as a tested procedure.**

## Get the tools (only if regenerating or resizing)

```bash
git clone --recurse-submodules https://github.com/GlassOnTin/exokey
cd exokey
make deps          # venv + pinned deps (requirements.txt) + the MyoHand submodule
make test          # the gates -- run the suite; each one caught a real bug (~5 min)
make stl           # regenerate out/gauntlet.stl (median hand) from the shipped design
make fit MM=192    # or your size
```

`make` hides the `PYTHONPATH=. OMP_NUM_THREADS=1` prefixes every script needs. Run `make` with
no target for the list. `make stl`/`fit` need the design files (`final_design.pkl`, `bone.npz`)
— they ship on the Release; a fresh clone does not have them, and producing them from scratch
is the expensive `make optimise` step (the cloud burst in [`cloud/hetzner.sh`](cloud/hetzner.sh)
exists for exactly that).

## What is not done yet

The structure is the finished part of this project. These are **not**:

- **Firmware.** Outlined only (`VISION.md §8.15l qqq`). No code, no BLE HID, no
  direction→QWERTY map. The device cannot type until someone writes it.
- **Electronics / PCB.** No board design. The Hall sensors sit on generic breakouts in carved
  pockets; the "harness" is a wire-routing plan, not a schematic.
- **The read-out is a prediction, not a measurement.** The magnetic figures and the spring
  come from a numpy model; the rest gap (3.5 mm) and the cradle lever (0.7) are guesses. The
  planned next physical step is a **stage-1 coupon** — TPU domes across a thickness/radius
  band, a printed seat, and a TLV493D breakout — measured against thresholds set in advance
  (`VISION.md §8.15l rrr`). Until that coupon exists, treat [BOM.md](BOM.md) as provisional.
- **Per-user fit** is first-order only (see above).

## Glossary

- **Gauntlet** — the printed lattice that arches over the back of the hand and carries the
  finger cups. The "bone."
- **Well / cup** — the cavity a fingertip sits in; its floor is what a downward press pushes
  against.
- **Flexure** — a part that bends *instead of* using a hinge or spring. Here, a thin printed
  TPU dome that returns the fingertip key to rest.
- **Thenar group** — the muscles at the base of the thumb (ADP, FPB, APB). The stock hand
  model lacks the adductor, so its thumb literally cannot press a key; adding this group is
  what makes the thumb work.
- **Effort (Σaᵢ³)** — how the model scores exertion: the sum of each muscle's activation
  cubed (the Crowninshield–Brand criterion). Lower is more comfortable.
- **Touch-limited vs load-limited** — a structure sized by what a *hand can bear against
  without pain* (a 1.5 mm minimum radius) rather than by the *forces* it carries. ExoKey is
  touch-limited, which is why it can be hollow.
- **Wolff's law** — bone grows where it is loaded and thins where it is not; the optimiser
  does the same to the lattice.
- **Entry-first mount** — a cup designed so the finger's slide-in route is kept clear *first*,
  and everything else fits around it.
- **Trabecular** — the spongy, strut-like internal pattern of real bone; the grown lattice
  resembles it.

## Errata (numbers that disagree across the repo)

- **Gate count.** The `README` historically said "109" and `VISION.md` said "57"; the suite
  currently collects **137**. `make test` is the authoritative, live count.
- **Stale file names.** `VISION.md §8.15l` cites `manufacture/wellmod.py`, `scripts/coupon.py`,
  and `tests/test_wellmod.py` — none of which exist. Their logic was consolidated into
  `manufacture/readout.py`, `manufacture/mount.py`, and `manufacture/entry.py` (tested by
  `tests/test_readout.py`, `tests/test_mount.py`, `tests/test_entry.py`). The flexure test
  coupons (`out/coupon_*.stl`) ship on the Release, but their generator script was not
  committed.

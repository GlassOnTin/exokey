# Stage-1 coupon — read-out bench test protocol

> **Why this exists.** Everything about the ExoKey key — that a fingertip press swings the Hall
> field ~200× above its noise, that the five directions are tellable apart, that a neighbour well
> does not false-trigger, that the TPU dome gives a ~20 gf key and survives — is a **prediction from
> a numpy model** (`manufacture/readout.py`, `manufacture/flexure.py`). Nothing has been built. This
> is the first physical test, and it decides whether the read-out is real. Two of the model's inputs
> are outright **guesses** it exists to settle: `REST_GAP` (3.5 mm) and `CRADLE_LEVER` (0.7).
>
> **Pre-registration.** The pass thresholds below are **fixed before any coupon is printed or
> measured**, and they are the model's own design gates (`tests/test_readout.py`) restated as bench
> measurements. Record the date you freeze them; do not edit a threshold after seeing a result. A
> result that misses its threshold is a finding, not a failure to be tuned away — that is the whole
> point of measuring.

Registered: __________ (fill in before the first measurement). Model revision: `git rev-parse HEAD`.

---

## What the model predicts (the numbers under test)

Magnet **Ø3 × 1 mm N42** (Br 1.29 T) · Hall **TLI493D-W2BW class** (0.098 mT/LSB, ±130 mT, ~0.2 mT
RMS noise) · rest gap **3.5 mm** · key travel **1.5 mm** · target spring **130.7 N/m** (196 mN / 1.5 mm).

| quantity | predicted | source |
|---|---|---|
| magnet dipole moment | 7.26 × 10⁻³ A·m² | `readout.moment` |
| rest field (gap 3.5 mm) | ≈ 19 mT | `readout.cyl_axial_B` |
| full-plunge field (gap 2.0 mm) | ≈ 61 mT | `readout.cyl_axial_B` |
| plunge swing ΔBz (rest→full) | **41.9 mT = 427 LSB = 209× noise** | `readout.delta_B('click')` |
| hard-stop field (gap 1.7 mm) | 80 mT = 62 % of range | `readout.cyl_axial_B` |
| lateral tilt signal (each of 4) | 7.1 mT = 36× noise (6.3 mT transverse + −3.3 mT plunge) | `readout.delta_B` |
| min pairwise angle, 5 directions | **78°** | `readout.discriminability` |
| classifier error (10⁵ noisy draws) | **0** | `readout.classify_mc` |
| neighbour crosstalk @ 18.6 mm | static 0.23 mT, modulation 0.055 mT | `readout.crosstalk` |
| scan power / life | 1.48 mA → 68 h on 100 mAh | `readout.scan_power` |
| TPU dome thickness for k=130 N/m | 0.322 mm (a=6 mm), 0.356 mm (a=7 mm) | `flexure.dome` |

---

## Coupons and rig

**Print (TPU membrane sweep — the printability question is real: 0.32 mm is at the FDM single-
perimeter floor):**
- Generate: `PYTHONPATH=. .venv/bin/python scripts/coupon.py` → six watertight
  `out/coupon_t{0.25,0.32,0.40}_a{6,7}.stl` (a rigid rim clamping a flat membrane of thickness `t`,
  radius `a`, with a central boss for the Ø3×1 mm magnet). This is exactly the clamped diaphragm
  `flexure.dome` sizes, so its force/travel is a direct check on k. Exported in **mm**.
- **Design point: `coupon_t0.32_a6.stl`.** Print the whole `t` sweep to find which thickness lays down
  without gaps on your nozzle (a 0.25 mm nozzle helps the thin ones).
- A rigid **seat/spacer** (any PA/PETG part, or an improvised feeler-gauge stack) holds the Hall under
  the coupon at the design 3.5 mm rest gap for T1/T3.
- **Softer flexures, because the flat membrane measured ~230 g at 1.5 mm — far too stiff (it stretches,
  not bends).** Two routes: a shallow **dome** (`scripts/coupon.py` also emits
  `out/coupon_dome_a6_t0.32_h{1,2,3}.stl` — it rolls instead of stretching, and can snap); or, softer
  still, **cast silicone** — print the two-part mold `scripts/dome_mold.scad`
  (→ `dome_mold_cavity.stl` + `dome_mold_core.stl`), pour a two-part silicone, press, cure. Silicone is
  the native keypad-dome material (soft, isotropic, near-immortal in fatigue) and the wall is set by the
  mold gap, not by print calibration — check platinum-cure inhibition against your filament first.

**Buy:**
- 5+ **Ø3 × 1 mm N42** discs (e.g. supermagnete S-03-01-N) — some spares; verify grade.
- 2+ **TLV493D / TLI493D-W2BW breakouts** (one under test, one as the neighbour for crosstalk).
- A microcontroller to read the Hall over I²C and log **raw (Bx, By, Bz) in LSB** at ≥ 500 Hz.

**Rig (the measurements need controlled displacement, not a finger):**
- **A printed field-map fixture** (`scripts/field_fixture.scad` → `fixture_hall_base.stl` +
  `fixture_magnet_sled.stl`): fixes the Hall, slides the magnet over it in X with a **caliper** setting
  the position at a set gap. It decouples the SIGNAL (rigid magnet vs Hall) from the flexure, so T1/T3
  need only your caliper + micrometer, not a load cell. Set the pocket to your breakout and verify the
  gap with a feeler gauge.
- A **micrometer / motorised linear stage**, ≤ 0.05 mm resolution, to set the plunge gap and lateral
  offset precisely (the model's x-axis is displacement — the bench must own it).
- A **force gauge / load cell**, mN resolution, for the spring curve.
- A tilt fixture (set 0–8° about the crown) for the lateral-direction signals.
- Optional: a thermal source (ambient → ~35 °C skin) and a cyclic actuator for fatigue.

---

## Tests — each isolates one claim, each has a frozen threshold

Isolate before you compose: **T1** (magnet+sensor, no dome) and **T2** (dome, no magnet) first, then
**T3+** on the full cradle. Every threshold below is the corresponding `tests/test_readout.py` gate.

### T1 — Field vs gap (magnet + Hall only) → confirms `REST_GAP` and the plunge budget
Mount a magnet on the stage over the Hall. Step the gap from ~4 mm down to the 1.7 mm hard stop,
logging Bz at each step. Fit against `cyl_axial_B`.
- **Pass:** ΔBz(3.5→2.0 mm) **≥ 20 mT AND ≥ 200 LSB AND ≥ 50× noise**; hard-stop field **< 130 mT
  (never clips) and ≤ 0.8× range**; measured B(z) tracks `cyl_axial_B` within **± 15 %**.
- **Also record (confirms the guess):** the field at the as-built rest gap — expect ~19 mT, and it
  must sit low-mid range (10–40 mT) so both rest and hard-stop stay unclipped.
- Gates: `test_the_full_plunge_dwarfs_the_noise_and_the_lsb`, `test_the_hard_stop_does_not_clip_the_sensor`.

### T2 — Spring rate & snap (dome only) → confirms the key feel and the FDM floor
Push each dome's centre with the force gauge to 1.5 mm, logging force vs displacement; note any
buckling snap. Repeat per (t, a).
- **Pass:** actuation force at 1.5 mm travel in **[0.12, 0.30] N** (soft enough to be comfortable,
  firm enough to return); the dome **returns to within 50 µm of rest** on release.
- **Record (not predicted — the model sizes the membrane, not the snap):** snap force/displacement,
  and **which (t, a) printed without gaps** at your nozzle. Printability is a pass/fail of its own.
- Gate: the k = 130 N/m target (`flexure.spring_rate`, `flexure.dome`).

### T3 — Five-direction signal & discriminability (full cradle) → confirms `CRADLE_LEVER`
On the assembled cradle (magnet on dome over Hall, on the PA seat), drive each of the five actions —
click (plunge) and forward/back/left/right (tilt) — to full travel and record (Bx, By, Bz). Build the
5×3 template from the measured means; then run a **live confusion matrix**: ≥ 10⁴ real presses across
the five directions, classified nearest-template.
- **Pass:** weakest direction **|ΔB| ≥ 2 mT AND ≥ 10× noise**; **min pairwise angle ≥ 25°**;
  **0 misclassifications** on the live 10⁴+ presses (predicted 0 at 10⁵).
- **This is where `CRADLE_LEVER` is measured** — the transverse field per mm of tilt travel. If the
  weakest lateral misses 2 mT, the real lever is below the 0.7 guess and the geometry needs the tilt
  amplified (a taller crown / longer lever), which this coupon quantifies.
- Gates: `test_the_weakest_direction_still_clears_the_noise`, `test_the_five_directions_are_mutually_discriminable`.

### T4 — Neighbour crosstalk (two cradles at well spacing)
Place a second magnet/cradle at the finger-well spacing (worst case ~15 mm, nominal 18.6 mm). Press
the neighbour full-travel; measure the field CHANGE it leaks onto this Hall (the static part is
baselined out; the modulation is what a baseline cannot catch).
- **Pass:** modulation **≤ 0.1 mT** (below the 0.2 mT noise); static **≤ 0.3 mT**.
- Gate: `test_neighbour_crosstalk_is_below_the_noise`.

### T5 — Baseline & drift (Earth field + temperature)
With the idle-gated baseline tracker running, rotate the assembly through orientations (Earth ±50 µT)
and warm it from ambient to ~35 °C. Log the resting field and any spurious direction triggers.
- **Pass:** resting-field drift is **fully removed by the baseline tracker** (no false trigger over
  the sweep); the raw orientation swing stays **≤ 10 % of the Schmitt on-threshold**.
- Gates: `test_earth_field_sits_inside_the_hysteresis_band`, `test_misalignment_is_absorbed_by_calibration`.

### T6 — Fatigue / creep (dome, ≥ 1000 cycles)
Cycle a passing dome to full travel ≥ 1000 times; re-run T2 (spring) and T1 (rest field) after.
- **Pass:** post-cycle spring rate within **± 20 %** of initial; rest-field drift **≤ 2 mT (~20 LSB)**
  — comfortably inside the 42 mT plunge margin, so the signal budget still holds.
- (No `test_readout.py` gate yet — this is the creep claim from `VISION.md §8.15l (rrr)` made testable;
  add a regression once measured.)

### T7 — Power (real sensor + MCU)
Measure the actual duty-cycled current of a sensor at 500 Hz scan plus the MCU/BLE average.
- **Pass:** total **< 5 mA**; life on 100 mAh **≥ 12 h**.
- Gate: `test_the_power_budget_lasts_a_working_day`.

---

## Results (fill in — predicted vs threshold vs measured)

| test | predicted | threshold (frozen) | measured | pass? |
|---|---|---|---|---|
| T1 plunge ΔBz | 41.9 mT / 427 LSB | ≥ 20 mT, ≥ 200 LSB, ≥ 50× noise | | |
| T1 rest field | 19 mT | 10–40 mT, unclipped | | |
| T1 hard-stop | 80 mT | < 130 mT, ≤ 0.8× range | | |
| T2 spring @1.5 mm | 0.196 N (k=130 N/m) | 0.12–0.30 N; returns ≤ 50 µm | | |
| T2 printability | dome t=0.32 mm | prints without gaps (some t,a) | | |
| T3 weakest direction | 7.1 mT / 36× noise | ≥ 2 mT, ≥ 10× noise | | |
| T3 min pairwise angle | 78° | ≥ 25° | | |
| T3 confusion matrix | 0 / 10⁵ | 0 / 10⁴⁺ | | |
| T4 crosstalk modulation | 0.055 mT | ≤ 0.1 mT | | |
| T5 orientation/temp drift | Earth ±50 µT | baselined, no false trigger | | |
| T6 post-1k-cycle spring | — | within ± 20 % | | |
| T7 total current / life | 1.48 mA / 68 h | < 5 mA / ≥ 12 h | | |

## What this coupon does NOT settle (state it, don't imply otherwise)

- The **tactile snap** (a shallow dome buckles; `flexure` sizes the linear membrane only) — measured
  as data in T2, but there is no model to pass/fail it against yet.
- The **firmware classifier** end-to-end (Schmitt thresholds, baseline tracker tuning) — T3 tests the
  *separability* of the signals, not a running firmware.
- **Population fit** — this is the median hand's geometry; per-finger lever tuning across the 5th–95th
  percentile is later work.
- The exact **TLI493D-W2BW noise** in its chosen range mode and its **I²C address count** — confirm
  against the ordered part; the 0.2 mT noise here is a datasheet mid-value.

When T1–T7 have measured values, turn each into a regression the way the model's predictions already
are — and, per `VISION.md`, whichever thresholds are missed are the most valuable rows in the table.

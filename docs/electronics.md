# ExoKey electronics — the read-out chain (design, not built)

> **Status.** This is the electronics *design* that the mechanical/magnetic model implies. **No
> firmware and no custom PCB exist yet**, and every magnetic figure is a prediction awaiting the
> stage-1 bench coupon ([COUPON.md](../COUPON.md)). This file writes the "no PCB yet" gap down as a
> schematic-in-words so it's a decision, not a hole. Tags: **DECIDED** vs **OPEN**.

## The chain

```
5× TLV493D  ──(I²C)──►  TCA9548A mux  ──(I²C)──►  XIAO nRF52840  ──(BLE HID)──►  host
 (fingertips)             (wrist)                    (wrist)
```

Five 3-axis Hall sensors, one under each finger well, feed a mux at the wrist, which a XIAO
nRF52840 polls and turns into Bluetooth keystrokes. Power is one small LiPo, charged over the XIAO's
USB-C.

## Per-sensor circuit — about as minimal as a digital sensor gets

The TLV493D/TLI493D is fully integrated (3-axis Hall + ADC + I²C on-die), so each fingertip node is:

- **1× bare Hall chip** (PG-TSOP-6) — `TLV493D-A1B6` for the bench, `TLI493D-W2BW` for the device.
- **1× 100 nF** decoupling cap, VDD–GND, close to the chip.
- **3.3 V** supply — straight off the XIAO's 3V3, no regulator, no level shifter.

That's it. No crystal, no analog front-end. **Pull-ups are per-*bus*, not per-sensor** (one 4.7–10 kΩ
pair at the MCU end, shared) — which is why each fingertip board is just *chip + one 0402 cap* and
fits the 6.4 × 6.4 mm pocket (`manufacture/mount.py:38`). All the shared parts live at the wrist.
**DECIDED.**

## Addressing — the fork that picks the mux

The `TLV493D-A1B6` sets its I²C address from the SDA level at power-up → **only 2 addresses**, and the
nRF52840 has **2 I²C peripherals**, so 2 × 2 = **4 sensors max** on shared buses. Five needs one of:

| path | how | copper | part |
|---|---|---|---|
| **Mux** (bench-driven default) | `TCA9548A` 8-ch I²C switch at the wrist; all five sensors at the *same* address, one channel live at a time | **more** — each channel needs its own SDA/SCL to the fingertip (signals fan out; power still shares a trunk) | common `A1B6` |
| **W2BW + 2 buses** (the repo's minimal-copper plan) | `TLI493D-W2BW` factory-address variants across the two hardware buses, **no mux** (`VISION.md §8.15l qqq`) | **less** — the shared Steiner-tree bus, 4 conductors, 275 mm | production part |

The **mux path** is the near-term choice because it works with the `A1B6` you can buy and reflow now,
and it also isolates lock-ups (below). The **W2BW path** is the minimal-copper graduation. This is a
real trade-off, not a default — **OPEN** which ships. Confirm the W2BW address count at BOM time.

## Lock-up recovery — three layers

The `A1B6` has a known bus-lock/power-up quirk (it can hold SDA low). Defence in depth:

1. **Firmware I²C recovery** — clock out 9 SCL pulses + STOP to release a slave holding SDA. Free.
2. **Mux channel isolation** — deselect a wedged channel so the upstream bus recovers; the four other
   sensors keep working. The `TCA9548A` RESET pin re-inits all channels.
3. **Power-gate** the sensor VDD rail from a GPIO / load switch, to hard power-cycle a channel the
   others can't unstick (costs a global re-baseline, ~a few dropped scans). **OPEN** whether needed.

## Harness

- **34 AWG enamelled copper**, four conductors twisted into a round **~0.6 mm bundle** (chuck the ends
  in a hand drill) — fits the modeled **1.36 mm** dorsal groove (`export_stl.py`, `0.4 + 0.07 ×
  conductors` mm radius). Get **solderable** (self-fluxing) enamel and **four colours** to trace the
  bus. Solid core is fine *because* it's bonded in the rigid grooves (no flex fatigue).
- Off-the-shelf jacketed 4-wire is too fat (the finest bundle sourced was 2.79 mm — it would saw
  through a 3 mm strut), so a self-made fine braid is a structural necessity, not a preference.
- **Internal-through-the-tubes routing** (idea, **OPEN**): the gauntlet tubes are already hollow, so
  short straight-ish strut segments could carry the braid *inside*, with printed **entry/exit ports**
  through the tube wall — external across the nodes (where the internal geometry pinches), internal in
  between. A future export feature.
- **Wire entry** at the wrist: `mount.housing` now carves a slot through the +y wall at the mux bay so
  the harness braid drops into the box. **DECIDED.**

## Power

- **Single-cell LiPo, a 401020 (4.0 × 10 × 20 mm)** — thin, which keeps the wrist box slim (its 4 mm
  thickness sets the box height at ~7 mm vs 9 mm for a 6 mm cell). Rated 200 mAh, but 0.8 cc at real
  LiPo density (~60–180 mAh/cc) is optimistic — treat as **~100–150 mAh**. The scan budget is ~1.5 mA
  (`readout.scan_power`), so figure **1–2+ days per charge** on real BLE-HID use. Charged over the
  XIAO's USB-C (solder to its BAT pads). **DECIDED (401020).**

## Wrist housing

`manufacture/mount.housing` — a box proud of the wrist holding **XIAO nRF52840 + LiPo + the mux
breakout** in a row (now **24 × 39.4 × 7 mm** with the 401020 cell), necked to the nearest live
struts, with the harness wire-slot. The mux (thinner than the LiPo) rides in the dead space.
A production wrist **PCB** would shrink the breakout (bare `TCA9548A` is 7.8 × 4.4 mm) — **OPEN**.

## What is NOT done

- **Firmware** — nothing exists. Needed: TLV493D init + 500 Hz scan → per-well 5×3 calibrated map →
  per-direction Schmitt (on 60 % / off 40 %) → idle-gated baseline tracker → `action_map` → BLE HID
  (`VISION.md §8.15l qqq`). The **bench sketch** (stream raw Bx/By/Bz over USB serial for the field
  map) is the first slice.
- **Custom PCBs** — the fingertip sensor boards (chip + cap, 6.4 mm) and the wrist board (mux + pull-ups
  + battery/charge). Everything above is breakout-and-bare-chip until then.
- **The read-out is still a prediction** — `REST_GAP`, `CRADLE_LEVER`, and the whole signal budget are
  measured by the stage-1 coupon ([COUPON.md](../COUPON.md)), not yet built.

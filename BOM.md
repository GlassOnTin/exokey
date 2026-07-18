# ExoKey — Bill of Materials

> ⚠ **This is a parts list for a device that has never been built.** ExoKey is, today,
> simulation. Every electrical and magnetic figure below is a **model prediction or an
> outright guess** awaiting a first physical bench coupon — they are tagged `SPEC` (a real
> part's datasheet value) or `GUESS` (a number the model chose that a print must confirm).
> Do not order in quantity before the stage-1 coupon (see [BUILD.md](BUILD.md) → "What is not
> done yet"). Sources for every value are in [`design/params.py`](design/params.py) and
> `VISION.md` §8.15l.

The device is one printed gauntlet carrying five finger wells. Each well is a 5-way magnetic
joystick: a disc magnet on a printed TPU cradle moves over a fixed 3-axis Hall sensor, and
firmware reads which of five directions the fingertip pushed.

## Electronics (per device)

| Qty | Part | Spec | Source |
|----:|------|------|--------|
| 5 | **3-axis Hall sensor**, Infineon **TLI493D-W2BW** (TLV493D-family footprint) | 12-bit, 0.098 mT/LSB, ±130 mT, ~0.2 mT RMS noise. One per finger. Split across the MCU's **two hardware I²C buses** using the W2BW address variants — no mux. `SPEC` (reconfirm exact I²C address count against the ordered variant) | `params.py:113` |
| 5 | **NdFeB disc magnet**, Ø3 × 1 mm, grade **N42**, e.g. supermagnete **S-03-01-N** | Br ≈ 1.29 T. Press-fits into the cradle dome (pocket bored Ø − 0.1 mm for interference). `SPEC` | `params.py:99–111` |
| 1 | **Seeed XIAO nRF52840** (BLE) | ~21 × 17.8 × 3.5 mm. Sits in the wrist housing; runs the (unwritten) firmware and the BLE HID keyboard. `SPEC` | `mount.py:238` |
| 1 | **LiPo battery, 100 mAh** | ~20 × 12 × 6 mm. Modelled ~68 h at a 500 Hz scan (~1.5 mA). `SPEC/estimate` | `mount.py:239` |
| — | **4-conductor wire** (VDD, GND, SDA, SCL), ~30 AWG | ~275 mm total on the shipped layout, routed as a shared bus in grooves on the dorsal struts. `SPEC` | `VISION.md §8.15l qqq-2` |

## Printed parts (you make these)

| Part | Material | Notes | Source |
|------|----------|-------|--------|
| **Gauntlet body** (the "bone": frame + 5 sensor mounts + wrist housing + wire grooves) | CF-PA12 (SLS) **or** FDM filament sliced hollow | One watertight solid, ~40 g at the median hand. Print settings in [BUILD.md](BUILD.md). | `out/gauntlet.stl` |
| **5 × cradle + dome flexure** (the moving key; carries the magnet) | **TPU** | The restoring spring, k ≈ 131 N/m (targets the Svalboard 20 gf / 1.5 mm key). Stiff plastics (PLA/PETG/ASA/glass-nylon) were **rejected** — they fatigue-fail. Dome membrane ~0.32 mm is at the FDM single-perimeter floor: needs a **0.25 mm nozzle** or a corrugation. `GUESS` on lever/gap | `flexure.py`, `mount.py:205` |

## Strap / retention

| Qty | Part | Spec | Source |
|----:|------|------|--------|
| ~1 | **Webbing band**, nylon/polyester, ~22 × 1.5 mm | The strap supplies the ~1 N hold-down pull; without it the frame deflects ~18× the gate. Loops watch-style lugs. `SPEC` | `strap.py`, `structure/lattice.py` |
| 2 | **Spring bars / printed pins**, ~2 mm (Ø1.1 mm through-hole) | Captured in the printed lugs; the TPU strap loops them in **shear** (a peeled adhesive bond fails). `GUESS` (no pin sourced) | `strap.py:32` |
| 1 | **Buckle / adjuster** | Covers the ~1.24× wrist-circumference spread across the 5th–95th percentile hand. `SPEC` | `strap.py:18` |

## Consumables

- **PU adhesive** (TPU↔TPU) and, if using glass-nylon anywhere, **vinyl-silane primer**. A
  materials choice, not modelled. `VISION.md §8.15f`
- Filament for supports: the part prints standing on the wrist, fingers up, **0 prop
  supports** (1021 self-supporting pillars fall out of the geometry).

## What you also need (not a part)

- A **3D printer** with a 0.4 mm nozzle (0.25 mm helps for the TPU dome), and TPU capability
  (or a second printer / service for the TPU cradles).
- The **firmware** — **does not exist yet** (outlined in `VISION.md §8.15l qqq`). No PCB
  either: the Hall sensors mount on small breakouts in carved pockets; there is no board
  design in this repo.

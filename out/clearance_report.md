# ExoKey Clearance Sweep Report

Exact-geometry clearance audit (structure/clearance.py) over 5 evenly spaced curl postures, 0.30 to 0.60, on the 50th-percentile hand fixture (manufacture/carrier_fixture.py). Spread and the abduction fan stay at the reference values; curl is the axis that flexes the chains relative to the metacarpal bed.

Gates: skin 1.50 mm, strut-strut 2.00 mm, pod-pod 0.25 mm. Skin measurements are exact surface distances against the per-pose flesh union solid (bonded socket/ball contacts excluded from the free minimum); samples the voxel prefilter cleared are certified >= gate, not measured.

| Curl | Skin samples | Skin violations | Bonded contacts | Skin min (mm) | Strut pairs | Strut min (mm) | Pod min (mm) | Worst pod pair |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| 0.300 | 899 | 0 | 37 | 1.76 | 190 | 7.50 | 3.50 | middle <-> ring |
| 0.375 | 919 | 0 | 43 | 1.76 | 190 | 7.50 | 1.79 | middle <-> ring |
| 0.450 | 938 | 0 | 42 | 1.76 | 190 | 7.50 | 0.48 | middle <-> ring |
| 0.525 | 958 | 0 | 46 | 1.76 | 190 | 7.50 | 0.00 | middle <-> ring |
| 0.600 | 976 | 6 | 56 | 1.33 | 190 | 7.50 | 0.00 | middle <-> ring |

**Sweep totals: 4690 skin samples across 5 postures, 6 violations, 224 bonded contacts.**

Worst free skin clearance across the sweep: **1.33 mm** (gate 1.50 mm; the shortfall is the documented sweep floor SKIN_FLOOR_CURL_MAX, measured on boom_index_link0 at the most-curl posture, see design/params.py).
Worst strut-strut clearance: **7.50 mm** (gate 2.00 mm).
Worst pod-pod clearance: **0.00 mm** (gate 0.25 mm — below gate, the documented stand-in floor, see below).

The pod-pod minimum at the curled end of the sweep is the documented stand-in floor: the keywell units are payload stand-ins for the delivered Svalboard kit (KIT_* constants unmeasured), their envelopes were reshaped to just clear at the reference posture, and no separate cups around adjacent curled fingertips can do better. Measured onset: the units touch from curl 0.525 and interpenetrate at 0.60. The kit session sets the real inter-unit number from the delivered clusters (KIT_PITCH).

<!-- clearance:min_skin_mm=1.33 min_strut_mm=7.50 min_pod_mm=0.00 poses=5 skin_violations=6 -->

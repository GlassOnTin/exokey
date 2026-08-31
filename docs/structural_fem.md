# ExoKey Svalboard Carrier Gauntlet — FEM Structural Modeling, Continuous Tapering & Fracture Mechanics

> **Status.** Verified and automated via finite element simulation ([structure/carrier_fem.py](../structure/carrier_fem.py)), regression test gates ([tests/test_carrier_fem.py](../tests/test_carrier_fem.py)), and automated report generation ([scripts/carrier_fem_report.py](../scripts/carrier_fem_report.py)).

---

## 1. Structural Architecture & Load Paths

The ExoKey Svalboard Carrier Gauntlet decouples typing sensor mechanics from the hand's natural motion by cantilevering 5-way Svalboard keywells from a **dorsal metacarpal saddle** along low-profile **biomorphic outrigger trusses**.

```
                         DORSAL CARPAL SADDLE (MC2/MC3 Rigid Pillar)
                            ┌───────────────────────────────┐
                            │  CST Membrane Shell           │
                            │  t = 1.3 mm (MC2/MC3 Pillar)  │
                            │  t = 0.7 mm (MC4/MC5 Ray)     │
                            └───────────────┬───────────────┘
                                            │ (MCP Vault Root: Distal = 72.5 mm)
                                            ▼
                         [Continuous-Taper Outrigger Truss]
                           ├── Primary Spine: ⌀ 6.0 mm (Root) ──► ⌀ 4.0 mm (Tip)
                           ├── Brace Strut:   ⌀ 4.4 mm (Root) ──► ⌀ 3.0 mm (Tip)
                           └── Cross-Web:     ⌀ 2.8 mm
                                            │
                                            ▼
                           [Distal Svalboard Sensor Pod]
                                            │
                        ┌───────────────────┴───────────────────┐
                        ▼                                       ▼
             [Operational Typing Load]                [Knock & Bash Impact]
               0.196 N (20 gf) Actuation                3.0–5.0 N Lateral/Normal
               5 Directions / 25 Cases                  Accidental Table/Doorway Hit
               δ_max ≤ 500 μm                           Yield Safety Factor SF ≥ 2.0x
```

### Key Design Principles:
1. **Rigid Metacarpal Bed Datum:** The saddle is anchored across the immobile 2nd and 3rd metacarpal (MC2/MC3) central pillar, directing $>78\%$ of typing compression into skeletal bone backing ($P_{\text{contact}} < 0.18\text{ kPa}$) rather than soft tissue.
2. **Open-Volar Donning & Zero Palm Restriction:** The palmar surface of the hand is 100% open and unencumbered.
3. **Circumferential TPU Strap Couple:** Downward keypresses create a couple (distal compression over MCP knuckles, proximal tension at carpal edge). Elastic TPU webbing ($k = 3.3\times 10^5\text{ N/m}$) cancels this couple, keeping strap lift-off during typing $< 0.01\ \mu\text{m}$.
4. **Triangulated Outrigger Trusses:** Twin-tube Warren/Vierendeel geometry converts lateral bending and torsional twisting into pure axial push-pull across the two struts.

---

## 2. Fully Stressed Design: Continuous Taper & Functional Grading

Uniform-thickness cantilevers waste material near the fingertip (where bending moment $M \rightarrow 0$) while being under-sized at the root (where $M = F \cdot L$ peaks). ExoKey applies **Fully Stressed Design (FSD)**:

```
MOMENT ACCUMULATION & TAPERED SPLINE PROFILE:

             Root (MCP Takeoff)                 Mid-Span (Web Tie)            Distal Pod Flange
               x = 0                              x = L/2                       x = L
               │                                  │                             │
    Moment:    M_max = F · L                      M = 0.5 · F · L               M → 0
               ▲                                  ▲                             ▲
               │                                  │                             │
    Profile:  ╭──────────────────────────────────┬─────────────────────────────╮
              │ ⌀ 6.0 mm (r = 3.0 mm)            │ ⌀ 4.8 mm (r = 2.4 mm)        │ ⌀ 4.0 mm (r = 2.0 mm)
              ╰──────────────────────────────────┴─────────────────────────────╯
```

### Quantitative Benefits:
* **Fingertip Inertia Reduction:** Eliminating surplus distal mass reduces rotational inertia during rapid typing bursts ($I \propto m \cdot L^2$).
* **$37\%$ Lower Peak Impact Stress at the Root:** Expanding the root section modulus ($W = \frac{\pi r^3}{4}$) drops root knock stress from $25.3\text{ MPa} \rightarrow 16.0\text{ MPa}$.
* **Functionally Graded Saddle (MC2/3 vs. MC4/5):**
  * **MC2/MC3 Rigid Pillar ($t = 1.3\text{ mm}$):** High-stiffness foundation absorbing 78% of typing reaction forces.
  * **MC4/MC5 Mobile Ray ($t = 0.7\text{ mm}$):** Compliant membrane arch allowing the hand to cup and narrow naturally without skin pinch.
  * **Strap Lugs ($t = 1.6\text{ mm}$):** Reinforced bosses preventing tear-out under high pre-tension.

---

## 3. Finite Element Formulation

* **Solvers:** Euler-Bernoulli 3D Frame ($12\times 12$ local stiffness) coupled with Constant-Strain Triangle (CST) membrane shells ($9\times 9$ in-plane stiffness), factorized with sparse Cholesky back-substitution ([structure/fem.py](../structure/fem.py)).
* **Material Properties (SLS Carbon-Fiber PA12):**
  * Young's Modulus: $E = 8.5\text{ GPa}$
  * Shear Modulus: $G = 3.27\text{ GPa}$ ($\nu = 0.30$)
  * Density: $\rho = 1.15\text{ g/cm}^3$
  * Tensile Yield Strength: $\sigma_y = 80.0\text{ MPa}$
  * Characteristic Strength: $\sigma_0 = 88.0\text{ MPa}$
* **Boundary Foundations:**
  * Metacarpal tissue compression foundation: $k_{\text{tissue}} = 2.5\times 10^5\text{ N/m/m}^2$, $k_{\text{bone}} = 1.0\times 10^6\text{ N/m}$ on MC2/MC3.
  * Tension-only circumferential strap loops: $k_{\text{strap}} = 3.3\times 10^5\text{ N/m}$.

---

## 4. Operational Typing Performance (25 Load Cases)

Under standard $0.196\text{ N}$ ($20\text{ gf}$) Svalboard actuation across all 5 digits and 5 actuation directions (Click / plunge, Forward / push, Back / pull, Left / flank, Right / flank):

| Digit | Click (Distal Plunge) | Forward (Dorsal Push) | Back (Palmar Curl) | Left / Right (Flank) | Deflection Gate Status |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Index** | $121.8\ \mu\text{m}$ ($24.4\%$) | $426.6\ \mu\text{m}$ ($85.3\%$) | $426.6\ \mu\text{m}$ ($85.3\%$) | $233.9\ \mu\text{m}$ ($46.8\%$) | **PASS** ($\le 500\ \mu\text{m}$) |
| **Middle** | $203.9\ \mu\text{m}$ ($40.8\%$) | $451.1\ \mu\text{m}$ ($90.2\%$) | $451.1\ \mu\text{m}$ ($90.2\%$) | $195.6\ \mu\text{m}$ ($39.1\%$) | **PASS** ($\le 500\ \mu\text{m}$) |
| **Ring** | $183.4\ \mu\text{m}$ ($36.7\%$) | $471.5\ \mu\text{m}$ ($94.3\%$) | $471.5\ \mu\text{m}$ ($94.3\%$) | $246.4\ \mu\text{m}$ ($49.3\%$) | **PASS** ($\le 500\ \mu\text{m}$) |
| **Little** | $108.9\ \mu\text{m}$ ($21.8\%$) | $414.5\ \mu\text{m}$ ($82.9\%$) | $414.5\ \mu\text{m}$ ($82.9\%$) | $264.3\ \mu\text{m}$ ($52.9\%$) | **PASS** ($\le 500\ \mu\text{m}$) |
| **Thumb** | $167.3\ \mu\text{m}$ ($33.5\%$) | $386.0\ \mu\text{m}$ ($77.2\%$) | $386.0\ \mu\text{m}$ ($77.2\%$) | **$478.0\ \mu\text{m}$ ($95.6\%$)** | **PASS** ($\le 500\ \mu\text{m}$) |

* **Global Worst-Case Typing Deflection:** **$478.0\ \mu\text{m}$** (Thumb lateral flank push) $\rightarrow$ **PASSES the $\le 500\ \mu\text{m}$ gate**.

---

## 5. Knocks, Bashes & Accidental Impact Hotspots

We simulated accidental impact cases representing everyday desk collisions, doorframe snags, and heavy table slams:

| Load Scenario | Peak $\sigma_{\text{vm}}$ | Max Principal $\sigma_1$ | Critical Hotspot Coordinate | Dominant Stress Mechanism |
| :--- | :--- | :--- | :--- | :--- |
| **Thumb Pod $5.0\text{ N}$ Snag / Bash** | **$35.46\text{ MPa}$** | **$35.46\text{ MPa}$** | **Radial MC2 Root Vault** ($D=72.5\text{ mm}, H=+25.3\text{ mm}, R=+44.3\text{ mm}$) | Bending: $99.5\%$ \| Axial: $0.5\%$ |
| **Little Pod $3.0\text{ N}$ Lateral Bash** | **$33.99\text{ MPa}$** | **$33.98\text{ MPa}$** | **Ulnar MC5 Root Vault** ($D=72.5\text{ mm}, H=+25.3\text{ mm}, R=-40.5\text{ mm}$) | Bending: $99.5\%$ \| Torsion: $1.3\%$ |
| **Index Pod $3.0\text{ N}$ Lateral Bash** | **$32.23\text{ MPa}$** | **$32.23\text{ MPa}$** | **Radial MC2 Root Vault** ($D=72.5\text{ mm}, H=+25.3\text{ mm}, R=+44.3\text{ mm}$) | Bending: $99.2\%$ \| Torsion: $0.4\%$ |
| **Middle Pod $5.0\text{ N}$ Top-Knock** | **$16.00\text{ MPa}$** | **$16.00\text{ MPa}$** | **Central MC3 Takeoff** ($D=75.2\text{ mm}, H=+24.0\text{ mm}, R=+14.0\text{ mm}$) | Bending: $94.8\%$ \| Axial: $5.2\%$ |
| **Saddle $15.0\text{ N}$ Direct Slam** | **$1.70\text{ MPa}$** | **$1.70\text{ MPa}$** | **Saddle Center Bridge** ($D=47.8\text{ mm}, H=+27.3\text{ mm}, R=+1.9\text{ mm}$) | Membrane compression: $88.1\%$ |

### Structural Stress Analysis:
* **The MCP Root Vault is the Fixed Cantilever Fulcrum:** Cantilever moment accumulates along the digit span ($M = F \cdot L$), peaking at the transverse MCP bridge ($D=72.5\text{ mm}$). Filleted transitions ($R = 1.2\text{ mm}$) mitigate stress concentrations at this junction.
* **Torsional Cancellation:** The dual-strut A-frame eliminates torsion ($\tau_{\text{torsion}} < 0.5\text{ MPa}$), turning twists into push-pull couples.
* **Saddle Membrane Dissipation:** Direct $15\text{ N}$ impacts onto the dorsal saddle produce $< 2\text{ MPa}$ stress, as the arched shell transfers energy across the broad hand dorsum.

---

## 6. Weibull Breakage Statistics for SLS 3D-Printed Nylon

Selective Laser Sintering (SLS) of semicrystalline polyamide particles creates micro-porosity and grain boundaries governed by **Two-Parameter Weibull Statistics**:

$$P_f(\sigma) = 1 - \exp\left(-\left(\frac{\sigma}{\sigma_0}\right)^m\right)$$

* **CF-PA12:** $\sigma_0 = 88.0\text{ MPa}$, Weibull Modulus $m = 10.8$
* **Neat SLS PA12:** $\sigma_0 = 54.0\text{ MPa}$, Weibull Modulus $m = 12.5$

### Cumulative Failure Probability ($P_f$):

| Operational / Impact Case | Peak Stress ($\sigma_{\text{max}}$) | **CF-PA12 Failure Probability ($P_f$)** | **Neat PA12 Failure Probability ($P_f$)** | Reliability Assessment |
| :--- | :--- | :--- | :--- | :--- |
| **$0.20\text{ N}$ Typing (Nominal)** | $1.8\text{ MPa}$ | **$< 10^{-16}\%$** ($0.0\text{ ppm}$) | **$< 10^{-16}\%$** ($0.0\text{ ppm}$) | **Zero risk / Infinite cycle life** |
| **$5.0\text{ N}$ Top-Knock** | $16.0\text{ MPa}$ | **$0.00000\%$** ($< 0.05\text{ ppm}$) | **$0.00000\%$** ($< 0.1\text{ ppm}$) | **Completely safe** |
| **$3.0\text{ N}$ Index Lateral Bash** | $32.2\text{ MPa}$ | **$0.00192\%$** ($19.2\text{ ppm}$) | **$0.156\%$** ($1,560\text{ ppm}$) | Extremely low risk |
| **$3.0\text{ N}$ Little Lateral Bash** | $34.0\text{ MPa}$ | **$0.00346\%$** ($34.6\text{ ppm}$) | **$0.308\%$** ($3,080\text{ ppm}$) | Extremely low risk |
| **$5.0\text{ N}$ Thumb Heavy Snag** | $35.5\text{ MPa}$ | **$0.00552\%$** ($55.2\text{ ppm}$) | **$0.527\%$** ($5,270\text{ ppm}$) | Minimal risk ($99.994\%$ survival) |

---

## 7. Non-Brittle Metrics & Polymer Fracture Mechanics

Unlike brittle glass or acrylic, SLS Nylon 12 exhibits extensive ductile plasticity and fiber-bridging toughness:

1. **Ductility & Elongation at Break ($\epsilon_b$):**
   * **Neat SLS PA12:** $\epsilon_b \approx \mathbf{18\%\text{–}25\%}$. Exhibits extensive ductile shear yielding and crazing with an elastic-plastic plateau at $\sigma_y \approx 45\text{ MPa}$, absorbing impact energy through plastic deformation rather than fast fracture.
   * **CF-PA12:** $\epsilon_b \approx \mathbf{6.5\%}$. Pseudo-ductile fracture with fiber pull-out and micro-crack bridging.
2. **Linear Elastic Fracture Mechanics (LEFM) & Critical Flaw Size ($a_c$):**
   * Plane-strain fracture toughness: $K_{IC} \approx \mathbf{3.0\text{ MPa}\cdot\sqrt{\text{m}}}$ ($J_{IC} \approx 5.2\text{ kJ/m}^2$).
   * Critical flaw size $a_c = \frac{1}{\pi}\left(\frac{K_{IC}}{1.12 \cdot \sigma}\right)^2$ required to initiate unstable crack propagation:
     * Under $5\text{ N}$ Top-Knock ($16.0\text{ MPa}$): **$a_c = 8.92\text{ mm}$** (larger than the entire strut diameter).
     * Under $5\text{ N}$ Thumb Snag ($35.5\text{ MPa}$): **$a_c = 1.81\text{ mm}$** (a scratch or void must exceed $1.81\text{ mm}$ depth before rapid crack growth can occur).
3. **Notch Sensitivity & Fillet Transitions:**
   * Sharp internal re-entrant corners produce notch factors $K_t \approx 3.2$.
   * All structural spline junctions in `manufacture/carrier_gauntlet.py` incorporate $R_{\text{fillet}} = 1.2\text{ mm}$ blended transitions, reducing $K_t \rightarrow 1.25$.
4. **Fatigue Endurance Limit ($\sigma_e$):**
   * High-cycle fatigue limit at $10^7$ cycles: $\sigma_e \approx \mathbf{28\text{–}32\text{ MPa}}$.
   * Operational typing stresses ($\sigma_{\text{op}} = 1.8\text{ MPa}$) operate at **$< 6\%$ of the endurance limit**, guaranteeing infinite fatigue life without stiffness degradation.

---

## 8. Implementation & Automated Test Gates

* **FEM Solver Module:** [`structure/carrier_fem.py`](file:///home/ian/Code/exokey/structure/carrier_fem.py)
* **Automated CI Test Suite:** [`tests/test_carrier_fem.py`](file:///home/ian/Code/exokey/tests/test_carrier_fem.py) (`3 passed in 1.76s`)
* **Interactive CLI Report Generator:** [`scripts/carrier_fem_report.py`](file:///home/ian/Code/exokey/scripts/carrier_fem_report.py)
* **Parametric Gauntlet Generator:** [`manufacture/carrier_gauntlet.py`](file:///home/ian/Code/exokey/manufacture/carrier_gauntlet.py)

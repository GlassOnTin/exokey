# ExoKey — Architectural Vision & Technical Disclosure

**A wearable, open-source, musculoskeletal keyboard carrier.** One adjustable five-direction keywell per digit, carried on a lightweight **high-modulus carbon-fiber central dorsal spine and branching transverse wishbone tree**, anchored over the dorsal metacarpal skeleton by a soft tension strap.

It brings DataHand and [Svalboard](https://svalboard.com/) ergonomic efficiency into a walk-away wearable form factor: you can stand up, move freely, and type with relaxed, natural hand postures.

---

## 1. Executive Summary & Core Thesis

Desktop keyboards lock the typist to a desk and force forearm pronation and wrist extension. Traditional data gloves or thumb-cluster pads fail because they lack crisp mechanical key reaction, allowing the key to yield under the finger.

**ExoKey solves wearable typing as a coupled musculoskeletal-structural problem:**
1. **Zero Palm Obstruction:** The entire palm and palmar finger surfaces remain open. Load reaction is carried through a high-rigidity dorsal skeleton anchored to the dorsal metacarpus.
2. **Sub-200 µm Structural Rigidity:** Aerospace-grade carbon-fiber tubes ($\varnothing 5.0\text{–}8.0\text{ mm}$, $E = 180\text{ GPa}$) ensure that typing loads ($0.196\text{ N} \approx 20\text{ gf}$) produce fingertip deflections under $165\ \mu\text{m}$, eliminating mushiness.
3. **Bio-Mechanical Optimization:** Keywell placement and actuation directions are co-designed with a 23-DOF, 39-muscle musculoskeletal hand model ([MyoSuite MyoHand](https://github.com/facebookresearch/myosuite)), minimizing muscle activation effort ($\Sigma a_i^3$).
4. **Full Conformal Adjustability:** Multi-link phalanx booms with locking spherical ball-collets adapt to the 5th–95th percentile hand geometry and diverse resting finger curvatures.

---

## 2. Enabling Architectural Disclosure (Defensive Prior Art)

*This section provides an enabling technical disclosure of the structural, mechanical, and kinematic inventions of ExoKey to establish definitive public prior art under 35 U.S.C. § 102 and international patent treaties (EPC Article 54, PCT Article 33).*

```
                  [ DORSAL METACARPAL SADDLE ANCHOR ]
                                 │
                                 │  ◄── PRIMARY CENTRAL SPINE (⌀ 8.0 mm CF Tube)
                                 ▼
                        [ MIDDLE KNUCKLE ] (MCP3 4-Way Manifold Hub)
                         /              \
      TRANSVERSE ARCH   /                \  TRANSVERSE ARCH
      (⌀ 6.0 mm CF)    ▼                  ▼ (⌀ 6.0 mm CF)
                [ RING KNUCKLE ]   [ INDEX KNUCKLE ] (MCP2 Hub)
                       │                  │
      TRANSVERSE ARCH  │                  │  ◄── DIRECT 1ST WEBSPACE ARCH (⌀ 6.0 mm CF)
      (⌀ 6.0 mm CF)    ▼                  ▼
                [ LITTLE KNUCKLE ] [ THUMB KNUCKLE ] (MCP1 / TMC Hub)
                       │                  │
                       ▼                  ▼
                 [LITTLE POD]        [THUMB POD]
```

### 2.1 Central Dorsal Backbone & Metacarpal Saddle Anchor
* **Metacarpal Saddle Base Hub:** Positioned over the proximal dorsal base of the 3rd metacarpal ($D \approx 25\text{ mm}$ distal to the wrist crease). It interfaces with a conformal dorsal bearing saddle secured by a soft TPU tension strap encircling the palm.
* **Primary Central Spine:** A single pultruded carbon-fiber tube ($\varnothing 8.0\text{ mm}$ OD / $\varnothing 6.0\text{ mm}$ ID, $E = 180\text{ GPa}$) runs along the neutral dorsal midline of the 3rd metacarpal, transferring multi-digit bending and torsional moments directly to the saddle anchor.

### 2.2 Continuous 5-Knuckle Transverse Arch Bridge
* **Wishbone Manifold:** Originating at a 4-way central manifold cross-fitting at the Middle Knuckle (MCP3), a high-modulus transverse carbon-fiber tube ($\varnothing 6.0\text{ mm}$ OD / $\varnothing 4.4\text{ mm}$ ID) spans laterally across the metacarpophalangeal line: `Little (MCP5) <-> Ring (MCP4) <-> Middle (MCP3) <-> Index (MCP2)`.
* **Daylight Skin Clearance:** The transverse arch maintains a positive clearance of $+4.0\text{ to }+4.6\text{ mm}$ above the dorsal skin and knuckle apices, preventing chafing and accommodating extensor tendon glide during finger articulation.

### 2.3 Direct 1st-Webspace Index-to-Thumb Outrigger Truss
* **Structural Integration:** The thumb outrigger connects directly to the **Index Knuckle Joint (MCP2)** via an arched carbon-fiber bridge ($\varnothing 6.0\text{ mm}$ OD) spanning across the 1st dorsal interosseous webspace into the Thumb Carpometacarpal/MCP hub (MCP1/TMC).
* **Deflection Suppression:** By anchoring to the closed transverse arch at the index knuckle rather than a long cantilever from the wrist, the thumb cantilever span is reduced by $50\%$, slashing thumb typing deflection from $489\ \mu\text{m} \rightarrow 148\ \mu\text{m}$ ($> 3.3\times$ stiffness increase).

### 2.4 Multi-Link Phalanx Booms with Locking Ball-Collet Joints
* **Phalangeal Kinematic Tracking:** Each digit ray branches from its MCP knuckle collar into a 3-link conformal carbon-fiber boom ($\varnothing 5.0\text{ mm}$ OD / $\varnothing 3.4\text{ mm}$ ID) tracking the individual phalanx segments (`MCP -> PIP -> DIP -> Keywell Pod`).
* **Modular Ball-Collet Mechanism:** Joints feature precision spherical collets (M2.5 clamp screw, $\pm 25^\circ$ angular cone, $360^\circ$ axial roll). Each link can be independently adjusted and locked to match personal finger length, joint curvature, and resting claw posture.
* **Non-Penetrating Clearance:** Booms hover $+1.1\text{ to }+1.8\text{ mm}$ above the dorsal flesh of flexed phalanges with zero skin penetration across full typing strokes.

### 2.5 Suspended 5-Way Directional Keywell Pod Carriers
* **Payload Interface:** Terminal boom links clamp into modular receiver cradles holding Svalboard 5-way directional switch clusters beneath each fingertip pad.
* **Actuation Ergonomics:** Supports 5 discrete orthogonal and axial inputs per digit (Click, Forward, Back, Left, Right) operating at $20\text{ gf}$ ($0.196\text{ N}$) threshold force.

---

## 3. Musculoskeletal Co-Design Formulation

Keywell geometry and typing directions are optimized directly against human physiology using MyoSuite's 23-DOF, 39-muscle Hill-type model.

### 3.1 Muscle Activation Cost (Crowninshield–Brand Metric)
Typing effort is formulated as the sum of cubed muscle activations across all 39 muscles:
$$\text{Effort} = \sum_{i=1}^{39} a_i^3, \quad a_i \in [0, 1]$$
Subject to static equilibrium under fingertip key reaction $F_{\text{key}} = 0.196\text{ N}$:
$$R(q)^T F_{\text{tendon}}(a) + J(q)^T F_{\text{key}} = 0$$

### 3.2 Thenar Intrinsic Group Restoration
Standard computational models (like stock MyoHand) omit thenar intrinsics, rendering the simulated thumb incapable of opposition. ExoKey integrates the complete thenar group ([`hand/thenar.py`](hand/thenar.py)):
* **Adductor Pollicis (ADP)** — Transverse and oblique heads
* **Flexor Pollicis Brevis (FPB)** — Deep and superficial heads
* **Abductor Pollicis Brevis (APB)**
* Result: Verified human-accurate pinch force of **$66.8\text{ N}$** (matching the physiological 45–70 N band).

### 3.3 Directional Performability & Muscle Recruitment
The musculoskeletal solver proves that directional capability is asymmetric across digits:

| Digit | Click (Flexion) | Forward (Extension) | Back (Curling) | Lateral In (Adduction) | Lateral Out (Abduction) |
| :--- | :---: | :---: | :---: | :---: | :---: |
| **Thumb** | $6.7 \times 10^{-8}$ (Optimal) | $7.2 \times 10^{-7}$ | $1.3 \times 10^{-7}$ | $1.4 \times 10^{-4}$ | $3.7 \times 10^{-7}$ |
| **Index** | $1.1 \times 10^{-7}$ (Optimal) | $2.5 \times 10^{-6}$ | $2.0 \times 10^{-6}$ | $1.6 \times 10^{-5}$ | $1.2 \times 10^{-3}$ |
| **Middle** | $1.2 \times 10^{-8}$ (Optimal) | $3.9 \times 10^{-6}$ | $4.8 \times 10^{-6}$ | High residual | High residual |
| **Ring** | $3.8 \times 10^{-7}$ (Optimal) | $4.6 \times 10^{-5}$ | $1.8 \times 10^{-5}$ | High residual | High residual |
| **Little** | $4.5 \times 10^{-5}$ | $3.7 \times 10^{-6}$ | $1.9 \times 10^{-6}$ (Optimal) | High residual | $3.7 \times 10^{-4}$ |

* **Click, Forward, Back** are effortless across all 4 long fingers ($4 \times 3 = 12$ inputs).
* **Thumb executes all 5 directions effortlessly** ($5$ inputs).
* Total effortless inputs = **17 directions** (fully covering 15 QWERTY half-hand characters + Space + Shift + Modifiers).

---

## 4. 3D Space-Frame Finite Element Method (FEM) Verification

ExoKey implements a full 3D Timoshenko / Euler-Bernoulli space-frame FEM solver ([`structure/fem.py`](structure/fem.py)) with **6 DOFs per node** ($u_x, u_y, u_z, \theta_x, \theta_y, \theta_z$).

### 4.1 Global Element Formulation
For each element of length $L$, axial area $A$, shear modulus $G$, torsion constant $J$, and principal moments $I_y, I_z$:
* Coupled 3D axial stiffness ($EA/L$)
* Torsional stiffness ($GJ/L$)
* Biaxial bending stiffnesses ($12EI/L^3, 6EI/L^2, 4EI/L$)
* Coordinate transformation $T_{12 \times 12} = \text{diag}(R, R, R, R)$ via 3D direction cosine rotation matrices.

### 4.2 Structural Load Case Analysis

```
==================== 3D SPACE-FRAME FEM REPORT ====================
MATERIAL: High-Modulus Carbon Fiber (E = 180 GPa, G = 6.0 GPa, σ_ult = 1200 MPa)

1. INDIVIDUAL 20 gf TYPING LOAD (Fz = 0.196 N per fingertip):
   • Index  : Deflection = 114.57 μm | Peak Stress = 1.87 MPa (SF = 643x)
   • Middle : Deflection = 163.21 μm | Peak Stress = 1.93 MPa (SF = 622x)
   • Ring   : Deflection = 134.99 μm | Peak Stress = 1.79 MPa (SF = 670x)
   • Little : Deflection =  99.11 μm | Peak Stress = 1.72 MPa (SF = 697x)
   • Thumb  : Deflection = 147.95 μm | Peak Stress = 1.43 MPa (SF = 837x)

2. SIMULTANEOUS 5-FINGER CHORD TYPING (1.0 N Total Downward Load):
   • Maximum System Deflection  = 325.53 μm
   • Peak von Mises Stress       = 2.63 MPa (Safety Factor = 456x)

3. ACCIDENTAL IMPACT / SNAG LOAD (2.0 N on Little Finger):
   • Maximum Deflection          = 1.01 mm
   • Peak von Mises Stress       = 17.57 MPa (Safety Factor = 68x)
===================================================================
```

---

## 5. Collision & Skin Clearance Verification

Collision avoidance is strictly enforced through a continuous **3D Signed-Distance Mesh Engine** ([`structure/collision.py`](structure/collision.py)):
* **Skin Clearance:** All 12 structural tube segments maintain a minimum signed distance $\ge +1.13\text{ mm}$ relative to the high-resolution flesh mesh.
* **Transverse Arch Clearance:** $+4.38\text{ to }+4.54\text{ mm}$ over the knuckle line.
* **Inter-Pod Clearance:** Minimum clearance between adjacent Svalboard keywells is **$+3.13\text{ mm}$** across all 10 pairwise digit combinations.

---

## 6. Defensive Publication & Patent Preemption Declaration

**Notice of Dedication to the Public Domain & Establishment of Prior Art:**

The authors of ExoKey hereby disclose and dedicate all structural concepts, kinematic arrangements, mechanical linkages, fabrication methods, and algorithmic optimization pipelines described herein to the public domain under **AGPL-3.0** (code) and **CERN-OHL-S v2 / CC-BY-SA 4.0** (hardware).

This public disclosure serves to invalidate and prevent any subsequent patent claims by third parties under 35 U.S.C. § 102 (novelty / prior art), EPC Article 54, and PCT Article 33. The dedicated inventions include:
1. A wearable keyboard carrier comprising a **central dorsal spine** mounted to a metacarpal saddle and branching at the knuckles.
2. A **transverse knuckle wishbone arch** bridging finger rays over the dorsal MCP joints.
3. A **direct 1st-webspace outrigger truss** connecting a thumb keywell directly to an index knuckle hub.
4. **Conformal phalanx-following boom links** interconnected by 3D multi-axis locking ball-collet joints.
5. Co-design optimization algorithms combining **Hill-type musculoskeletal muscle activation metrics** with **3D space-frame finite element deflection constraints**.

---

## 7. Validated Engineering Evolution

Key architectural milestones proven through regression testing:
* **Palm vs. Dorsal Backbone:** Palm-mounted brackets restricted hand closing and curled finger clearance; the central dorsal spine provides uninhibited finger flex and superior load distribution.
* **Single Spine vs. Parallel Cantilevers:** 5 parallel booms originating from the wrist suffered severe lateral compliance ($> 2.5\text{ mm}$ snag deflection); the central spine + transverse wishbone tree achieved $> 5\times$ higher torsional rigidity at lower mass.
* **Modular Ball-Collets vs. Monolithic Prints:** Monolithic 3D prints required hours of re-printing for a 1 mm hand length change; modular ball-collet carbon tubes provide continuous $\pm 15\text{ mm}$ reach adjustment and $\pm 25^\circ$ angle adaptation in seconds.

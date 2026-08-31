# ExoKey

**A wearable Svalboard.** One adjustable 5-way finger well per digit — each sensing five discrete directions — carried on an ultra-lightweight **carbon-fiber central dorsal spine and branching wishbone tree**, secured over the back of the hand by a soft tension strap.

It brings DataHand / Svalboard ergonomic efficiency into a walk-away wearable form factor: you can stand up, move around, and type with relaxed, natural hand postures.

---

## Interactive 3D Model & Visualizers

* **Live 3D Web Viewer:** [glassontin.github.io/exokey](https://glassontin.github.io/exokey/out/)
* **Local Interactive Assembly:** Run `PYTHONPATH=. .venv/bin/python scripts/branching_spine_view.py` and open `out/branching_spine_view.html`.
* **Documentation & Technical Guides:**
  * **[VISION.md](VISION.md)** — Architectural thesis, enabling prior art disclosures, and musculoskeletal mechanics.
  * **[docs/structural_fem.md](docs/structural_fem.md)** — Full 3D space-frame FEM formulation, load cases, and stress margins.
  * **[BUILD.md](BUILD.md)** — Fabrication, print settings, and assembly guide.
  * **[BOM.md](BOM.md)** — Bill of materials and off-the-shelf hardware list.

---

## Core Mechanical Architecture

```
                  [ DORSAL METACARPAL SADDLE ANCHOR ]
                                 │
                                 │  ◄── PRIMARY CENTRAL SPINE (⌀ 8.0 mm High-Modulus CF Tube)
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

1. **Metacarpal Saddle Anchor Base:**  
   Clamps to the dorsal skeleton over the base of the 3rd metacarpal ($D = 25\text{ mm}$), securing the load path directly into the hand's rigid dorsal bridge with zero palm obstruction.
2. **Primary Central Spine:**  
   A high-modulus pultruded carbon fiber tube ($\varnothing 8.0\text{ mm}$ OD / $\varnothing 6.0\text{ mm}$ ID, $E = 180\text{ GPa}$) running along the 3rd metacarpal midline to a 4-way CNC titanium/aluminum manifold hub at the middle knuckle.
3. **Transverse Dorsal Knuckle Arch:**  
   A continuous cross-knuckle wishbone ($\varnothing 6.0\text{ mm}$ OD CF tube) spanning across `Little (MCP5) <-> Ring (MCP4) <-> Middle (MCP3) <-> Index (MCP2)` with positive daylight clearance ($> 4.0\text{ mm}$) above the skin.
4. **Direct 1st Webspace Thumb Bridge:**  
   Connects the Thumb Knuckle (MCP1/TMC) directly to the **Index Knuckle (MCP2)** across the 1st webspace, closing the structural truss and dropping thumb typing deflection by over $4\times$.
5. **Conformal 3-Link Phalanx Booms:**  
   Straight carbon fiber links ($\varnothing 5.0\text{ mm}$ OD) track the dorsal contour of each phalanx chain (`MCP -> PIP -> DIP -> Pod`) using M2.5 locking ball-collet hubs for multi-axis personal ergonomic adjustment.
6. **Svalboard 5-Way Keywells:**  
   Suspends five modular Svalboard directional switch units beneath the fingertips, capturing click, forward, back, left, and right motions with ultra-low operating force ($20\text{ gf} \approx 0.196\text{ N}$).

---

## 3D Space-Frame FEM Performance

The structure is verified using a full **3D Space-Frame Finite Element Solver** ([`structure/fem.py`](structure/fem.py)) with 6 DOFs per node ($u_x, u_y, u_z, \theta_x, \theta_y, \theta_z$):

| Digit Ray | Structural Load Path | 3D FEM Tip Deflection ($F_z = 0.20\text{ N}$) | Peak von Mises Stress | Ultimate Safety Factor |
| :--- | :--- | :---: | :---: | :---: |
| **Index** | Middle Knuckle $\rightarrow$ Arch $\rightarrow$ Index Knuckle $\rightarrow$ Booms | **$114.6\ \mu\text{m}$** | $1.87\text{ MPa}$ | **$643\times$** |
| **Middle** | Central Spine $\rightarrow$ Middle Knuckle $\rightarrow$ Booms | **$163.2\ \mu\text{m}$** | $1.93\text{ MPa}$ | **$622\times$** |
| **Ring** | Middle Knuckle $\rightarrow$ Arch $\rightarrow$ Ring Knuckle $\rightarrow$ Booms | **$135.0\ \mu\text{m}$** | $1.79\text{ MPa}$ | **$670\times$** |
| **Little** | Ring Knuckle $\rightarrow$ Arch $\rightarrow$ Little Knuckle $\rightarrow$ Booms | **$99.1\ \mu\text{m}$** | $1.72\text{ MPa}$ | **$697\times$** |
| **Thumb** | Index Knuckle $\rightarrow$ Webspace Bridge $\rightarrow$ Thumb Knuckle $\rightarrow$ Booms | **$147.9\ \mu\text{m}$** | $1.43\text{ MPa}$ | **$837\times$** |

* **Deflection Gate ($\le 200\ \mu\text{m}$):** **PASSED** (Crisp, zero-mush keyfeel)
* **Simultaneous 5-Finger Chord Typing ($1.0\text{ N}$):** Peak stress **$2.63\text{ MPa}$** (SF = $456\times$)
* **Accidental Snag / Impact Load ($2.0\text{ N}$):** Peak stress **$17.57\text{ MPa}$** (SF = $68\times$)
* **Skin Collision Clearance:** Continuous signed-distance audit confirms daylight **$\ge 1.13\text{ mm}$** everywhere (zero penetration).
* **Inter-Pod Clearance:** **$\ge 3.13\text{ mm}$** between all 10 pairwise combinations of keywell pods.

---

## Quickstart

```bash
git clone --recurse-submodules https://github.com/GlassOnTin/exokey
cd exokey
make deps          # venv + pinned dependencies
make test          # regression test suite (FEM, ball-collets, collision gates)
make view          # build and open 3D interactive HTML model
```

---

## Defensive Prior Art & Patent Preemption Declaration

**All design principles, mechanical linkages, structural load paths, and co-design algorithms documented in this repository are hereby published openly to the public domain.**

This publication constitutes prior art under 35 U.S.C. § 102 and international patent treaties (including the European Patent Convention Article 54 and PCT regulations). Specifically dedicated to the public domain to prevent patenting by third parties:
1. The **central dorsal spine backbone** anchored to a metacarpal saddle base and transmitting multi-digit typing reaction loads.
2. The **continuous transverse knuckle arch bridge** spanning across the metacarpophalangeal joints with positive skin standoff.
3. The **direct 1st-webspace index-to-thumb outrigger bridge** anchoring the thumb structure to the index knuckle.
4. The **conformal multi-link phalanx boom chain** with spherical ball-collet adjustment joints tracking finger ray ergonomics.
5. The **co-design computational optimization pipeline** coupling Hill-type musculoskeletal effort with 3D space-frame FEM structural gates.

---

## License

**[AGPL-3.0](LICENSE)** for software and firmware; **CERN-OHL-S v2** / Creative Commons **CC-BY-SA 4.0** for CAD, mechanics, and hardware documentation.

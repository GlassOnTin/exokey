# ExoKey — Bill of Materials (BOM)

This Bill of Materials specifies all hardware, raw stock, fasteners, and off-the-shelf components required to build one complete **ExoKey Wearable Musculoskeletal Keyboard Carrier**.

---

## 1. Modular Carbon Fiber Space-Frame & Joint Hardware

| Qty | Part Description | Specification | Recommended Source / Part No. |
| :---: | :--- | :--- | :--- |
| **2 packs** (20 pcs) | **Hardened Steel Ballstuds (⌀ 4.8 mm)** | ⌀ 4.8 mm ball, M3 × 5.0 mm threaded shank, black oxide steel | **Yeah Racing YA-0562** (Modelsport UK / 1/10 RC) |
| **1 pc** (~0.5 m) | **Primary Central Spine Tube** | ⌀ 8.0 mm OD × ⌀ 6.0 mm ID pultruded high-modulus carbon fiber tube | Carbon fiber tube stock ($E \ge 180\text{ GPa}$) |
| **1 pc** (~0.5 m) | **Transverse Knuckle Arch & Thumb Tube** | ⌀ 6.0 mm OD × ⌀ 4.4 mm ID pultruded carbon fiber tube | Carbon fiber tube stock ($E \ge 180\text{ GPa}$) |
| **1 pc** (~1.0 m) | **Phalanx Outrigger Boom Tubes** | ⌀ 5.0 mm OD × ⌀ 3.4 mm ID pultruded carbon fiber tube | Carbon fiber tube stock ($E \ge 180\text{ GPa}$) |
| **1 sheet** (100×150 mm) | **Dogbone Clamp Plate Stock** | $2.0\text{ mm}$ (or $2.5\text{ mm}$) 6061-T6 or 7075-T6 aluminum flat plate | Metal stockists / eBay |
| **20 pcs** | **M3 Knurled Threaded Inserts** | M3 internal thread, ⌀ 4.0–4.2 mm OD, 4.0–5.0 mm length (brass) | Standard brass heat-set / epoxy inserts |
| **10 pcs** | **Pinch Clamping Screws** | M2.5 × 8.0 mm (or 10.0 mm) ISO 4762 Grade 12.9 / Stainless Socket Cap | Fastener supplier |

---

## 2. Keywells & Sensor Electronics

| Qty | Component | Specification | Notes |
| :---: | :--- | :--- | :--- |
| **5 units** | **Svalboard 5-Way Key Clusters** | 5 discrete magnetic tactile switches per cluster (Center Plunge + 4 Directional Paddles) | Svalboard Open Hardware / DataHand compatible |
| **1 pc** | **Microcontroller Board** | Seeed Studio XIAO nRF52840 (BLE 5.0, USB-C, LiPo charger) | Mounts in dorsal saddle / wrist electronics housing |
| **1 pc** | **LiPo Battery** | $3.7\text{ V}$, $150\text{–}300\text{ mAh}$ ultra-thin pouch battery with JST-PH2.0 | Provides 40+ hours continuous wireless BLE typing |
| **1 spool** | **Flexible Micro-Harness Ribbon** | 30 AWG ultra-flexible silicone ribbon cable (VDD, GND, SDA, SCL) | Routes along dorsal carbon tubes into wrist housing |

---

## 3. Ergonomic Chassis & Anatomical Strap

| Qty | Component | Specification | Fabrication Method |
| :---: | :--- | :--- | :--- |
| **1 pc** | **Dorsal Metacarpal Saddle Anchor** | Conformal anatomical saddle plate matching dorsal 3rd metacarpal bed | 3D Printed in CF-PA12 (SLS / FDM) or CNC milled |
| **1 pc** | **4-Way Middle Knuckle Manifold Hub** | 4-port cross-fitting hub (Spine + Arch + MCP3 outrigger) | 3D Printed in Titanium / SLS CF-PA12 or CNC machined |
| **1 pc** | **Circumferential Tension Strap** | $20\text{–}25\text{ mm}$ wide soft elastic TPU or silicone-lined Velcro webbing | 95-Shore A TPU 3D print or medical-grade band |

---

## 4. Consumables & Assembly Tooling

* **Structural Epoxy Adhesive:** 3M Scotch-Weld DP420 (or high-strength Araldite) for bonding M3 brass inserts inside carbon fiber tube ends.
* **Friction Paste / Loctite:** Loctite 243 (medium threadlocker) for M2.5 dogbone pinch screws; optional diamond friction gel for extreme ball grip.
* **CNC Tooling for Dogbone Clamp Milling:**
  * ⌀ 2.05 mm & ⌀ 2.7 mm drill bits.
  * ⌀ 4.76 mm (3/16") or ⌀ 4.8 mm ball-nose endmill (or $90^\circ$ spot/chamfer tool) for ball pockets.
  * ⌀ 3.175 mm (1/8") or ⌀ 2.0 mm flat carbide endmill for profile contouring.

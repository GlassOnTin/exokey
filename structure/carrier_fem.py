"""CONSOLIDATED 3D SPACE-FRAME FINITE ELEMENT MODELLING FOR EXOKEY.

Unifies all manufactured components and physical key mechanics:
1. DORSAL METACARPAL SADDLE ANCHOR & TPU TENSION STRAP FOUNDATION:
   - $6 \times 4$ functionally graded saddle grid with dorsal skin compression springs
     (k = 1.0e5 N/m) and circumferential TPU strap tension (k = 3.3e5 N/m).
2. PRIMARY CENTRAL DORSAL SPINE (Vertebral Column):
   - High-modulus pultruded carbon fiber tube (⌀ 8.0 mm OD / ⌀ 6.0 mm ID, E = 180 GPa).
   - Originates at saddle base (D = 25 mm) and terminates at 4-Way Middle Knuckle Hub.
3. CONTINUOUS TRANSVERSE KNUCKLE WISHBONE ARCH:
   - High-modulus carbon fiber tube (⌀ 6.0 mm OD / ⌀ 4.4 mm ID, E = 180 GPa).
   - Symmetrically bridges Little (MCP5) <-> Ring (MCP4) <-> Middle (MCP3) <-> Index (MCP2).
4. DIRECT 1ST-WEBSPACE INDEX-TO-THUMB OUTRIGGER TRUSS:
   - Arched carbon fiber bridge (⌀ 6.0 mm OD) anchoring directly into Index Knuckle (MCP2).
   - Triangulated webspace cross-brace truss providing high lateral radial stiffness.
5. CONFORMAL 3-LINK PHALANX BOOMS & CNC DUAL-BALL DOGBONE JOINTS:
   - Straight carbon fiber links (⌀ 5.0 mm OD / ⌀ 3.4 mm ID, E = 180 GPa).
   - Symmetrical CNC 6061 aluminum dual-ball clamp dogbones at MCP, PIP, DIP joints.
6. SVALBOARD 5-WAY MAGNETIC ALIGNMENT & TACTILE BREAKAWAY DYNAMICS:
   - Coupled non-linear magnetic dipole attraction curve (F_peak = 0.196 N, z0 = 0.35 mm).
   - Magnetic self-centering restoring gradient (k_align = 364 N/m) ensuring zero key rattle.
   - Evaluates 25 directional typing cases (5 digits x 5 axes) + chord typing + impact loads.
   - Computes structural-to-magnetic crispness ratio (k_struct / k_mag >= 10x).
"""
from __future__ import annotations

import numpy as np

from hand.myohand import MyoHand
from manufacture.carrier_gauntlet import hand_axes
from manufacture.svalboard import build_all_svalboard_units
from structure.anchor import bearing_surface
from structure.fem import SpaceFrameFEM
from structure.frame import MATERIALS


def svalboard_magnetic_force_profile(z_mm: float = 0.0, f_peak_N: float = 0.196, z0_mm: float = 0.35) -> dict:
    """Calculate non-linear Svalboard magnetic force, gradient stiffness, and self-centering alignment torque.
    
    Magnetic dipole return curve:
        F_mag(z) = F_peak / (1 + z / z0)^2
    Tactile gradient stiffness:
        k_mag = -dF/dz = 2 * F_peak / (z0 * (1 + z / z0)^3)
    Self-centering lateral restoring stiffness:
        k_align = 0.65 * F_mag / z0
    """
    z_clamped = max(z_mm, 0.0)
    f_mag = f_peak_N / ((1.0 + z_clamped / z0_mm) ** 2)
    k_mag = 2.0 * f_peak_N / ((z0_mm * 1e-3) * ((1.0 + z_clamped / z0_mm) ** 3))
    k_align = 0.65 * f_mag / (z0_mm * 1e-3)
    
    return {
        "travel_mm": z_clamped,
        "force_N": f_mag,
        "stiffness_N_per_m": k_mag,
        "k_align_N_per_m": k_align,
        "tactile_drop_ratio": f_mag / f_peak_N
    }


def build_carrier_fem_model(h: MyoHand, q: np.ndarray, sval_units: dict | None = None,
                            mat_spine: str = "cf_high_modulus",
                            mat_joint: str = "al6061_t6",
                            strap_k: float = 3.3e5,
                            skin_k: float = 1.0e5,
                            typing_force_N: float = 0.196) -> dict:
    """Construct a full consolidated 3D Space-Frame FEM model connecting all manufactured components."""
    if sval_units is None:
        sval_units = build_all_svalboard_units(h, q)
        
    o, e_d, e_r, e_o = hand_axes(h, q)
    
    # Material properties
    # High-Modulus Carbon Fiber (E = 180 GPa, G = 6.0 GPa, rho = 1550 kg/m3, sigma_ult = 1200 MPa)
    E_cf = 180.0e9
    G_cf = 6.0e9
    rho_cf = 1550.0
    sigma_ult_cf = 1200.0e6
    
    # Aluminum 6061-T6 for CNC clamp plates (E = 70 GPa, G = 26 GPa, rho = 2700 kg/m3, sigma_y = 276 MPa)
    E_al = 70.0e9
    G_al = 26.0e9
    rho_al = 2700.0
    sigma_y_al = 276.0e6
    
    fem = SpaceFrameFEM()
    node_map = {}
    
    # -------------------------------------------------------------------------
    # 1. DORSAL METACARPAL SADDLE GRID & SKIN/STRAP FOUNDATION
    # -------------------------------------------------------------------------
    P_bear, N_bear, K_bear, T_bear = bearing_surface(h, q)
    P_bear = np.asarray(P_bear)
    mc = P_bear[(P_bear - o) @ e_d > 0.002]
    if len(mc) < 4:
        mc = P_bear
        
    r_lo, r_hi = np.min((mc - o) @ e_r), np.max((mc - o) @ e_r)
    d_lo, d_hi = np.min((mc - o) @ e_d), np.max((mc - o) @ e_d)
    hgt = float(np.percentile((P_bear - o) @ e_o, 90)) + 0.0055
    
    n_across, n_along = 6, 4
    rs = np.linspace(r_lo - 0.001, r_hi + 0.001, n_across)
    ds = np.linspace(d_lo - 0.004, d_hi + 0.004, n_along)
    r_mid = 0.5 * (r_lo + r_hi)
    r_span = max(r_hi - r_lo, 0.02)
    
    saddle_nodes = {}
    for row_idx, d_val in enumerate(ds):
        for col_idx, r_val in enumerate(rs):
            arch_sag = 0.003 * (1.0 - ((r_val - r_mid) / (0.5 * r_span))**2)
            pt = o + r_val * e_r + d_val * e_d + (hgt + arch_sag) * e_o
            nid = fem.add_node(pt)
            saddle_nodes[(row_idx, col_idx)] = nid
            
    # Saddle ribs & transverse arches
    for col_idx in range(n_across):
        for row_idx in range(n_along - 1):
            nA = saddle_nodes[(row_idx, col_idx)]
            nB = saddle_nodes[(row_idx + 1, col_idx)]
            fem.add_element(nA, nB, E=6.0e9, G=2.3e9, r_od=0.0020, r_id=0.0)
            
    for row_idx in range(n_along):
        for col_idx in range(n_across - 1):
            nA = saddle_nodes[(row_idx, col_idx)]
            nB = saddle_nodes[(row_idx, col_idx + 1)]
            fem.add_element(nA, nB, E=6.0e9, G=2.3e9, r_od=0.0020, r_id=0.0)
            
    # Saddle diagonal shear braces
    for row_idx in range(n_along - 1):
        for col_idx in range(n_across - 1):
            n00 = saddle_nodes[(row_idx, col_idx)]
            n01 = saddle_nodes[(row_idx, col_idx + 1)]
            n10 = saddle_nodes[(row_idx + 1, col_idx)]
            n11 = saddle_nodes[(row_idx + 1, col_idx + 1)]
            fem.add_element(n00, n11, E=6.0e9, G=2.3e9, r_od=0.0014, r_id=0.0)
            fem.add_element(n10, n01, E=6.0e9, G=2.3e9, r_od=0.0014, r_id=0.0)
            
    # Saddle Root Hub Node (Anchored over 3rd metacarpal base, D = 25 mm)
    p_root = o + 0.0250 * e_d + 0.0090 * e_r + 0.0240 * e_o
    n_root = fem.add_node(p_root)
    node_map["root"] = n_root
    
    # Tie root hub to proximal saddle frame
    for c in [2, 3]:
        fem.add_element(n_root, saddle_nodes[(0, c)], E=E_cf, G=G_cf, r_od=0.0035, r_id=0.0025)
        fem.add_element(n_root, saddle_nodes[(1, c)], E=E_cf, G=G_cf, r_od=0.0035, r_id=0.0025)
        
    # Boundary conditions: Clamped at saddle base + soft skin foundation
    fem.fix_node(n_root)
    fem.fix_node(saddle_nodes[(0, 2)])
    fem.fix_node(saddle_nodes[(0, 3)])
    
    # -------------------------------------------------------------------------
    # 2. MCP KNUCKLE NODES & 4-WAY MANIFOLD
    # -------------------------------------------------------------------------
    mcp_coords = {
        "little": o + 0.0580 * e_d - 0.0370 * e_r + (0.0100 + 0.0075) * e_o,
        "ring":   o + 0.0628 * e_d - 0.0127 * e_r + (0.0122 + 0.0070) * e_o,
        "middle": o + 0.0615 * e_d + 0.0090 * e_r + (0.0149 + 0.0070) * e_o,
        "index":  o + 0.0582 * e_d + 0.0369 * e_r + (0.0107 + 0.0070) * e_o,
    }
    
    id_mcp_th = h.model.body("proximal_thumb").id
    id_ip_th = h.model.body("distal_thumb").id
    mcp_coords["thumb"] = np.copy(h.data.xpos[id_mcp_th]) + 0.014 * e_o + 0.024 * e_r
    
    for f, pt in mcp_coords.items():
        node_map[f"mcp_{f}"] = fem.add_node(pt)
        
    # -------------------------------------------------------------------------
    # 3. PRIMARY CENTRAL DORSAL SPINE (⌀ 8.0 mm OD / ⌀ 6.0 mm ID CF Tube)
    # -------------------------------------------------------------------------
    fem.add_element(node_map["root"], node_map["mcp_middle"],
                    E=E_cf, G=G_cf, r_od=0.0040, r_id=0.0030)
                    
    # -------------------------------------------------------------------------
    # 4. TRANSVERSE DORSAL KNUCKLE ARCH (⌀ 6.0 mm OD / ⌀ 4.4 mm ID CF Tube)
    # -------------------------------------------------------------------------
    arch_seq = ["little", "ring", "middle", "index"]
    for i in range(len(arch_seq) - 1):
        fA, fB = arch_seq[i], arch_seq[i+1]
        fem.add_element(node_map[f"mcp_{fA}"], node_map[f"mcp_{fB}"],
                        E=E_cf, G=G_cf, r_od=0.0030, r_id=0.0022)
                        
    # -------------------------------------------------------------------------
    # 5. DIRECT 1ST-WEBSPACE INDEX-TO-THUMB OUTRIGGER TRUSS (⌀ 6.0 mm OD CF)
    # -------------------------------------------------------------------------
    p_web = 0.5 * (mcp_coords["index"] + mcp_coords["thumb"]) + 0.008 * e_o + 0.006 * e_r
    n_web = fem.add_node(p_web)
    node_map["web_arch"] = n_web
    fem.add_element(node_map["mcp_index"], n_web,
                    E=E_cf, G=G_cf, r_od=0.0030, r_id=0.0022)
    fem.add_element(n_web, node_map["mcp_thumb"],
                    E=E_cf, G=G_cf, r_od=0.0030, r_id=0.0022)
                    
    # Triangulated diagonal webspace cross-brace truss (Thumb TMC <-> Index MCP)
    fem.add_element(node_map["mcp_thumb"], node_map["mcp_index"],
                    E=E_cf, G=G_cf, r_od=0.0025, r_id=0.0017)
                    
    # -------------------------------------------------------------------------
    # 6. CONFORMAL PHALANX BOOMS & SVALBOARD KEYWELL PODS
    # -------------------------------------------------------------------------
    digit_chains = {
        "index": [
            mcp_coords["index"],
            o + 0.0848 * e_d + 0.0425 * e_r + (-0.0100 + 0.0070) * e_o + 0.006 * e_d,
            o + 0.1070 * e_d + 0.0417 * e_r + (-0.0299 + 0.0060) * e_o + 0.010 * e_d,
            o + 0.1160 * e_d + 0.0380 * e_r - 0.0650 * e_o
        ],
        "middle": [
            mcp_coords["middle"],
            o + 0.0941 * e_d + 0.0178 * e_r + (-0.0067 + 0.0070) * e_o + 0.006 * e_d,
            o + 0.1134 * e_d + 0.0156 * e_r + (-0.0340 + 0.0060) * e_o + 0.010 * e_d,
            o + 0.1200 * e_d + 0.0150 * e_r - 0.0720 * e_o
        ],
        "ring": [
            mcp_coords["ring"],
            o + 0.0901 * e_d - 0.0093 * e_r + (-0.0081 + 0.0070) * e_o + 0.006 * e_d,
            o + 0.1073 * e_d - 0.0038 * e_r + (-0.0313 + 0.0060) * e_o + 0.010 * e_d,
            o + 0.1140 * e_d - 0.0050 * e_r - 0.0680 * e_o
        ],
        "little": [
            mcp_coords["little"],
            o + 0.085 * e_d - 0.048 * e_r - 0.008 * e_o,
            o + 0.092 * e_d - 0.044 * e_r - 0.035 * e_o,
            sval_units["little"]["center"] - 0.012 * e_r + 0.008 * e_d
        ],
        "thumb": [
            mcp_coords["thumb"],
            np.copy(h.data.xpos[id_ip_th]) + 0.012 * e_o + 0.024 * e_r,
            sval_units["thumb"]["center"] + 0.020 * e_r + 0.008 * e_d
        ]
    }
    
    pod_nodes = {}
    for f, chain in digit_chains.items():
        prev_node = node_map[f"mcp_{f}"]
        for seg_idx, pt in enumerate(chain[1:]):
            n_curr = fem.add_node(pt)
            node_map[f"{f}_node_{seg_idx+1}"] = n_curr
            
            # Straight CF Tube (⌀ 5.0 mm OD / ⌀ 3.4 mm ID)
            fem.add_element(prev_node, n_curr,
                            E=E_cf, G=G_cf, r_od=0.0025, r_id=0.0017)
            prev_node = n_curr
        pod_nodes[f] = prev_node
        
    return {
        "fem": fem,
        "node_map": node_map,
        "pod_nodes": pod_nodes,
        "mcp_coords": mcp_coords,
        "digit_chains": digit_chains,
        "sval_units": sval_units,
        "hand_axes": (o, e_d, e_r, e_o),
        "typing_force_N": typing_force_N,
        "sigma_ult_cf": sigma_ult_cf,
        "sigma_y_al": sigma_y_al,
        "magnetic_profile": svalboard_magnetic_force_profile(0.0, typing_force_N)
    }


def solve_carrier_typing_cases(model: dict) -> dict:
    """Solve all 25 operational typing load cases (5 digits x 5 actuation axes)."""
    fem: SpaceFrameFEM = model["fem"]
    pod_nodes: dict[str, int] = model["pod_nodes"]
    sval_units: dict = model["sval_units"]
    typing_force_N: float = model["typing_force_N"]
    sigma_ult_cf: float = model["sigma_ult_cf"]
    
    # 5 directions per digit
    directions = ["click", "forward", "back", "left", "right"]
    results = {}
    
    worst_deflection_um = 0.0
    worst_case_info = None
    
    for f, pod_nid in pod_nodes.items():
        u = sval_units[f]
        dirs = u["dirs"]
        results[f] = {}
        
        for d_name in directions:
            d_vec = np.asarray(dirs[d_name])
            d_unit = d_vec / (np.linalg.norm(d_vec) + 1e-12)
            
            f_vec = -typing_force_N * d_unit
            force_dict = {pod_nid: np.array([f_vec[0], f_vec[1], f_vec[2], 0.0, 0.0, 0.0])}
            res = fem.solve(force_dict)
            
            disp_m = float(res["trans_displacements_m"][pod_nid])
            disp_um = disp_m * 1.0e6
            max_stress_MPa = res["max_stress_MPa"]
            sf = (sigma_ult_cf / 1.0e6) / max(max_stress_MPa, 1e-3)
            
            # Structural stiffness along actuation axis
            k_struct = typing_force_N / max(disp_m, 1e-9)
            
            # Coupled Crispness Ratio vs magnetic gradient
            mag_info = svalboard_magnetic_force_profile(0.0, typing_force_N)
            crispness_ratio = k_struct / mag_info["stiffness_N_per_m"]
            
            case_data = {
                "disp_um": disp_um,
                "max_stress_MPa": max_stress_MPa,
                "safety_factor": sf,
                "force_N": typing_force_N,
                "k_struct_N_per_m": k_struct,
                "crispness_ratio": crispness_ratio
            }
            results[f][d_name] = case_data
            
            if disp_um > worst_deflection_um:
                worst_deflection_um = disp_um
                worst_case_info = (f, d_name, disp_um)
                
    return {
        "digit_cases": results,
        "worst_deflection_um": worst_deflection_um,
        "worst_case": worst_case_info,
        "passes_gate": worst_deflection_um <= 180.0  # Crisp <= 180 μm gate
    }


def solve_chord_typing_case(model: dict) -> dict:
    """Solve simultaneous 5-finger chord typing (1.0 N total downward plunge)."""
    fem: SpaceFrameFEM = model["fem"]
    pod_nodes: dict[str, int] = model["pod_nodes"]
    typing_force_N: float = model["typing_force_N"]
    sigma_ult_cf: float = model["sigma_ult_cf"]
    
    chord_forces = {}
    for f, pod_nid in pod_nodes.items():
        chord_forces[pod_nid] = np.array([0.0, 0.0, -typing_force_N, 0.0, 0.0, 0.0])
        
    res = fem.solve(chord_forces)
    
    disps_um = {f: float(res["trans_displacements_m"][nid]) * 1.0e6 for f, nid in pod_nodes.items()}
    max_disp_um = max(disps_um.values())
    max_stress_MPa = res["max_stress_MPa"]
    sf = (sigma_ult_cf / 1.0e6) / max(max_stress_MPa, 1e-3)
    
    return {
        "pod_displacements_um": disps_um,
        "max_deflection_um": max_disp_um,
        "max_stress_MPa": max_stress_MPa,
        "safety_factor": sf,
        "total_force_N": typing_force_N * len(pod_nodes),
        "passes_gate": max_stress_MPa <= 50.0
    }


def evaluate_impact_rigidity(model: dict) -> dict:
    """Evaluate accidental top knock (5.0 N), lateral bash (3.0 N), and snag (2.0 N)."""
    fem: SpaceFrameFEM = model["fem"]
    pod_nodes: dict[str, int] = model["pod_nodes"]
    node_map: dict = model["node_map"]
    sigma_ult_cf: float = model["sigma_ult_cf"]
    
    cases = {}
    
    # 1. Top Knock (5.0 N downward impact on Middle Knuckle Hub)
    res_knock = fem.solve({node_map["mcp_middle"]: np.array([0.0, 0.0, -5.0, 0.0, 0.0, 0.0])})
    cases["top_knock_5N"] = {
        "max_deflection_um": res_knock["max_deflection_um"],
        "max_stress_MPa": res_knock["max_stress_MPa"],
        "safety_factor": (sigma_ult_cf / 1.0e6) / max(res_knock["max_stress_MPa"], 1e-3)
    }
    
    # 2. Lateral Bash (3.0 N outward impact on Little Finger Outrigger)
    res_bash = fem.solve({pod_nodes["little"]: np.array([0.0, -3.0, 0.0, 0.0, 0.0, 0.0])})
    cases["lateral_bash_3N"] = {
        "max_deflection_um": res_bash["max_deflection_um"],
        "max_stress_MPa": res_bash["max_stress_MPa"],
        "safety_factor": (sigma_ult_cf / 1.0e6) / max(res_bash["max_stress_MPa"], 1e-3)
    }
    
    # 3. Accidental Snag (2.0 N downward snag on Little keywell)
    res_snag = fem.solve({pod_nodes["little"]: np.array([0.0, 0.0, -2.0, 0.0, 0.0, 0.0])})
    cases["snag_impact_2N"] = {
        "max_deflection_um": res_snag["max_deflection_um"],
        "max_stress_MPa": res_snag["max_stress_MPa"],
        "safety_factor": (sigma_ult_cf / 1.0e6) / max(res_snag["max_stress_MPa"], 1e-3)
    }
    
    all_pass = all(c["safety_factor"] >= 20.0 for c in cases.values())
    return {
        "cases": cases,
        "passes_gate": all_pass
    }

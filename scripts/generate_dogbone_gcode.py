"""PARAMETRIC CNC G-CODE GENERATOR FOR ARBITRARY N-WAY CLAMP PLATES (DUET 2 / RRF).

Generates CNC toolpaths for machining symmetrical 2-piece clamping sandwich plates
for ANY number of radial branches (N = 2, 3, 4, 5, ...):
- N = 2: Standard Linear Symmetrical Dogbone (Phalanx links, Thumb bridge)
- N = 3: 120° Axially Symmetric Tri-Lobe (Ring MCP4, Index MCP2)
- N = 4: 90° Symmetrical Quad-Lobe Cross (Middle MCP3 4-Way Hub)
- N >= 5: Symmetrical N-Star / Multi-Lobe Manifold Clamps

Features:
- Parametric ball diameter (4.8 mm Yeah Racing YA-0562 default), arm radius, sheet thickness.
- Equidistant radial branch angles (360° / N) guaranteeing even clamping force distribution.
- Op 1: Center Clamping Holes (Peck Drilling with full chip clearing).
- Op 2: Radial Ball Pockets (Hemispherical plunge / 90° chamfer seat with dwell).
- Op 3: Multi-Lobe Perimeter Profile Milling with 3D Bridge Holding Tabs.

Compatible with Duet 2 (RepRapFirmware / RRF CNC mode) & Duet Web Control.

Usage:
    # Generate full ExoKey assembly kit (2-way, 3-way, 4-way):
    PYTHONPATH=. .venv/bin/python scripts/generate_dogbone_gcode.py --preset exokey

    # Generate custom 4-way cross clamps:
    PYTHONPATH=. .venv/bin/python scripts/generate_dogbone_gcode.py --branches 4 --pairs 2
"""
from __future__ import annotations

import argparse
import os
import math
from dataclasses import dataclass
import numpy as np


@dataclass
class CNCConfig:
    # Stock & Geometry
    stock_thickness_mm: float = 2.0
    ball_diameter_mm: float = 4.8
    arm_radius_mm: float = 7.5         # Radius from center bolt to each ball pocket (15mm C-C for N=2)
    plate_width_mm: float = 7.0
    pocket_depth_mm: float = 1.10
    branches: list[int] | None = None  # List of branch counts per part
    
    # Screw / Drill Settings
    screw_type: str = "M2.5"
    clearance_hole_dia: float = 2.7
    tap_drill_dia: float = 2.05
    
    # Tooling
    tool_drill_dia: float = 2.5
    tool_ball_dia: float = 4.8
    tool_mill_dia: float = 3.175       # 1/8" flat endmill for profile
    
    # Speeds & Feeds (Aluminum 6061)
    spindle_rpm: int = 18000
    feed_plunge_drill: float = 120.0   # mm/min
    feed_plunge_pocket: float = 80.0   # mm/min
    feed_cut_profile: float = 450.0    # mm/min
    feed_plunge_profile: float = 150.0 # mm/min
    feed_rapid: float = 2000.0         # mm/min
    
    # Milling Strategy
    stepdown_mm: float = 0.5
    tab_width_mm: float = 2.0
    tab_height_mm: float = 0.6
    z_safe_mm: float = 5.0
    z_retract_mm: float = 1.5


def format_header(title: str, config: CNCConfig) -> list[str]:
    return [
        f"; ========================================================",
        f"; ExoKey Parametric N-Way Clamping Plate Generator",
        f"; Job: {title}",
        f"; Machine: Duet 2 CNC (RepRapFirmware)",
        f"; Stock: {config.stock_thickness_mm:.2f} mm Aluminum Sheet",
        f"; Ball Diameter: ⌀ {config.ball_diameter_mm:.1f} mm | Pocket Depth: {config.pocket_depth_mm:.2f} mm",
        f"; Clamping Screw: {config.screw_type}",
        f"; ========================================================",
        "G21            ; Metric units (mm)",
        "G90            ; Absolute positioning",
        "G94            ; Feed rate units mm/min",
        "G17            ; XY plane selection",
        f"G0 Z{config.z_safe_mm:.2f} F{config.feed_rapid:.0f} ; Retract to safe Z",
        f"M3 S{config.spindle_rpm}   ; Spindle ON ({config.spindle_rpm} RPM)",
        "G4 P1500       ; Wait 1.5s for spindle to reach full speed",
        ""
    ]


def format_footer(config: CNCConfig) -> list[str]:
    return [
        "",
        "; ========================================================",
        "; Job Completion & Safe Park",
        "; ========================================================",
        f"G0 Z{config.z_safe_mm * 4:.2f} F{config.feed_rapid:.0f} ; Retract to high safe Z",
        "M5             ; Spindle Stop",
        "M9             ; Coolant/Air blast OFF",
        "G0 X0 Y100     ; Park table forward for part removal",
        "M84 S0         ; Keep steppers engaged or idle",
        "; End of Program"
    ]


def build_n_way_layout(config: CNCConfig) -> list[dict]:
    """Calculate compact grid packing coordinates for arbitrary N-way clamp plates."""
    items = []
    branch_list = config.branches if config.branches else [2, 2, 2, 2]
    
    cur_x = 12.0
    r_max = config.arm_radius_mm + config.plate_width_mm / 2.0 + 2.0
    
    for idx, n in enumerate(branch_list):
        spacing_x = 2.0 * r_max + 4.0
        
        # Calculate N radial ball pocket offsets
        # For N=2, align horizontally (0°, 180°)
        # For N >= 3, distribute evenly (0°, 360/N, 2*360/N, ...)
        pocket_offsets = []
        for k in range(n):
            ang_deg = k * (360.0 / n)
            rad = math.radians(ang_deg)
            ox = config.arm_radius_mm * math.cos(rad)
            oy = config.arm_radius_mm * math.sin(rad)
            pocket_offsets.append((ox, oy, ang_deg))
            
        # Top Plate (Clearance Hole)
        y_top = r_max + 6.0
        pockets_top = [(cur_x + ox, y_top + oy) for ox, oy, _ in pocket_offsets]
        items.append({
            "id": f"{idx+1}_top",
            "branches": n,
            "role": "top_clearance",
            "center": (cur_x, y_top),
            "drill_hole": (cur_x, y_top),
            "pocket_offsets": pocket_offsets,
            "pockets": pockets_top
        })
        
        # Bottom Plate (Tap Hole)
        y_bot = y_top + 2.0 * r_max + 6.0
        pockets_bot = [(cur_x + ox, y_bot + oy) for ox, oy, _ in pocket_offsets]
        items.append({
            "id": f"{idx+1}_bot",
            "branches": n,
            "role": "bottom_tap",
            "center": (cur_x, y_bot),
            "drill_hole": (cur_x, y_bot),
            "pocket_offsets": pocket_offsets,
            "pockets": pockets_bot
        })
        
        cur_x += spacing_x
        
    return items


def generate_op1_drilling(config: CNCConfig, items: list[dict]) -> list[str]:
    """Op 1: Center Clamping Holes (Peck Drilling)."""
    lines = [
        "; --------------------------------------------------------",
        "; OPERATION 1: Center Clamping Screw Holes (Peck Drilling)",
        "; Tool: Center / Twist Drill",
        "; --------------------------------------------------------"
    ]
    peck_depth = 0.7
    total_depth = config.stock_thickness_mm + 0.4
    
    for idx, it in enumerate(items):
        hx, hy = it["drill_hole"]
        lines.append(f"; Part {idx+1}: {it['branches']}-Way {it['role']} at ({hx:.2f}, {hy:.2f})")
        lines.append(f"G0 X{hx:.3f} Y{hy:.3f} F{config.feed_rapid:.0f}")
        lines.append(f"G0 Z{config.z_retract_mm:.3f}")
        
        curr_z = 0.0
        while curr_z > -total_depth:
            curr_z = max(curr_z - peck_depth, -total_depth)
            lines.append(f"G1 Z{curr_z:.3f} F{config.feed_plunge_drill:.0f}")
            lines.append(f"G0 Z{config.z_retract_mm:.3f} ; Chip clearing retract")
            if curr_z > -total_depth:
                lines.append(f"G0 Z{curr_z + 0.15:.3f}")
                
        lines.append(f"G0 Z{config.z_retract_mm:.3f}")
        lines.append("")
        
    return lines


def generate_op2_ball_pockets(config: CNCConfig, items: list[dict]) -> list[str]:
    """Op 2: Radial Ball Pockets (Hemispherical plunge / 90° chamfer seat with dwell)."""
    lines = [
        "; --------------------------------------------------------",
        f"; OPERATION 2: Spherical Ball Pockets ({config.pocket_depth_mm:.2f}mm depth)",
        f"; Tool: ⌀ {config.ball_diameter_mm:.1f}mm Ball Endmill / 90° Chamfer",
        "; --------------------------------------------------------"
    ]
    z_final = -config.pocket_depth_mm
    
    for idx, it in enumerate(items):
        cx, cy = it["center"]
        lines.append(f"; Part {idx+1} {it['branches']}-Way Clamps at center ({cx:.2f}, {cy:.2f})")
        for p_idx, (bx, by) in enumerate(it["pockets"]):
            lines.append(f"; Pocket {p_idx+1}/{it['branches']} at ({bx:.3f}, {by:.3f})")
            lines.append(f"G0 X{bx:.3f} Y{by:.3f} F{config.feed_rapid:.0f}")
            lines.append(f"G0 Z{config.z_retract_mm:.3f}")
            
            z_step = 0.35
            z_curr = 0.0
            while z_curr > z_final:
                z_curr = max(z_curr - z_step, z_final)
                lines.append(f"G1 Z{z_curr:.3f} F{config.feed_plunge_pocket:.0f}")
                
            lines.append("G4 P350 ; Dwell for smooth spherical seat")
            lines.append(f"G0 Z{config.z_retract_mm:.3f}")
            
        lines.append("")
        
    return lines


def generate_op3_profiling(config: CNCConfig, items: list[dict]) -> list[str]:
    """Op 3: Multi-Lobe Perimeter Profile Milling with 3D Bridge Holding Tabs."""
    lines = [
        "; --------------------------------------------------------",
        "; OPERATION 3: Multi-Lobe Perimeter Contour with 3D Tabs",
        "; Tool: ⌀ 3.175mm (1/8\") Flat Carbide Endmill",
        "; --------------------------------------------------------"
    ]
    r_tool = config.tool_mill_dia / 2.0
    r_lobe = (config.plate_width_mm / 2.0) + r_tool
    total_depth = config.stock_thickness_mm + 0.15
    num_passes = math.ceil(total_depth / config.stepdown_mm)
    tab_z_threshold = -(config.stock_thickness_mm - config.tab_height_mm)
    
    for idx, it in enumerate(items):
        px, py = it["center"]
        n = it["branches"]
        lines.append(f"; Part {idx+1}: {n}-Way Contour at ({px:.2f}, {py:.2f})")
        
        # Build polygon perimeter loop points
        # Each lobe extends outwards along angle ang_deg with arc around tip
        contour_pts = []
        for k in range(n):
            ang_deg = k * (360.0 / n)
            rad = math.radians(ang_deg)
            # Tip of the lobe
            tx = px + (config.arm_radius_mm + r_lobe) * math.cos(rad)
            ty = py + (config.arm_radius_mm + r_lobe) * math.sin(rad)
            contour_pts.append((tx, ty))
            
            # Midpoint valley between this lobe and the next
            next_ang_deg = (k + 1) * (360.0 / n)
            mid_rad = math.radians(0.5 * (ang_deg + next_ang_deg))
            r_valley = max(r_lobe * 0.9, config.arm_radius_mm * 0.45)
            vx = px + r_valley * math.cos(mid_rad)
            vy = py + r_valley * math.sin(mid_rad)
            contour_pts.append((vx, vy))
            
        start_pt = contour_pts[0]
        lines.append(f"G0 X{start_pt[0]:.3f} Y{start_pt[1]:.3f} F{config.feed_rapid:.0f}")
        lines.append(f"G0 Z{config.z_retract_mm:.3f}")
        
        for pass_idx in range(1, num_passes + 1):
            z_cut = -min(pass_idx * config.stepdown_mm, total_depth)
            lines.append(f"; Pass {pass_idx}/{num_passes} at Z={z_cut:.3f}")
            lines.append(f"G1 Z{z_cut:.3f} F{config.feed_plunge_profile:.0f}")
            
            for pt_idx, pt in enumerate(contour_pts[1:] + [start_pt]):
                # Add holding tab on the second valley
                is_tab_valley = (pt_idx == 3)
                if is_tab_valley and z_cut < tab_z_threshold:
                    lines.append(f"G1 X{pt[0]:.3f} Y{pt[1]:.3f} F{config.feed_cut_profile:.0f}")
                    lines.append(f"G1 Z{tab_z_threshold:.3f} F{config.feed_plunge_profile:.0f} ; Holding tab")
                    lines.append(f"G1 Z{z_cut:.3f} F{config.feed_plunge_profile:.0f}")
                else:
                    lines.append(f"G1 X{pt[0]:.3f} Y{pt[1]:.3f} F{config.feed_cut_profile:.0f}")
                    
        lines.append(f"G0 Z{config.z_retract_mm:.3f}")
        lines.append("")
        
    return lines


def main():
    parser = argparse.ArgumentParser(description="Parametric Duet 2 CNC G-code Generator for N-Way Clamping Plates")
    parser.add_argument("--preset", type=str, default="exokey", choices=["exokey", "custom"], help="Presets: 'exokey' (Full Kit) or 'custom'")
    parser.add_argument("--branches", type=str, default="2,2,2,2", help="Comma-separated branch counts for custom preset (e.g. 2,3,4)")
    parser.add_argument("--pairs", type=int, default=1, help="Multiplier for pair count (default: 1)")
    parser.add_argument("--ball-dia", type=float, default=4.8, help="Ball stud diameter in mm (default: 4.8 for Yeah Racing YA-0562)")
    parser.add_argument("--arm-radius", type=float, default=7.5, help="Radius from center to ball pockets in mm (default: 7.5 mm = 15mm C-C)")
    parser.add_argument("--thick", type=float, default=2.0, help="Stock aluminum sheet thickness in mm (default: 2.0)")
    parser.add_argument("--screw", type=str, default="M2.5", choices=["M2", "M2.5", "M3"], help="Center clamp screw size (default: M2.5)")
    parser.add_argument("--outdir", type=str, default="out/cnc", help="Output directory for G-code files")
    args = parser.parse_args()

    os.makedirs(args.outdir, exist_ok=True)
    
    if args.preset == "exokey":
        # Complete ExoKey Hardware Kit:
        # - 6 pairs of 2-way Dogbones (Phalanx Booms + Thumb Bridge)
        # - 2 pairs of 3-way Tri-Lobes (MCP4 Ring + MCP2 Index)
        # - 1 pair of 4-way Quad-Lobes (MCP3 Middle Knuckle Cross-Hub)
        branch_list = [2] * (6 * args.pairs) + [3] * (2 * args.pairs) + [4] * (1 * args.pairs)
    else:
        raw_list = [int(b.strip()) for b in args.branches.split(",") if b.strip()]
        branch_list = raw_list * args.pairs
        
    cfg = CNCConfig(
        branches=branch_list,
        ball_diameter_mm=args.ball_dia,
        arm_radius_mm=args.arm_radius,
        stock_thickness_mm=args.thick,
        pocket_depth_mm=1.10 if args.ball_dia <= 5.0 else 1.30,
        plate_width_mm=7.0 if args.ball_dia <= 5.0 else 8.0,
        tool_ball_dia=args.ball_dia,
        screw_type=args.screw
    )
    
    items = build_n_way_layout(cfg)
    
    # Master G-code Program
    combined_lines = []
    combined_lines.extend(format_header(f"Complete N-Way Clamping Plate Kit ({len(branch_list)} pairs)", cfg))
    
    combined_lines.append("; --- TOOL 1: Spot / Drill ---")
    combined_lines.extend(generate_op1_drilling(cfg, items))
    
    combined_lines.append("M5 ; Stop Spindle")
    combined_lines.append(f"G0 Z{cfg.z_safe_mm * 4:.2f} ; High Retract")
    combined_lines.append("M0 \"Tool Change: Insert ⌀ 4.8mm Ball Endmill / 90-deg Chamfer Tool & Re-Zero Z\" ; Pause for DWC")
    combined_lines.append(f"M3 S{cfg.spindle_rpm} ; Spindle ON")
    combined_lines.append("G4 P1500")
    
    combined_lines.append("; --- TOOL 2: Ball Pockets ---")
    combined_lines.extend(generate_op2_ball_pockets(cfg, items))
    
    combined_lines.append("M5 ; Stop Spindle")
    combined_lines.append(f"G0 Z{cfg.z_safe_mm * 4:.2f} ; High Retract")
    combined_lines.append("M0 \"Tool Change: Insert ⌀ 3.175mm (1/8in) Flat Endmill & Re-Zero Z\" ; Pause for DWC")
    combined_lines.append(f"M3 S{cfg.spindle_rpm} ; Spindle ON")
    combined_lines.append("G4 P1500")
    
    combined_lines.append("; --- TOOL 3: Profile with 3D Tabs ---")
    combined_lines.extend(generate_op3_profiling(cfg, items))
    
    combined_lines.extend(format_footer(cfg))
    
    combined_path = os.path.join(args.outdir, "dogbone_clamp_plates_complete.gcode")
    with open(combined_path, "w") as f:
        f.write("\n".join(combined_lines))
        
    # Export 3D STL models of the machined clamps
    from manufacture.dogbone_clamps import build_nway_clamp_assembly
    
    # 2-way Dogbone STL
    arm_2way = [np.array([-cfg.arm_radius_mm*1e-3, 0.0, 0.0]), np.array([cfg.arm_radius_mm*1e-3, 0.0, 0.0])]
    stl_2way = build_nway_clamp_assembly(arm_2way, width=cfg.plate_width_mm*1e-3, plate_thick=cfg.stock_thickness_mm*1e-3, ball_dia=cfg.ball_diameter_mm*1e-3)
    stl_2way.export(os.path.join(args.outdir, "clamp_2way_dogbone.stl"))
    
    # 3-way Tri-Lobe STL
    arm_3way = [
        np.array([cfg.arm_radius_mm*1e-3 * math.cos(math.radians(ang)), cfg.arm_radius_mm*1e-3 * math.sin(math.radians(ang)), 0.0])
        for ang in [0, 120, 240]
    ]
    stl_3way = build_nway_clamp_assembly(arm_3way, width=cfg.plate_width_mm*1e-3, plate_thick=cfg.stock_thickness_mm*1e-3, ball_dia=cfg.ball_diameter_mm*1e-3)
    stl_3way.export(os.path.join(args.outdir, "clamp_3way_trilobe.stl"))
    
    # 4-way Quad-Cross STL
    arm_4way = [
        np.array([cfg.arm_radius_mm*1e-3 * math.cos(math.radians(ang)), cfg.arm_radius_mm*1e-3 * math.sin(math.radians(ang)), 0.0])
        for ang in [0, 90, 180, 270]
    ]
    stl_4way = build_nway_clamp_assembly(arm_4way, width=cfg.plate_width_mm*1e-3, plate_thick=cfg.stock_thickness_mm*1e-3, ball_dia=cfg.ball_diameter_mm*1e-3)
    stl_4way.export(os.path.join(args.outdir, "clamp_4way_quadcross.stl"))
    
    print(f"Generated CNC G-code and 3D STLs in {args.outdir}/:")
    print(f"  • G-code: {combined_path} ({len(combined_lines)} lines)")
    print(f"  • STLs: clamp_2way_dogbone.stl, clamp_3way_trilobe.stl, clamp_4way_quadcross.stl")
    print(f"Total Parts: {len(items)} plates ({len(branch_list)} matching pairs)")
    print(f"Breakdown: {branch_list.count(2)}x 2-Way Dogbones | {branch_list.count(3)}x 3-Way Tri-Lobes | {branch_list.count(4)}x 4-Way Quad-Crosses")


if __name__ == "__main__":
    main()

"""PARAMETRIC CNC G-CODE GENERATOR FOR DUAL-BALL DOGBONE CLAMP PLATES (DUET 2 / RRF).

Generates toolpaths for machining symmetrical dual-ball armature clamping plates
from aluminum sheet stock (e.g. 2.0 mm - 3.0 mm 6061-T6 / 7075 aluminum).

Features:
- Parametric ball spacing, plate width, sheet thickness, and screw size (M2, M2.5, M3).
- Generates separate G-code files per operation or a single combined program with M0 pauses.
- Op 1: Center Clamping Holes (Peck Drilling with full chip clearing).
- Op 2: Ball Pockets (Hemispherical plunging or conical seat chamfering with dwell).
- Op 3: Perimeter Profile Milling with 3D Bridge Holding Tabs.

Compatible with Duet 2 (RepRapFirmware / RRF CNC mode) & Duet Web Control.

Usage:
    PYTHONPATH=. .venv/bin/python scripts/generate_dogbone_gcode.py --pairs 4 --spacing 16.0 --thick 2.5 --screw M2.5
"""
from __future__ import annotations

import argparse
import os
import math
from dataclasses import dataclass


@dataclass
class CNCConfig:
    # Stock & Geometry
    stock_thickness_mm: float = 2.0
    ball_diameter_mm: float = 4.8
    ball_spacing_mm: float = 15.0
    plate_width_mm: float = 7.0
    pocket_depth_mm: float = 1.10
    num_pairs: int = 4
    spacing_x_mm: float = 26.0
    spacing_y_mm: float = 13.0
    
    # Screw / Drill Settings
    screw_type: str = "M2.5"  # M2, M2.5, M3
    clearance_hole_dia: float = 2.7   # Top plate clearance
    tap_drill_dia: float = 2.05       # Bottom plate tap drill (M2.5 = 2.05mm, M3 = 2.5mm)
    
    # Tooling
    tool_drill_dia: float = 2.5
    tool_ball_dia: float = 4.8
    tool_mill_dia: float = 3.175     # 1/8" flat endmill for profile
    
    # Speeds & Feeds (Aluminum 6061)
    spindle_rpm: int = 18000
    feed_plunge_drill: float = 120.0   # mm/min
    feed_plunge_pocket: float = 80.0   # mm/min
    feed_cut_profile: float = 450.0    # mm/min
    feed_plunge_profile: float = 150.0 # mm/min
    feed_rapid: float = 2000.0         # mm/min
    
    # Milling Strategy
    stepdown_mm: float = 0.5           # mm per pass for profiling
    tab_width_mm: float = 2.0          # holding tab width
    tab_height_mm: float = 0.6         # holding tab height
    z_safe_mm: float = 5.0             # safe clearance height
    z_retract_mm: float = 1.5          # rapid height inside job


def format_header(title: str, config: CNCConfig) -> list[str]:
    return [
        f"; ========================================================",
        f"; ExoKey Dual-Ball Armature Clamp Plate Generator",
        f"; Job: {title}",
        f"; Machine: Duet 2 CNC (RepRapFirmware)",
        f"; Stock Thickness: {config.stock_thickness_mm:.2f} mm Aluminum",
        f"; Ball Spacing: {config.ball_spacing_mm:.1f} mm C-C | Pocket Depth: {config.pocket_depth_mm:.2f} mm",
        f"; Total Parts: {config.num_pairs * 2} plates ({config.num_pairs} pairs)",
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


def get_part_origins(config: CNCConfig) -> list[tuple[float, float, str]]:
    """Generate (x, y, plate_type) origins for all parts in a compact grid."""
    parts = []
    # 2 rows per pair: Row 0 = Top Plate (Clearance), Row 1 = Bottom Plate (Tap drill)
    cols = min(config.num_pairs, 4)
    rows = math.ceil(config.num_pairs / cols) * 2
    
    pair_idx = 0
    for r in range(0, rows, 2):
        for c in range(cols):
            if pair_idx >= config.num_pairs:
                break
            x0 = c * config.spacing_x_mm + config.plate_width_mm
            y_top = r * config.spacing_y_mm + config.plate_width_mm
            y_bot = (r + 1) * config.spacing_y_mm + config.plate_width_mm
            
            parts.append((x0, y_top, "top_clearance"))
            parts.append((x0, y_bot, "bottom_tap"))
            pair_idx += 1
            
    return parts


def generate_op1_drilling(config: CNCConfig, parts: list[tuple[float, float, str]]) -> list[str]:
    """Op 1: Center Clamping Holes (Peck Drilling)."""
    lines = [
        "; --------------------------------------------------------",
        "; OPERATION 1: Center Clamping Screw Holes (Peck Drilling)",
        "; Tool: Center / Twist Drill",
        "; --------------------------------------------------------"
    ]
    
    peck_depth = 0.8
    total_depth = config.stock_thickness_mm + 0.5  # through into spoilboard
    
    for idx, (px, py, ptype) in enumerate(parts):
        lines.append(f"; Part {idx+1}: {ptype} at ({px:.2f}, {py:.2f})")
        lines.append(f"G0 X{px:.3f} Y{py:.3f} F{config.feed_rapid:.0f}")
        lines.append(f"G0 Z{config.z_retract_mm:.3f}")
        
        # Manual peck loop for universal RRF compatibility
        curr_z = 0.0
        while curr_z > -total_depth:
            curr_z = max(curr_z - peck_depth, -total_depth)
            lines.append(f"G1 Z{curr_z:.3f} F{config.feed_plunge_drill:.0f}")
            lines.append(f"G0 Z{config.z_retract_mm:.3f} ; Full chip clearing retract")
            if curr_z > -total_depth:
                lines.append(f"G0 Z{curr_z + 0.2:.3f} ; Rapid back to cut level")
                
        lines.append(f"G0 Z{config.z_retract_mm:.3f}")
        lines.append("")
        
    return lines


def generate_op2_ball_pockets(config: CNCConfig, parts: list[tuple[float, float, str]]) -> list[str]:
    """Op 2: Ball Pockets (Hemispherical / Conical Seat Plunging with Dwell)."""
    lines = [
        "; --------------------------------------------------------",
        "; OPERATION 2: Dual Spherical Ball Pockets (1.3mm depth)",
        "; Tool: ⌀ 6.0mm Ball Nose or 90° Chamfer Tool",
        "; --------------------------------------------------------"
    ]
    
    dx = config.ball_spacing_mm / 2.0
    z_final = -config.pocket_depth_mm
    
    for idx, (px, py, ptype) in enumerate(parts):
        lines.append(f"; Part {idx+1} Ball Pockets at X={px:.2f}, Y={py:.2f}")
        for pocket_idx, ox in enumerate([-dx, dx]):
            bx, by = px + ox, py
            lines.append(f"; Pocket {pocket_idx+1} at ({bx:.3f}, {by:.3f})")
            lines.append(f"G0 X{bx:.3f} Y{by:.3f} F{config.feed_rapid:.0f}")
            lines.append(f"G0 Z{config.z_retract_mm:.3f}")
            
            # Step down plunge with smooth dwell
            z_step = 0.4
            z_curr = 0.0
            while z_curr > z_final:
                z_curr = max(z_curr - z_step, z_final)
                lines.append(f"G1 Z{z_curr:.3f} F{config.feed_plunge_pocket:.0f}")
                
            lines.append("G4 P350 ; Dwell 350ms for smooth spherical finish")
            lines.append(f"G0 Z{config.z_retract_mm:.3f}")
            
        lines.append("")
        
    return lines


def generate_op3_profiling(config: CNCConfig, parts: list[tuple[float, float, str]]) -> list[str]:
    """Op 3: Perimeter Profile Milling with 3D Bridge Holding Tabs."""
    lines = [
        "; --------------------------------------------------------",
        "; OPERATION 3: Dogbone Perimeter Profile with Holding Tabs",
        "; Tool: ⌀ 3.175mm (1/8\") Flat Carbide Endmill",
        "; --------------------------------------------------------"
    ]
    
    r_tool = config.tool_mill_dia / 2.0
    half_L = config.ball_spacing_mm / 2.0
    r_end = (config.plate_width_mm / 2.0) + r_tool
    waist_y = (config.plate_width_mm / 2.0) * 0.85 + r_tool  # slight dogbone waist
    
    total_depth = config.stock_thickness_mm + 0.15
    num_passes = math.ceil(total_depth / config.stepdown_mm)
    
    tab_half_w = config.tab_width_mm / 2.0
    tab_z_threshold = -(config.stock_thickness_mm - config.tab_height_mm)
    
    for idx, (px, py, ptype) in enumerate(parts):
        lines.append(f"; Part {idx+1} Dogbone Contour at ({px:.2f}, {py:.2f})")
        
        # Lead-in point (top waist)
        x_start = px - half_L
        y_start = py + r_end
        
        lines.append(f"G0 X{x_start:.3f} Y{y_start:.3f} F{config.feed_rapid:.0f}")
        lines.append(f"G0 Z{config.z_retract_mm:.3f}")
        
        for pass_idx in range(1, num_passes + 1):
            z_cut = -min(pass_idx * config.stepdown_mm, total_depth)
            lines.append(f"; Pass {pass_idx}/{num_passes} at Z={z_cut:.3f}")
            lines.append(f"G1 Z{z_cut:.3f} F{config.feed_plunge_profile:.0f}")
            
            # Profile loop:
            # 1. Top straight across to right lobe
            x_tab_l = px - tab_half_w
            x_tab_r = px + tab_half_w
            
            # Top waist with tab
            if z_cut < tab_z_threshold:
                # Up and over top tab
                lines.append(f"G1 X{x_tab_l:.3f} Y{py + waist_y:.3f} F{config.feed_cut_profile:.0f}")
                lines.append(f"G1 Z{tab_z_threshold:.3f} F{config.feed_plunge_profile:.0f} ; Tab bridge")
                lines.append(f"G1 X{x_tab_r:.3f} Y{py + waist_y:.3f} F{config.feed_cut_profile:.0f}")
                lines.append(f"G1 Z{z_cut:.3f} F{config.feed_plunge_profile:.0f}")
            else:
                lines.append(f"G1 X{px + half_L:.3f} Y{py + waist_y:.3f} F{config.feed_cut_profile:.0f}")
                
            # Right rounded lobe (G2 clockwise arc)
            lines.append(f"G1 X{px + half_L:.3f} Y{py + r_end:.3f} F{config.feed_cut_profile:.0f}")
            lines.append(f"G2 X{px + half_L:.3f} Y{py - r_end:.3f} R{r_end:.3f}")
            
            # Bottom waist with tab
            if z_cut < tab_z_threshold:
                lines.append(f"G1 X{x_tab_r:.3f} Y{py - waist_y:.3f} F{config.feed_cut_profile:.0f}")
                lines.append(f"G1 Z{tab_z_threshold:.3f} F{config.feed_plunge_profile:.0f} ; Tab bridge")
                lines.append(f"G1 X{x_tab_l:.3f} Y{py - waist_y:.3f} F{config.feed_cut_profile:.0f}")
                lines.append(f"G1 Z{z_cut:.3f} F{config.feed_plunge_profile:.0f}")
            else:
                lines.append(f"G1 X{px - half_L:.3f} Y{py - waist_y:.3f} F{config.feed_cut_profile:.0f}")
                
            # Left rounded lobe (G2 clockwise arc)
            lines.append(f"G1 X{px - half_L:.3f} Y{py - r_end:.3f} F{config.feed_cut_profile:.0f}")
            lines.append(f"G2 X{px - half_L:.3f} Y{py + r_end:.3f} R{r_end:.3f}")
            
        lines.append(f"G0 Z{config.z_retract_mm:.3f}")
        lines.append("")
        
    return lines


def main():
    parser = argparse.ArgumentParser(description="Duet 2 CNC G-code Generator for Dual-Ball Dogbone Clamps")
    parser.add_argument("--pairs", type=int, default=4, help="Number of clamp plate pairs to machine (default: 4)")
    parser.add_argument("--ball-dia", type=float, default=4.8, help="Ball stud diameter in mm (default: 4.8 for Yeah Racing YA-0562)")
    parser.add_argument("--spacing", type=float, default=15.0, help="Ball pocket center-to-center distance in mm (default: 15.0)")
    parser.add_argument("--thick", type=float, default=2.0, help="Stock aluminum sheet thickness in mm (default: 2.0)")
    parser.add_argument("--screw", type=str, default="M2.5", choices=["M2", "M2.5", "M3"], help="Center clamp screw size (default: M2.5)")
    parser.add_argument("--outdir", type=str, default="out/cnc", help="Output directory for G-code files")
    args = parser.parse_args()

    os.makedirs(args.outdir, exist_ok=True)
    
    cfg = CNCConfig(
        num_pairs=args.pairs,
        ball_diameter_mm=args.ball_dia,
        ball_spacing_mm=args.spacing,
        stock_thickness_mm=args.thick,
        pocket_depth_mm=1.10 if args.ball_dia <= 5.0 else 1.30,
        plate_width_mm=7.0 if args.ball_dia <= 5.0 else 8.0,
        tool_ball_dia=args.ball_dia,
        screw_type=args.screw
    )
    
    parts = get_part_origins(cfg)
    
    # 1. Combined G-code program with M0 Tool-Change Pauses for Duet
    combined_lines = []
    combined_lines.extend(format_header("Complete 3-Op Dual-Ball Clamp Plate Suite", cfg))
    
    combined_lines.append("; --- TOOL 1: Spot / Drill ---")
    combined_lines.extend(generate_op1_drilling(cfg, parts))
    
    combined_lines.append("M5 ; Stop Spindle")
    combined_lines.append(f"G0 Z{cfg.z_safe_mm * 4:.2f} ; High Retract")
    combined_lines.append("M0 \"Tool Change: Insert ⌀ 4.8mm Ball Endmill / 90-deg Chamfer Tool & Re-Zero Z\" ; Pause for DWC")
    combined_lines.append(f"M3 S{cfg.spindle_rpm} ; Spindle ON")
    combined_lines.append("G4 P1500")
    
    combined_lines.append("; --- TOOL 2: Ball Pockets ---")
    combined_lines.extend(generate_op2_ball_pockets(cfg, parts))
    
    combined_lines.append("M5 ; Stop Spindle")
    combined_lines.append(f"G0 Z{cfg.z_safe_mm * 4:.2f} ; High Retract")
    combined_lines.append("M0 \"Tool Change: Insert ⌀ 3.175mm (1/8in) Flat Endmill & Re-Zero Z\" ; Pause for DWC")
    combined_lines.append(f"M3 S{cfg.spindle_rpm} ; Spindle ON")
    combined_lines.append("G4 P1500")
    
    combined_lines.append("; --- TOOL 3: Profile with 3D Tabs ---")
    combined_lines.extend(generate_op3_profiling(cfg, parts))
    
    combined_lines.extend(format_footer(cfg))
    
    combined_path = os.path.join(args.outdir, "dogbone_clamp_plates_complete.gcode")
    with open(combined_path, "w") as f:
        f.write("\n".join(combined_lines))
        
    # 2. Also output individual OP files (handy if zeroing per tool independently)
    # Op 1 file
    op1_lines = format_header("Op 1 - Center Hole Peck Drilling", cfg)
    op1_lines.extend(generate_op1_drilling(cfg, parts))
    op1_lines.extend(format_footer(cfg))
    with open(os.path.join(args.outdir, "dogbone_op1_drilling.gcode"), "w") as f:
        f.write("\n".join(op1_lines))
        
    # Op 2 file
    op2_lines = format_header("Op 2 - Spherical Ball Pockets", cfg)
    op2_lines.extend(generate_op2_ball_pockets(cfg, parts))
    op2_lines.extend(format_footer(cfg))
    with open(os.path.join(args.outdir, "dogbone_op2_ball_pockets.gcode"), "w") as f:
        f.write("\n".join(op2_lines))
        
    # Op 3 file
    op3_lines = format_header("Op 3 - Dogbone Perimeter Profiling", cfg)
    op3_lines.extend(generate_op3_profiling(cfg, parts))
    op3_lines.extend(format_footer(cfg))
    with open(os.path.join(args.outdir, "dogbone_op3_profiling.gcode"), "w") as f:
        f.write("\n".join(op3_lines))
        
    print(f"Generated CNC G-code files in {args.outdir}/:")
    print(f"  • {combined_path} ({len(combined_lines)} lines)")
    print(f"  • {os.path.join(args.outdir, 'dogbone_op1_drilling.gcode')}")
    print(f"  • {os.path.join(args.outdir, 'dogbone_op2_ball_pockets.gcode')}")
    print(f"  • {os.path.join(args.outdir, 'dogbone_op3_profiling.gcode')}")
    print(f"\nTotal Parts: {len(parts)} plates ({cfg.num_pairs} matching sets)")


if __name__ == "__main__":
    main()

"""PARAMETRIC CNC G-CODE GENERATOR FOR DUAL-BALL DOGBONES & 3-WAY TRI-LOBE CLAMPS (DUET 2 / RRF).

Generates toolpaths for machining symmetrical clamping plates from 2.0 mm aluminum sheet:
1. 2-WAY SYMMETRICAL DOGBONE CLAMPS (Phalanx Boom Links & Thumb Bridge)
2. 3-WAY AXIALLY SYMMETRIC TRI-LOBE CLAMPS (120° Knuckle Junctions)

Features:
- Parametric ball diameter (4.8 mm Yeah Racing YA-0562 default), spacing, sheet thickness.
- Op 1: Center Clamping Holes (Peck Drilling with chip clearing).
- Op 2: Ball Pockets (Hemispherical plunge / 90° seat chamfer with dwell).
- Op 3: Perimeter Profile Milling with 3D Bridge Holding Tabs.

Compatible with Duet 2 (RepRapFirmware / RRF CNC mode) & Duet Web Control.

Usage:
    PYTHONPATH=. .venv/bin/python scripts/generate_dogbone_gcode.py --style mixed --screw M2.5
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
    style: str = "mixed"  # dogbone, trilobe, mixed
    
    # Screw / Drill Settings
    screw_type: str = "M2.5"
    clearance_hole_dia: float = 2.7
    tap_drill_dia: float = 2.05
    
    # Tooling
    tool_drill_dia: float = 2.5
    tool_ball_dia: float = 4.8
    tool_mill_dia: float = 3.175
    
    # Speeds & Feeds (Aluminum 6061)
    spindle_rpm: int = 18000
    feed_plunge_drill: float = 120.0
    feed_plunge_pocket: float = 80.0
    feed_cut_profile: float = 450.0
    feed_plunge_profile: float = 150.0
    feed_rapid: float = 2000.0
    
    # Milling Strategy
    stepdown_mm: float = 0.5
    tab_width_mm: float = 2.0
    tab_height_mm: float = 0.6
    z_safe_mm: float = 5.0
    z_retract_mm: float = 1.5


def format_header(title: str, config: CNCConfig) -> list[str]:
    return [
        f"; ========================================================",
        f"; ExoKey Symmetrical Clamping Plate Generator",
        f"; Job: {title}",
        f"; Machine: Duet 2 CNC (RepRapFirmware)",
        f"; Stock: {config.stock_thickness_mm:.2f} mm Aluminum Sheet",
        f"; Ball Diameter: ⌀ {config.ball_diameter_mm:.1f} mm | Pocket Depth: {config.pocket_depth_mm:.2f} mm",
        f"; Clamp Style: {config.style.upper()}",
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


def get_layout(config: CNCConfig) -> list[dict]:
    """Generate part centers, types, and ball pocket coordinates."""
    items = []
    
    if config.style in ["dogbone", "mixed"]:
        n_dog = 4 if config.style == "mixed" else config.num_pairs
        for i in range(n_dog):
            # Top Plate
            x_top = 10.0 + i * 26.0
            y_top = 10.0
            dx = config.ball_spacing_mm / 2.0
            items.append({
                "type": "dogbone",
                "role": "top_clearance",
                "center": (x_top, y_top),
                "drill_hole": (x_top, y_top),
                "pockets": [(x_top - dx, y_top), (x_top + dx, y_top)],
                "spacing": config.ball_spacing_mm
            })
            # Bottom Plate
            y_bot = 24.0
            items.append({
                "type": "dogbone",
                "role": "bottom_tap",
                "center": (x_top, y_bot),
                "drill_hole": (x_top, y_bot),
                "pockets": [(x_top - dx, y_bot), (x_top + dx, y_bot)],
                "spacing": config.ball_spacing_mm
            })
            
    if config.style in ["trilobe", "mixed"]:
        n_tri = 2 if config.style == "mixed" else config.num_pairs
        start_x = 10.0 if config.style == "trilobe" else 10.0 + (4 * 26.0)
        R_arm = 9.0
        for i in range(n_tri):
            # Top Tri-Lobe
            x_top = start_x + i * 32.0 + 12.0
            y_top = 14.0
            pockets_top = [
                (x_top + R_arm * math.cos(math.radians(ang)), y_top + R_arm * math.sin(math.radians(ang)))
                for ang in [0, 120, 240]
            ]
            items.append({
                "type": "trilobe",
                "role": "top_clearance",
                "center": (x_top, y_top),
                "drill_hole": (x_top, y_top),
                "pockets": pockets_top,
                "radius": R_arm
            })
            # Bottom Tri-Lobe
            y_bot = 46.0
            pockets_bot = [
                (x_top + R_arm * math.cos(math.radians(ang)), y_bot + R_arm * math.sin(math.radians(ang)))
                for ang in [0, 120, 240]
            ]
            items.append({
                "type": "trilobe",
                "role": "bottom_tap",
                "center": (x_top, y_bot),
                "drill_hole": (x_top, y_bot),
                "pockets": pockets_bot,
                "radius": R_arm
            })
            
    return items


def generate_op1_drilling(config: CNCConfig, items: list[dict]) -> list[str]:
    """Op 1: Center Clamping Screw Holes (Peck Drilling)."""
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
        lines.append(f"; Part {idx+1}: {it['type']} ({it['role']}) at ({hx:.2f}, {hy:.2f})")
        lines.append(f"G0 X{hx:.3f} Y{hy:.3f} F{config.feed_rapid:.0f}")
        lines.append(f"G0 Z{config.z_retract_mm:.3f}")
        
        curr_z = 0.0
        while curr_z > -total_depth:
            curr_z = max(curr_z - peck_depth, -total_depth)
            lines.append(f"G1 Z{curr_z:.3f} F{config.feed_plunge_drill:.0f}")
            lines.append(f"G0 Z{config.z_retract_mm:.3f} ; Chip clear")
            if curr_z > -total_depth:
                lines.append(f"G0 Z{curr_z + 0.15:.3f}")
                
        lines.append(f"G0 Z{config.z_retract_mm:.3f}")
        lines.append("")
        
    return lines


def generate_op2_ball_pockets(config: CNCConfig, items: list[dict]) -> list[str]:
    """Op 2: Ball Pockets (Hemispherical plunging with dwell)."""
    lines = [
        "; --------------------------------------------------------",
        f"; OPERATION 2: Spherical Ball Pockets ({config.pocket_depth_mm:.2f}mm depth)",
        f"; Tool: ⌀ {config.ball_diameter_mm:.1f}mm Ball Endmill / 90° Chamfer",
        "; --------------------------------------------------------"
    ]
    z_final = -config.pocket_depth_mm
    
    for idx, it in enumerate(items):
        lines.append(f"; Part {idx+1} {it['type']} Pockets at center ({it['center'][0]:.2f}, {it['center'][1]:.2f})")
        for p_idx, (bx, by) in enumerate(it["pockets"]):
            lines.append(f"; Pocket {p_idx+1} at ({bx:.3f}, {by:.3f})")
            lines.append(f"G0 X{bx:.3f} Y{by:.3f} F{config.feed_rapid:.0f}")
            lines.append(f"G0 Z{config.z_retract_mm:.3f}")
            
            z_step = 0.35
            z_curr = 0.0
            while z_curr > z_final:
                z_curr = max(z_curr - z_step, z_final)
                lines.append(f"G1 Z{z_curr:.3f} F{config.feed_plunge_pocket:.0f}")
                
            lines.append("G4 P350 ; Dwell for smooth surface finish")
            lines.append(f"G0 Z{config.z_retract_mm:.3f}")
            
        lines.append("")
        
    return lines


def generate_op3_profiling(config: CNCConfig, items: list[dict]) -> list[str]:
    """Op 3: Perimeter Profile Milling with 3D Bridge Holding Tabs."""
    lines = [
        "; --------------------------------------------------------",
        "; OPERATION 3: Perimeter Contour Profiling with 3D Tabs",
        "; Tool: ⌀ 3.175mm (1/8\") Flat Carbide Endmill",
        "; --------------------------------------------------------"
    ]
    r_tool = config.tool_mill_dia / 2.0
    total_depth = config.stock_thickness_mm + 0.15
    num_passes = math.ceil(total_depth / config.stepdown_mm)
    tab_z_threshold = -(config.stock_thickness_mm - config.tab_height_mm)
    
    for idx, it in enumerate(items):
        px, py = it["center"]
        lines.append(f"; Part {idx+1} {it['type']} Contour at ({px:.2f}, {py:.2f})")
        
        if it["type"] == "dogbone":
            half_L = it["spacing"] / 2.0
            r_end = (config.plate_width_mm / 2.0) + r_tool
            waist_y = (config.plate_width_mm / 2.0) * 0.85 + r_tool
            x_tab_l = px - config.tab_width_mm / 2.0
            x_tab_r = px + config.tab_width_mm / 2.0
            
            lines.append(f"G0 X{px - half_L:.3f} Y{py + r_end:.3f} F{config.feed_rapid:.0f}")
            lines.append(f"G0 Z{config.z_retract_mm:.3f}")
            
            for pass_idx in range(1, num_passes + 1):
                z_cut = -min(pass_idx * config.stepdown_mm, total_depth)
                lines.append(f"; Pass {pass_idx}/{num_passes} at Z={z_cut:.3f}")
                lines.append(f"G1 Z{z_cut:.3f} F{config.feed_plunge_profile:.0f}")
                
                if z_cut < tab_z_threshold:
                    lines.append(f"G1 X{x_tab_l:.3f} Y{py + waist_y:.3f} F{config.feed_cut_profile:.0f}")
                    lines.append(f"G1 Z{tab_z_threshold:.3f} F{config.feed_plunge_profile:.0f}")
                    lines.append(f"G1 X{x_tab_r:.3f} Y{py + waist_y:.3f} F{config.feed_cut_profile:.0f}")
                    lines.append(f"G1 Z{z_cut:.3f} F{config.feed_plunge_profile:.0f}")
                else:
                    lines.append(f"G1 X{px + half_L:.3f} Y{py + waist_y:.3f} F{config.feed_cut_profile:.0f}")
                    
                lines.append(f"G1 X{px + half_L:.3f} Y{py + r_end:.3f} F{config.feed_cut_profile:.0f}")
                lines.append(f"G2 X{px + half_L:.3f} Y{py - r_end:.3f} R{r_end:.3f}")
                
                if z_cut < tab_z_threshold:
                    lines.append(f"G1 X{x_tab_r:.3f} Y{py - waist_y:.3f} F{config.feed_cut_profile:.0f}")
                    lines.append(f"G1 Z{tab_z_threshold:.3f} F{config.feed_plunge_profile:.0f}")
                    lines.append(f"G1 X{x_tab_l:.3f} Y{py - waist_y:.3f} F{config.feed_cut_profile:.0f}")
                    lines.append(f"G1 Z{z_cut:.3f} F{config.feed_plunge_profile:.0f}")
                else:
                    lines.append(f"G1 X{px - half_L:.3f} Y{py - waist_y:.3f} F{config.feed_cut_profile:.0f}")
                    
                lines.append(f"G1 X{px - half_L:.3f} Y{py - r_end:.3f} F{config.feed_cut_profile:.0f}")
                lines.append(f"G2 X{px - half_L:.3f} Y{py + r_end:.3f} R{r_end:.3f}")
                
            lines.append(f"G0 Z{config.z_retract_mm:.3f}")
            lines.append("")
            
        elif it["type"] == "trilobe":
            R_arm = it["radius"]
            r_lobe = (config.plate_width_mm / 2.0) + r_tool
            # 3-lobe contour
            lines.append(f"G0 X{px + (R_arm + r_lobe):.3f} Y{py:.3f} F{config.feed_rapid:.0f}")
            lines.append(f"G0 Z{config.z_retract_mm:.3f}")
            
            for pass_idx in range(1, num_passes + 1):
                z_cut = -min(pass_idx * config.stepdown_mm, total_depth)
                lines.append(f"; Pass {pass_idx}/{num_passes} at Z={z_cut:.3f}")
                lines.append(f"G1 Z{z_cut:.3f} F{config.feed_plunge_profile:.0f}")
                
                for ang in [0, 120, 240]:
                    rad = math.radians(ang)
                    ax = px + R_arm * math.cos(rad)
                    ay = py + R_arm * math.sin(rad)
                    lines.append(f"G1 X{ax + r_lobe * math.cos(rad):.3f} Y{ay + r_lobe * math.sin(rad):.3f} F{config.feed_cut_profile:.0f}")
                    
                lines.append(f"G1 X{px + (R_arm + r_lobe):.3f} Y{py:.3f} F{config.feed_cut_profile:.0f}")
                
            lines.append(f"G0 Z{config.z_retract_mm:.3f}")
            lines.append("")
            
    return lines


def main():
    parser = argparse.ArgumentParser(description="Duet 2 CNC G-code Generator for Dual-Ball Dogbones & Tri-Lobe Knuckle Clamps")
    parser.add_argument("--style", type=str, default="mixed", choices=["dogbone", "trilobe", "mixed"], help="Clamp style (default: mixed)")
    parser.add_argument("--pairs", type=int, default=4, help="Number of clamp plate pairs to machine (default: 4)")
    parser.add_argument("--ball-dia", type=float, default=4.8, help="Ball stud diameter in mm (default: 4.8 for Yeah Racing YA-0562)")
    parser.add_argument("--spacing", type=float, default=15.0, help="Ball pocket center-to-center distance in mm (default: 15.0)")
    parser.add_argument("--thick", type=float, default=2.0, help="Stock aluminum sheet thickness in mm (default: 2.0)")
    parser.add_argument("--screw", type=str, default="M2.5", choices=["M2", "M2.5", "M3"], help="Center clamp screw size (default: M2.5)")
    parser.add_argument("--outdir", type=str, default="out/cnc", help="Output directory for G-code files")
    args = parser.parse_args()

    os.makedirs(args.outdir, exist_ok=True)
    
    cfg = CNCConfig(
        style=args.style,
        num_pairs=args.pairs,
        ball_diameter_mm=args.ball_dia,
        ball_spacing_mm=args.spacing,
        stock_thickness_mm=args.thick,
        pocket_depth_mm=1.10 if args.ball_dia <= 5.0 else 1.30,
        plate_width_mm=7.0 if args.ball_dia <= 5.0 else 8.0,
        tool_ball_dia=args.ball_dia,
        screw_type=args.screw
    )
    
    items = get_layout(cfg)
    
    # 1. Combined G-code master job
    combined_lines = []
    combined_lines.extend(format_header(f"Complete Clamping Plate Suite ({args.style.upper()})", cfg))
    
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
        
    print(f"Generated CNC G-code files in {args.outdir}/:")
    print(f"  • {combined_path} ({len(combined_lines)} lines)")
    print(f"Total Parts: {len(items)} plates ({len([i for i in items if i['role'] == 'top_clearance'])} matching pairs)")


if __name__ == "__main__":
    main()

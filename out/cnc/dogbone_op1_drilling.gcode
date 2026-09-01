; ========================================================
; ExoKey Dual-Ball Armature Clamp Plate Generator
; Job: Op 1 - Center Hole Peck Drilling
; Machine: Duet 2 CNC (RepRapFirmware)
; Stock Thickness: 2.00 mm Aluminum
; Ball Spacing: 15.0 mm C-C | Pocket Depth: 1.10 mm
; Total Parts: 8 plates (4 pairs)
; ========================================================
G21            ; Metric units (mm)
G90            ; Absolute positioning
G94            ; Feed rate units mm/min
G17            ; XY plane selection
G0 Z5.00 F2000 ; Retract to safe Z
M3 S18000   ; Spindle ON (18000 RPM)
G4 P1500       ; Wait 1.5s for spindle to reach full speed

; --------------------------------------------------------
; OPERATION 1: Center Clamping Screw Holes (Peck Drilling)
; Tool: Center / Twist Drill
; --------------------------------------------------------
; Part 1: top_clearance at (7.00, 7.00)
G0 X7.000 Y7.000 F2000
G0 Z1.500
G1 Z-0.800 F120
G0 Z1.500 ; Full chip clearing retract
G0 Z-0.600 ; Rapid back to cut level
G1 Z-1.600 F120
G0 Z1.500 ; Full chip clearing retract
G0 Z-1.400 ; Rapid back to cut level
G1 Z-2.400 F120
G0 Z1.500 ; Full chip clearing retract
G0 Z-2.200 ; Rapid back to cut level
G1 Z-2.500 F120
G0 Z1.500 ; Full chip clearing retract
G0 Z1.500

; Part 2: bottom_tap at (7.00, 20.00)
G0 X7.000 Y20.000 F2000
G0 Z1.500
G1 Z-0.800 F120
G0 Z1.500 ; Full chip clearing retract
G0 Z-0.600 ; Rapid back to cut level
G1 Z-1.600 F120
G0 Z1.500 ; Full chip clearing retract
G0 Z-1.400 ; Rapid back to cut level
G1 Z-2.400 F120
G0 Z1.500 ; Full chip clearing retract
G0 Z-2.200 ; Rapid back to cut level
G1 Z-2.500 F120
G0 Z1.500 ; Full chip clearing retract
G0 Z1.500

; Part 3: top_clearance at (33.00, 7.00)
G0 X33.000 Y7.000 F2000
G0 Z1.500
G1 Z-0.800 F120
G0 Z1.500 ; Full chip clearing retract
G0 Z-0.600 ; Rapid back to cut level
G1 Z-1.600 F120
G0 Z1.500 ; Full chip clearing retract
G0 Z-1.400 ; Rapid back to cut level
G1 Z-2.400 F120
G0 Z1.500 ; Full chip clearing retract
G0 Z-2.200 ; Rapid back to cut level
G1 Z-2.500 F120
G0 Z1.500 ; Full chip clearing retract
G0 Z1.500

; Part 4: bottom_tap at (33.00, 20.00)
G0 X33.000 Y20.000 F2000
G0 Z1.500
G1 Z-0.800 F120
G0 Z1.500 ; Full chip clearing retract
G0 Z-0.600 ; Rapid back to cut level
G1 Z-1.600 F120
G0 Z1.500 ; Full chip clearing retract
G0 Z-1.400 ; Rapid back to cut level
G1 Z-2.400 F120
G0 Z1.500 ; Full chip clearing retract
G0 Z-2.200 ; Rapid back to cut level
G1 Z-2.500 F120
G0 Z1.500 ; Full chip clearing retract
G0 Z1.500

; Part 5: top_clearance at (59.00, 7.00)
G0 X59.000 Y7.000 F2000
G0 Z1.500
G1 Z-0.800 F120
G0 Z1.500 ; Full chip clearing retract
G0 Z-0.600 ; Rapid back to cut level
G1 Z-1.600 F120
G0 Z1.500 ; Full chip clearing retract
G0 Z-1.400 ; Rapid back to cut level
G1 Z-2.400 F120
G0 Z1.500 ; Full chip clearing retract
G0 Z-2.200 ; Rapid back to cut level
G1 Z-2.500 F120
G0 Z1.500 ; Full chip clearing retract
G0 Z1.500

; Part 6: bottom_tap at (59.00, 20.00)
G0 X59.000 Y20.000 F2000
G0 Z1.500
G1 Z-0.800 F120
G0 Z1.500 ; Full chip clearing retract
G0 Z-0.600 ; Rapid back to cut level
G1 Z-1.600 F120
G0 Z1.500 ; Full chip clearing retract
G0 Z-1.400 ; Rapid back to cut level
G1 Z-2.400 F120
G0 Z1.500 ; Full chip clearing retract
G0 Z-2.200 ; Rapid back to cut level
G1 Z-2.500 F120
G0 Z1.500 ; Full chip clearing retract
G0 Z1.500

; Part 7: top_clearance at (85.00, 7.00)
G0 X85.000 Y7.000 F2000
G0 Z1.500
G1 Z-0.800 F120
G0 Z1.500 ; Full chip clearing retract
G0 Z-0.600 ; Rapid back to cut level
G1 Z-1.600 F120
G0 Z1.500 ; Full chip clearing retract
G0 Z-1.400 ; Rapid back to cut level
G1 Z-2.400 F120
G0 Z1.500 ; Full chip clearing retract
G0 Z-2.200 ; Rapid back to cut level
G1 Z-2.500 F120
G0 Z1.500 ; Full chip clearing retract
G0 Z1.500

; Part 8: bottom_tap at (85.00, 20.00)
G0 X85.000 Y20.000 F2000
G0 Z1.500
G1 Z-0.800 F120
G0 Z1.500 ; Full chip clearing retract
G0 Z-0.600 ; Rapid back to cut level
G1 Z-1.600 F120
G0 Z1.500 ; Full chip clearing retract
G0 Z-1.400 ; Rapid back to cut level
G1 Z-2.400 F120
G0 Z1.500 ; Full chip clearing retract
G0 Z-2.200 ; Rapid back to cut level
G1 Z-2.500 F120
G0 Z1.500 ; Full chip clearing retract
G0 Z1.500


; ========================================================
; Job Completion & Safe Park
; ========================================================
G0 Z20.00 F2000 ; Retract to high safe Z
M5             ; Spindle Stop
M9             ; Coolant/Air blast OFF
G0 X0 Y100     ; Park table forward for part removal
M84 S0         ; Keep steppers engaged or idle
; End of Program
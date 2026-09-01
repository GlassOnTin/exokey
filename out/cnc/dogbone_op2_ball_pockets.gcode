; ========================================================
; ExoKey Dual-Ball Armature Clamp Plate Generator
; Job: Op 2 - Spherical Ball Pockets
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
; OPERATION 2: Dual Spherical Ball Pockets (1.3mm depth)
; Tool: ⌀ 6.0mm Ball Nose or 90° Chamfer Tool
; --------------------------------------------------------
; Part 1 Ball Pockets at X=7.00, Y=7.00
; Pocket 1 at (-0.500, 7.000)
G0 X-0.500 Y7.000 F2000
G0 Z1.500
G1 Z-0.400 F80
G1 Z-0.800 F80
G1 Z-1.100 F80
G4 P350 ; Dwell 350ms for smooth spherical finish
G0 Z1.500
; Pocket 2 at (14.500, 7.000)
G0 X14.500 Y7.000 F2000
G0 Z1.500
G1 Z-0.400 F80
G1 Z-0.800 F80
G1 Z-1.100 F80
G4 P350 ; Dwell 350ms for smooth spherical finish
G0 Z1.500

; Part 2 Ball Pockets at X=7.00, Y=20.00
; Pocket 1 at (-0.500, 20.000)
G0 X-0.500 Y20.000 F2000
G0 Z1.500
G1 Z-0.400 F80
G1 Z-0.800 F80
G1 Z-1.100 F80
G4 P350 ; Dwell 350ms for smooth spherical finish
G0 Z1.500
; Pocket 2 at (14.500, 20.000)
G0 X14.500 Y20.000 F2000
G0 Z1.500
G1 Z-0.400 F80
G1 Z-0.800 F80
G1 Z-1.100 F80
G4 P350 ; Dwell 350ms for smooth spherical finish
G0 Z1.500

; Part 3 Ball Pockets at X=33.00, Y=7.00
; Pocket 1 at (25.500, 7.000)
G0 X25.500 Y7.000 F2000
G0 Z1.500
G1 Z-0.400 F80
G1 Z-0.800 F80
G1 Z-1.100 F80
G4 P350 ; Dwell 350ms for smooth spherical finish
G0 Z1.500
; Pocket 2 at (40.500, 7.000)
G0 X40.500 Y7.000 F2000
G0 Z1.500
G1 Z-0.400 F80
G1 Z-0.800 F80
G1 Z-1.100 F80
G4 P350 ; Dwell 350ms for smooth spherical finish
G0 Z1.500

; Part 4 Ball Pockets at X=33.00, Y=20.00
; Pocket 1 at (25.500, 20.000)
G0 X25.500 Y20.000 F2000
G0 Z1.500
G1 Z-0.400 F80
G1 Z-0.800 F80
G1 Z-1.100 F80
G4 P350 ; Dwell 350ms for smooth spherical finish
G0 Z1.500
; Pocket 2 at (40.500, 20.000)
G0 X40.500 Y20.000 F2000
G0 Z1.500
G1 Z-0.400 F80
G1 Z-0.800 F80
G1 Z-1.100 F80
G4 P350 ; Dwell 350ms for smooth spherical finish
G0 Z1.500

; Part 5 Ball Pockets at X=59.00, Y=7.00
; Pocket 1 at (51.500, 7.000)
G0 X51.500 Y7.000 F2000
G0 Z1.500
G1 Z-0.400 F80
G1 Z-0.800 F80
G1 Z-1.100 F80
G4 P350 ; Dwell 350ms for smooth spherical finish
G0 Z1.500
; Pocket 2 at (66.500, 7.000)
G0 X66.500 Y7.000 F2000
G0 Z1.500
G1 Z-0.400 F80
G1 Z-0.800 F80
G1 Z-1.100 F80
G4 P350 ; Dwell 350ms for smooth spherical finish
G0 Z1.500

; Part 6 Ball Pockets at X=59.00, Y=20.00
; Pocket 1 at (51.500, 20.000)
G0 X51.500 Y20.000 F2000
G0 Z1.500
G1 Z-0.400 F80
G1 Z-0.800 F80
G1 Z-1.100 F80
G4 P350 ; Dwell 350ms for smooth spherical finish
G0 Z1.500
; Pocket 2 at (66.500, 20.000)
G0 X66.500 Y20.000 F2000
G0 Z1.500
G1 Z-0.400 F80
G1 Z-0.800 F80
G1 Z-1.100 F80
G4 P350 ; Dwell 350ms for smooth spherical finish
G0 Z1.500

; Part 7 Ball Pockets at X=85.00, Y=7.00
; Pocket 1 at (77.500, 7.000)
G0 X77.500 Y7.000 F2000
G0 Z1.500
G1 Z-0.400 F80
G1 Z-0.800 F80
G1 Z-1.100 F80
G4 P350 ; Dwell 350ms for smooth spherical finish
G0 Z1.500
; Pocket 2 at (92.500, 7.000)
G0 X92.500 Y7.000 F2000
G0 Z1.500
G1 Z-0.400 F80
G1 Z-0.800 F80
G1 Z-1.100 F80
G4 P350 ; Dwell 350ms for smooth spherical finish
G0 Z1.500

; Part 8 Ball Pockets at X=85.00, Y=20.00
; Pocket 1 at (77.500, 20.000)
G0 X77.500 Y20.000 F2000
G0 Z1.500
G1 Z-0.400 F80
G1 Z-0.800 F80
G1 Z-1.100 F80
G4 P350 ; Dwell 350ms for smooth spherical finish
G0 Z1.500
; Pocket 2 at (92.500, 20.000)
G0 X92.500 Y20.000 F2000
G0 Z1.500
G1 Z-0.400 F80
G1 Z-0.800 F80
G1 Z-1.100 F80
G4 P350 ; Dwell 350ms for smooth spherical finish
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
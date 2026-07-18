// ExoKey silicone dome mold -- COUPON.md. Cast a soft dome (the native keypad-rubber technology).
// Cavity (bottom) + core (top). Pour two-part silicone into the cavity, press the core (pins align
// it), clamp, cure, demold, trim flash. The wall t is set by the mold GAP, not by print luck.
// Export:  -D 'part="cavity"'  /  -D 'part="core"' .  "section" shows the seated pair, cut, to check t.
part = "both";          // "cavity" | "core" | "both" | "section"

a     = 6;              // dome base radius (mm)
h     = 2;              // dome rise
t     = 0.6;            // shell wall -- silicone, thicker than TPU is fine
rim_w = 3;              // skirt width
rim_d = 1.2;            // skirt thickness
D     = 34;             // mold outer diameter
Hc    = 12;             // cavity height
Ht    = 9;              // core body height
pin_d = 2.0;            // alignment pins
vent  = 1.2;            // apex air-escape / fill hole
$fn   = 160;

Rout = (a*a + h*h) / (2*h);     // outer (dome-convex) sphere radius
zc   = Rout - h;                // its centre, above the rim plane z=0
Rin  = Rout - t;                // inner sphere -> uniform normal wall t
pinx = D/2 - 4;

module cavity() {
  difference() {
    cylinder(h = Hc, d = D);
    translate([0,0, Hc + zc]) sphere(r = Rout);                          // dome-outer bowl at the top
    translate([0,0, Hc - rim_d]) cylinder(h = rim_d + 1, d = 2*(a+rim_w)); // annular skirt recess
    for (s = [-1,1]) translate([s*pinx, 0, Hc - 4]) cylinder(h = 5, d = pin_d + 0.25); // pin holes
  }
}

// core in PRINT orientation: flat seating face at z=0, bump protruding DOWN (-z), body up (+z).
module core() {
  union() {
    difference() {
      union() {
        cylinder(h = Ht, d = D);                                        // full-width presser (covers the pins)
        intersection() {                                                // dome-inner bump, protruding down
          translate([0,0, zc]) sphere(r = Rin);
          translate([0,0, -(h+2)]) cylinder(h = h + 2, d = D);          // keep only the z<0 cap
        }
      }
      translate([0,0, -(h+2)]) cylinder(h = Ht + h + 4, d = vent);      // apex vent up the axis
    }
    for (s = [-1,1]) translate([s*pinx, 0, -3.8]) cylinder(h = 3.8, d = pin_d);  // alignment pins, point down
  }
}

if (part == "cavity") cavity();
if (part == "core")   core();
if (part == "both") { cavity(); translate([D + 6, 0, 0]) core(); }
if (part == "section")                                                  // seated, cut on +x to reveal the t-gap
  difference() {
    union() { cavity(); translate([0,0, Hc]) core(); }
    translate([0, -D, -5]) cube([D, 2*D, Hc + Ht + 10]);
  }

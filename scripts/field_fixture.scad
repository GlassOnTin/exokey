// ExoKey field-map fixture -- COUPON.md tests T1 (plunge) / T3 (lateral).
// Fix the Hall, slide the magnet over it in X (a caliper sets X), hold the gap; log (Bx,By,Bz).
// Export one part at a time:  openscad -D 'part="base"'  /  -D 'part="sled"'
part = "both";          // "base" | "sled" | "both"

gap     = 3.5;          // magnet face -> Hall chip (mm). VERIFY with a feeler gauge, tune to your board.
hall_l  = 15;           // Hall breakout pocket L,W,depth -- SET to your breakout.
hall_w  = 13;
hall_d  = 2.0;
mag_d   = 3.0;          // magnet diameter
mag_t   = 1.0;          // magnet thickness
sled_w  = 12;
sled_l  = 16;
sled_h  = gap + mag_t + 2;
wall    = 2;
clr     = 0.3;
railz   = 5;
base_l  = 60; base_w = 30; base_h = 6;
chan_w  = sled_w + 2*clr;
$fn = 72;

module hall_base() {
  difference() {
    union() {
      cube([base_l, base_w, base_h]);
      translate([0, base_w/2 + chan_w/2,        base_h]) cube([base_l, wall, railz]);  // +y guide rail
      translate([0, base_w/2 - chan_w/2 - wall, base_h]) cube([base_l, wall, railz]);  // -y guide rail
      translate([0, base_w/2 - chan_w/2 - wall, base_h]) cube([wall, chan_w + 2*wall, railz]); // x=0 datum stop
    }
    translate([base_l/2 - hall_l/2, base_w/2 - hall_w/2, base_h - hall_d])
      cube([hall_l, hall_w, hall_d + 0.3]);                                            // Hall pocket
    translate([base_l/2 - 2, base_w/2 + hall_w/2, base_h - hall_d])
      cube([4, base_w/2, hall_d + 0.3]);                                               // wire slot out +y
  }
}

module magnet_sled() {
  difference() {
    cube([sled_l, sled_w, sled_h]);
    translate([sled_l/2, sled_w/2, gap])  cylinder(h = sled_h, d = mag_d + 0.15);      // magnet slot (top-load, rests at z=gap)
    translate([sled_l/2, sled_w/2, -0.1]) cylinder(h = gap + 0.2, d = mag_d - 0.8);    // window to the Hall
  }
}

if (part == "base" || part == "both") hall_base();
if (part == "sled" || part == "both") translate([base_l/2 - sled_l/2, -sled_w - 6, 0]) magnet_sled();

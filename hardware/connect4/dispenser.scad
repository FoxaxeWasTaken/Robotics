// dispenser.scad - the token dispenser as the simulation builds it (so101_bringup/sim_scene.py,
// _add_dispenser; ADR-041): three straight rails stacked in a vertical plane, sloping 25 degrees. The tokens
// stand on edge between two walls and slide down to a stop; the lowest place of a rail is its pick station,
// where the walls are low: the open jaws go 5 mm under the token's centre.
//
// Frame: the dispenser frame of so101_connect4/layout.py. Origin at the centre of the token waiting at the
// lowest station, x outwards from the arm, z up; the tokens stand in the x-z plane.
//
// Parts (set `part`, then render and export): "rail" (three times, the same part: its two skirts stand on the
// walls of the rail below, a notch on the nub at that rail's end), "stand" (the lowest rail sits astride it),
// "stand_foot" (twice), "marker_plate", "leg" (twice), "assembly" (a view of the whole, not for printing).
include <dimensions.scad>

part = "assembly";
clearance = 0.3;   // mm between two printed parts that slide into each other
stop_brace = 0;    // mm: the stop's fin made longer towards the arm. 0 is the simulation's fin, 3 mm long
                   // and 2 mm wide: fragile in print, but a longer one was never tried with the gripper
plate_t = 2;       // mm, the marker plate
leg = 12;          // mm, a leg's side
$fn = 48;

r = token_diameter / 2;
gap = dispenser_slot_gap;
wall = dispenser_wall;
start = -r - 3;                                                     // the stop's back, along the rail
cut = r + dispenser_rail_wall_height * tan(dispenser_slope) + 2;    // where the station's low walls end [MES-77]
end = (dispenser_tokens_per_rail - 1) * token_diameter + r + 4;
floor_z = -r - dispenser_floor;                                     // a rail's underside, off its tokens' centre
// From a rail to the next one up, along the rail and across it (layout.station)
up = (dispenser_station_step + dispenser_rail_pitch * sin(dispenser_slope)) / cos(dispenser_slope);
lift = dispenser_rail_pitch - (r + dispenser_floor) - (dispenser_rail_wall_height - r);  // between a wall's top and the floor above
table = -dispenser_placement_station_z - placement_riser;           // the table's level in the dispenser frame
nub = 2;                                                            // mm along a rail: what places a rail on the one below
nub_h = 3;

module tilted() { rotate([0, -dispenser_slope, 0]) children(); }    // x up the rail, z off its floor

module rail() {  // in the rail's own frame
  // the floor, and the stop: a fin in the tokens' plane, the jaws close on either side of it
  translate([start - stop_brace, -gap / 2 - wall, floor_z]) cube([end - start + stop_brace, gap + 2 * wall, dispenser_floor]);
  translate([start - stop_brace, dispenser_stop_offset - dispenser_stop_width / 2, -r])
    cube([3 + stop_brace, dispenser_stop_width, token_diameter]);
  for (side = [1, -1]) translate([0, side * (gap + wall) / 2 - wall / 2, 0]) {
    translate([start, 0, -r]) cube([cut - start, wall, dispenser_station_wall_height]);   // low at the station
    translate([cut, 0, -r]) cube([end - cut, wall, dispenser_rail_wall_height]);          // high behind it
    // a nub at the wall's end, for the notch of the rail above
    translate([end - nub, 0, -r + dispenser_rail_wall_height]) cube([nub, wall, nub_h]);
    // a skirt: it stands on the wall of the rail below, its notch on that wall's nub; the lowest rail's
    // skirts hang on either side of the stand
    difference() {
      translate([start, 0, floor_z - lift]) cube([end - start, wall, lift]);
      translate([end - up - nub - clearance, -1, floor_z - lift - 1]) cube([nub + 2 * clearance, wall + 2, nub_h + clearance + 1]);
    }
  }
}

function station(k) = k * [up * cos(dispenser_slope) - dispenser_rail_pitch * sin(dispenser_slope), 0,
                           up * sin(dispenser_slope) + dispenser_rail_pitch * cos(dispenser_slope)];
function world(x, z) = [x * cos(dispenser_slope) - z * sin(dispenser_slope), x * sin(dispenser_slope) + z * cos(dispenser_slope)];

beam = gap - 2 * clearance;                 // the stand's thickness: it fits between a rail's two skirts
a = world(start, floor_z);                  // under the lowest rail's front end
b = world(end - up, floor_z);               // and where its tab hooks
arm_x = -dispenser_markers_plate - 6 - 20;  // the arm that carries the marker plate reaches 20 mm under it
plate_top = -dispenser_markers_below;

module stand_outline() {
  polygon([a, b, [b[0], table], [a[0], table]]);
  // the arm under the marker plate, and its foot down to the lowest rail's front
  polygon([[arm_x, plate_top - plate_t], [a[0], plate_top - plate_t], [a[0] + 12, a[1] - 8], [a[0], a[1] - 22],
           [arm_x, plate_top - plate_t - 10]]);
}

module stand() {  // a frame in the tokens' plane; two half-lap notches for its feet
  translate([0, beam / 2, 0]) rotate([90, 0, 0]) linear_extrude(beam) difference() {
    stand_outline();
    offset(delta = -12) polygon([a, b, [b[0], table], [a[0], table]]);   // lightened
    for (x = [a[0] + 20, b[0] - 20]) translate([x - 3 - clearance, table - 1]) square([6 + 2 * clearance, 11]);
  }
  // on one face, the nub for the notch of the lowest rail's skirt
  tilted() translate([end - up - nub, beam / 2, floor_z - lift]) cube([nub, gap / 2 + wall - beam / 2, nub_h]);
}

module stand_foot() {  // a bar across the stand, on edge, with the notch that takes the stand
  difference() {
    translate([-3, -60, 0]) cube([6, 120, 20]);
    translate([-4, -beam / 2 - clearance, 10]) cube([8, beam + 2 * clearance, 11]);
  }
}

// The marker plate (layout.dispenser_marker_centers): flat, facing up, before the lowest station, on the
// camera's side; four markers at its corners.
mx = -dispenser_markers_plate - 6;
my = -dispenser_markers_aside;
plate_lo = [mx - dispenser_markers_span[0] - dispenser_markers_plate / 2, my - dispenser_markers_span[1] - dispenser_markers_plate / 2];
plate_hi = [mx + dispenser_markers_plate / 2, gap / 2 + wall];
leg_at = [for (x = [plate_lo[0] + 12, plate_hi[0] - 12]) [x, plate_lo[1] + 12]];

module marker_plate() {
  difference() {
    translate([plate_lo[0], plate_lo[1], plate_top - plate_t]) cube([plate_hi[0] - plate_lo[0], plate_hi[1] - plate_lo[1], plate_t]);
    for (dx = [0, dispenser_markers_span[0]], dy = [0, dispenser_markers_span[1]])   // the markers' seats, 0.3 mm
      translate([mx - dx - dispenser_markers_plate / 2, my - dy - dispenser_markers_plate / 2, plate_top - 0.3]) difference() {
        cube([dispenser_markers_plate, dispenser_markers_plate, 1]);
        translate([0.5, 0.5, -1]) cube([dispenser_markers_plate - 1, dispenser_markers_plate - 1, 3]);
      }
  }
  // under it: two ribs astride the stand's arm, and a socket for each leg
  for (side = [1, -1]) translate([arm_x, side * (beam / 2 + clearance + 1) - 1, plate_top - plate_t - 6])
    cube([plate_hi[0] - arm_x, 2, 6]);
  for (at = leg_at) translate([at[0], at[1], plate_top - plate_t - 8]) difference() {
    translate([-leg / 2 - 2, -leg / 2 - 2, 0]) cube([leg + 4, leg + 4, 8]);
    translate([-leg / 2 - clearance, -leg / 2 - clearance, -1]) cube([leg + 2 * clearance, leg + 2 * clearance, 10]);
  }
}

module leg() { cube([leg, leg, plate_top - plate_t - table]); }

if (part == "rail") translate([0, 0, r + dispenser_floor + lift]) rail();        // on its skirts: the floor bridges 9.5 mm
else if (part == "stand") rotate([90, 0, 0]) stand();                            // flat, its nub up
else if (part == "stand_foot") stand_foot();
else if (part == "marker_plate") rotate([180, 0, 0]) marker_plate();             // its top face on the bed
else if (part == "leg") rotate([0, 90, 0]) leg();
else {
  for (k = [0 : dispenser_rails - 1]) translate(station(k)) tilted() {
    color("gainsboro") rail();
    color("gold") for (i = [0 : dispenser_tokens_per_rail - 1])
      translate([i * token_diameter, token_thickness / 2, 0]) rotate([90, 0, 0]) cylinder(d = token_diameter, h = token_thickness);
  }
  color("silver") stand();
  color("gray") for (x = [a[0] + 20, b[0] - 20]) translate([x, 0, table]) stand_foot();
  color("white") marker_plate();
  color("gray") for (at = leg_at) translate([at[0] - leg / 2, at[1] - leg / 2, table]) leg();
}

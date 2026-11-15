// grid.scad - the Connect Four grid as the simulation builds it (so101_bringup/sim_scene.py, _add_grid):
// two plates with round windows, the walls between the columns, the bar the tokens land on, the two side
// posts that carry the markers, the depth guide above the columns.
//
// Frame: the grid frame of so101_connect4/layout.py. Origin at the centre of cell (column 0, row 0) in the
// slots' mid-plane, x along the columns, z up, +y the front face (the camera's side).
//
// Parts (set `part`, then render and export): "back" (a plate with the walls and the floor on it), "front"
// (a plate), "post" (twice: the far one is its mirror, `far = true`), "foot" (twice), "guide" (twice, the
// same part on both faces), "assembly" (a view of the whole, not for printing).
// A plate is 275 x 212 mm: on a smaller bed print it in two pieces with `first` and `last` (the columns
// kept, 0 to 6), for instance 0-3 then 4-6; the cut is in the middle of a wall.
include <dimensions.scad>

part = "assembly";
first = 0;
last = grid_columns - 1;
far = false;       // "post": the post beyond the last column
tenon = 8;         // mm of each plate over a post's tongue
clearance = 0.3;   // mm between two printed parts that slide into each other
floor_t = 6;       // mm, the bar the tokens land on (the sim's slot_floor)
foot_length = 120; // mm across the grid; check the near foot against the arm's riser before printing
$fn = 72;

length = grid_columns * grid_column_pitch;
x0 = -grid_column_pitch / 2;                    // the first column's outer wall
z0 = -grid_row_pitch / 2;                       // the slots' floor
height = grid_rows * grid_row_pitch + grid_headroom;
entry = z0 + height;                            // the slot entries
table = entry - grid_slot_entry_height;         // the table's level in the grid frame
depth = grid_slot_gap + 2 * grid_plate;
edge = markers_plate / 2 + 2;                   // layout.marker_centers

module plate(front) {
  y = front ? grid_slot_gap / 2 : -grid_slot_gap / 2 - grid_plate;
  difference() {
    translate([x0 - tenon, y, z0 - floor_t]) cube([length + 2 * tenon, grid_plate, height + floor_t]);
    for (c = [0 : grid_columns - 1], r = [0 : grid_rows - 1])
      translate([c * grid_column_pitch, y - 1, r * grid_row_pitch])
        rotate([-90, 0, 0]) cylinder(d = grid_window_diameter, h = grid_plate + 2);
  }
}

module ribs() {  // the walls between the columns and the floor, as deep as the slot
  for (i = [0 : grid_columns])
    translate([x0 + i * grid_column_pitch - grid_wall / 2, -grid_slot_gap / 2, z0])
      cube([grid_wall, grid_slot_gap, height]);
  translate([x0, -grid_slot_gap / 2, z0 - floor_t]) cube([length, grid_slot_gap, floor_t]);
}

module columns_kept() {  // the columns first..last of a part that runs along the grid
  xa = first == 0 ? x0 - tenon - 1 : x0 + first * grid_column_pitch;
  xb = last == grid_columns - 1 ? x0 + length + tenon + 1 : x0 + (last + 1) * grid_column_pitch;
  intersection() {
    children();
    translate([xa, -100, table - 1]) cube([xb - xa, 200, 400]);
  }
}

module back() { columns_kept() { plate(false); ribs(); } }
module front() { columns_kept() plate(true); }

module marker_seat(z) {  // the outline of a marker's white square, 0.3 mm deep, on the post's front face
  x = x0 - (grid_frame_margin + markers_plate) / 2;
  translate([x, depth / 2 - 0.3, z - markers_plate / 2]) difference() {
    cube([markers_plate, 1, markers_plate]);
    translate([0.5, -1, 0.5]) cube([markers_plate - 1, 3, markers_plate - 1]);
  }
}

module near_post() {
  difference() {
    union() {
      // the body, from the table to the slot entries
      translate([x0 - grid_frame_margin, -depth / 2, table]) cube([grid_frame_margin - tenon, depth, entry - table]);
      // under the plates, up to the first column: what the plates stand on
      translate([x0 - tenon, -depth / 2, table]) cube([tenon, depth, z0 - floor_t - table]);
      // the tongue between the two plates
      translate([x0 - tenon, -grid_slot_gap / 2 + clearance, z0 - floor_t])
        cube([tenon - grid_wall / 2 - clearance, grid_slot_gap - 2 * clearance, height + floor_t]);
    }
    marker_seat(entry - edge);  // layout.marker_centers: the top marker
    marker_seat(table + edge);  // and the bottom one
  }
}

module post() {
  if (far) translate([2 * x0 + length, 0, 0]) mirror([1, 0, 0]) near_post();
  else near_post();
}

module foot() {  // a collar around a post's bottom with a flange across the grid; the post still touches the table
  w = grid_frame_margin + 2 * clearance;
  d = depth + 2 * clearance;
  difference() {
    union() {
      translate([-w / 2 - 3, -foot_length / 2, 0]) cube([w + 6, foot_length, 4]);
      translate([-w / 2 - 3, -d / 2 - 3, 0]) cube([w + 6, d + 6, 25]);
    }
    translate([-w / 2, -d / 2, -1]) cube([w, d, 30]);
  }
}

module guide() {  // one strip of the depth guide: it sits on a plate's top edge, a skirt down its outer face
  g = grid_slot_gap / 2;
  o = grid_depth_guide_opening / 2;
  columns_kept()
    translate([x0, 0, entry]) rotate([90, 0, 90]) linear_extrude(length)
      polygon([[g, 0], [o, grid_depth_guide_height], [o + 2, grid_depth_guide_height],
               [g + grid_plate + 1.6, 0], [g + grid_plate + 1.6, -10], [g + grid_plate, -10], [g + grid_plate, 0]]);
}

if (part == "back") rotate([90, 0, 0]) back();            // the plate on the bed, the walls up
else if (part == "front") rotate([-90, 0, 0]) front();
else if (part == "post") rotate([90, 0, 0]) post();       // on its back face, the markers' seats up
else if (part == "foot") foot();
else if (part == "guide") rotate([-90, 0, 0]) guide();    // on its skirt
else {
  color("royalblue") { back(); front(); }
  color("steelblue") { near_post(); translate([2 * x0 + length, 0, 0]) mirror([1, 0, 0]) near_post(); }
  color("lightsteelblue", 0.6) { guide(); mirror([0, 1, 0]) guide(); }
  color("gray") for (x = [x0 - grid_frame_margin / 2, x0 + length + grid_frame_margin / 2])
    translate([x, 0, table]) foot();
}

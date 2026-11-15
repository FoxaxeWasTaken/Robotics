// dimensions.scad - the Connect Four parts' dimensions, in mm (angles in degrees).
// Each value is the one of ros2_ws/src/so101_connect4/config/connect4_grid.yaml (in m and rad there), which
// the simulation is built from; a name is the YAML path, its keys joined by "_".
// tools/ci/check_scad_dimensions.py fails when one differs: change the YAML first, then this file.

grid_columns = 7;
grid_rows = 6;
grid_column_pitch = 37;
grid_row_pitch = 34;
grid_slot_gap = 9.5;
grid_slot_entry_height = 250;
grid_depth_guide_height = 30;
grid_depth_guide_opening = 30;
grid_headroom = 2;
grid_wall = 2.2;
grid_plate = 3;
grid_window_diameter = 29;
grid_frame_margin = 45;

markers_size = 30;
markers_plate = 39;

placement_riser = 100;

token_diameter = 34;
token_thickness = 6.5;

dispenser_rails = 3;
dispenser_tokens_per_rail = 7;
dispenser_slope = 25;
dispenser_slot_gap = 9.5;
dispenser_wall = 2;
dispenser_floor = 3;
dispenser_station_wall_height = 7;
dispenser_rail_wall_height = 30;
dispenser_stop_width = 2;
dispenser_stop_offset = -0.45;
dispenser_rail_pitch = 40;
dispenser_station_step = 40;
dispenser_markers_span = [90, 165];
dispenser_markers_size = 30;
dispenser_markers_plate = 39;
dispenser_markers_aside = 35;
dispenser_markers_below = 20;
dispenser_placement_station_z = 20;

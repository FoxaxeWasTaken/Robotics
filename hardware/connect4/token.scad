// token.scad - a Connect Four token (docs/project/phase2/P2-02 §7): 21 in each of the two colors.
// Print flat, no support. The grid's slots leave it 0.8 mm along a column and 3 mm across: print one, then
// drop it through one column of the grid before printing the 41 others.
include <dimensions.scad>

chamfer = 0.6;  // on both edges: the token slides on the rails and into the depth guide
$fn = 96;

module token() {
  r = token_diameter / 2;
  rotate_extrude()
    polygon([[0, 0], [r - chamfer, 0], [r, chamfer], [r, token_thickness - chamfer],
             [r - chamfer, token_thickness], [0, token_thickness]]);
}

token();

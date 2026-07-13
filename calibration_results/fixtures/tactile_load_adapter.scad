// Parametric load-transfer puck for tactile-skin weight calibration.
// Print upright with the small foot on the build plate.

$fn = 120;

foot_diameter = 8.0;
foot_height = 4.0;
top_diameter = 36.0;
top_height = 4.0;
edge_chamfer = 0.5;

module chamfered_cylinder(diameter, height, chamfer) {
    cylinder(d1=diameter - 2 * chamfer, d2=diameter, h=chamfer);
    translate([0, 0, chamfer])
        cylinder(d=diameter, h=height - 2 * chamfer);
    translate([0, 0, height - chamfer])
        cylinder(d1=diameter, d2=diameter - 2 * chamfer, h=chamfer);
}

union() {
    chamfered_cylinder(foot_diameter, foot_height, edge_chamfer);
    translate([0, 0, foot_height])
        chamfered_cylinder(top_diameter, top_height, edge_chamfer);
}

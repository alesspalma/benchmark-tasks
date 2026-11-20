// End-effector adapter coupon and split cable sleeve gauge, dimensions in mm.
// Export the stock plate profile as DXF in "profile" mode before machining.
part = "plate";  // plate, profile, or sleeve-gauge
$fn = 96;

module cable_aperture(radius, axial = 23) {
    hull() {
        translate([-axial, 0]) circle(r = radius);
        translate([axial, 0]) circle(r = radius);
    }
}

module profile() {
    difference() {
        square([154, 168], center = true);
        translate([0, -40]) cable_aperture(21);
        for (x = [-52, 52], y = [-55, 55])
            translate([x, y]) circle(d = 8.4);
    }
}

if (part == "profile")
    profile();
else if (part == "sleeve-gauge")
    linear_extrude(height = 14) difference() {
        cable_aperture(20.75);
        cable_aperture(18.25);
    }
else difference() {
    linear_extrude(height = 10) profile();
    for (x = [-52, 52], y = [-55, 55])
        translate([x, y, 5]) cylinder(d = 13, h = 5.1);
}

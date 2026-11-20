// Bench anti-backlash sleeve. Print in dimensionally stable nylon or PETG.
// Select "cam", "follower", or "body" and export separately.
part = "cam";
$fn = 90;

bore = 28.3;
outer = 48;
r0 = 16;
r1 = 21;
rise = 1.25;  // mm per radian
segments = 32;

function polar(r, a, z) = [r*cos(a), r*sin(a), z];

module ramp_cell(a, b, z1, z2) {
    polyhedron(
        points = [
            polar(r0, a, 0), polar(r1, a, 0),
            polar(r1, b, 0), polar(r0, b, 0),
            polar(r0, a, z1), polar(r1, a, z1),
            polar(r1, b, z2), polar(r0, b, z2),
        ],
        faces = [[0,1,2,3], [7,6,5], [7,5,4], [0,4,5,1], [1,5,6,2],
                 [2,6,7,3], [3,7,4,0]],
        convexity = 4
    );
}

module cam() {
    difference() {
        union() {
            difference() {
                cylinder(h=25, d=outer);
                translate([0,0,-1]) cylinder(h=27, d=bore);
            }
            // Opposed facing surfaces run on three equal ramps. The 6 degree
            // returns are outside the motion allowed by the limit pin.
            for (i=[0:2])
                translate([0,0,24.99])
                    rotate([0,0,120*i])
                        for (k=[0:segments-1])
                            ramp_cell(114*k/segments, 114*(k+1)/segments,
                                      0.1+rise*114*k/segments*PI/180,
                                      0.1+rise*114*(k+1)/segments*PI/180);
        }
        translate([18,0,-.01]) cylinder(h=4, d=1.8);
        translate([23,0,10]) cube([3,3,17], center=true);
    }
}

module follower() {
    difference() {
        cylinder(h=7, d=44);
        translate([0,0,-1]) cylinder(h=9, d=bore);
        for (i=[0:2])
            rotate([0,0,120*i]) translate([18.5,0,-1]) cylinder(h=9, d=2.6);
        for (i=[0:2])
            rotate([0,0,120*i+60]) translate([19,0,-1]) cylinder(h=9, d=3.4);
    }
}

module body() {
    difference() {
        union() {
            cylinder(h=71, d=44);
            translate([0,0,71]) cylinder(h=25, d=45, $fn=6);
        }
        translate([0,0,-1]) cylinder(h=98, d=bore);
        for (i=[0:2])
            rotate([0,0,120*i+60]) translate([19,0,-1]) cylinder(h=98, d=3.0);
        translate([18,0,-.01]) cylinder(h=5, d=1.8);
    }
}

if (part == "cam") cam();
if (part == "follower") follower();
if (part == "body") body();

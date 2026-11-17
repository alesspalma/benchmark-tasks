// Dimensioned reconstruction of an ordinary stir-bar restrainer grip.
// Millimetres. Original task geometry; not OEM CAD.
// Core pocket: 12 along local X, 10 along Y and 8 along Z (magnetization).
// Print the two halves and encapsulate the magnet permanently; no loose magnet
// or exposed adhesive is used in a trial. Field calibration is required.
$fn=96;
part="assembly"; // "assembly", "upper", "lower", "core"

module exterior() {
    union() {
        cube([80,25,10],center=true);
        translate([0,0,18.5]) cylinder(d=16,h=27,center=true);
        translate([0,0,35.5]) cylinder(d=32,h=7,center=true);
    }
}
module case() {
    difference() {
        exterior();
        cube([12.2,10.2,8.2],center=true);
    }
}
if(part=="assembly") case();
if(part=="upper") intersection() {
    case(); translate([0,0,50]) cube([100,100,100],center=true);
}
if(part=="lower") intersection() {
    case(); translate([0,0,-50]) cube([100,100,100],center=true);
}
if(part=="core") cube([12,10,8],center=true);

// Ordinary T-shaped headphone stand, millimetres; original MIT geometry.
$fn=48;
union(){
 translate([0,0,6])cube([130,200,12],center=true);
 translate([0,0,114.5])cube([12,12,205],center=true);
 translate([0,0,215])cube([60,40,10],center=true);
}

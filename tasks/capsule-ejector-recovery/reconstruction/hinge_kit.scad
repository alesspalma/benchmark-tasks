// M6 bearing and 0..85 degree stop kit. Units:mm; MIT.
part="StopPlate";
$fn=64;
module ArcSlot(){for(a=[0:1:84]) hull(){
translate([15*cos(a),0,-15*sin(a)]) rotate([90,0,0]) cylinder(d=3.4,h=8,center=true);
translate([15*cos(a+1),0,-15*sin(a+1)]) rotate([90,0,0]) cylinder(d=3.4,h=8,center=true);}}
module StopPlate(){difference(){
union(){rotate([90,0,0]) cylinder(d=44,h=4,center=true);translate([0,0,18]) cube([44,4,12],center=true);}
rotate([90,0,0]) cylinder(d=6.2,h=8,center=true);ArcSlot();
for(x=[-16,16]) translate([x,0,20]) rotate([90,0,0]) cylinder(d=3.4,h=8,center=true);}}
module StopArm(){difference(){union(){rotate([90,0,0]) cylinder(d=24,h=4,center=true);translate([7.5,0,0]) cube([15,4,8],center=true);translate([15,0,0]) rotate([90,0,0]) cylinder(d=8,h=4,center=true);}
rotate([90,0,0]) cylinder(d=6.2,h=8,center=true);translate([15,0,0]) rotate([90,0,0]) cylinder(d=3.1,h=8,center=true);
for(x=[-5,5]) translate([x,0,6]) rotate([90,0,0]) cylinder(d=3.4,h=8,center=true);}}
module Spacer(){difference(){rotate([90,0,0]) cylinder(d=10,h=4,center=true);rotate([90,0,0]) cylinder(d=6.2,h=8,center=true);}}
if(part=="StopPlate") StopPlate();
if(part=="StopArm") StopArm();
if(part=="Spacer") Spacer();

// Assembly components for the supplied sampler model. Units:mm; MIT.
use <sampler.scad>
part="RodCore";
$fn=64;
module RodCore(){ union(){
translate([0.0, 0.0, 2.0]) cube([40.0, 40.0, 4.0],center=true);
translate([0.0, 18.5, 10.0]) cube([34.0, 3.0, 20.0],center=true);
translate([-18.5, 0.0, 10.0]) cube([3.0, 40.0, 20.0],center=true);
translate([18.5, 0.0, 10.0]) cube([3.0, 40.0, 20.0],center=true);
translate([0.0, 0.0, 21.5]) cube([40.0, 40.0, 3.0],center=true);
translate([0.0, 0.0, 166.5]) cube([6.0, 6.0, 287.0],center=true);
}}
module Button(){difference(){translate([0,0,310]) cylinder(r=15,h=10,center=true);translate([0,0,307.45]) cube([6.5,6.5,5.1],center=true);}}
module StopCollar(){difference(){translate([0,0,253]) cube([10,10,4],center=true);translate([0,0,253]) cube([6.3,6.3,4.2],center=true);}}
module StrokeSpacer(){difference(){translate([0,0,275]) cylinder(r=7,h=10,center=true);translate([0,0,275]) cylinder(r=6,h=10.2,center=true);}}
if(part=="RodCore") RodCore();
if(part=="Button") Button();
if(part=="StopCollar") StopCollar();
if(part=="StrokeSpacer") StrokeSpacer();
if(part=="OuterLeft") intersection(){Outer();translate([-1000,-1000,-1000]) cube([1000,2000,2000]);}
if(part=="OuterRight") intersection(){Outer();translate([0,-1000,-1000]) cube([1000,2000,2000]);}

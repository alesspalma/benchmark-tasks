// Internal stop-pin clearance for the printed housing. Units:mm; MIT.
use <coffee_machine.scad>
part="BasePhysical";
hinge=[50,0,235];
$fn=64;
module StopPinWindow(){
  for(a=[0:1:84]) hull(){
    translate(hinge+[15*cos(a),-96,-15*sin(a)]) rotate([90,0,0]) cylinder(d=4,h=22,center=true);
    translate(hinge+[15*cos(a+1),-96,-15*sin(a+1)]) rotate([90,0,0]) cylinder(d=4,h=22,center=true);
  }
}
module BasePhysical(){difference(){Base();StopPinWindow();}}
module BaseFrontPhysical(){intersection(){BasePhysical();translate([-1000,-1000,-1000]) cube([1000,2000,2000]);}}
module BaseRearPhysical(){intersection(){BasePhysical();translate([0,-1000,-1000]) cube([1000,2000,2000]);}}
if(part=="BasePhysical") BasePhysical();
if(part=="BaseFrontPhysical") BaseFrontPhysical();
if(part=="BaseRearPhysical") BaseRearPhysical();

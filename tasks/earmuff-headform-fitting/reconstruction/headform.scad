// Millimetres. Original printable adult headform; MIT license.
// Ear centres205mm above support; crown335mm; temporal width145mm.
$fn=96;
module ellipsoid(r,p){translate(p)scale(r)sphere(1);}
module headform(){
  translate([0,0,7])cube([190,200,14],center=true);
  ellipsoid([46,44,70],[0,0,72]);
  intersection(){ellipsoid([95,88,120],[0,0,215]);translate([0,0,215])cube([210,145,250],center=true);}
  ellipsoid([73,63,68],[-16,0,157]);
  ellipsoid([24,16,26],[-97,0,218]);
  for(s=[-1,1])ellipsoid([14,9,30],[4,s*77.5,205]);
}
headform();

// Original task reconstruction,MIT license. Units:mm.
// Select part; F6 render; exportSTL. Standard hardware specified inREADME.
part = "Outer";
$fn=64;
module Outer() { union() {
  translate([0.0, -22.5, 20.5]) cube([48.0, 3.0, 29.0],center=true);
  polyhedron(points=[[-24.0, -24.0, 0.0], [-24.0, -24.0, 6.0], [-24.0, -21.0, 6.0], [24.0, -24.0, 0.0], [24.0, -24.0, 6.0], [24.0, -21.0, 6.0]],faces=[[2, 1, 0], [4, 5, 3], [4, 3, 0], [1, 4, 0], [5, 4, 1], [2, 5, 1], [5, 2, 0], [3, 5, 0]],convexity=10);
  translate([0.0, 22.5, 17.5]) cube([48.0, 3.0, 35.0],center=true);
  translate([-22.5, 0.0, 17.5]) cube([3.0, 42.0, 35.0],center=true);
  translate([-15.5, 0.0, 36.5]) cube([17.0, 48.0, 3.0],center=true);
  translate([0.0, -15.5, 36.5]) cube([14.0, 17.0, 3.0],center=true);
  translate([-8.0, 0.0, 154.0]) cube([2.0, 18.0, 232.0],center=true);
  translate([0.0, -8.0, 154.0]) cube([14.0, 2.0, 232.0],center=true);
  translate([22.5, 0.0, 17.5]) cube([3.0, 42.0, 35.0],center=true);
  translate([15.5, 0.0, 36.5]) cube([17.0, 48.0, 3.0],center=true);
  translate([0.0, 15.5, 36.5]) cube([14.0, 17.0, 3.0],center=true);
  translate([8.0, 0.0, 154.0]) cube([2.0, 18.0, 232.0],center=true);
  translate([0.0, 8.0, 154.0]) cube([14.0, 2.0, 232.0],center=true);
  translate([-10.0, 0.0, 180.0]) cube([4.0, 24.0, 80.0],center=true);
  translate([0.0, -10.0, 180.0]) cube([16.0, 4.0, 80.0],center=true);
  translate([10.0, 0.0, 180.0]) cube([4.0, 24.0, 80.0],center=true);
  translate([0.0, 10.0, 180.0]) cube([16.0, 4.0, 80.0],center=true);
  translate([-5.125, 0.0, 43.0]) cube([3.75, 14.0, 10.0],center=true);
  translate([0.0, -5.125, 43.0]) cube([6.5, 3.75, 10.0],center=true);
  translate([5.125, 0.0, 43.0]) cube([3.75, 14.0, 10.0],center=true);
  translate([0.0, 5.125, 43.0]) cube([6.5, 3.75, 10.0],center=true);
  translate([-5.125, 0.0, 260.0]) cube([3.75, 14.0, 10.0],center=true);
  translate([0.0, -5.125, 260.0]) cube([6.5, 3.75, 10.0],center=true);
  translate([5.125, 0.0, 260.0]) cube([3.75, 14.0, 10.0],center=true);
  translate([0.0, 5.125, 260.0]) cube([6.5, 3.75, 10.0],center=true);
  translate([-12.5, 0.0, 194.0]) cube([11.0, 36.0, 4.0],center=true);
  translate([0.0, -12.5, 194.0]) cube([14.0, 11.0, 4.0],center=true);
  translate([12.5, 0.0, 194.0]) cube([11.0, 36.0, 4.0],center=true);
  translate([0.0, 12.5, 194.0]) cube([14.0, 11.0, 4.0],center=true);
} }
if(part=="Outer") Outer();
module Rod() { union() {
  translate([0.0, 0.0, 2.0]) cube([40.0, 40.0, 4.0],center=true);
  translate([0.0, 18.5, 10.0]) cube([34.0, 3.0, 20.0],center=true);
  translate([-18.5, 0.0, 10.0]) cube([3.0, 40.0, 20.0],center=true);
  translate([18.5, 0.0, 10.0]) cube([3.0, 40.0, 20.0],center=true);
  translate([0.0, 0.0, 21.5]) cube([40.0, 40.0, 3.0],center=true);
  translate([0.0, 0.0, 166.5]) cube([6.0, 6.0, 287.0],center=true);
  translate([0.0, 0.0, 310.0]) cylinder(r=15,h=10,center=true);
} }
if(part=="Rod") Rod();
module Source() { union() {
  translate([0.0, 0.0, 2.0]) cube([111.0, 111.0, 3.0],center=true);
  translate([-54.0, 0.0, 76.0]) cube([3.0, 111.0, 145.0],center=true);
  translate([0.0, -54.0, 76.0]) cube([105.0, 3.0, 145.0],center=true);
  translate([54.0, 0.0, 76.0]) cube([3.0, 111.0, 145.0],center=true);
  translate([0.0, 54.0, 76.0]) cube([105.0, 3.0, 145.0],center=true);
} }
if(part=="Source") Source();
module Jar() { union() {
  translate([0.0, 0.0, 2.0]) cube([116.0, 116.0, 3.0],center=true);
  translate([-56.5, 0.0, 51.0]) cube([3.0, 116.0, 95.0],center=true);
  translate([0.0, -56.5, 51.0]) cube([110.0, 3.0, 95.0],center=true);
  translate([56.5, 0.0, 51.0]) cube([3.0, 116.0, 95.0],center=true);
  translate([0.0, 56.5, 51.0]) cube([110.0, 3.0, 95.0],center=true);
} }
if(part=="Jar") Jar();
module Stand() { union() {
  translate([0.0, 0.0, -3.0]) cube([116.0, 116.0, 6.0],center=true);
  translate([-52.0, 0.0, 15.0]) cube([4.0, 108.0, 30.0],center=true);
  translate([52.0, 0.0, 15.0]) cube([4.0, 108.0, 30.0],center=true);
  translate([0.0, -52.0, 15.0]) cube([108.0, 4.0, 30.0],center=true);
  translate([0.0, 52.0, 15.0]) cube([108.0, 4.0, 30.0],center=true);
} }
if(part=="Stand") Stand();
module Containment() { union() {
  translate([0.0, 0.0, 1.0]) cube([526.0, 626.0, 3.0],center=true);
  translate([-261.5, 0.0, 27.5]) cube([3.0, 626.0, 50.0],center=true);
  translate([0.0, -311.5, 27.5]) cube([520.0, 3.0, 50.0],center=true);
  translate([261.5, 0.0, 27.5]) cube([3.0, 626.0, 50.0],center=true);
  translate([0.0, 311.5, 27.5]) cube([520.0, 3.0, 50.0],center=true);
  translate([0.0, 0.0, -0.5]) cube([526.0, 626.0, 8.0],center=true);
} }
if(part=="Containment") Containment();
if(part=="granule") sphere(r=3.4);

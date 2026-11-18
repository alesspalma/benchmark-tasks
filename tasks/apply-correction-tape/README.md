# Apply correction tape to one entry

The robot grasps a dry correction-tape dispenser, covers the marked HOLD
entry on a clipped stock sheet and leaves PACK and SHIP readable. The
side-action housing measures approximately 82 × 35 × 16 mm. Its 4 mm
transfer width follows [Tombow MONO 68620][product]. The outline, mass and
small transfer-edge flex are approximated. The [manufacturer's application
instructions][technique] use smooth pressure, an angle of roughly 45° and
a lift to release the coating at the end.

## Native prototype

From this folder, use the Robocurve YAM simulation environment.

```sh
python -m pip install -e .
python run.py --device cuda:0 --physics-device cpu --no-cameras --output runs/demo
python -c 'from correction_tape.paper import write_svg; write_svg("stock-log.svg")'
```

The fixed scene uses seed 1415158438, a maximum of 480 steps at 20 Hz and
PhysX substeps at 240 Hz. The native demonstration's controller measures
the tool and clipboard poses for in-hand alignment and commands the YAM
through the ordinary joint targets. The benchmark observation interface
contains three RGB images and joint coordinates. The native runner
produces a trace and score receipt.

An A5 sheet sits under the normal clip on a 232 × 165 × 6 mm fibreboard
clipboard. Print the generated sheet at 100%. The YAM bench top is 747.5 mm
above the floor. In the robot station frame, forward is +X and the left-arm side
is +Y. The clipboard centre is X 382 mm, Y −93 mm. Orient it so the words read
along +X from PACK to SHIP. Set the housing broad-face-down with the transfer
nose pointing toward −X. Place the nose at X 302 mm, Y −270 mm. Reset with a fresh
sheet and an applicator with dry coating at the tip.

Both free props use native gravity and contacts. The simulated paper stays
flat on the clipboard as a thin rigid surface. The transfer blade has
a 2.5 mm spring travel that approximates compliance at the tip. The environment
projects the physically loaded blade's swept outline onto the sheet and displays
persistent film from the measured poses. It omits adhesive layers, the carrier
through the case and paper tearing. A photo of the real sheet would show how
much ink remains uncovered, along with tape on either neighbouring entry.

The native feasibility trials grasped and returned the dispenser. The contact
stroke did not cover the target, and the prototype was parked with a score of 0.

[product]: https://www.tombowusa.com/products/mono-correction-tape-original-white
[technique]: https://www.tombowusa.com/pages/support-orders-frequently-asked-questions

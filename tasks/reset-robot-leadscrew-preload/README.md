# Restore the preload of a robot leadscrew nut

The two flanks of a lead-screw nut have to share axial load without excessive friction. An overdriven compensating cam can leave a nut with little measurable play and enough drag to burden the axis. This is a commissioning operation for a robot axis before the drive is mounted and powered. Hold the free front nut steady, turn the cam down its ramps until the flanks relax, and let the torsion spring bring it back to a useful preload. The camera sees the relative turn of the exposed cam and housing. The evaluator checks the resulting fit after the robot releases it. [Novanta's installation procedure for the MDrive external linear actuator](https://www.novantaims.com/application-note/installing-anti-backlash-nut/)

## Make and reset the test stage

Use a cold, disconnected linear-axis assembly. The bench build uses the documented torsion-compensated ramp arrangement and stocked lead-screw components. The test does not require a complete powered robot.

| Component | Parts and dimensions |
| --- | --- |
| Drive screw | Right-hand steel TR16×4 trapezoidal screw, 337 mm long, with piloted ends matching two supported bearings. Purchase a TR16×4 blank and order ends turned to fit the chosen bearing supports. A standard 6803 bearing takes a 17 mm shoulder, so use a separate piloted collar or use Ø12 mm turned ends with 6001 bearings. Put the supports 300 mm apart on the 360 × 125 × 10 mm aluminium plate. Axis height is 109 mm above the bench. Fit a 36 mm handwheel and one axial thrust collar at each bearing. |
| Nut halves | Two TR16×4 dry polymer flange nuts matched to the screw, e.g. igus JFRM-2835TR16x4. The sleeve bore in the supplied parts accepts their nominal 28 mm body with clearance. The front flange mounts rigidly to the body. The rear flange attaches to the follower; three smooth 3 mm guide pins prevent relative rotation while leaving up to 3 mm axial travel. Give both halves the same thread phase while the guide pins are loose, then secure the flange fasteners. |
| Rotor sleeve and follower | Export the parts from [cam_stack.scad](fixture/cam_stack.scad). Assemble cam, front sleeve and rear follower coaxially. Print or machine the cam in nylon or acetal with 48 mm OD, 28.3 mm bore and a 25 mm sleeve. The three rising tracks span 114° and rise 1.25 mm per radian. Smooth the tracks. Three M2.5 screws with round heads indexed every 120° on the follower ride the tracks. Fit a 3 mm travel pin against the two shoulders of a 114° sector, with undertravel toward the low end and an excess-preload end above the working range. Leave axial clearance between all non-ramp faces. |
| Preload springs | Put a wound torsion spring between the front sleeve and the cam. Target 31 N mm/rad, with its unloaded index 83° above the point where the ramps become active. Engage the tangs in the marked blind holes of the body and the cam. Install a lightly sprung axial washer stack between rear flange and ramp follower. Select the stack to read 16.8 ± 1.0 N per mm over 5–22 N. Set the contact take-up at 0.08 mm of ramp rise, measured with the free nut halves and a depth gauge. |
| Housing support | A nylon-lined split cradle supports the front sleeve radially. Give the assembly room to rotate freely between soft stops at ±115° and allow a few millimetres of screw-driven axial float. The moving nut case has an accessible hex section opposite the brass-coloured cam. The cradle leaves the case free under the operator's torque. Use straight 15 mm square base risers to align the cradle with the bearing centres. |

On the table, the screw is parallel to the forward axis, its cam centred 374 mm in front of the arm base line and 3 mm left of the station midline. Set the case flats in their horizontal orientation and use a removable paper witness marker to put the cam 57° up from its neutral mark. Verify negligible axial play and at least 20 N mm running drag before starting a trial. Return the handwheel index to vertical. The right and left YAM bases are at opposite sides of the worktop, 610 mm apart. Both jaws start open and clear of the drive.

Before using a newly built rig, check force versus follower displacement with a 0–25 N spring scale. With the cam near its low stop, verify at least 0.30 mm play, then gently push it up its ramps and verify that both load and drag increase. Retain the same washers and dry lubrication for subsequent trials. A different spring arrangement should reproduce the measured response and remain freely reversible by hand. [Stock nut dimensions](https://www.igus.com/contentData/Product_Files/Download/pdf/2016%20drylin%20section.pdf)

## Score a trial

* **0.25** once the cam crosses the relief position that opens at least 0.30 mm reversal play. Calibrate this test mark on the rig before running it and observe the two cam and sleeve witness marks in the trial video.
* **0.5** when the resulting preload gives at most 0.10 mm reversal play under 5 N in each axial direction and at most 18.5 N mm running drag.
* **1.0** when the 0.5 conditions persist through a test with both robot grippers clear, during which the handwheel turns at least 16° under a 20 N mm torque cap. Keep a minimum 1.2 s hands-clear interval.
* **0** in other states. Either arm can hold or adjust the nut.

The physical check indexes the free housing in place with the cradle's removable lock after the robot has released it. Hold the screw against rotation for the reversal test. Use a 0.01 mm plunger indicator against the body and pull and push at 5 N along the screw. The difference of the two indicator readings is reversal play. Then unlock the screw, leave the nut restrained from turning, and take the running torque from a low-range torque screwdriver or spring scale tangent to a marked lever arm. Tare bearing friction with an unloaded screw. At a 100 mm lever arm, 0.20 N applies the capped 20 N mm for the wheel-travel test. The evaluator stops after checking 16° of rotation and does not alter the preload. Use the camera to verify the robot has withdrawn.

The native collar and housing use real contacts, and collar rotation is relayed into a thrust and friction model with no automatic alignment. The model uses ramp slope times the measured collar angle for rise, then multiplies take-up beyond clearance by axial stiffness. Sleeve friction and screw-flank running drag rise with thrust. The reversed load estimate combines travel before flank contact and displacement from ±5 N on the matched thread stiffness. During the hands-clear check the physics simulator applies a bounded torque at the native screw joint. The screw moves under that torque only when the modeled thread drag permits it. None of these measurements enter the reference controller.

## Restart and run

The packaged Isaac Lab Arena YAM embodiment uses a fixed setup. The reference outputs standard joint targets at 20 Hz, with seed `1415158438` and a 420-action limit. Start in the activated simulator environment:

```bash
python -m pip install -e . --no-deps
python -B run.py --device cuda:0 --physics-device cpu --output /tmp/preload-demo
python -B run.py --device cuda:0 --physics-device cpu --no-cameras --policy hold --output /tmp/preload-hold
python -B run.py --device cuda:0 --physics-device cpu --no-cameras --policy no-brace --output /tmp/preload-no-brace
python -B run.py --device cuda:0 --physics-device cpu --no-cameras --policy undertravel --output /tmp/preload-undertravel
```

Use `preload` as the Inspect Robots embodiment and `preload_scripted` for the reference. The two failure controls attempt the turn with the case unsupported or release the cam before it rides down the ramp. Both leave excessive drag. Every run records its state trace and receipt. The three camera recordings include the native reset and every action frame, at 640 × 360 and 20 fps.

## Assets and license

The bench, lighting, screw stage, nut and cam geometry and fixture CAD are original procedural assets covered by [MIT](LICENSE). The [YAM geometry and terms](https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/assets/yam/LICENSE.i2rt) and [MIT wrist camera calibration](https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc126418ea5e74ac567133d/LICENSE) come through the pinned adapter.

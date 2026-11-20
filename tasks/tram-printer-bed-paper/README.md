# Set a 3D printer's corner height with a paper feeler

A spring-supported build plate must be an even distance from the nozzle so the first layer adheres across a printed robot part. Correct one corner of an unpowered Ender-3 style printer, with the cold brass nozzle held over the spring screw. Turn the wheel with one gripper. With the wheel released and that gripper clear, use the other to slide the paper feeler back and forth under the nozzle. Finish by releasing the paper. Either gripper can perform either operation. [Creality Ender-3 procedure](https://store.creality.com/blogs/all/ender-3-calibration) · [Manufacturer manual](https://file2-cdn.creality.com/product/31903413-a549-4167-8e80-ca3ca0be402d)

## Build and reset

Use a Creality Ender-3 equipped with the factory four manual bed wheels or a matching open-frame filament printer. Keep the printer cold and unplugged throughout the trial. Use a standard smooth 235 mm build surface and a clean nominal 0.4 mm brass nozzle. Place the Y carriage and X hot-end carriage using the ordinary front-right service position, with the nozzle tip centered directly over the M4 support screw. Keep them stationary during one corner trial. The front of the printer faces the YAM bases. The source asset dimensions use +X away from those bases and +Y toward the left arm.

| Part | Size, material and measured placement |
| --- | --- |
| Wooden workbench | 1,100 × 1,250 mm, surface 747.5 mm above the floor. YAM base centers are 610 mm apart. |
| Cold printer | 0.40 m deep aluminum V-slot base and full 0.41 m tall uprights. The center is (453, 0) mm in the horizontal plane. The carriage carries the 235 × 235 mm bed at nominal height 856 mm. |
| Bed and adjustment screws | Smooth spring-steel surface over an aluminum heater, approximately 5.2 mm overall thickness. Four 59 mm wide nylon thumbwheels on M4 × 0.7 screws centered 85 mm from the bed center along X and Y. Installed wheel contact surfaces are at approximately 820 mm height. |
| Hot end | Brass nozzle with a fixed X, Y and Z carriage position. The tip is at (368, −85, 856.083) mm when it is unloaded in the simulation. |
| Paper | Dry, uncoated 80 gsm office paper, around 0.108 mm thick. Cut a 179 × 58 mm strip with clean edges. Fold the near 18 mm over itself twice to give four layers. Lay the strip flat with its centerline joining the nozzle and the point at (310, 44) mm. Leave the folded end overhanging at that point. |

Level all four corners to the midpoint of the gauge band before measuring the reset. Leave three wheel settings fixed. Back off the front-right one by 48 degrees, checking that the 0.135 mm feeler initially passes at that corner. Confirm the paper slides freely initially. Set the grippers open and clear. The cold nozzle and bed stay on the printer. Use new paper if a strip picks up grit, tears or stays creased.

## Score a physical trial

Use ordinary steel thickness gauges after the grippers withdraw and a fixed video of the paper edge. Use metric feeler blades nearest 0.065, 0.10 and 0.135 mm. The lower blade should pass, and the upper blade should meet resistance. The front-right wheel must have moved at least 2 degrees in the raising direction from its starting index. Judge the gap with the wheel released. Apply minimal force to the gauge and keep it flat. For the light-drag criterion, corroborate surface contact from a faint rub trace at the nozzle line on the original, intact paper and compare paper edge motion against a millimetre scale fixed beside the front of the bed. Assess the trial at its scheduled end or once the full result has held for a second.

- **0.25** for a gap over 0.10 mm and no more than 0.135 mm.
- **0.5** for a gap from 0.065 to 0.10 mm.
- **0.75** for the target gap and at least 12 mm of cumulative intact paper travel under light drag.
- **1.0** for the target gap and verified sliding plus one second with both grippers clear of the wheel, paper and bed and the paper resting on the bed. Use a 20 mm clearance around the paper and 48 mm from the wheel center. A fingertip above the build area must be at least 25 mm from the plate plane.
- **0** otherwise.

The wheel physically acts on a spring-supported platform. The path of four screw seats is calculated from the 0.70 mm thread lead, and drives the bed's vertical, roll and pitch springs. The passive axial nozzle element represents combined head and plate compliance under light paper loads. Contact is carried by the 0.108 mm thick paper, build surface and brass tip. Sliding credit requires consecutive paper positions under measured nozzle deflection while the paper footprint covers the tip and the gap is in the target band. The reference uses two short rim grasps, reads wheel angle after the first release and computes the remaining turn with the same reachable wrist range. It then finds the folded edge from the current paper pose and slides the sheet with the opposite gripper. The test uses an original model at Ender-3 scale. The original paper's shallow end curl bends at a passive hinge, and the working leaf stays thin.

## Simulate

Run with the pinned Isaac Lab Arena and Inspect Robots configuration in `pyproject.toml`. Model evaluation uses the `printer_bed` embodiment. Scripted evaluation uses the same two parallel grippers, seated printer and ordinary joint action interface. The fixed trial seed is `1415158438`. The simulation runs 240 Hz physics with 660 actions at 20 Hz, a maximum of 33 simulated seconds. Each command writes the fixed layout, full sequence and scorer receipt. The `--no-cameras` option runs the same physics with camera rendering disabled.

```bash
python -m pip install -e . --no-deps
python run.py --device cuda:0 --physics-device cpu --output /tmp/printer-bed-demo
python run.py --no-cameras --policy hold --output /tmp/printer-bed-idle
python run.py --no-cameras --policy undertravel --output /tmp/printer-bed-undertravel
python run.py --no-cameras --policy no-paper --output /tmp/printer-bed-no-paper
```

The model and source assets are original and MIT-licensed. Robot and camera rig assets are provided by the pinned YAM adapter. [YAM license](https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/assets/yam/LICENSE.i2rt) · [Camera asset license](https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc126418ea5e74ac567133d/LICENSE)

# Meter 5 mL through a three-valve pipette filler

A rubber safety filler gives a graduated glass pipette independent controls
for evacuating the bulb, aspirating liquid and venting for delivery. The
operator pinches A while squeezing the bulb, closes A before letting the bulb
recover, pinches S to raise the meniscus to the graduation, then moves the
pipette to the receiving vessel and pinches E. This is a millilitre-scale
liquid-transfer procedure used in teaching and research labs. The operation
tests independent valve control, maintaining pipette immersion during the
draw, and moving the filled pipette without losing the calibrated column.
The [Fischer safety filler][fischer] and [BRAND laboratory guide][brand]
document the apparatus and method.

## Setup and reset

Use a level worktop 747.5 mm above the floor and two YAM arms. Coordinates
are measured from the midpoint between the arm bases, with X forward and
Y left. Place a blue source beaker at (331, 70) mm and a clear receiving
beaker at (331, −88) mm. Each glass beaker has a 46 mm inner diameter,
1.8 mm wall, 3 mm bottom and 53 mm overall height. Secure both beakers to
the worktop. Fill the source with 62.0 mL of blue-dyed water and leave the
receiver empty.

Fit a 50 mL natural-rubber A/S/E safety filler to a 350 mm long, 10 mm
outside diameter, 10 mL serological glass pipette with 0.1 mL graduations.
A [PYREX 7085-10][pyrex] is an example with those dimensions. Mark the
graduation at the 5.0 mL sample column height. Start with A facing the
left arm and S and E accessible from the right. Start the pipette upright
in an open-top collar support on a steel retort stand, with the pipette
tip at (331, 139) mm and 79 mm above the worktop. The weighted stand has
a 94 × 110 × 12 mm base centred at (455, 162) mm and a 14 mm diameter post
centred at (467, 183) mm. The fork supports a 31 × 25 × 6 mm pipette collar
374 mm above the worktop. Its two prongs leave a 19 mm centre opening facing
the beakers. The removable 27 × 31 × 42 mm rubber sleeve is centred 226 mm
above the tip, with its flat grip faces turned 28° toward the left arm.
Start with both grippers open, the pipette and bulb dry, all valves released
and the bulb relaxed at atmospheric pressure.

To reset, put both beakers at their marks, refill the source to 62.0 mL
and empty the receiver. Empty and dry the pipette, let the bulb expand,
vent A and return the pipette to its stand. Use water only. Use a spill tray
under the work area for physical trials.

## Score a trial

Tare the empty receiver and weigh it on a balance with 0.01 g increments
after the trial. Calculate delivered volume by dividing water mass by its
density at room temperature. Use a tared absorbent liner under the vessels
and transfer path to measure any spill. Inspect the graduated tube for
retained water and watch the upper glass and filler for overdraw. A spill
of more than 0.20 mL at any point or any water reaching the bulb sets the
score to zero.

| Delivered volume and remaining water | Score |
| --- | ---: |
| 4.80–5.20 mL delivered and at most 0.06 mL retained in the pipette | 1.0 |
| Error from 5.0 mL is at most 0.50 mL and the higher score does not apply | 0.75 |
| Error at most 1.00 mL | 0.50 |
| Error at most 3.00 mL | 0.25 |
| Other outcomes | 0 |

Either arm can steady the tool or operate the valves. The score is based
on delivered volume, retained volume, spill and avoiding overdraw.

## Run

Use the supported Robocurve Isaac Lab-Arena environment and work from this
task directory. The deterministic seed is 1415158438. The episode limit
is 700 actions, or 35 seconds at 20 Hz.

    python -m pip install -e .
    python run.py --device cuda:0 --physics-device cpu --output runs/demo
    python run.py --device cuda:0 --physics-device cpu --no-cameras --policy hold --output runs/hold
    python run.py --device cuda:0 --physics-device cpu --no-cameras --policy no-charge --output runs/no-charge
    python run.py --device cuda:0 --physics-device cpu --no-cameras --policy no-delivery --output runs/no-delivery

The runner writes the three 640 × 360 views at 20 fps, initial images,
per-action scores and state measurements in trace.jsonl, and a fingerprinted
receipt. The Inspect Robots entries are pipette_transfer and
pipette_transfer_scripted.

## Simulation fidelity

The workcell and tool use native contacts. The grippers move the four
separate spring-return joints representing the bulb squeeze and three valves.
A lumped pneumatic model uses valve travel, bulb width, pressure, head and
tip immersion to compute aspiration, retention and drainage. It conserves
the water volume across the two vessels, pipette and any spill. Water volumes
render as surfaces and a graduated liquid column. The bulb compliance is
lumped and the liquid model is quasi-static.

[fischer]: https://fischertech.com/safety-pipette-fillers
[brand]: https://www.fishersci.se/content/dam/fssite/eu/brands/b/brand/22624_BRAND_Volumetric_Measurement_in_the_Laboratory.pdf
[pyrex]: https://ecatalog.corning.com/life-sciences/b2b/US/en/Glassware/Serological-Pipets%2C-Glass/PYREX%C2%AE-Reusable-Glass-Serological-Pipets%2C-Color-coded%2C-Colored-Markings%2C-To-Deliver/p/7085-10

## Third-party assets and licenses

The pipette, filler, stand and beakers are generated from this task's MIT-licensed source.
The station assets below are supplied by runtime dependencies.

| Asset | Source and use | Applicable license |
| --- | --- | --- |
| Left and right I2RT YAM arms, parallel grippers and collision hulls | The same I2RT model at commit `120c3c81400171174604e503943f8d1ebc891058`, converted from URDF to USD by the [pinned YAM adapter][adapter]. | [MIT][i2rt-license]. |
| Black and white arm-link meshes | I2RT links 2 and 3 split for the Dream Labs station appearance and [supplied with the adapter][split-meshes]. | [MIT][i2rt-license]. |
| Glass optics | The virtual pipette and beakers use the [OmniGlass](https://docs.omniverse.nvidia.com/materials-and-rendering/latest/materials.html#omniglass) shader supplied by Isaac Sim, with a USD Preview Surface fallback. | NVIDIA Omniverse end-user license. The shader is a runtime component. |
| Station worktop | The 1.10 × 1.30 × 0.06 m colored cuboid generated by the adapter's [station room][room]. | Procedural geometry with no separate asset license. The [adapter has no declared repository license][adapter-license]. |
| Station floor, dome light and lamp | Procedural floor box and untextured lights from the same [station room][room]. The dome light uses Isaac Lab-Arena. | The [adapter has no declared repository license][adapter-license]. Isaac Lab-Arena is [Apache-2.0][arena-license]. |
| Fixed overview camera (`top_cam`) | Virtual RGB camera using [YAMLAb optics][calibration], with its pose set by this task and the sensor spawned by the [adapter][cameras]. | [MIT for the YAMLAb calibration][calibration-license] and the task-authored pose. The [adapter has no declared repository license][adapter-license]. |
| Left wrist camera (`left_cam`) | Virtual RGB camera mounted to the left gripper by the [adapter][cameras] with the [YAMLAb left wrist calibration][calibration]. | [MIT for the YAMLAb calibration][calibration-license]. The [adapter has no declared repository license][adapter-license]. |
| Right wrist camera (`right_cam`) | Virtual RGB camera mounted to the right gripper by the [adapter][cameras] with the [YAMLAb right wrist calibration][calibration]. | [MIT for the YAMLAb calibration][calibration-license]. The [adapter has no declared repository license][adapter-license]. |

[adapter]: https://github.com/kairosiann/yam-isaac-lab-arena/tree/0059d02a1a91bc6834c1d0846022891df0c78730
[adapter-license]: https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/README.md#license
[i2rt-license]: https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/assets/yam/LICENSE.i2rt
[split-meshes]: https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/assets/yam/dream/SOURCE
[room]: https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/scene.py
[cameras]: https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/embodiment.py
[arena-license]: https://github.com/isaac-sim/IsaacLab-Arena/blob/c7edf3a9d5d8276565208f72422f8f8a0806d6cf/LICENSE.md
[calibration]: https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc126418ea5e74ac567133d/yamlab/configs/robot/yam.yaml
[calibration-license]: https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc126418ea5e74ac567133d/LICENSE

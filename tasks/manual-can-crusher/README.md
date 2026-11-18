# Compact an aluminum can in a wall-mounted hand crusher

Place the empty can in the open cradle and pull the handle through the first
buckle and the remainder of the compression stroke. Release the handle to let
the ram return, then pick up the flattened can and put it into the bin.
This saves space where aluminum is already kept in a separate recycling
stream.
[EPA guidance on sorting and compaction](https://www.epa.gov/recycle/how-do-i-recycle-common-recyclables)

## Run

Install Isaac Lab-Arena, Inspect Robots, and the YAM adapter according to the
Robocurve walkthrough. Install the task from this directory.

```bash
python -m pip install -e .
python run.py --device cuda:0 --physics-device cpu --output runs/demo
```

The embodiment is `can_crusher` and the reference policy is
`can_crusher_scripted`. This runner reads the native handle angle for trajectory
tracking during the press. The fixed layout uses seed 1415158438. The trial allows
920 actions at 20 Hz, or 46 seconds, with native physics at 480 Hz. The
runner records a receipt and all three native camera views at 640 × 360.
Use `--policy load-only` for the ordinary load and release without pulling
the lever, or `--policy hold --no-cameras` for a stationary control.

## Setup

Mount a full-size hand can crusher with a fixed base plate and a guided ram on
a rigid vertical support. The scene uses a 340 mm pull lever, steel guides,
a slotted connection between the lever pin and the ram, and a preloaded return
spring. The fixed support and vertical compression follow the operation in
[US5009155A](https://patents.google.com/patent/US5009155A/en).
Spring-return manual operation is documented in
[US5331889A](https://patents.google.com/patent/US5331889A/en).
All distances in the table are measured from the midpoint of the two pedestal
base mounting axes. X points from the arms toward the wall, Y to the arms'
left and Z upward.

| Item | Material and size | Initial configuration |
| --- | --- | --- |
| Supports | Two bolted metal YAM pedestals, about 160 × 160 × 833 mm, with 220 mm square feet | Y ±305 mm, arm mounting surface Z 820 mm. Hands open and clear. |
| Vertical mount | Plywood or structural panel, 32 × 580 × 1,240 mm | Front face X 492 mm. Crusher centered at Y 0. |
| Can crusher | Rigid back plate, steel hinge pins, guides and pressure faces, 140 × 98 mm. 1.4 mm rubber cradle pad and 36 mm-wide, 3 mm-high buckling boss with a round footprint | Lever pivot at X 457 mm, Z 832 mm. Open cradle centered on X 408 mm, Y 0, Z 757 mm. The crossbar is 340 mm from its pivot. Open clearance is 171 mm between the pad and the boss. Lever starts raised and reaches a 115° closed stop. |
| Can | Empty standard 12 fl oz aluminum beverage can with its original pull tab, about Ø66 × 122 mm, 14 g | Upright, on the side shelf at X 333 mm, Y 198 mm and Z 748 mm. |
| Side shelf | Plywood, 265 × 190 × 18 mm | Shelf top at Z 748 mm. Its center is 39 mm behind the can along X. |
| Collection bin | Ordinary 301 × 224 × 204 mm plastic bin on a rigid shelf, initially empty | Opening centered at X 342 mm, Y 0. Bin floor at Z 220 mm. |

Use one empty, drained can of the same standard format per trial. Clear the
collection bin, return the lever to its upper stop and place the can upright
on the shelf between trials.

For a fabricated equivalent, use 6 mm guide rods on 154 mm centers over the
back of the two pressure faces. The pin is 105 mm from the handle pivot. The
roller travels in two horizontal channels attached to the ram. Build the
channels from off-the-shelf channel stock and fasten the frame to the vertical
support. The depth and width of the anvil support and the three starting
positions are given above. Exact geometry and dimensions can be exported as
USD and a JSON inventory from this directory.

```bash
python - <<'PY'
from isaaclab.app import AppLauncher

app = AppLauncher(headless=True, device="cpu").app
try:
    from can_crusher.builder import create_assets

    create_assets("runs/geometry")
finally:
    app.close()
PY
```

## Scoring

Measure the can's maximum length along its original axis between two flat
surfaces after it has been released. Use trial video to confirm where it
ended and the final position of the lever.

- **1.0** when the can is no more than 35 mm high, rests in the collection
  bin for at least 0.6 seconds, and the lever is within 12° of its open stop.
- **0.5** for a can no more than 70 mm high whose final position is in the
  cradle or bin, when the full conditions are unmet.
- **0.25** for a can seated in the cradle when the compaction conditions are
  unmet.
- **0.0** for other outcomes.

## Simulation

Robot grasps, the lever, the slotted drive-pin linkage, the return spring and
contact with the bin run in PhysX. The upper cap contact uses a reduced
axial law from the measured ram gap, can alignment and support. It compresses
the native shell joint and applies the opposing load through the ram to the
lever. The spring model uses 30 kN/m initial axial stiffness, a 300 N
buckling threshold, and 4 mm rebound. The shallow convex boss localizes
initial buckling, a feature documented for household crushers in
[US4301722A](https://patents.google.com/patent/US4301722A/en). The shell wall
folds with the end-to-end distance. Exact crease geometry and sheet tearing
are outside the model. A published test of a hand-lever crusher reports
22–89 N at the grip for aluminum cans.
[Hand-lever load reference](https://patents.google.com/patent/US4290354A/en)

## Third-party assets and licenses

The crusher, can, bin, shelves, support panel and robot pedestals are generated
from this task's source under the included MIT license. The assets below are
provided by runtime dependencies.

| Asset | Source and use | Applicable license |
| --- | --- | --- |
| Left and right I2RT YAM arms, parallel grippers and collision hulls | The same I2RT model at commit `120c3c81400171174604e503943f8d1ebc891058`, converted from URDF to USD by the [pinned YAM adapter][adapter]. | [MIT][i2rt-license]. |
| Black and white arm-link meshes | I2RT links 2 and 3 split for the Dream Labs station appearance and [supplied with the adapter][split-meshes]. | [MIT][i2rt-license]. |
| Station floor, dome light and lamp | Procedural floor box and untextured lights from the adapter's [station room][room]. The dome light uses Isaac Lab-Arena. | The [adapter has no declared repository license][adapter-license]. Isaac Lab-Arena is [Apache-2.0][arena-license]. |
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

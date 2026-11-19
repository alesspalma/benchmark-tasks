# Get a folding walker ready for a seated person

An adult two-button walker is standing on the floor directly ahead of a seated
training manikin. Each side frame is folded inward and its release button is
recessed. The walker can look usable before its spring pins engage. Loading an
unlocked walker can fold a side frame and cause the user to fall. The robot
opens both sides until they lock, then steps back so the walker is available
for use.

> Get the folding walker ready for the person in the chair.

Two-button walkers have independently folding sides. The clinical setup check
is that the walker is locked open before use.
[Drive Medical two-button walker][drive] · [Mayo Clinic folding walker handbook, p. 2][mayo]

## Setup and reset

Use an adult four-ferrule, two-button walker of the 10200-1 type, a straight
chair, and a seated training manikin. The full-size test frame uses 25.4 mm
aluminum main tubes with rubber grip sleeves and ferrules. It is about 510 mm
wide and 430 mm deep when open, weighs 2.47 kg and has its grips 825 mm above
the floor. The grip sleeves span 164 mm along the rails and are 36 mm in
diameter. Both vertical hinges lie on the line through the front feet,
438 mm apart. Use a smooth level floor with ordinary dry rubber contact.

Place the seat surface 484 mm above the floor. The chair faces the robot bases. Put
the walker in front of the manikin with its open side toward the chair and
the walker centreline on the seat centreline. The baseline layout has 0.775 m
between the front hinge line and the centre of the seat. The manikin's shoes
are 0.107 m beyond the open walker's rear feet. Use both release buttons to
fold inward from the open orientation and set the two angles as specified
below. Set the frame down on its four feet and verify that both pins are
recessed. The scene has no living subject.

The world origin is midway between the bases at their horizontal position and
at the standard YAM station floor datum. In the simulation the floor is at
z = −0.0125 m. The x-axis points from the robot bases toward the chair. The
positive y-axis points toward the left robot base.

| Object | Reset dimensions and pose |
| --- | --- |
| Left and right YAM arms | Standard parallel-jaw grippers. Pedestal tops at (0, ±0.305, 0.750) m. Bases face +x. Home joint targets [0, 1, 1, −0.6, 0, 0] rad with both grippers open. |
| Steel robot pedestals | 140 × 140 mm column cross sections with a 204 × 205 × 12 mm floor plate per arm. |
| Walker | Front hinge line centred at (0.215, 0) m on the floor. Button caps are 28 mm in diameter with 7 mm nominal projection when engaged. Begin with the left side 23° and the right side 18° inward from the open stops. |
| Chair | Wood seat 413 × 435 × 44 mm, centre at (0.990, 0, 0.449) m. Backrest 34 × 434 × 317 mm. Four 24 mm diameter tubular steel legs. |
| Seated manikin | Rigid plastic adult training manikin with clothed torso, arms on lap and shoes flat on the floor. Shoulder span about 0.40 m; two flat 214 × 101 mm shoes. Shoe tips are at x = 0.718 m and centres are at y = ±0.113 m. |

Reset the walker manually after every run. Press its release buttons, restore
the folds with a protractor, and position all four feet on the floor. The
angle setting is reproducible with a top photo or a cardboard angle template.

## Scoring

Score **1.0** when all the following hold together for 0.75 s:

- Both side-frame locks engage. Both buttons project and each side is within
  4° of the open stop. To verify on hardware, after the trial use a spring
  scale to apply 8 N inward at each ordinary grip, 207 mm from its hinge,
  for one second. The lock must hold within 4° and the button must stay up.
  Test with the chair empty of any live subject.
- All four ferrules are no more than 12 mm above the floor and the walker is
  steady. In video the front hinge line travels no more than 17 mm in the
  final 0.75 s and the frame turns by no more than 7.5°.
- The hinge-line centre is 140–300 mm forward of the base line, within
  65 mm of the chair centreline, and square within 12°. The walker opens
  toward the chair.
- Both gripper centres are at least 85 mm clear of both grip centres. The
  robot has let go of the walker.

Score **0.0** otherwise. With either side unlocked, the walker is unsafe to
offer to the user. Judge the score with video, a tape measure, a protractor
and the brief lock test. The verifier permits either ordering and either
approach, including opening the sides independently.

## Run

Use the Robocurve Isaac Lab-Arena and Inspect Robots environment. From this
directory, install the task using its `pyproject.toml` and launch the demo
with native cameras.

```bash
python -m pip install -e .
python run.py --device cuda:0 --physics-device cpu --proof-load --output runs/demo
python run.py --device cuda:0 --physics-device cpu --no-cameras --policy hold --output runs/hold
python run.py --device cuda:0 --physics-device cpu --no-cameras --policy one-side --output runs/one-side
inspect-robots run --embodiment folding_walker --policy folding_walker_scripted \
  --instruction "Get the folding walker ready for the person in the chair." \
  --max-steps 400 --seed 1415158438 --scorer task_score -E cam_width=640 -E cam_height=360
```

The fixed scene uses seed 1415158438. Trials run at 20 control actions per
second for at most 400 actions. This is 20 s of robot time. The native runner
saves the initial frame and one native frame per action from each camera. It
also saves `receipt.json`, `trace.jsonl` and the final native state. It refuses
to overwrite an existing run directory. Opening, engagement, released
stability and final placement are separate diagnostics. The optional
`--proof-load` check runs after the episode and records the response to the
8 N per-grip load, separately from the rollout and its score.

| Native trial | Outcome |
| --- | --- |
| Bilateral opening and release | 1.0 in 199 actions. Both buttons rose, the frame stood on four feet and both hands released. |
| Stationary | 0.0 after all 400 actions. Both sides remained unlocked. |
| Premature release | 0.0. The left lock engaged and the right side stayed 6.44° from open. The frame stood on four feet in the placement area with both arms clear. |
| Released bench load | Both locks held for 1 s under an inward 8 N per grip. Peak fold angles were 0.743° and 0.728°. |

## Simulation and fidelity

The model includes full-size native frame and floor contacts, independent
vertical hinges, physical button springs, and a free-moving walker. A cam
effort depresses each release button while its side rotates away from the
open position. The modeled catch reacts against inward hinge motion after
its button rises in the open recess. This preserves the lock's dependence
on real side movement and its loaded bearing response without modelling
individual pin faces. The opening reference drives only the YAM action
interface and relies on contact at both ordinary handgrips. Numerical spring
and friction constants are engineering settings for the bench test. The
bearing law uses 285 N·m/rad stiffness, 3 N·m·s/rad damping and a 14 N·m
limit. The frictional bushings and rubber foot pads keep the folded
walker steady until an arm pushes on it. The
model does not qualify walker strength for human loading.

This submission tests functional engagement of both independently folding
side locks on a full-size frame. The nearest placement task on the board
evaluates position of a rigid miniature frame (TK-T8VMWU). The setting also
differs from parked-wheel tests on rollators. Side folding can make the
frame collapse even when its feet initially stand still.

[drive]: https://shop.drivemedical.com/us/en/products/mobility/walkers/push-button/deluxe-folding-walker-two-button/p/709-1
[mayo]: https://mcforms.mayo.edu/mc1200-mc1299/mc1234-158.pdf

## Third-party assets and licenses

The walker, chair, manikin, and robot pedestals are generated from this task's
source under the included MIT license. The remaining assets are provided by
runtime dependencies.

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

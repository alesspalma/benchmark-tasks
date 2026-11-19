# Load a printed page into a clear sheet protector

This prototype uses a full-size Letter sheet and a ring-bound, top-loading
polypropylene sheet protector. One YAM gripper opens and supports the clear
film while the other handles the loose page. The intended outcome is a flat,
fully contained sheet with both hands clear.

**Validation status:** The native reference has not achieved full insertion.
The scene and commands are retained for reproducing and diagnosing contact
failures. It is not a submission candidate.

## Setup and reset

Use an [Avery 75540 top-loading Letter sheet protector][protector] or an
equivalent clear polypropylene, reinforced three-hole document pocket. Open
a standard three-ring binder flat and load the pocket on its rings above the
filed pages. Place a single 80 g/m², 279.4 × 215.9 mm printed sheet on a
sloped plastic document stand next to the binder. Keep the page entirely
outside the sleeve, print face up. No page edge is in the pocket at reset.

The YAM worktop is 747.5 mm high, with +X forward and +Y left. The horizontal
pocket's entrance is at X 485 mm, centred at Y 5 mm and Z 791.5 mm. The
sealed end is at X 774 mm and the clear internal width is 220 mm. The upper
lip projects 9 mm toward the robot from the lower lip. That separation is a
simulation dimension and should be checked on the chosen physical pocket.
The binder has two roughly 284 × 250 × 6 mm boards and 38 mm of filed pages.
The stand supports the sheet at a 19° slope toward the table with its top
edge centre at X 209 mm, Y −244 mm and Z 901 mm. Leave about 22 mm of
paper beyond the backing tray at this edge so the gripper can approach
both faces. Use a low full-width support ledge at the bottom of the stand.
Reset by removing the sheet, flattening or replacing creased stock, closing
the pocket, restoring the marked positions and opening both grippers.

## Run and score

Use the challenge YAM environment. Run from this folder.

```sh
python -m pip install -e .
python run.py --device cuda:0 --physics-device cpu --output runs/demo
python run.py --device cuda:0 --physics-device cpu --no-cameras --policy hold --output runs/hold
python run.py --device cuda:0 --physics-device cpu --no-cameras --policy on-top --output runs/on-top
```

The fixed reset uses seed 1415158438, 480 Hz physics and 20 Hz control.
The limit is 700 actions (35 seconds). The runner records three native
640 × 360 feeds and writes scores and the source fingerprint to
`receipt.json`. Use a top and a side photo and a ruler to inspect where
the page rests. All three edges of the film opposite or to the sides of
the mouth are sealed on a physical protector.

- **1.0** when the entire sheet lies inside and beneath the film, the
  sheet lies flat with less than 6 mm height variation, the lip is at
  most 1.03° open, and both gripper tips are at least 35 mm clear. The
  completed state must be still for at least 0.45 seconds.
- **0.5** for a released, flat sheet at the level of the lower film if
  five to eight of nine equally spaced corner, edge-centre and centre
  samples are inside the pocket and beneath the upper film.
- **0.0** otherwise, including a page resting only on top of the
  protector.

## Simulation model

The visible polypropylene surfaces are 0.10 mm translucent skins.
The native contact geometry for the film and paper is 0.65 mm thick
to stabilize collision tests. The binder physically supports the lower
film. A passive hinge with a 7.4° motion limit approximates the upper
film opening and closing. A light spring and the film's native mass
close it when released. The prototype uses a rigid page to isolate
pinch and insertion feasibility. This approximation does not represent
page buckling or film flex across the mouth, and both matter for a
finished version of the benchmark.

## Third-party assets and licenses

The task-authored sheet, print, binder, film, stand and joint geometry
use the included MIT license. Avery specifies the commercial format and
supplies no mesh, texture or logo bundled here. The runner and YAM
simulation setup reuse the same team's `tasks/core-an-apple` scaffold.

| Asset | Source and use | License |
| --- | --- | --- |
| I2RT YAM arm, gripper geometries and split arm appearance | I2RT URDF and meshes supplied by the [pinned adapter][adapter] | [MIT][i2rt-license] |
| Station worktop, floor, lights and camera support | Procedural assets from the [station scene][room] | The adapter has [no declared repository license][adapter-license]. The runtime is [Apache-2.0][arena-license]. |
| Overview and wrist cameras | [YAMLAb calibration][calibration], supplied by the adapter, with the overview pose authored by this task | [MIT for calibration][calibration-license] |

[protector]: https://www.avery.com/products/sheet-protectors/75540
[adapter]: https://github.com/kairosiann/yam-isaac-lab-arena/tree/0059d02a1a91bc6834c1d0846022891df0c78730
[adapter-license]: https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/README.md#license
[i2rt-license]: https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/assets/yam/LICENSE.i2rt
[room]: https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/scene.py
[arena-license]: https://github.com/isaac-sim/IsaacLab-Arena/blob/c7edf3a9d5d8276565208f72422f8f8a0806d6cf/LICENSE.md
[calibration]: https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc126418ea5e74ac567133d/yamlab/configs/robot/yam.yaml
[calibration-license]: https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc126418ea5e74ac567133d/LICENSE

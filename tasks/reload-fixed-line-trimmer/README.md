# Reload a fixed-line string trimmer

Restore both cutting lines on the empty head of a residential string trimmer. Each 8 in nylon strip must be aligned with one small outboard eyelet, pushed through the inward-sliding spring grip far enough to reach the service recess, and left secure against outward pull. The two arms carry and feed the separate strips. The fixed lines can be loaded in either order with either arm. [ECHO GT-225 manual, nylon line installation on page 12](https://www.echo-usa.com/getattachment/06eeedf7-d33a-432d-8ada-40560c9f94c9/v2_GT-225_29_020622_es.pdf) · [ECHO Rapid Loader two-line head and precut nylon](https://www.echo-usa.com/trimmer-brushcutter-accessories/rapid-loader%C2%AE-head)

## Physical setup and reset

Use an ECHO 21560056 Rapid Loader two-line head, installed on an inert GT-225 curved-shaft trimmer or a spare compatible shaft. Remove the spark-plug lead and drain fuel before fixture assembly. A compatible stock head and shaft can also be bolted into a wood cradle with its cutting face up. This test stops at the fitted, released lines. The head does not spin.

Use a laminate workbench 747.5 mm high, 1,250 mm wide and 1,800 mm long so it fully supports the shaft and powerhead with the head accessible from the YAM mounts. The robot base centers are 610 mm apart at bench height. The horizontal coordinates below use X forward from the robot base midpoint and positive Y to the left.

| Component | Reset |
| --- | --- |
| Head and debris guard | Molded nylon head approximately 108 mm across with two brass eyelets, steel spring retainers, a lower flange and a central service recess. Keep the cutting face up, axis at X = 390 mm and Y = 0, eyelet height 87.5 mm above the bench. Orient the two eyelets toward opposite Y sides. Fix the shaft on the bench in this pose with a padded cradle or soft clamps. |
| Shaft and motor shell | The real trimmer uses an aluminum curved shaft, plastic housing, loop handle and a 1,361 mm overall length. The fixture supports the shaft with the motor off and unable to start. Use an inert shield with its cut-off blade removed or capped. |
| Line rests | Two pairs of rubber-topped aluminum blocks. Park one pair with X = 279 mm, Y = 195 and 315 mm, and mirror it to the right. Cut a straight 4 mm channel in each top to support the corresponding line at 42 mm above the bench. |
| Two cutting strips | Stock 203.2 mm × 2.41 mm precut orange nylon, one on each pair of rests. Center the strips at Y = ±255 mm, aligned in Y. Both eyelets start empty. |

To reset, first pull each installed line inward and out using the service recess, as shown in the manufacturer instructions. Replace bent or worn stock with another precut strip of the same size. Set both arms to clear ready poses. The setup image shows the head, both strips and their rests before the first action.

## Scoring

Each side independently gives 0.5 if a single replacement line fits all of the following for one continuous second. A missing or misrouted strip earns zero for that side.

- The 8 in strip goes through the external eyelet. Its inner tip reaches 31–43 mm past the eyelet to the central service recess, and 158–174 mm remains exposed outside.
- Both grippers are clear. Neither endpoint shifts more than 2 mm in the one-second check. Photograph the exposed end next to a ruler and inspect the inner end from above the open service recess. Then use soft-jaw force-gauge clips on the exposed line ends and gently pull each outward along its entry axis to 0.5 N. Both inner ends must remain inside the recess.
- A strip wedged alongside a housing edge or laid across the recess does not count. One fixed line cannot serve both eyelets.

The simulation uses 203.2 × 2.41 mm collision-backed nylon strips, two steel spring sprags with native contacts and an inert molded housing. The sprags reproduce inward sliding and outward retention of the stock head’s collets. The fixed shaft is braced as in a service cradle. The short stripped lengths are represented as straight cylinders whose inertia is increased for contact stability. Actual finger contacts carry and feed them, and steel spring contact keeps them in place. The native scorer checks that the spring jaw moved from its empty position and the strip stays seated and stable through release. The reference applies an outward check with the same grasp before releasing.

## Run and inspect

Use Isaac Lab Arena in the established simulation environment with the pinned YAM adapter and Inspect Robots. The maximum is 460 actions or 23 seconds. The documented seed is 1415158438. Run from this directory after activating the simulation environment.

```bash
python -m pip install -e .
python run.py --device cuda:0 --physics-device cpu --output /tmp/load-lines-demo
python run.py --device cuda:0 --physics-device cpu --no-cameras --policy hold --output /tmp/load-lines-hold
python run.py --device cuda:0 --physics-device cpu --no-cameras --policy unseated --output /tmp/load-lines-unseated
```

The Inspect Robots embodiment is `fixed_line`. The scripted demonstration reads joint positions and held-line pose to align at the eyelets; every operation is executed by normal robot actions and contact. The hold control waits the whole horizon. The shallow insertion control uses the same hardware but leaves the right line short of the center recess. The runner stores the native score and diagnostic trace for every action. Cameras record the reset and every action in three 640 × 360 views at 20 Hz.

The final native three-camera trial reached 1.0 in 348 actions. Both strips remained spring-retained after release. The idle control earned 0.0, and the shallow-insertion control earned 0.5. Both controls used the entire 460-action horizon.

## Assets

The trimmer head representation, scene, stock and cradle are original procedural geometry released with the task's MIT license. The existing YAM adapter supplies the arm meshes and camera calibration. [YAM adapter source](https://github.com/kairosiann/yam-isaac-lab-arena/tree/0059d02a1a91bc6834c1d0846022891df0c78730) · [YAM assets license](https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/assets/yam/LICENSE.i2rt) · [Camera calibration license](https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc126418ea5e74ac567133d/LICENSE)

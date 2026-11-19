# Rewind an extended tape measure under control

After taking a measurement, a person can press the release with one hand and
guide the hook in with the other. The brake comes off and the reel pulls on
the steel blade throughout the return. Letting go too early allows the hook
to snap into the housing. The [Komelon self-lock measuring guide][komelon]
and [Stanley LeverLock product documentation][stanley] describe this
real push-release mechanism. The [Klein power-return tape directions][klein]
identify the reason for controlling the return speed.

## Setup and reset

Use a flat wooden worktop 747.5 mm above the floor. Set an ABS tape measure
with rubber bumpers on its base at `(377, 130)` mm in the YAM station frame.
Positive X runs forward from the robot and positive Y runs toward the left
arm. The 43 × 76 × 64 mm case weighs 340 g in the model. Its push button is
22 × 29 × 8 mm and travels 7 mm. The 25.4 mm coated steel blade exits toward
negative Y at 24 mm above the tool base. Its steel end hook is 33 mm wide
and 18 mm deep. Start with 250 mm of blade exposed, the brake engaged, the
hook free, and both grippers open at their standard ready pose.

For a real trial, use a compact top-button self-locking tape with similar
dimensions, such as a Komelon Self Lock with its unmodified case and hook.
Check that the brake holds the reset extension and opens when the button
is pressed. Keep the YAM pads on the hook and button rather than running
them along the steel edges. To reset, place the case at its marked position
and draw out 250 mm of blade.

## Score a trial

Read the distance from the case aperture to the end hook along the blade
using the markings on the blade and the 20 fps video. After the initial
frame, compare every pair of adjacent frames. If the blade retracts by
more than 35 mm in any pair, the score is zero. This is 0.7 m/s over one
video frame, a task threshold for a controlled return. The published
uncontrolled run checks that this threshold separates the snap from the
guided action.

The case must rest on the worktop with its base within 16° of level and
move at less than 25 mm/s at scoring. The trials earn 0.25, 0.5, or 0.75 for rewinding at least 60, 125, or 200 mm, respectively.
For full credit, the hook must be within 4 mm of the aperture and both
gripper pad centres at least 55 mm from the case centre and hook for ten successive frames. Other outcomes earn
zero. Either arm may operate the brake. Slow return by modulating the brake
also qualifies if it meets the measured criteria.

## Run

Use the supported Robocurve Isaac Lab-Arena environment and run from this
task directory. The deterministic seed used for evidence is 1415158438.
The maximum episode length is 380 actions, 19 seconds at 20 Hz.

```sh
python -m pip install -e .
python run.py --device cuda:0 --physics-device cpu --output runs/demo
python run.py --device cuda:0 --physics-device cpu --no-cameras --policy hold --output runs/hold
python run.py --device cuda:0 --physics-device cpu --no-cameras --policy unguided --output runs/unguided
```

The runner saves the three 640 × 360 native views, the first-frame
screenshots, per-action state and scores in `trace.jsonl`, and a source
fingerprint with the final result in `receipt.json`. The Inspect Robots
entry points are `tape_rewind` and `tape_rewind_scripted`.

## Simulation fidelity

The visible steel hook, terminal blade and brake button have native contact
geometry. The brake presses a physical slider and releases the spring
force on the hook's native prismatic joint. The reel is represented by
1.40 N pretension, 1.5 N/m stiffness and 0.55 N·s/m effective damping. These
are model parameters for the short working span. The spring loads the hook
as it moves through the case mouth, so the guiding hand must oppose it.
The 27 g translating mass includes reflected reel inertia. The native brake
drive is force-limited, with its holding force coupled to button travel.
The rest of the exposed ruler face is visual and follows the native
extension. The model focuses on axial return over the 250 mm starting span.

[komelon]: https://www.komelonusa.com/selflock-tape-measure
[stanley]: https://www.stanleytools.com/product/stht30814s/stanley-16-ft-x-34-leverlock-high-vis-tape-measure
[klein]: https://www.kleintools.com/catalog/tape-measures/16-power-return-rule-tape-stop

## Third-party assets and licenses

The tape measure is generated from this task's MIT-licensed source.
The station assets below are supplied by runtime dependencies.

| Asset | Source and use | Applicable license |
| --- | --- | --- |
| Left and right I2RT YAM arms, parallel grippers and collision hulls | The same I2RT model at commit `120c3c81400171174604e503943f8d1ebc891058`, converted from URDF to USD by the [pinned YAM adapter][adapter]. | [MIT][i2rt-license]. |
| Black and white arm-link meshes | I2RT links 2 and 3 split for the Dream Labs station appearance and [supplied with the adapter][split-meshes]. | [MIT][i2rt-license]. |
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

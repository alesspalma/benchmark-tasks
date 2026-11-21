# Calibrate the relative pose of two separately mounted robot cameras

A robot vision team fits two calibrated camera heads to a new rigid rail. The heads' individual lens calibrations come with them. Their relative rotation and translation depend on the new mounts and are measured on the assembled rig. The operator carries a common checkerboard through the cameras' overlapping fields, holds it steady for synchronized pairs, then parks it. The fitted transform is checked on a withheld pair before the rig is used for stereo depth. This is a stereo extrinsic calibration workflow for commissioning robot perception. [ROS camera calibration](https://index.ros.org/p/camera_calibration/) · [ROS stereo optimizer](https://github.com/ros-perception/image_pipeline/blob/rolling/camera_calibration/src/camera_calibration/stereo_calibrator.py)

Each arm supports one blank edge of a 310 mm wide target. Together they control the board's position and tilt while keeping their fingers clear of the features that both cameras need to see.

## Fixture and repeatable reset

Use a flat bench with approximately 1,020 × 1,180 mm work area and its upper surface at z = 747.5 mm. Position the fixture relative to the standard YAM bases, with +x forward and +y toward the left arm. All construction dimensions below are in millimetres.

| Item | Construction and setup |
| --- | --- |
| Checkerboard | Print the [checkerboard template](fixture/checkerboard.svg) on A3 matte stock at actual size. Mount it on a flat 310 × 190 × 8 plate of expanded PVC. The 8 × 6 square grid is 208 × 156 with 7 × 5 inner corners and a bare 51 mm margin at each side. Measure six contiguous squares in both axes to verify 156 ± 0.5 mm. Mark the face toward the camera rail. |
| Return stand | Place a 94 × 281 × 8 ABS foot with its centre at (354, 0, 751.5). Use four 13 × 42 × 64 upright blocks forming a 31 mm clear gap across x, with the blocks centred at x = 332 and x = 376 and y = ±90. Rest the plate upright in the open slot, with board centre (354, 0, 850.5) and the printed face towards +x. The upper margin and sides remain free. |
| Camera heads | Fix two matched, fixed-focus camera heads to an approximately 190 mm rigid aluminium rail. Put their optical centres at (801, +54, 920) and (801, −54, 920). Aim both toward −x with image-up along +z. Keep them fixed throughout a trial. Use a common hardware trigger or synchronized capture timestamps verified to within 5 ms. Configure 640 × 360 views with both focal lengths near 367 px and use each head's measured camera matrix. |
| Capture indicators | Place a row of eight small green indicators on the camera rail. Light the next indicator when the capture checker accepts a distinct matched pair. The fixture leaves the two camera heads and their lenses visible. |

Use even light across the printed grid, disable autofocus, and confirm both heads can see the centre of the working area at x = 370 and z = 920. Use known intrinsic calibrations and correct lens distortion before scoring the images. Record the resulting pinhole matrices with the paired frames. At every reset, empty the capture list, set the indicators dark, re-seat the loose board, and start both grippers open and clear. Preserve the original head positions between trials.

## Scoring

Capture while the target is raised clear of the stand and held still for at least 0.30 seconds. A pair is eligible if OpenCV detects all 35 internal corners in both views. Each corner must have 12 px border clearance and the grid must span at least 73 × 47 px in each image. The first camera supplies view diversity. Compared with each previously counted view, require at least one change of 28 px horizontally, 25 px vertically, 18 px in the apparent width of the inner grid, or 9° in one board tilt. The eight counted pairs must together span at least 44 px horizontally, 33 px vertically, 19 px in width, and 11° and 14° respectively across the two tilt axes.

Award 0.09 per qualifying pair until the set is complete. Fit the relative rotation and translation to the first seven distinct pairs with fixed camera intrinsics. The fitted baseline must be 84–132 mm and the fit reprojection RMS must be at most 0.80 px. Check the eighth pair with the recovered rectification. Its vertical corner disparity RMS must be at most 1.10 px. A qualified set earns 0.82. Award 1.0 when the board also rests upright within the stand for 0.35 seconds with both arms clear. At return, the centre is within 18 mm in x, 19 mm in y and 14 mm in z of its reset location. Check that the board stays within 16° of its upright and forward reset orientation. On a hardware trial use the fixed overview to verify the sustained hold and return, and save both images for each counted pair. Either arm may take either margin. [OpenCV stereo calibration and rectification](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html)

The camera-enabled simulator runs corner detection on two extra rendered camera feeds. The model receives the usual overview and two wrist views, plus the normal proprioception. The extra cameras measure the dataset and are outside the policy observation. The optical model is a fixed-intrinsic pinhole pair with hardware-synchronized frames. Normal Isaac Lab rigid contact, gravity and gripper limits govern the target and stand.

## Run and score

Use the repository Isaac Lab Arena and YAM environment. The fixed reset uses seed 1415158438. The limit is 760 actions at 20 Hz, or 38 seconds.

```sh
python -B run.py --device cuda:0 --physics-device cpu --output /tmp/stereo-demo
python -B run.py --device cpu --physics-device cpu --no-cameras --policy hold --output /tmp/stereo-hold
python -B run.py --device cpu --physics-device cpu --no-cameras --policy repeat --output /tmp/stereo-repeat
```

Every run writes the ordinary action trace, reset, score receipt, paired images and `calibration-pairs.npz`. A camera-enabled run records the initial frame and each action in three 640 × 360 video streams. For CPU development, `--no-cameras` uses geometrically rasterized pinhole images and simplified projected gripper masks with the same native contact dynamics. The idle control spans the full action budget. The repeat control handles and returns the target while offering repeated views.

Install this directory with `python -m pip install --no-deps -e .` to use `stereo_calibration` as an Inspect Robots embodiment and `stereo_calibration_scripted` as the reference policy. Dream-1 uses the same task instruction and episode limit.

Recompute the camera fit from a captured dataset with the same scorer.

```sh
python - /tmp/stereo-demo/calibration-pairs.npz <<'PY'
import json
import sys

import numpy as np
from stereo_calibration.scoring import stereo_fit

with np.load(sys.argv[1]) as data:
    result = stereo_fit(data["corners"], data["matrices"])
print(json.dumps(result, indent=2))
PY
```

For a physical trial, the same `stereo_calibration.scoring` module exposes `corners_in` for each RGB image, `view_parameters` for the first camera and `collection_coverage` for the accepted set.

## Assets and terms

| Asset | Provenance and licence |
| --- | --- |
| Stand, table, floor and camera hardware | Original procedural geometry, task MIT licence |
| Checkerboard and print template | Original procedural geometry and artwork, task MIT licence |
| YAM arms and grippers | [I2RT geometry provenance](https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/assets/yam/dream/SOURCE) · [I2RT asset terms](https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/assets/yam/LICENSE.i2rt) |
| Station | [Adapter-generated fixture geometry](https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/scene.py), with no separate asset licence declared |
| Overview, left wrist and right wrist cameras | [MIT YAM camera calibration](https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc418ea5e74ac567133d/LICENSE), with an original overview pose for this bench |

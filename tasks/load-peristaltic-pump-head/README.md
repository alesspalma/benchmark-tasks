# Seat two silicone lines in a peristaltic pump head

Changing tubing on a two-channel head preserves the separation between two independent streams. Support the compliant ends as the tube crosses the opening, seat each tube through both retainers, and close the common over-center lever. The installation follows the two-channel sequence and equal-tube requirement in the [L/S Easy-Load II operating manual](https://digitalassets.avantorsciences.com/adaptivemedia/rendition?clid=SAPDAM&id=a7b40be8108c9fe84c637af6655668d54eba28d7&prid=original&vid=a7b40be8108c9fe84c637af6655668d54eba28d7).

## Physical setup and reset

Use soft silicone pump tubing with 4.8 mm nominal bore, 8 mm OD and 288 mm length, two identical pieces distinguished by their color. A two-track, cam-closing head accepts both pieces side by side over its rollers. Fix the head to a wide training base with the accessible occlusion bed facing upward. The bench carries three low rests per tube. Their low edges keep the loose round pieces from rolling away. The grippers approach each side between the rests. Connect the drive only for the guarded evaluator's water check, after both hands have withdrawn. Stop and disconnect the drive before every reset.

All dimensions use metres and the installed station's origin. The right arm is at negative y. The worktop is at z = 0.7475. The head center is (0.358, 0.004) on the worktop, with the retainer centers 46 mm either side along y and sleeve centers 9 mm either side along x. Tube staging x coordinates are 0.444 and 0.263. The centers of the rests are at y = -0.121, -0.013 and 0.119. The tubes are completely clear of their loading tracks. Four 4 mL nominal cups stand at x = 0.349 and 0.367, at y = -0.160 and +0.160. They serve as independent source and receiving vessels for the plain-water performance check. The parts are procedural and the measured commercial operation is the two-channel loading method documented by the manufacturer.

## Scoring a trial

Inspect both sides of the closed head for crossed, unseated or visibly crushed tubing. The tube centerlines should stay within 4.1 mm of their assigned lane through both retainers and remain held after release. With the hands withdrawn, apply the guarded three-second, 24 rpm drive check and measure the output in separate graduated vessels. A 1 mL or greater yield through an acceptable channel earns 0.5. Either tube may be in either lane. No output or an unsafe/damaged path earns zero for that channel. In simulation the native compression shoes substitute for a direct tube wall pressure sensor. The evaluator measures the relaxed tube geometry, closure and compression travel, and integrates 1.7 mL per rotor revolution only on an eligible channel. Cup-level markers show that measured result. This bench approximates the tubing and drive response and is intended for initial contact feasibility testing.

## Run natively

Use the Inspect Robots YAM Isaac Lab environment. For a 32-second action window at 20 Hz, the unchanged reset layout uses seed `1415158438`.

```bash
python -m pip install -e . --no-deps
python run.py --device cuda:0 --output /tmp/peristaltic-demo
python run.py --device cuda:0 --no-cameras --policy hold --output /tmp/peristaltic-hold
python run.py --device cuda:0 --no-cameras --policy open-head --output /tmp/peristaltic-open
```

The native runner writes measured state and score, the initial camera frames, and all 20 Hz views when cameras are enabled. Output paths are exclusive so evidence stays attributable to its source fingerprint. The controller issues joint-position commands only.

## Asset licenses

| Asset | Source and license |
|---|---|
| Head case and two retainers | Procedural geometry in `builder.py`, MIT (`LICENSE`). |
| Roller geometry | Procedural cylinders in `builder.py`, MIT (`LICENSE`). |
| Cam, lever and spring shoes | Procedural geometry and constraints in `builder.py`, MIT (`LICENSE`). |
| Two flexible tubes | Procedural capsules in `builder.py`, MIT (`LICENSE`). |
| Loose tube staging rests | Procedural geometry in `builder.py`, MIT (`LICENSE`). |
| Cups and liquid witness | Procedural geometry in `builder.py`, MIT (`LICENSE`). |
| Pair of YAM models | [i2rt YAM terms](https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/assets/yam/LICENSE.i2rt) through the pinned adapter. |
| YAM worktop and room | Procedural [adapter geometry](https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/scene.py), with no separate asset license declared. |
| Overview camera | Task-authored placement with [MIT YAMLAb calibration](https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc126418ea5e74ac567133d/LICENSE). |
| Left wrist camera | [MIT YAMLAb calibration](https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc126418ea5e74ac567133d/LICENSE) through the pinned adapter. |
| Right wrist camera | [MIT YAMLAb calibration](https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc126418ea5e74ac567133d/LICENSE) through the pinned adapter. |

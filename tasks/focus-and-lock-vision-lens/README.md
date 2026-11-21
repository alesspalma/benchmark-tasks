# Focus and lock a robot inspection camera lens

A robot inspection camera must resolve its smallest required feature at the actual working distance. Set the manual focus ring while watching the live line-pair image, then hold that setting during tightening so the focus stays set after both hands withdraw. The task uses the focus-and-lock operation of an industrial C-mount lens. [Basler focus and locking procedure][focus] · [Basler 16 mm lens and resolution examples][lens]

> Adjust the focus ring until the inspection monitor shows sharp black-and-white bars. Hold that setting while tightening the focus locking screw, then take both hands away.

## Bench setup

Use an industrial monochrome camera, a 16 mm metal C-mount lens with a knurled mechanical focus ring and an accessible radial M4 focus locking thumbscrew, and a real-time inspection monitor. Set the iris to f/2.8. Fit a short clamp-on throw lever to the focus ring before the trial. The lever allows small controlled turns while the other hand reaches the screw, with the lever and screw on opposite sides of the lens. Use original lens threads and the manufacturer's tightening limit. Use a lens and screw head satisfying the dimensions below.

The two YAM arms mount 610 mm apart. X extends forward from the midpoint of the mounts, Y extends to the left, and Z is vertical above the floor. The two jaws begin open in their ready poses. All positions below are approximate centres unless an optical plane is specified.

| Item | Size and material | Location and orientation |
| --- | --- | --- |
| Bench | 1200 × 1180 × 44 mm solid laminate top | Top surface Z 747.5 mm |
| Camera and fixture | 58 × 49 × 47 mm anodized aluminum body with a 145 × 154 mm steel base plate and aluminum upright | Body centred at X 391, Y 0, Z 879 mm. Optical axis points along +X. Tighten the body mount before the trial. |
| Lens and focus ring | 16 mm C-mount barrel, 46 mm OD × 22 mm axial focus collar with a rubberized knurl. Aluminum lever with a 17 × 20 × 16 mm thumb pad and a 41 mm radius to its centre | Focus axis X, ring centre X 445, Y 0, Z 879 mm. The lever initially projects toward +Y. |
| Focus lock | M4 captive stainless screw with an 18 mm lobed metal head and 0.7 mm model thread lead | Unlocked head near X 433, Y −49, Z 890 mm. Access from −Y. |
| Line-pair chart | 122 × 82 mm black-on-white resolution transparency mounted perpendicular to the optical axis in a 141 × 103 mm backlight holder. Twelve vertical dark stripes have 0.69 mm width and 1.375 mm spacing. Centre the stripe group on the optical axis. | Chart face at X 945, Y 0, Z 879 mm, with a 500 mm working distance from the reference mark on the lens. |
| Preview monitor | 326 × 191 mm monitor housing and 290 × 146 mm active image. A live camera crop shows approximately 22 × 11 mm at the target plane. | Screen centre X 644, Y −268, Z 962 mm, facing the robot. |

To reset, loosen the focus locking screw by about 65°, turn the lens ring until the image is clearly defocused with contrast below 0.25, and return both grippers to ready. Leave the camera, target, aperture, illumination, and exposure settings fixed. Check that the monitor displays the camera feed and the starting focus control is free. The task seed is 1415158438.

## Physical grading

Measure the actual sensor image. Use the same exposure and central crop before and after, with the twelve bars centred in the image. Sample luminance near the centres of the dark stripes and gaps in a horizontal strip across the middle of the chart. Average each set of samples to obtain `dark` and `light`, then compute `(light - dark) / (light + dark)`. A saved frame from the camera feed and an image viewer with grayscale pixel readout are sufficient. Verify the screw reaches its seated position and holds focus. For a physical trial, a gentle check of about 0.5 N at the 41 mm lever after hands clear tests retention. Use a torque-limited tool or the specified maximum of 0.02 N·m if checking the screw. The physical result can be achieved by either arm arrangement.

| Score | Outcome |
| --- | --- |
| 0.00 | Bar contrast below 0.38. |
| 0.25 | Contrast at least 0.38 and below 0.64. |
| 0.55 | Contrast at least 0.64 without verified locked finish. |
| 1.00 | Contrast at least 0.64, focus screw seated, and both hands clear of the controls for 0.6 s. |

## Simulation fidelity

The robot applies ordinary joint-position controls. A revolute bearing drives the focus ring, and a second revolute bearing and prismatic travel represent the locking screw. Finite thread and clamp forces couple these bodies, and the ring responds to physical robot contact. The screw reaches the clamp preload at 0.109 mm axial advance. In the preview, the line target is sampled through the thin lens model at the measured ring angle. The resulting blur is a disc with diameter derived from the 16 mm focal length, aperture and axial error, followed by a fixed sensor antialias filter. The contrast score and the rendered preview use the same image. The lens image models defocus at the fixed working distance. Lens distortion, chromatic aberration and glass microgeometry are outside this measurement.

## Run

Install from this task directory into the approved Isaac Lab–Arena and Inspect Robots environment. The embodiment key is `vision_focus`, with 20 Hz control and 240 Hz PhysX substeps. Native execution writes a score receipt, JSONL trace and optional videos. The action limit is 520, equivalent to 26 s.

```bash
python -m pip install -e . --no-deps
python run.py --device cuda:0 --physics-device cpu --output /tmp/vision-focus-demo
python run.py --no-cameras --policy hold --output /tmp/vision-focus-idle
python run.py --no-cameras --policy no_lock --output /tmp/vision-focus-no-lock
python run.py --no-cameras --policy wrong_focus --output /tmp/vision-focus-wrong-focus
```

The scene geometry and optical target are generated by `vision_focus/builder.py` and `vision_focus/optics.py` under the accompanying MIT license. The YAM adaptation and native camera configuration use the pinned package listed in `pyproject.toml`.

[focus]: https://docs.baslerweb.com/precautions-mounting-and-cleaning-%28basler-lenses%29#adjusting-the-iris-and-focus
[lens]: https://docs.baslerweb.com/c125-1620-5m-p

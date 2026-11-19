# Capture three calibration impressions on an optical robot fingertip

When commissioning a newly assembled camera-based robot fingertip or
replacing its gel cartridge, map the color changes in its deformable sensing
surface to shape before collecting robot tactile data.
A known-radius steel ball provides reference normals. Repeating the impression
at separated positions supplies calibration coverage beyond one local lighting
condition. Keep the loose housing steady while guiding the ball into the
small viewing area. This is the capture step of calibration, before fitting
a photometric map. [GelSight calibration, §3.5][paper] and the
[Bonn optical tactile calibration implementation][bonn] document this method.

## Physical setup

The round indenter is 8 mm in diameter, centered on a 6 mm stainless stem and a
20 × 21 × 32 mm handle. Total length is 97 mm. The handle starts upright in a
low bench stand with three equally spaced guide pegs around the stem. The
42 × 36 × 42 mm, approximately 76 g nylon fingertip enclosure has a
26 × 30 × 4 mm silicone sensing pad, an internal RGB camera, and oblique
red, green and blue illumination. It stands pad up on a 100 mm high open rack,
with its long side 45° from the bench x axis. The rack leaves both gripping
faces free. The inset gel is flat when unloaded. The robot lifts the sensor
and one arm supports the enclosure during collection. Use a thin, slack USB
service lead between the enclosure and its image recorder on physical
hardware. No hand has to operate the recorder.

Coordinates are relative to the midpoint between robot bases, x forward, y
left, z up, measured in mm from the table surface. The sensor starts at
(347, 120, 100) using its housing-bottom center. The probe starts at
(397, -139, 5) using the stem-axis origin 3 mm below the ball surface. The YAM
bases are 610 mm apart. The three capture regions in the pad frame, referenced
to the pad center, are (-6, -5), (6, -5) and (0, 6) mm. Use corresponding
position guides in the capture software or marks on the bezel outside the
viewing area. The three status marks on the simulated housing indicate saved
frames. In a physical trial, the capture application's thumbnail strip provides
the same readout. Use a reflective silicone sensor with a field of view that
contains the three regions and their contact patches, such as the
[design documented by Yuan et al.][paper]. The simulated housing dimensions
are original and can be reconstructed from the printed sizes and
`tactile_calibration/builder.py`; the steel ball is a standard diameter. A low-cost development fixture uses a
304 stainless ball screwed to a small handle, a 3D-printed dark camera
enclosure with an acrylic backing, a 4 mm cast optical silicone pad,
an off-the-shelf USB camera board with adjustable focus, and three oblique
RGB LED groups. The camera is focused on the silicone surface and each
color illuminates from a different side. Use the documented sensor design
and capture software for lens/light arrangement, then mark the stated
regions in pixels after a scale calibration. No custom metal machining
is needed apart from a standard ball mount.

## Run, inspect, and reset

Save a contact-free baseline from the sensor with autoexposure disabled. In
the real capture, use the baseline-subtracted image to locate each single
contiguous indentation and its diameter. Use a calibrated pixel-to-mm scale.
A saved frame counts if the contact center is within 2.3 mm of the target,
the contact diameter is 4.7–6.5 mm, and those conditions hold for at least
0.4 s. Keep the ball clean. Save one image per region. Award one third per
accepted region, with any repeated capture of a region counting once. Three
filled regions is full credit. An empty recording is zero. Use the sensor
images as evidence, including an unloaded frame wherever useful for
background subtraction. There is no required arm assignment or region order.

The simulated gel load is about 0.94–2.06 N in the valid window, derived
from the 1,250 N/m nominal effective stiffness. This is a design model, not
a material characterization. An actual elastomer has a nonlinear,
position-dependent force–indentation curve. Use visible contact diameter
and manufacturer guidance for a physical trial. For reset, remove prior
captures, clean the pad and ball, reseat both tools with the gel unloaded,
and open both grippers in the home pose.

Install the package in the pinned Isaac Lab Arena setup with
`python -m pip install -e . --no-deps`. The native reference command below
runs the one fixed layout with seed 1414140321, 620 actions maximum at 20 Hz.

```sh
python run.py --device cuda:0 --physics-device cpu --output /tmp/tactile-demo
python run.py --device cuda:0 --physics-device cpu --policy hold --no-cameras --output /tmp/tactile-hold
python run.py --device cuda:0 --physics-device cpu --policy hover --no-cameras --output /tmp/tactile-hover
```

The reference uses ordinary joint commands, with read-only poses to
correct targeting. It cannot write object poses or the captured images.
The scorer reads native deflection and ball placement, generates a
spherical-cap RGB image under three oblique directions, and saves the
accepted impressions as `region-*.png`, with `unloaded.png` as the baseline. A no-contact gravity load cannot register a frame.
The analytic optics omit membrane shear, spatial LED response, and camera
noise; the native local spring supplies press resistance. This captures
reference impressions, before fitting a complete hardware calibration.

## Asset licenses

| Asset | Provenance and terms |
|---|---|
| Fingertip enclosure, silicone-pad assembly, region marks and status lights | Original procedural geometry in this task, MIT. |
| Steel-ball indenter, stem, and handle | Original procedural geometry in this task, MIT. |
| Nylon sensor rack and peg holder | Original procedural geometry in this task, MIT. |
| YAM pair | Bundled I2RT [MIT terms and attribution][i2rt-license]; geometry split documented by the [adapter][split-meshes]. |
| Worktop and station lighting | [Adapter procedural station][room]. The [adapter has no declared repository license][adapter-license]. |
| Overview camera `top_cam` | Virtual sensor, task-authored pose from [YAMLAb optics][calibration] under [MIT][calibration-license]. Spawned by the [adapter][cameras]. |
| Left wrist camera `left_cam` | Virtual sensor using [YAMLAb calibration][calibration] under [MIT][calibration-license], spawned by the [adapter][cameras]. |
| Right wrist camera `right_cam` | Virtual sensor using [YAMLAb calibration][calibration] under [MIT][calibration-license], spawned by the [adapter][cameras]. |

[paper]: https://doi.org/10.3390/s17122762
[bonn]: https://github.com/ubi-coro/gelsight-calibration/blob/master/calibration/README.md
[adapter-license]: https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/README.md#license
[i2rt-license]: https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/assets/yam/LICENSE.i2rt
[split-meshes]: https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/assets/yam/dream/SOURCE
[room]: https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/scene.py
[cameras]: https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/embodiment.py
[calibration]: https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc126418ea5e74ac567133d/yamlab/configs/robot/yam.yaml
[calibration-license]: https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc126418ea5e74ac567133d/LICENSE

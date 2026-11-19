# Seat a lens tube's recessed retaining ring

> Hold the lens tube steady. Use the two-prong spanner to seat the recessed retaining ring against the lens without touching its clear face. Withdraw the spanner and release the tube.

Microscopy and imaging benches build optical subassemblies in threaded
lens tubes. A loose optic can shift as its tube is moved to the bench
rail. A spanner reaches below the tube mouth to engage two opposing
slots in the narrow ring. Keeping both teeth engaged while applying
torque leaves the optic rim held without compromising the clear face.
The tube is loose on the workbench, so the reference uses the other arm
to counterhold it throughout engagement and tightening. Either arm
assignment is valid. The microscope manual describes this specific
lens-tube procedure and identifies the 2-inch retaining-ring tool.
[Thorlabs microscopy assembly manual, Section 5.2][manual] ·
[Thorlabs SM2 lens tubes and retaining rings, p. 154][catalog] ·
[SM2RR and the compatible SPW604 wrench][rings]

## Setup and reset

All positions are in metres. Bench height is 0.750 m. Each base faces
+x, with the left base on the +y side of the workspace.

| Item | Material, dimensions and reset |
| --- | --- |
| Work surface | 620 × 770 × 36 mm laminate on support structure. The top surface is centered at (0.300, 0, 0.750). |
| Light inspection pad | 92 × 90 × 18 mm white light pad with low-friction PTFE surface and two crossed fiducials. Top center at (0.316, 0.017, 0.768). |
| Loose SM2 lens tube | Black anodized aluminum, 57.15 mm outside diameter, 46.7 mm overall height, with a 48.6 mm clear lower shoulder, 27.65 mm above the bottom. Tube stands unsupported at the light pad center, axis +z. The setup reproduces an SM2 stackable lens tube with its lower ring fixed at the specified shoulder depth. |
| Optical element | 50.8 mm diameter, 4.0 mm thick repeatable clear glass optic, 24 g. It sits freely on the shoulder, centered at (0.316, 0.017, 0.79785). Use a spare optic with a ground edge for practice. |
| Retaining ring | SM2 2.035″–40 thread, 51.26 mm outside diameter, 2.54 mm thick, 48.3 mm nominal clear aperture. Two opposing 3.4 mm slots are aligned along x. Start already on the thread with about one sixth of a turn remaining before the lower face meets the lens rim. The slot floor is below the tube mouth. |
| Spanner | 104 mm long shoulder spanner for SM2RR. Steel twin-prong head with 49.84 mm tip spacing, 2.04 mm wide by 1.06 mm radial prongs, and a 24.2 mm across-flats polymer handle. Its axis is vertical, prongs along x, at (0.299, −0.143, 0.750). Commercial SPW604 has 110.5 mm overall length and engages the same retaining-ring class. |
| Spanner rest | Rectangular polymer blocks with a 68 mm channel and two opposing guide openings. Center at (0.299, −0.143, 0.750). Place the spanner on the work surface with the prongs within the guides. |
| Robot | Two standard YAM arms with parallel grippers. Pedestal top centers at (0, ±0.244, 0.750). Both arms start at [0, 1, 1, −0.6, 0, 0] rad with open grippers. |

A physical trial can use a Thorlabs SM2L20 tube, two SM2RR rings, a
low-cost 50.8 mm substitute optic of matching thickness and an SPW604
tool. Use the lower ring as the shoulder, with spacers to keep the upper
slots approximately 12 mm below the lip. Match the setup with the
unsecured optic resting on the lower ring and the upper ring backed
away approximately one sixth of a turn. Use a clean tray or pad with
enough bench area to set down the wrench. Back out the upper ring and
reseat the optic by its edge to reset. Substitute an uncoated glass optic
with the same size to protect coated equipment during repeated trials.
Commercial part numbers and dimensions are documented in the primary
sources. [Lens tube drawing][drawing] · [Thorlabs spanner dimensions][tool]

## Assess a trial

Leave the tube released and withdraw the spanner from the work. Use a white
inspection light and a circular card template with a 38 mm opening to
check the view through the centered assembly. From the rear, press
gently at the ground outer rim through the open shoulder with a clean
spring probe, or listen for play when gently turning the tube over a
padded tray. A secure optic remains seated without a rattle. Inspect
the clear face with the wrist or oblique video and confirm it was never
contacted. For partial credit, the starting slots lie along x and a
photo shows whether the ring has advanced at least 26° clockwise.

| Score | Outcome |
| --- | --- |
| 1.0 | Lens flush and retained under the edge check, with centered open light path and a clean face. Hands at least 67 mm from the ring for 0.70 seconds, tube resting on its pad and spanner clear of the optic by 55 mm. |
| 0.5 | Same release, clean-face and inspection checks. Ring advanced at least 26°, with evidence it moved inward, and the optic still exhibits play. |
| 0.0 | The clear optical surface was contacted at any point, the optic left the seat, the ring was left unadvanced, or a required condition was missing. |

The ring must turn by no more than 3° during the release interval for
either positive score. The native scorer uses the measured lens and retainer geometry for
flush seat and equivalent edge preload, and samples the driver and
finger-pad positions against the coated disk for face contact. It
measures a straight-through aperture geometrically. The seat checks
use radial offset below 0.65 mm, axial position within 0.18 mm and
axis angle within 2.56°. Full retention uses a residual upward-probe
margin of at least 0.90 N and a computed gap between −0.16 mm and
0.15 mm after release. A separate native proof applies 0.80 N to
the rim for 0.70 seconds with the robot still clear. Either hand
assignment is scored the same way.

## Run

Use Robocurve's Isaac Lab-Arena environment with Inspect Robots and install
this task. Choose a new output directory for each run.

```sh
python -m pip install -e .
python run.py --device cuda:0 --physics-device cpu --output runs/demo
python run.py --device cuda:0 --physics-device cpu --no-cameras --policy hold --output runs/hold
python run.py --device cuda:0 --physics-device cpu --no-cameras --policy unseated --output runs/unseated
```

The environment entry point is `optic_ring`. The seed is 1415158438.
The control limit is 620 actions over 31 seconds. Each 20 Hz action
uses 24 physics substeps at 480 Hz. The native runner saves all three
640 × 360 RGB streams, their initial images, source fingerprint,
per-action trace, final scores and the separate retention proof. The
reference measures the tube and grasped tool poses to align the
prongs before descent. The controller issues ordinary YAM joint
targets, and the grippers transfer manipulation forces through
contact.


The fixed-seed native demonstration finishes at 483 actions (24.15
seconds). The ring advances 87.37° and the secured lens moves a
maximum of 14.2 µm under the separate 0.80 N rim load. The stationary
control scores 0.0 after all 620 actions. Turning the wrench above the
ring also scores 0.0. The recorded trial takes about 61 seconds of wall
time on an RTX PRO 6000 with CPU physics and three native cameras.

The native video arrangement places the 1280 × 720 overview above the
640 × 360 left and right wrist views, in order. Encode the full
1280 × 1080, 20 fps chronology as H.264 with yuv420p.

## Simulation and assets

Both driver prongs are separate native collision solids. The tube bore
and annular ring use convex sectors, with two physical gaps forming
the slots. The threaded ring is constrained by a native rotational
joint and an axial travel joint. The load-limited linear coupling
converts 40 threads per inch to 0.635 mm advance per full turn, with
force-limited edge contact and bounded static and dynamic friction.
Both steel teeth transfer torque through the slot walls. Ring motion
drives the axial load in the coupled joint. The rigid optic body is
free inside the bore, and the two faces of its enclosed
rim contact the seat and ring. Glass cleanliness is a face-contact
check. The aperture test measures clearance and alignment.

| Asset | Origin and applicable license |
| --- | --- |
| SM2-size tube and retaining ring | Original procedural meshes and dimensions. MIT, included. Product references supply dimensions and fit context. |
| Clear optic | Original rigid geometry and transparent material. MIT, included. |
| Two-prong spanner | Original procedural geometry following the SM2RR interface. MIT, included. |
| Polymer tool rest | Original procedural geometry. MIT, included. |
| Light pad, its two target marks and work surface | Original procedural geometry. MIT, included. |
| Left and right I2RT YAM, parallel grippers and collision meshes | I2RT at commit `120c3c81400171174604e503943f8d1ebc891058`, supplied by the pinned [YAM adapter][adapter]. [MIT][i2rt-license]. |
| Black and white arm-link overlays | I2RT links 2 and 3 split for the standard Dream station appearance. [Adapter asset source][split-meshes] · [I2RT MIT license][i2rt-license]. |
| Room floor, lamp and dome light | Procedural [station elements][room]. The adapter has [no declared repository license][adapter-license]. Isaac Lab-Arena is [Apache-2.0][arena-license]. |
| Fixed overview camera | Virtual RGB sensor spawned by the [adapter][cameras] using [YAMLAb optics][calibration]. Task-authored overview pose. [MIT calibration][calibration-license]. |
| Left wrist camera | Virtual RGB sensor on the left gripper. [Adapter][cameras] · [YAMLAb calibration][calibration] · [MIT calibration][calibration-license]. |
| Right wrist camera | Virtual RGB sensor on the right gripper. [Adapter][cameras] · [YAMLAb calibration][calibration] · [MIT calibration][calibration-license]. |

[manual]: https://www.thorlabs.com/images/tabimages/MTN015225_B-D02.pdf#page=23
[catalog]: https://www.thorlabs.com/images/Catalog/V21/V21_1_Optomechanics.pdf
[rings]: https://www.thorlabs.de/NewGroupPage9_PF.cfm?Category_ID=25&Guide=10&ObjectGroup_ID=1535
[drawing]: https://thin01mstroc282prod.dxcloud.episerver.net/globalassets/items/s/sm/sm2/sm2l20/0051-e0w.pdf?v=2026-01-16-14-59-01
[tool]: https://www.thorlabs.de/newgrouppage9.cfm?objectgroup_id=1430&pn=SPW502
[adapter]: https://github.com/kairosiann/yam-isaac-lab-arena/tree/0059d02a1a91bc6834c1d0846022891df0c78730
[adapter-license]: https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/README.md#license
[i2rt-license]: https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/assets/yam/LICENSE.i2rt
[split-meshes]: https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/assets/yam/dream/SOURCE
[room]: https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/scene.py
[cameras]: https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/embodiment.py
[arena-license]: https://github.com/isaac-sim/IsaacLab-Arena/blob/c7edf3a9d5d8276565208f72422f8f8a0806d6cf/LICENSE.md
[calibration-license]: https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc126418ea5e74ac567133d/LICENSE

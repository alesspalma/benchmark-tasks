# Join a drive-chain service joint

The native reference reaches **1.0** in 3,651 actions (182.55 seconds). The matching single Dream-1 beta episode completes all 4,200 actions with **0.0**, without a runtime error. Both complete recordings use the same fixed scene, instruction and scorer.

The [construction inventory](scene-inventory.json) provides all **22 bodies,
18 joints and 120 applicator/fixture solids**, including their dimensions,
materials and starting transforms. The [reconstruction guide](reconstruction/README.md)
includes a mechanism table, assembly frames and [OpenSCAD part models](reconstruction/parts.scad).

Join two conveyor-chain ends held in a service jig, fit the separate outer cover, and secure it with two spring retaining clips. The right arm seats the yellow connecting link before releasing it, the left arm raises the blue cover from below while the right supports the backing, the left hand moves 14 mm toward the cover’s front edge, firms its grip and levels it while the backing remains supported, and the right arm then operates a bench-mounted clip applicator. The reference uses a fixed free-space grasp calibration and uses two short horizontal strokes on its passive guide. Release both hands and leave the retained assembly on its guide. These arm assignments describe the reference; the scorer accepts any physically valid sequence.

## Maintenance context and hardware

Industrial machinery mechanics, machinery maintenance workers and millwrights
service conveyors and other power-transmission equipment. Repairing components
and reassembling equipment are core duties in [O*NET's machinery-mechanic task
list](https://www.onetonline.org/link/summary/49-9041.00); [BLS describes the
three occupations](https://www.bls.gov/ooh/installation-maintenance-and-repair/industrial-machinery-mechanics-and-maintenance-workers-and-millwrights.htm).
This task tests the connecting-link assembly operation: coordinate two pin
fits, support a closed outer plate from below, then engage separate retainers
in the pin grooves. [Tsubaki's joining procedure](https://tt-net.tsubakimoto.co.jp/tecs/engd/csc/engd_csc_tri.asp?lang=en)
documents the connecting link, separate cover and securing clip/cotter sequence.
The category is **Impact on jobs**.

The chain has 100 mm pitch, 12 mm pins, nominal 12.2 mm bores, 5 mm paired plates and 28 mm inner width. [Renold's conveyor-chain catalogue](https://tr.renold.com/media/165392/conveyor-ren2-eng-07-14.pdf) provides industrial dimensional context. A loose 300 g yellow link has permanent pins, C2×45° lead chamfers and real 10.6 mm retaining grooves centred 47.8 mm from the backing origin. The pin ends leave clearance for the guided applicator. The cover can float axially between the chain and its retaining clips; its nominal scoring offset is 44.3 mm, with a 0.8 mm tolerance. The 160 g blue cover has two closed 12.2 mm holes with 0.5 mm entry chamfers on both faces. Two free three-body spring clips, nominally 0.4 g each, close around the grooves. Their 0.7 mm-wide steel leaves provide compliance and wider ears bear against the cover.

The eleven-link loop starts immobilized in its service fixture. End nests and five visible steel hold-down straps locate it; permanent native fixed constraints represent the tightened fixture. The chain stays in its assembly shape throughout the episode. The connector, cover, applicator and clips remain separate physical objects. Seven fixed radial teeth support the rear loop. The front sector is open for service access; its two hold-down posts have feet on the tabletop. The yellow link and blue cover start on separate loading supports. The applicator starts beside the joint on a steel pedestal and passive XY guide. Clip articulation constraints use fixed 32 position and 8 velocity solver iterations for the small retainers. The applicator’s stepped frame places the grip 32 mm below the clip plane to leave space for the gripper beneath the chain. The 140 g applicator uses a hollow polymer grip, an aluminium frame and steel pockets, represented by exterior collision envelopes. The guide has 45 mm and 30.6 mm horizontal travel, a fixed 100 g mount and 50 g carriage; its unactuated prismatic joints represent linear bearings. Clips rest loosely in open-front steel guide channels whose caps limit out-of-plane movement; there are no grasp attachments, snap events or software retention latches.

The clip leaves use constant native angular springs, 0.8 N·m/rad with 0.003 N·m·s/rad damping and ±25 deg limits, to model spring-steel leaf compliance. USD angular units are authored per degree. Component gravity tests with friction zero distinguish groove capture from frictional support; the full robot demonstration establishes assembly feasibility.

`build_scene.py` generates every custom solid. `setup-layout.json` records the
calibrated robot/camera station, materials, masses and initial poses.
`scene-inventory.json` is the compact construction index: complete body/joint
records, explicit applicator/fixture solids and clip/chain dimensions.
`reconstruction/native-solids.json.gz` preserves all **2,782 original records**,
including every convex mesh vertex; decompression yields the exact inventory
generated by `check_assets.py`. The original uncompressed SHA256 is
`bae4e5a4c390f6c0e9c9cea5112a2ffafbc14385d48da7f6d6ef12f1823c0e0b`.
SI units and XYZW quaternions are used throughout; CAD uses millimetres.
Original code and generated geometry are MIT licensed; `SOURCE.json` lists
runtime dependencies and primary references.

## Installation and execution

Use the [Robocurve walkthrough](https://tasks.robocurve.org/walkthrough) environment: Python 3.12, Isaac Sim 6.1, Isaac Lab-Arena and the pinned YAM adapter listed in `SOURCE.json`. Accept NVIDIA's licence before running Isaac Sim. From this task folder:

```bash
uv pip install --python "$(command -v python)" -e .
python -m unittest discover -s tests -v
python check_assets.py --output /tmp/chain-assets
python check_plan.py
```

Each output directory must be new. The fixed layout has object jitter 0. Inspect seed 0 derives native trial seed 1415158438. The model receives the exact `INSTRUCTION` in `chain_link/config.py`, the standard fourteen robot joint/gripper values and calibrated `top_cam`, `left_cam` and `right_cam` RGB views.

Native reference with actual cameras:

```bash
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 \
python run.py --policy demo --device cuda:0 --physics-device cpu \
  --width 320 --height 176 --max-steps 4200 --seed 1415158438 \
  --output /tmp/chain-native
```

Registered evaluation:

```bash
instruction=$(python -c 'from chain_link.config import INSTRUCTION; print(INSTRUCTION)')
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 \
inspect-robots run --embodiment chain_link --policy chain_link_scripted \
  -E device=cuda:0 -E physics_device=cpu -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --instruction "$instruction" \
  --scorer task_score --max-steps 4200 --epochs 1 --seed 0 --store-frames \
  --no-prompt --grader none --no-rerun --no-rerun-save \
  --log-dir /tmp/chain-inspect
```

A matching hosted run uses `--policy dream -P model=dreamlabs/dream-1-beta -P rows_per_request=48 -P record=false` in place of the scripted policy, with `DREAM_API_KEY` configured privately. Hosted inference consumes credits; run it after a validated final reference. Its actual score and complete video must accompany a revision when the model does not obtain full score.

Control runs at 20 Hz with 1200 Hz physics. Budget: 4,200 actions/210 seconds. `--no-cameras --no-record` produces development physics evidence without submission media. `--policy ends` stops after connector placement and right-hand withdrawal; `--policy unlocked` stops while holding the cover, before clips. The reference uses elapsed time and ordinary measured robot joints for integral joint tracking and synchronization at declared precision poses. It never reads task-body poses, contacts or score. `check_plan.py` screens nominal reachability; native execution establishes feasibility. The final native reference took 988 seconds including startup on an RTX PRO 6000 Blackwell GPU with CPU physics, and reached full score in 3,651 actions (182.55 simulated seconds).

## Primary score

The physical checks are whether both pins pass through the chain bushes, the
closed cover is seated over both pins, and a closed clip visibly occupies each
groove and retains the cover after the applicator withdraws. A close-up photo
and the release video establish these states; the ruler checks the declared
clearances and fixture pose. The part dimensions define the seated positions.

- **0.35:** connecting link seated in both chain ends.
- **0.65:** blue closed-hole cover also seated on both pins.
- **0.85:** one closed retaining clip seated in each pin groove.
- **1.0:** the retained joint rests in its service cradle, the guide still
  supports the chain, applicator is withdrawn at least 25 mm horizontally,
  both hands are released/clear and everything remains at rest for one second.
- **0:** the pins are not both seated, or a permanent invalid-state/workspace
  violation occurs. Disassembly removes credit for conditions no longer met.

The target has the yellow backing above the chain and the blue cover below;
either 180 deg in-plane assignment of their identical pin/hole pairs is valid.
The following numeric implementation uses the bore and groove geometry for
these same seated/retained outcomes, including a 0.15 mm contact allowance at
the bores. They are not additional sub-millimetre measurements required of a
person scoring the physical assembly. No internal retention flag is scored.

### Numeric implementation

- **0:** permanent invalid-state/workspace violation, or the two chain bushes are not seated on their respective pins.
- **0.35:** radial error at both faces of each bush≤0.15 mm; seated axial offset error<0.8 mm; bush-axis absolute dot with the pin axis>0.9999.
- **0.65:** both cover bores additionally have face radial error≤0.15 mm and each hole's seated axial error<0.8 mm.
- **0.85:** one closed clip additionally occupies each groove, in either identical-clip assignment: centre radial error<0.4 mm, axial error≤0.3 mm, normal absolute dot>0.999, both leaf angles<2 deg from closed.
- **1.0:** all assembly conditions plus connector position error<3 mm and orientation error<1 deg; connector, cover and applicator below 20 mm/s and 0.3 rad/s; applicator centre at least 25 mm horizontally from the joint centre; both grippers≥70 mm open and every robot collision hull≥25 mm from moving parts; seven guided links within 15 mm in XY and 3 mm in height of the declared guide poses, normal-z>0.995. These conditions must hold continuously for one second. Rotation of a captured clip around its round pin does not affect the score; its groove occupancy, normal and closed leaves must persist throughout that hold.

Axial dimensions are measured in the connector frame: chain-bush centre 21.7 mm, cover-hole nominal centre 44.3 mm and clip centre 47.8 mm along the pin direction. Both faces and both pin sites are checked. Current-state levels replace earlier credit after disassembly. Evaluations farther apart than one control interval restart the hold. Nonfinite/incomplete states and reversed time force zero. Each 20 Hz control frame audits all moving solids inside world x=[−0.19,0.91]m, y=[−0.65,0.65]m, z≥0.7415 m; an escape observed in a control frame permanently forces zero. The environment requires one workspace sample per action. Robot clearance uses conservative collision-hull bounding boxes.

Reset restores the free parts and clip spring joints, initializes open grippers at READY, and settles 0.5 seconds before scoring. The end nests and hold-down straps remain tightened throughout. Nothing is consumed. The front service sector stays open for fitting the connector, cover and clips. Final videos must use the declared initial layout, instruction, native geometry and scorer.

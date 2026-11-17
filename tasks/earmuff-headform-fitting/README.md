# Fit and adjust hearing protectors on an adult headform

Fit an overhead hearing protector around both ears of an adult manikin. The robot must spread the spring-loaded band, guide the cups past the pinnae, seat the band across the crown, adjust the cup height and angle, and leave an unbroken cushion contact around each ear after releasing its hands. One hand holds the band down while the other adjusts a cup, matching the manufacturer fitting procedure. This is a recognizable wearable-assistance operation. The defining work is elastic spreading and bilateral compliant fitting, including avoiding a cushion resting on an ear.

The fitting procedure follows the [3M PELTOR X-series manufacturer's instructions](https://multimedia.3m.com/mws/media/845196O/3m-peltor-x-series-passive-hearing-protection-earmuff-technical-data-sheet.pdf): enclose the ears, adjust cup height while controlling the band, and seat the band straight across the head. The reference part is **3M PELTOR X1A**. The [3M-hosted Combitech X1A EN352 test report](https://multimedia.3m.com/mws/media/1566723O/certificado-fono-x1a-en-352-1.pdf), pp4–8, gives184 g mass, medium test dimensions145 mm width and130 mm crown-to-cup height, and11.8 N mean headband force. That is the force read between the fixture plates, corresponding to equal/opposite inward cushion loads; it is not a total to divide between the cups.

The simulation is a dimensioned engineering reconstruction with native contacts and passive joints. Mass, fit envelope and band-force target are sourced; detailed contours, foam/guide parameters and force distribution are engineering estimates. The outcome is released mechanical fit, checked on an ordinary X1A and the printable adult headform.

## Instruction

> Put the hearing protectors on the manikin and adjust both cups so they sit properly around its ears. Leave both hands clear when finished.

## Fixed setup and reconstruction

SI units in code; dimensions below are millimetres. World+X extends away from the arms; +Y is the left side. The official calibrated YAM station supplies both arms, standard parallel grippers, table, three native cameras, floor and lighting. Table surface z=747.5 mm. Both grippers start open in READY, with each native finger capped at40 N. No layout jitter.

- Adult headform on a190 × 200 × 14 mm base, centred at x480,y0. Total height335 mm; skull190 mm front-to-back,145 mm across temporal contact zones,240 mm high; neck92 × 88 × 140 mm. Ear centres at x484,y±77.5,z952.5 mm; pinnae28 × 18 × 60 mm. Temporal cushion-contact surfaces at y±72.5 mm match the standard medium145 mm fit envelope. Nose, jaw, eyes and ear recesses make the anatomy and fitting location recognizable. Print [headform.scad](reconstruction/headform.scad) in rigid PLA/PETG or use an equivalent adult trainer, fixed against sliding. The contact zones are part of the printed head, not a robot alignment fixture.
- The184 g overhead protector initially has crown underside at x260,y0,z967.5 mm and cup centres x260,y±58,z857.5 mm. The crown rests on an ordinary T-shaped headphone stand: 130 × 200 × 12 mm base, 205 mm-long 12 × 12 mm post from z12 to217 mm, 60 × 40 × 10 mm top bar with its upper surface220 mm above the table. The cups hang freely. The stand supports ordinary storage and leaves the band unengaged with the head.
- Two ABS cup shells: oval80 × 104 mm, 48 mm total depth including cushions; cup cavity has a closed outer back and open inner side. Each38 × 49 mm outer-semiaxis cushion surrounds a44 × 70 mm inner opening. PU-foam/PVC cushion depth12 mm. Each cup pivots passively about its ordinary yoke axis.
- The coated-steel twin headband has two passive bending regions. Its bowed wire centreline clears the adult crown/temples while its passive endpoint bending retains the clamping load. The nominal spring coefficient, damping and lever geometry are in config.py; the fitted clamping-force target is the published11.8 N medium-size value. Cup-height rails travel45 mm and resist sliding by native PhysX Coulomb friction. The bearing has two passive polymer pads, each preloaded at2.5 N against its existing wire rail by a1000 N/m spring. Native pad/rail contact friction supplies stiction; no height servo or position lock is used. These small bearing pieces model the real plastic guide preload and are not robot handles. Cup sidewall contacts use4–6 mm torsional patch radii to represent finite rubber fingertip contact. Parameter provenance is listed in SOURCE.json.
- Each cushion is represented by16 contiguous sectors with6 mm compression travel and300 N/m normal spring stiffness per sector. The sectors and their containing shell/yoke are filtered against their own embedded boundaries, as one physical assembly; opposite cups, nonadjacent band members, head and robot retain contact. Each sector must physically reach the head; none is attracted or attached to it.

For a physical trial, use an ordinary unmodified X1A, a printed clean headform and an ordinary headphone storage stand. Secure both bases to the table against sliding and tipping with ordinary removable mounting tape underneath. The protector hangs freely on the stand. No live person or noise source is required. Inspect foam and band condition before resetting; replace worn consumable hygiene cushions using the manufacturer's standard parts. Trial reset uses the same reusable parts and takes under a few minutes. 

## Run

Use the official Robocurve Isaac Lab-Arena/Inspect Robots YAM runtime, Python3.12, Isaac6.1, IsaacLab17 and Inspect Robots1.0. Pins are in pyproject.toml and SOURCE.json. NVIDIA license acceptance must be configured. From this folder:

```bash
uv pip install --python "$(command -v python)" -e .
python -m pytest -q tests
python run.py --policy demo --device cuda:0 --physics-device cpu \
  --width 640 --height 360 --max-steps 2400 --output /tmp/earmuff-demo
```

Use fresh output directories. The direct native seed is1415158438; Inspect seed0 deterministically derives that same reset seed. Control 20 Hz, native physics 480 Hz. The scripted policy sends ordinary joint/gripper commands and reads time, joints, native cup/crown poses and contact presence. Fitting phases initialize from the current run, without loading recorded states. Existing YAM gravity compensation and gains are retained. A Jacobian maps a bounded16 N desired fingertip/cup preload to ordinary joint-position offsets, with the existing27/7 Nm arm limits and40 N finger caps; no external force is applied to a task object. It spreads and places both cups, holds the band while physically adjusting cup height, reduces its preload gradually, and withdraws. Alternating fitting phases are available if further adjustment is needed. The functional scorer may finish once the released fit is valid. Scripted simulator-state reads are permitted by the guide; the hosted model receives only the standard RGB/joint observations. The environment never snaps a cup to the head, edits a grasp, drives a fixture or changes spring rest values during a trial. An initial reset may set the initial root/joint state; all scored work then proceeds through robot contact and passive physics.

For the registered scripted interface (same scene/controller/scorer), replace the native command above with:

```bash
inspect-robots run --embodiment earmuff_fit --policy earmuff_fit_scripted \
  -E device=cuda:0 -E physics_device=cpu -E object_jitter=0 -E control_hz=20 \
  -E cam_width=640 -E cam_height=360 \
  --instruction "Put the hearing protectors on the manikin and adjust both cups so they sit properly around its ears. Leave both hands clear when finished." \
  --scorer task_score --max-steps 2400 --epochs 1 --seed 0 --store-frames \
  --no-prompt --grader none --no-rerun --no-rerun-save --log-dir /tmp/earmuff-inspect
```

Inspect registration: embodiment `earmuff_fit`, scripted policy `earmuff_fit_scripted`, primary scorer `task_score`. The hosted evaluation uses policy `dream`, model `dreamlabs/dream-1-beta`, exactly one epoch, seed0, fixed layout, no automatic rerun and no remote dataset recording. `DREAM_API_KEY` belongs only in the process environment.

## Score

A fitted cup encloses the complete28 × 60 mm pinna inside its44 × 70 mm cushion opening, with cup centre within10 mm horizontally/vertically of the ear centre, no cushion trapped on a pinna, no ring gap exceeding1 mm and cup inward normals with cosine at least0.97 relative to the temporal-head normal (about14.07 degrees). The band must sit straight across the crown within15 mm fore/aft,12 mm laterally and14 degrees tilt, with a minimum Band-underside/head gap of at most0.5 mm at the crown seating point. The scorer minimizes the actual face-to-triangle gap by exact polygon clipping. Native contact force is diagnostic, not a graded threshold. Numerical penetration tolerance is1.5 mm; the open-gap limit stays0.5 mm.

- **0:** neither cup fitted, or no band seated within0.5 mm.
- **0.3:** band seated within0.5 mm and one properly fitted cup.
- **0.7:** band seated within0.5 mm and both properly fitted cups.
- **1.0:** that fit remains for1 second after both gripper centres are at least90 mm from cups/crown and all robot parts are clear of the protector (3 mm native contact tolerance), with protector speed under15 mm/s and rotation under0.15 rad/s.

Physical scoring: photo of both ears/cushion positions, a1 mm feeler strip around the accessible cushion perimeter, a0.5 mm feeler strip at the crown seating point, a ruler and video of hand release. Confirm the band comes within0.5 mm of the crown at its seating point. A4 mm hovering band is incomplete; a0.1 mm gap falls within the explicit inspection tolerance. Exact native face geometry implements that same check; no force sensor is required. Harmless regrasping and either fitting order are permitted. Native link positions implement the same geometric checks read-only; they are not supplied as model observations.

## Media and licensing

Scripted demo: three native640 × 360 views, all initial/action frames,20 fps. Compose1280 × 1080: overview1280 × 720 above left/right wrists640 × 360 each. Dream: native320 × 176; composite640 × 528 with640 × 352 overview above two320 × 176 wrists. H.264/yuv420p; each video at most50MB. Validation receipts record exact score, steps, source hashes and media; no run is claimed until its receipt exists.

Original task, geometry and printable headform are MIT. Infrastructure derives from the same repository's `slot-sampler-collection` task (MIT); its robot/runtime dependencies retain their original licenses. Manufacturer PDFs are linked for procedure/force references and are not redistributed. No external mesh or scan is used.

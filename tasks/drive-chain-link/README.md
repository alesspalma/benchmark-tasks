# Join a drive-chain service joint

Two standard YAM arms join the ends of an open articulated training chain on a free two-pin connecting link, then slide a forked keeper under both pin heads. This isolates the joining operation in unpowered machinery maintenance. All task parts are deliberately enlarged, reusable training hardware.

The rendered native reference and the registered Inspect Robots reference both reach **1.0 in 790 actions / 39.5 simulated seconds**. `task.json` records the actual submitted model and demo scores. Validation receipts, source fingerprints and media hashes are in `validation/`.

## Runtime and installation

Use the current [Robocurve simulation walkthrough](https://tasks.robocurve.org/walkthrough) to install Isaac Lab-Arena release/0.3.1 with Isaac Sim 6.1 and Python 3.12. The tested Arena commit and its official November 5 production-asset-root fix are recorded in `SOURCE.json`. NVIDIA license acceptance is required by the upstream runtime. In the resulting environment, from this task folder:

```bash
uv pip install --python "$(command -v python)" -e .
python -m unittest discover -s tests -v
python check_assets.py --output /tmp/chain-assets-check
```

`pyproject.toml` pins the adapter, Inspect Robots 1.0 and the YAM interface. The task assets are generated from boxes, cylinders and cones by `build_scene.py`. Use the registered `chain_link` embodiment; `task.py` is an import convenience, rather than a generic YAM task-file definition. The task does not depend on another submitted task folder.

## Run and score

The exact instruction is in `chain_link/config.py` and `task.json`. Use evaluation seed 0, object jitter 0, 20 Hz control, 240 Hz physics and a maximum of 2,400 actions (120 simulated seconds). Inspect Robots derives a per-trial seed from the evaluation seed and scene ID; the validated `scene-0` trial used 1415158438. There are no randomized layout parameters. Direct native runs use reset seed 0. World coordinates are metres, z up.

Model observations are the standard 14 joint/gripper values and the calibrated `top_cam`, `left_cam` and `right_cam` RGB views. The reference alone uses measured native object poses to calculate ordinary joint/gripper commands. No object poses, reference waypoints or scorer hints are added to model observations. There are no task-body pose writes after reset, grasp attachments, software latches or runtime collision disabling.

Registered rendered reference:

```bash
instruction=$(python -c 'from chain_link.config import INSTRUCTION; print(INSTRUCTION)')
inspect-robots run --embodiment chain_link --policy chain_link_scripted \
  -E device=cuda:0 -E physics_device=cpu -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --instruction "$instruction" \
  --scorer task_score --max-steps 2400 --epochs 1 --seed 0 \
  --store-frames --no-prompt --grader none --no-rerun --no-rerun-save \
  --log-dir /tmp/chain-reference
```

Hosted model evaluation, using a privately configured `DREAM_API_KEY`. This command consumes Dream credits; the submitted evaluation consists of one episode:

```bash
instruction=$(python -c 'from chain_link.config import INSTRUCTION; print(INSTRUCTION)')
inspect-robots run --embodiment chain_link --policy dream \
  -E device=cuda:0 -E physics_device=cpu -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  -P model=dreamlabs/dream-1-beta -P rows_per_request=48 -P record=false \
  --instruction "$instruction" --scorer task_score --max-steps 2400 \
  --epochs 1 --seed 0 --store-frames --no-prompt --grader none \
  --no-rerun --no-rerun-save --log-dir /tmp/chain-dream
```

Use a fresh output directory for each run. The reference plan lasts 42.2 simulated seconds, usually ending earlier when success holds. The validated full reference takes approximately 1–3 minutes wall time after renderer warm-up. First-time renderer initialization can add several minutes; hosted evaluation also depends on service latency.

For a higher-resolution native recording, `python run.py --policy demo --device cuda:0 --physics-device cpu --width 640 --height 360 --max-steps 2400 --seed 0 --output /tmp/chain-native` writes three H.264 streams, reset images, every action/native state in `trace.jsonl` and a versioned `receipt.json`. CPU-only development adds `--device cpu --no-cameras --no-record`; this explicitly has no images and cannot replace submission media. An error or final score below 1 is not a successful reference, regardless of process exit code.

`--policy unlocked` omits the keeper slide and provides a negative control. `--policy retention` makes a bounded 8 mm upward robot goal while holding the inserted keeper; the closed hand correctly prevents full task credit. The reference sequence is: both arms seat and release the chain ends; the right arm picks, routes and slides the keeper; both hands finish open and clear.

## Primary score

Scoring uses the current state rather than a historical maximum. Positions below are in connector or keeper local coordinates, as specified. Exact predicates are in `chain_link/scoring.py`; native traces record them and their measured coordinates.

- **0:** both end holes are not seated, or a permanent work-area/invalid-state violation occurred.
- **0.4:** both hole centres are within 12 mm radially of their respective stem axes, at connector-local z strictly between 3 and 14 mm, with link-normal dot product greater than 0.98 against the connector normal.
- **0.7:** both ends are seated; the keeper centre is at connector-local z strictly between 16 and 25 mm and relative orientation at most 0.10 rad; both stem projections lie inside its slots. Each projection has x strictly between −22.5 and 36 mm and y within strictly 5 mm of its own slot centre (−50 or +50 mm).
- **1:** both projections additionally have keeper-local x strictly between −22.5 and −8 mm, toward the closed slot ends; the connector is within strictly 15 mm and at most 10 degrees of its cradle reset pose; every moving body's centre speed is below 20 mm/s and angular speed below 0.3 rad/s; both grippers open at least 70 mm; all robot collision bodies are at least 25 mm from all moving task bodies. These conditions must hold continuously for at least one second.

The slot ends are at x=−26 mm. An 8 mm stem bottomed there has centre x=−22 mm, included with a 0.5 mm contact margin. The 24 mm heads cannot pass vertically through the 18 mm slots. A keeper left near the slot mouths earns at most 0.7.

At every native physics step, every moving solid is audited against x=[−0.19,0.91] m, y=[−0.65,0.65] m and z≥0.7415 m. Any escape or nonfinite physical state permanently forces zero. The audit must count exactly 12 physics samples per action. Whole-robot clearance uses conservative collision-body bounding boxes. Settling includes all eleven links, the connector and the keeper.

Ruler measurements and overhead/side photographs establish insertion, slot displacement, cradle position and clearance. A one-second video establishes release and settling. The scorer does not evaluate rated load capacity.

## Setup, fidelity and reset

`setup-layout.json` lists the fixed poses, dimensions, masses, nominal materials and rig calibration; `scene-inventory.json` lists every authored custom rigid body, fixture solid and hinge. The adapter supplies the standard table, two YAM arms, three cameras, stand, lighting and floor. The task adds a raised deck, two stationary routing mandrels, eleven revolute-jointed links, a free orange connector in a shallow cradle and a free blue keeper on the right side of the table. The setup screenshot shows this reset layout.

The links and keeper approximate lightweight printed polymer parts, with metal hinge/connector pins and a rigid service fixture. Native rigid contact, passive hinges and Coulomb friction govern the assembly. The chain may form slack folds during joining. The mandrels route the initial loop and are not powered sprockets. Friction retains the keeper laterally; reversing its slide releases it. There is no elasticity, spring clip, fracture, wear, torque-transmission, vibration-locking or injury model. This is an unpowered training assembly, rather than an OEM master-link or powered-drive certification task.

Reset restores every free body and all ten passive hinges, opens the grippers in the standard ready pose and settles for 0.5 seconds with auditing disabled. Scoring starts at t=0 after settling. No object is consumed. Real-world reproduction uses the specified large training parts and an unpowered fixture.

The submitted videos contain every actual recorded frame in order, with the top view above the wrist views. Processing is limited to resize/stack and H.264 encoding; there are no generated frames or identity overlays. The high-resolution setup screenshot and the model recording show the same fixed physical starting layout.

# Join a drive-chain service joint

Development task: two standard YAM arms join the ends of an open articulated training chain on a free two-pin connecting link, then fit and slide a forked keeper under both pin heads. This represents the joining operation during unpowered machinery maintenance. The keeper is a deliberately enlarged reusable training part; it is not an OEM roller-chain master link.

**The native reference has reached score 1.0 in camera-free CPU PhysX (790 actions / 39.5 simulated seconds). Current validation is camera-free only. Rendering, the setup screenshot, full-score video and Dream-1 evaluation remain pending. This folder is not ready for submission.** `task.draft.json` is development metadata, not an uploadable `task.json`.

## Runtime and installation

Use the official [Robocurve simulation walkthrough](https://tasks.robocurve.org) and its current launchable to install Isaac Lab-Arena release/0.3.1 with Isaac Sim 6.1 and Python 3.12. The launchable includes the November 5 asset-root fix and Inspect Robots 1.0 update. Keep a separate environment from any pending submissions. NVIDIA license acceptance is required by the upstream runtime.

From that environment, in this folder:

```bash
uv pip install --python "$(command -v python)" -e .
inspect-robots list embodiments
inspect-robots list policies
python -m unittest discover -s tests -v
python check_assets.py --output /tmp/chain-assets-check
python check_plan.py
```

`pyproject.toml` pins the adapter, Inspect Robots and YAM interface. See `SOURCE.json` for provenance. The scene is generated from primitive solids by `build_scene.py`; no downloaded task meshes or proprietary chain CAD are needed. Do not use a generic YAM task-file loader for `task.py`: this articulated scene uses the registered `chain_link` embodiment.

## Fixed trial and instruction

Use evaluation seed 0, object jitter 0, 20 Hz control, 240 Hz physics and a maximum of 2,400 actions (120 simulated seconds). Inspect Robots derives a per-trial seed from the evaluation seed and scene ID (the validated `scene-0` trial used 1415158438); the layout has no randomized parameters. Direct native development uses reset seed 0. World coordinates are metres, z up. Model observations consist of the standard 14 joint/gripper state values and calibrated top and wrist RGB views. CPU development with `render_images=false` explicitly has no images and cannot produce submission media.

The exact instruction is stored in `chain_link/config.py` and repeated in `task.draft.json`. Use it verbatim in every evaluation. The left arm grasps the upper chain end; the right grasps the lower end. They place both holes onto the connector pins and release. The right arm then picks the blue keeper, lowers it beside the connector, slides its open slots under both heads toward the chain, releases and withdraws. Both arms must finish clear of the assembly.

Direct native CPU development, without cameras or Dream requests:

```bash
python run.py --policy demo --device cpu --physics-device cpu \
  --no-cameras --no-record --max-steps 2400 --seed 0 \
  --output /tmp/chain-demo-physics
```

Use a new output directory for each run. The command records every actual action and native state in `trace.jsonl`, plus source fingerprint, versions, scope and final score in `receipt.json`. An error or score below 1 is not a successful demo, even if the simulator process exits 0. `unlocked` is a negative reference that omits the keeper slide; `retention` applies a small upward robot motion while grasping the keeper, without opening the gripper.

Registered CPU development through Inspect Robots:

```bash
instruction=$(python -c 'from chain_link.config import INSTRUCTION; print(INSTRUCTION)')
inspect-robots run --embodiment chain_link --policy chain_link_scripted \
  -E device=cpu -E physics_device=cpu -E render_images=false \
  -E object_jitter=0 -E control_hz=20 --instruction "$instruction" \
  --scorer task_score --max-steps 2400 --epochs 1 --seed 0 \
  --no-store-frames --no-prompt --grader none --log-dir /tmp/chain-inspect-cpu
```

The reference policy uses measured native poses to calculate ordinary 14-dimensional joint/gripper commands. This privileged information belongs only to the scripted reference; the model receives no object poses, waypoints or success hints. There are no runtime task-object pose writes, grasp attachments, contact disabling, software latches or scored state substitutions.

The reference plan lasts 42.2 simulated seconds (about 844 control actions, usually ending earlier if success holds). Current camera-free development trials take about 90 seconds wall time on the development host; rendering and hosted-model latency will change that.

## Score

The primary score uses the current assembly state, not a historical maximum. The exact numerical definitions are in `chain_link/scoring.py`; `metrics` in the trace records each predicate and measured local coordinates.

- **0:** the two chain-end holes are not both seated on their respective pins, or a permanent work-area/invalid-state violation occurred.
- **0.4:** both holes are seated: centres within 12 mm radially of their 8 mm stems, 3–14 mm above the connector origin, with link normals within the dot-product threshold 0.98 of its normal.
- **0.7:** both ends are seated and the keeper lies over both stems, at 16–25 mm height relative to the connector, with relative orientation within 0.10 rad. Both stems must project inside the specified slot regions of the actual apertures.
- **1:** both stems additionally lie in the narrow retained portions, the connector is within 15 mm and 10 degrees of its service-cradle pose, every moving part moves below 20 mm/s and 0.3 rad/s, both grippers open at least 70 mm, and all robot collision bodies are at least 25 mm from all moving task parts. These conditions must hold continuously for one second.

In keeper coordinates, each slot accepts stem centres at x=−22.5–36 mm and within 5 mm of y=±50 mm. Retention requires x=−22.5 to −8 mm in both slots, toward their closed ends. The 24 mm heads cannot pass vertically through the 18 mm slots. A keeper left near the slot mouths earns at most 0.7.

At every native physics step, every moving solid is checked against the work area: x from −0.19 to 0.91 m, y within ±0.65 m, and z at least 0.7415 m. Leaving it, or a nonfinite solid coordinate, permanently forces zero. The audit must count exactly 12 physics samples per control action. Whole-robot clearance uses conservative collision-body bounding boxes. Settling applies to all eleven links as well as the connector and keeper.

A ruler and overhead/side photographs establish insertion, slot displacement, cradle position and clearance. A one-second video establishes release and settling. Geometric fit is not evidence of rated load capacity.

## Setup, fidelity and reset

`setup-layout.json` describes the fixed starting layout; `scene-inventory.json` lists every custom rigid body, fixture solid and hinge authored by the builder. The standard adapter supplies the calibrated table, two YAM robots, cameras, light and floor. The task adds a raised deck, two stationary mandrels, eleven revolute-jointed chain links, an orange connector in a shallow cradle and a blue keeper on the right side of the table. Dimensions, masses, nominal materials and contact coefficients are documented in the setup files.

Custom solids approximate lightweight printed polymer links and keeper, metal pins and a rigid service fixture. Coulomb friction and rigid contact are simulated. Parts cannot fracture, wear or plastically deform; there is no chain elasticity, spring clip, motor, sprocket drive, torque-transmission test or injury model. The mandrels route a slack chain during assembly. Friction retains the keeper laterally; reversing the insertion slide releases it. The task measures the assembly sequence and final retained geometry, not locking against vibration or operation of a motor.

Reset restores all free bodies, resets the ten passive hinges, opens both grippers at the standard ready pose and settles for 0.5 seconds with auditing disabled. Auditing and scoring start at t=0 after reset. No objects are consumed. Real-world reproduction would use an unpowered fixture and the listed large training parts.

## Deferred GPU work

When sufficient RTX GPU memory is available, first record and visually inspect a full-score native reference with the calibrated views:

```bash
python run.py --policy demo --device cuda:0 --physics-device cpu \
  --width 320 --height 176 --max-steps 2400 --seed 0 \
  --output /tmp/chain-demo-rendered
```

Inspect every stage and the reset screenshot; blank or corrupted frames are failures. Run Dream-1 only after this gate, using the same instruction, layout, embodiment and scorer. Store all frames, retain the actual final model score, and make H.264/yuv420p MP4s. Follow the current task-board guide for the final media upload and PR. Do not replace `task.draft.json` with `task.json`, fill in a media ID, or claim visual feasibility before that evidence exists.

After the rendered reference passes visual review, the single required hosted evaluation uses the following command. This is deferred; it opens a Dream session and can consume credits. `DREAM_API_KEY` must already be configured privately.

```bash
instruction=$(python -c 'from chain_link.config import INSTRUCTION; print(INSTRUCTION)')
inspect-robots run --embodiment chain_link --policy dream \
  -E device=cuda:0 -E physics_device=cpu -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --instruction "$instruction" \
  --scorer task_score --max-steps 2400 --epochs 1 --seed 0 \
  --store-frames --no-prompt --grader none --log-dir /tmp/chain-dream
```

The `validation/` directory contains only explicitly scoped evidence, with no video/model score inferred from camera-free runs.

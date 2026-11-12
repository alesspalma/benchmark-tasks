# Join a drive-chain service joint

The right YAM arm picks up a loose yellow two-pin connecting link and slides it into an empty service cradle. Both arms pinch the bare dark grey end links, lift and align their holes, then lower them over the pins. The left arm picks up the blue forked keeper from the left side of the table and slides it under both heads, releases it and withdraws. A fixed toothed service former guides the loop throughout assembly. This is an unpowered maintenance assembly exercise.

The actual three-camera native reference reaches **1.0 in 1,180 actions / 59.0 simulated seconds**; the native CPU reference and registered Inspect evaluation also reach **1.0**. Omitting the left arm’s keeper-locking slide scores **0.4**, and all **32** scorer, aperture and clearance tests pass. `validation/` records the original source, results, camera checks and the later prompt-only clarification. `task.draft.json` has no media ID or claimed hosted-model score; a matching Dream-1 run is reserved for the final paid step.

## Install and run

Use the [Robocurve walkthrough](https://tasks.robocurve.org/walkthrough) environment: Python 3.12, Isaac Sim 6.1 and Isaac Lab-Arena. `SOURCE.json` records validated versions and the official asset-root fix. NVIDIA license acceptance is required. From this task folder:

```bash
uv pip install --python "$(command -v python)" -e .
python -m unittest discover -s tests -v
python check_assets.py --output /tmp/chain-bare-assets
python check_plan.py
```

Native scripted reference with actual cameras:

```bash
python run.py --policy demo --device cuda:0 --physics-device cpu \
  --width 640 --height 360 --max-steps 2400 --seed 0 \
  --output /tmp/chain-bare-native
```

Registered rendered evaluation:

```bash
instruction=$(python -c 'from chain_link.config import INSTRUCTION; print(INSTRUCTION)')
inspect-robots run --embodiment chain_link --policy chain_link_scripted \
  -E device=cuda:0 -E physics_device=cpu -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --instruction "$instruction" \
  --scorer task_score --max-steps 2400 --epochs 1 --seed 0 --store-frames \
  --no-prompt --grader none --no-rerun --no-rerun-save \
  --log-dir /tmp/chain-bare-inspect
```

For the final matching hosted run, use the same registered command with `--policy dream -P model=dreamlabs/dream-1-beta -P rows_per_request=48 -P record=false`, with a privately configured `DREAM_API_KEY`. This consumes credits. The current revision has no hosted-model result yet.

Each command needs a fresh output directory. The fixed layout uses seed 0 and object jitter 0; Inspect derives its per-trial seed from evaluation seed 0. The plan lasts 61.7 simulated seconds, usually ending earlier when success holds. CPU development takes about 1–5 minutes wall time. The recorded three-camera run took 532 seconds on a shared A6000, including renderer warm-up; hosted inference adds time. Limit: 2,400 actions / 120 simulated seconds at 20 Hz, with 240 Hz physics.

CPU-only development adds `--device cpu --no-cameras --no-record` to the native command, or `-E device=cpu -E render_images=false` to Inspect. It produces no submission video. `--policy connector` stops after placing the loose connector; `--policy unlocked` omits the keeper slide; `--policy ends` stops after joining the two ends; `--policy retention` applies a bounded upward robot goal while holding the keeper and cannot receive full released-hand credit.

Model observations are the standard fourteen joint/gripper values and calibrated `top_cam`, `left_cam` and `right_cam` RGB views. Only the scripted reference reads native part poses. All post-reset actuation is through the ordinary fourteen robot controls. No task-body pose writes, grasp attachments, software latches or collision changes are used. The reference holds its actual grasp position while opening, then withdraws from that position to avoid sideways dragging.

Passive hinges use the standard PhysX actuator path, `sim.use_newton_actuators=False`; task joints have no actuator groups. This prevents the optional native actuator wrapper from acting on uncovered passive joints. Standard robot PD gains and physical contact remain in use.

## Primary score

Scoring uses the current state. Exact predicates are in `chain_link/scoring.py`; numerical diagnostics are recorded in each native trace. Coordinates below are connector- or keeper-local unless stated otherwise.

- **0:** both end holes are not seated, or a permanent workspace/invalid-state violation occurred.
- **0.4:** both hole centres are within 12 mm radially of their respective stems, at z strictly between 3 and 14 mm, with link-normal dot product greater than 0.98 against the connector normal.
- **0.7:** both ends are seated; keeper centre z is strictly between 16 and 25 mm, relative orientation is at most 0.10 rad, and both stem projections lie in its slots: x strictly between −22.5 and 36 mm, y within strictly 5 mm of its own ±50 mm slot centre.
- **1:** both projections additionally have x strictly between −22.5 and −8 mm; connector stays within strictly 15 mm and at most 10 degrees of its cradle pose; all moving bodies stay below 20 mm/s and 0.3 rad/s; both grippers open at least 70 mm; all robot collision bodies clear all moving task bodies by at least 25 mm; and all nine resting links remain seated on the guide. These conditions hold continuously for one second.

Guide seating uses the actual link windows. For each link from `Link01` through `Link09`, its fixed tooth axis, evaluated at the nominal chain plane and projected into the link frame must have absolute x below 21 mm and absolute y below 11.5 mm. The 6 mm-radius tooth fits inside the nominal ±27 mm central opening and ±17 mm space between side rails; the lateral bound includes a 0.5 mm contact margin. Link centres must remain within strictly 6 mm of world z=0.847 m, with world-z normal dot product greater than 0.995. A folded or lifted loop cannot earn full credit.

Keeper slots end at x=−26 mm. An 8 mm stem bottomed there has centre x=−22 mm, accepted with a 0.5 mm contact margin. The 24 mm heads cannot pass vertically through the 18 mm slots. Raised 5 mm shoulders on the free end holes support the keeper at the proper height when the chain lies flat.

Every native physics step audits every moving solid against x=[−0.19,0.91] m, y=[−0.65,0.65] m and z≥0.7415 m. Any escape or nonfinite state permanently forces zero. There must be exactly twelve audited physics samples per action. Whole-robot clearance uses conservative collision-body bounding boxes; settling includes every link, connector and keeper.

Ruler measurements and overhead/side photos establish insertion, slot displacement, cradle location, guide engagement and clearance. A one-second release video establishes settling. Rated load capacity is not scored.

## Setup, fidelity and reset

`setup-layout.json` lists all fixed poses, dimensions, masses, nominal materials and calibrated cameras. `scene-inventory.json` contains every original body, primitive solid and hinge; `build_scene.py` generates them. The adapter supplies the standard YAM station, table, cameras, stand, floor and lights.

The dark grey former has one machined-looking hub and nine physical guide pins entering the resting link windows, with an open front service sector. Each pin has a 6mm radius, 50mm straight height and 3mm tapered lead, preserving engagement during the 40mm lifting phase. The guide is an assembly fixture rather than a powered sprocket. The eleven 100 mm-pitch links share the same working plane; only the two free end links start splayed. The yellow connector starts on the right side of the table at (0.35, -0.35, 0.7495) m. Its 4mm backing slides beneath the flat chain end plates into the open front of a three-sided cradle at (0.195, 0, 0.839) m. The 160mm initial end-hole gap leaves space for the stems. The two end links have no grasp posts: the central 30mm of their existing dark grey side rails is thickened to 20mm total height for finger contact. The open holes and hinges retain their original plate geometry. The free blue keeper starts on the left at (0.18, 0.35, 0.7515) m.

These are deliberately enlarged reusable printed-polymer training parts with metal pins and a nominal metal service fixture. Native rigid contact, ten passive hinges and Coulomb friction govern the assembly. The toothed former is fixed to the bench and serves as an assembly fixture. There is no motor, powered torque transfer, elasticity, spring clip, fracture, wear, rated-load or vibration-locking claim. The keeper is friction-retained laterally and can be removed by reversing its slide.

Reset restores all free bodies and ten passive hinges, opens the grippers at ready and settles for 0.5 seconds with auditing disabled. Scoring begins at t=0 after settling. Nothing is consumed. Final media must show the current loose-part starting layout and exact instruction. The matching hosted run and media upload remain pending.

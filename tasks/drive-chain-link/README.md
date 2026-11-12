# Reconnect a drive chain on two sprockets

This revision repairs an open chain drive. Eleven links are already routed on two locked, toothed sprockets, with their connecting link missing. The right arm inserts the orange two-pin link through the two free ends and withdraws. The left arm then fits the blue underside keeper. The final chain must form a closed, retained loop correctly seated on both wheels, with both hands clear.

**Revised rendered demo and registered evaluation: 1.0 in 773 actions / 38.65 simulated seconds.** The old joint-only Dream video and media ID do not validate this goal. `task.draft.json` deliberately has no media ID or claimed model score. A fresh hosted evaluation is required before submission; no additional paid model call has been made for this revision.

## Install and run

Use the current [Robocurve walkthrough](https://tasks.robocurve.org/walkthrough) to install Isaac Lab-Arena release/0.3.1, Isaac Sim 6.1 and Python 3.12. `SOURCE.json` records the validated adapter and Arena versions and official asset-root fix. NVIDIA license acceptance is required by the upstream runtime. In that environment, from this task folder:

```bash
uv pip install --python "$(command -v python)" -e .
python -m unittest discover -s tests -v
python check_assets.py --output /tmp/chain-repair-assets
python check_plan.py
```

Use the registered `chain_link` embodiment; the articulated scene is generated from the primitive builder and does not depend on another submitted task folder.

Native reference with actual cameras:

```bash
python run.py --policy demo --device cuda:0 --physics-device cpu \
  --width 640 --height 360 --max-steps 2400 --seed 0 \
  --output /tmp/chain-repair-demo
```

Registered evaluation:

```bash
instruction=$(python -c 'from chain_link.config import INSTRUCTION; print(INSTRUCTION)')
inspect-robots run --embodiment chain_link --policy chain_link_scripted \
  -E device=cuda:0 -E physics_device=cpu -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --instruction "$instruction" \
  --scorer task_score --max-steps 2400 --epochs 1 --seed 0 \
  --store-frames --no-prompt --grader none --no-rerun --no-rerun-save \
  --log-dir /tmp/chain-repair-inspect
```

Use a fresh output directory. CPU-only development adds `--device cpu --no-cameras --no-record` to the native command, or `-E device=cpu -E render_images=false` to Inspect Robots. It does not provide submission media. The reference plan is approximately 42 simulated seconds. A trial permits at most 2,400 actions / 120 simulated seconds at 20 Hz, with 240 Hz physics. Native development takes about two minutes wall time; rendering, warm-up and hosted inference add time.

The layout has no randomized parameters. Use native reset seed 0 and Inspect evaluation seed 0; Inspect derives its per-trial seed from that seed and scene ID. Model observations are the standard 14 joint/gripper values and the three calibrated RGB views. Only the scripted reference uses measured native part poses, converted to normal robot commands. There are no task-body state writes after reset, grasp attachments, runtime collision changes or software latches.

For the final hosted evaluation, use the same registered command with `--policy dream -P model=dreamlabs/dream-1-beta -P rows_per_request=48 -P record=false`. This opens a paid model session. Run it once only after the revised rendered reference passes visual review and the additional credit use is authorized. Every task requires a model video under the current guide; lack of budget does not permit replacing it with an earlier task's recording.

## Primary score

Scoring uses the current state and does not keep a past maximum. **All nonzero credit requires correct routing**, so assembling the joint on the bench or lifting the loop above the wheels scores zero.

Routing requires all of the following:

- The ordered chain-joint polygon, including the missing/installed closing edge, is simple: no crossings, overlapping nonadjacent segments or reversals along the same segment.
- Its winding magnitude is 1 around each sprocket centre (numerical winding tolerance 0.01).
- Each of four specified pitch seats on each wheel is within 5 mm of an actual chain-joint centre in XY. The left seats are at 90, 150, 210 and 270 degrees; the right at −90, −30, 30 and 90 degrees, on the 100 mm pitch circles.
- All chain-joint heights are within 4 mm of z=0.858 m, and every link normal has world-z dot product greater than 0.995.

With routing correct:

- **0:** both chain ends are not installed on the connector, or a permanent work-area/invalid-state violation occurred.
- **0.4:** each end-hole centre lies within 10 mm radially of its own connector stem, at connector-local z strictly between −22 and −10 mm, with relative normal dot product greater than 0.99.
- **0.7:** both ends are installed and the keeper centre is at connector-local z strictly between −40 and −30 mm, relative orientation at most 0.10 rad, and both stem projections lie inside its slots: keeper-local x strictly between −21.5 and 35 mm, y strictly within 5 mm of the corresponding ±50 mm slot centre.
- **1:** both stems additionally have keeper-local x strictly between −21.5 and −8 mm; the connector is within strictly 8 mm and at most 0.10 rad of its installed pose; all moving body centres move below 20 mm/s and spin below 0.3 rad/s; both grippers open at least 70 mm; conservative whole-robot clearance from all moving parts is at least 25 mm. These conditions hold continuously for at least one second.

The closed slot ends are at x=−26 mm. A 10 mm stem bottomed there has centre x=−21 mm, accepted with a 0.5 mm contact margin. The 24 mm heads cannot pass vertically through the 20 mm slots.

At every native physics step, every moving solid is checked against x=[−0.19,0.91] m, y=[−0.65,0.65] m and z≥0.7415 m. Any escape or nonfinite physical state permanently forces zero. The runtime requires exactly 12 audited physics samples per action. Settling includes all eleven links, connector and keeper. `scoring.py` and trace metrics expose every numerical predicate.

These conditions can be checked using ruler measurements and overhead/side photographs, plus a one-second release/settling video. The task does not score powered operation or rated load capacity.

## Setup, fidelity and reset

`setup-layout.json` gives fixed coordinates, sizes, masses, nominal materials and camera calibration. `scene-inventory.json` lists every authored custom body, solid and hinge; `build_scene.py` is authoritative. The standard adapter supplies the YAM robots, table, cameras, stand, floor and lighting.

The custom service rig has two locked six-tooth sprockets and physical guide rims, with a front access window. Dual chain plates provide real clearance around the teeth. Physical service-holder channels align the free ends during pin insertion while leaving room for the underside keeper. The deck opening permits its extended handle to pass beneath the joint. The orange connecting link starts on a pickup stand to the right and the blue keeper on a supported area to the left.

The training links, rollers, caps, keeper and handles approximate printed polymer parts; connector pins and the fixed service rig are nominal metal. Native rigid contact, Coulomb friction and ideal passive revolute bearings govern motion. The retained connecting link uses friction against lateral withdrawal. There is no elasticity, wear, fracture, powered torque-transmission, rated-strength or vibration-locking test. Shaft locking and the service holders represent an unpowered maintenance setup.

Reset restores all free parts and ten hinge positions, opens both grippers at ready and settles for 0.5 seconds with auditing disabled. Scoring begins after settling. Nothing is consumed. A task is feasible only when the full repair, including correct routing and release, is demonstrated within the action budget.

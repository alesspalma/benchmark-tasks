# Repair and hand-test a chain drive — unfinished revision

**Not ready for submission or a paid model run. No full-score demonstration exists for this functional-drive revision.** Native development checks show that the compact joint closes, but the closed chain binds during rotation. The isolated driver and the open chain move. A complete circuit of the repaired chain has not been achieved. The planned reference also needs an IK correction during crank approach.

The intended task is to insert an orange two-pin connecting link upward through the chain ends, fit the compact blue retaining plate above them, and hand-crank the repaired link once around both sprockets. The final assembly must stay connected and routed, with both hands released. The drive has no motor or active shaft controller.

The previous static version's score of 1.0 applied to assembly on locked sprockets. Its media and receipts do not validate this revision. `task.draft.json` has no media ID or model score. Prior successful submissions are separate tasks.

## Implemented changes

- Compact 45 × 140 × 6 mm orange backing with upward pins; its tall grasp handle is removed.
- Compact 44 × 136 × 6 mm blue keeper with a short in-plane grip tab; its extended handle is removed.
- Two native passive sprocket axles, with a hand crank and a passive knob bearing. Axle and knob travel limits are ±3,600 degrees.
- Upper guide supports outside the moving shaft path, continuous side guides and a low-friction service bed. Primitive dimensions and all fixed starting poses are recorded in `scene-inventory.json` and `setup-layout.json`.
- A scorer that requires a complete physical circuit and matching motion of both sprockets before full credit. A stationary assembled joint is capped at 0.7.

Robot actuation remains the standard fourteen joint/gripper controls. The task environment adds no task-body pose writes after reset, grasp attachments, software latches, direct shaft forces or runtime collision changes. Development-only shaft-load probes are separate and cannot count as a robot demonstration.

## Development commands

Use the Robocurve walkthrough environment: Python 3.12, Isaac Sim 6.1, Isaac Lab-Arena and the adapter versions in `SOURCE.json`. NVIDIA license acceptance is required. From this task folder:

```bash
uv pip install --python "$(command -v python)" -e .
python -m unittest discover -s tests -v
python check_assets.py --output /tmp/chain-functional-assets
python check_plan.py
```

The unit tests cover failure gates and fork geometry. USD checks cover asset structure. Neither establishes physical feasibility. `check_plan.py` currently fails at the right hand's crank approach; do not interpret a partial plan as a demonstration.

An unpaid native assembly diagnostic can be run with a fresh output directory:

```bash
python run.py --policy assembly --device cuda:0 --physics-device cpu \
  --width 640 --height 360 --max-steps 2400 --seed 0 \
  --output /tmp/chain-functional-assembly
```

The full planned policy is `--policy demo`; it is unfinished. CPU-only development uses `--device cpu --physics-device cpu --no-cameras --no-record` and provides no video evidence. The layout is fixed, with seed 0, 20 Hz controls, 240 Hz physics and a limit of 2,400 actions / 120 simulated seconds. The proposed complete reference is approximately 80 simulated seconds, but its runtime and success have not been validated.

The standard PhysX actuator path is selected with `cfg.sim.use_newton_actuators=False`; passive task joints have no actuator groups. Only the scripted reference reads object poses. Model observations remain the standard joint/gripper state and three calibrated RGB cameras.

## Proposed score

All nonzero credit requires a simple, correctly routed loop around both sprocket centres, the appropriate rotating pitch seats within 10 mm of chain joints, chain-joint heights within 6 mm of their working plane and level chain links. These conditions are measured from the current native state.

- **0:** disconnected chain, incorrect routing, nonfinite state or permanent workspace escape.
- **0.4:** both end holes installed on their respective connector stems.
- **0.7:** the keeper additionally covers both stems above the chain. Full retention requires the stems at the closed slot ends.
- **1.0:** a retained, routed connection survives a full circuit: both axles travel at least approximately two revolutions, their accumulated phase difference stays within 0.15 rad and the repaired link travels one full revolution around the drive centre. Then all moving parts settle and both hands remain open and clear for one second.

`scoring.py` is authoritative for the exact numerical predicates. Motion is audited every native physics step. Breaking or derailing the connection clears movement history. Spinning axles without chain travel cannot complete the task.

## Fidelity and remaining work

These are enlarged rigid training parts, with ideal passive hinges, lubricated chain/sprocket contact and a custom friction-retained keeper. There is no elasticity, wear, fracture, rated-load or powered-operation claim. `SOURCE.json` records software provenance; `build_scene.py` generates all original primitive assets.

Remaining work is mechanical: resolve binding of the closed joint, validate keeper retention throughout circulation, fix the reference approach and demonstrate the complete task with real robot controls. Registered evaluation, current-source rendered evidence, a matching hosted-model recording and final submission metadata must follow successful physical validation. Existing diagnostic images are not final submission media.

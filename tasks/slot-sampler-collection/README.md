# Collect a granule sample with a closing slot probe

Insert a closed two-sleeve sampling probe into the granule tub. One arm holds the outer sleeve while the other rotates the inner sleeve to align the buried side windows. After material enters, close the window, withdraw the retained sample, discharge it into the sample jar and park the probe. The goal is at least 5 grams of actual delivered material with both hands released.

Development status: **not ready for submission**. Native full-score validation failed; calibration has stopped after repeated unsuccessful trials. The latest reference retains a small sample, spills material during discharge, and fails to park the probe. Native score: **0.0**. No Dream-1 episode or media upload has been made for this task. `task.draft.json` is preparation material, not a submission.

## Industrial operation

[Sampling Systems describes the insert-closed, open, close, withdraw and invert procedure](https://sampling.com/slot_sampler.html) for its two-sleeve slot samplers. The [Canadian Grain Commission sampling handbook](https://www.canada.ca/en/grain-commission/services/grain-quality/grain-sampling/sampling-systems-handbook.html) describes double-sleeve sampling and gentle closing and agitation. Material sampling supports incoming-goods and batch quality checks. [BLS includes samplers in quality-control inspection work](https://www.bls.gov/ooh/production/quality-control-inspectors.htm).

The task isolates physical capture and retention inside a closing probe. Its single-port, 300 mm long, 32 mm OD tool is an original fabricable design, with ordinary support handles and a turning lever. A square storage guide stays below the working window. The material is free-flowing polymer calibration granules; the score measures transferred mass. This is a substep of industrial sampling, motivating **Impact on jobs**.

## Install and run

Use Python 3.12 and the [official Isaac Lab-Arena walkthrough](https://tasks.robocurve.org/walkthrough), with NVIDIA license acceptance configured. Package versions and the pinned official YAM adapter are recorded in `SOURCE.json`. From this folder:

```bash
uv pip install --python "$(command -v python)" -e .
python -m pytest -q tests
python check_assets.py --output /tmp/sampler-usd
python check_plan.py
python run.py --policy demo --device cuda:0 --physics-device cpu \
  --width 320 --height 176 --max-steps 2400 --seed 1415158438 \
  --output /tmp/sampler-native
```

Each output directory must be new. CPU physics is available with `--physics-device cpu`; `--no-cameras --device cpu --physics-device cpu` produces no submission video. Modes `hold`, `insert`, `open`, `retain` and `deliver` stop after individual substeps. Native runs with 1,960 grains and three cameras take several minutes, including startup. The hard limit is 2,400 actions / 120 simulated seconds at 20 Hz, with 240 Hz physics. Reset has zero object jitter. Inspect seed 0 derives direct-run trial seed 1415158438.

Registered evaluation after installation:

```bash
instruction=$(python -c 'from slot_sampler.config import INSTRUCTION; print(INSTRUCTION)')
inspect-robots run --embodiment slot_sampler --policy slot_sampler_scripted \
  -E device=cuda:0 -E physics_device=cpu -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --instruction "$instruction" \
  --scorer task_score --max-steps 2400 --epochs 1 --seed 0 --store-frames \
  --no-prompt --grader none --no-rerun --no-rerun-save --log-dir /tmp/sampler-inspect
```

The reference reads only elapsed time and ordinary joint measurements. Standard URDF kinematics and nominal fixture geometry define its taught motion. Runtime object poses, contacts, grain locations, sample mass and score are scoring diagnostics and are unavailable to the controller. All post-reset actuation is through the standard 14 robot controls. The sleeve hinge is passive; there are no scripted captures, body pose changes, grasp attachments or runtime collision changes.

## Score and measurements

Each simulated granule has 0.18 g mass. Whole grains at rest inside the jar contribute to delivered mass only if they entered the buried side window with the probe upright and remained fully inside after it closed. Contacting a window or dropping unrelated grains directly into the jar does not mark them as retained sample.

- **0:** under 2 g delivered, or any permanent invalidation occurred.
- **0.4:** at least 2 g and under 5 g delivered.
- **0.7:** at least 5 g delivered.
- **1:** at least 5 g delivered; probe within 12 mm of its stored pose, with vertical-axis cosine above 0.98 and sleeve within 0.08 rad of closed; probe velocity below 15 mm/s and 0.15 rad/s; both grippers open over 40 mm; and every robot collision body over 3 mm clear of the probe. These conditions persist for one second.

A grain must fit wholly between the jar walls, floor and rim, including its 3.4 mm radius and a 0.5 mm contact tolerance, and move below 15 mm/s. A grain falling below table top-5 mm permanently forces zero, even if later recovered. The probe's whole collision envelopes must stay in x=[-0.19,0.91] m, y=[-0.65,0.65] m and z=[table top-5 mm,2.0] m. Nonfinite states also force zero. Material trajectories and whole-body bounds are audited at 240 Hz. A balance, ruler, photos and a release video establish the real-world outcome; native grain identities implement the equivalent provenance check without adding mass.

## Setup, construction and reset

World x extends away from the robot bases, +y is the left arm's side, and table top z is 0.7475 m. `config.py` defines the fixed reset; `builder.py` defines every native body and joint. `check_assets.py` generates the original USD geometry. The adapter supplies the standard YAM arms, grippers, table, floor, lights and top/left-wrist/right-wrist cameras.

- Probe outer tube: nominal aluminum, 300 mm long, 32 mm OD, 1.5 mm wall, closed pointed tip; 0.28 kg including external support handles. Working window spans local z=15–85 mm over a 90 degree circumferential sector.
- Inner sleeve: nominal aluminum, 310 mm long, 27.5 mm mean diameter, 1 mm wall, closed distal end and open proximal discharge; 0.09 kg including turning lever. Native passive hinge range is -90 to 0 degrees. At reset the two windows are offset 90 degrees; turning -90 degrees aligns them. Outer support grips are at z=255 mm. The lever extends 68 mm radially at z=312 mm.
- Fixed probe stand: polymer, 90×90 mm foot, 120 mm guide height, squared aperture matching the probe's distal storage collar. Probe nominal origin is (0.43,0.38,0.7685) m.
- Source tub: fixed polymer tray, inner 105×105 mm, 145 mm wall height, 3 mm wall thickness; floor reference (0.43,0.02,0.7475) m.
- Calibration granules: 1,960 individual native rigid spheres, 6.8 mm diameter, 0.18 g each (352.8 g total; nominal density about 1.09 g/cm³). Reset lattice is 14×14×10 at 7.1 mm pitch, followed by settling.
- Sample jar: fixed square polymer cup, inner 80×80 mm, 70 mm walls of 3 mm thickness; floor reference (0.35,0.30,0.7475) m, initially empty.

Native rigid contacts provide entry, granular flow, closure, retention and discharge. Nominal static/dynamic friction is 0.35/0.25 on the probe and 0.65/0.5 on the fixtures and 0.30/0.20 on the granules. The single port and spherical calibration pellets define this particular reproducible sample-transfer task. Reset restores every grain and the passive sleeve, opens the grippers at ready and settles for 2.5 seconds before scoring. Nothing is consumed. All custom assets and code are original and MIT licensed; reference manufacturers supply no meshes.

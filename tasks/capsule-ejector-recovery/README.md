# Empty a capsule machine's collector and load a fresh capsule

A used capsule remains in the loading carrier, and six spent capsules are in the removable collector. Open the hinged head, discharge the spent capsule, remove and empty the collector into the recycling tub, refit it, seat the fresh capsule and close the head. The reference opens the head with the right arm, uses the left arm to slide out, empty and refit the collector by its front bar, and loads the fresh capsule with the left. Scoring accepts other physical solutions.

Native demonstration: **1.0**, completed in **2324 actions / 116.2 simulated seconds**, within the 2,400-action limit (seed 1415158438). The recorded source matches `validation/native-attempt.json`. All seven spent capsules are collected, the compact cassette remains fitted after release, the fresh capsule is seated, and the loaded head closes within its stated tolerance. The package remains a draft pending model evaluation and final submission checks. No Dream-1 episode or media upload has been made; `task.draft.json` is preparation material.

## Motivation and endpoint

[Nespresso's Atelier troubleshooting](https://www.nespresso.com/qa/en/machine-assistance/atelier-machine-assistance) directs users whose lever will not close to empty the capsule container and check for a blocked capsule. The task isolates this mechanical recovery: ordinary fresh-capsule insertion will not dispose of the accumulated spent capsules. The endpoint is a restored loading/ejection assembly, with the fresh capsule seated and its cover shut.

This original cutaway fixture uses a passive lifting fork, a hinged cover, a removable collector and real-size rigid capsules. The machine is cold and unpowered. The task measures mechanical clearing and reloading; the machine has no water circuit. This context motivates **Societal relatability**.

## Install and run

Use Python 3.12 and the [official Isaac Lab-Arena walkthrough](https://tasks.robocurve.org/walkthrough), with NVIDIA license acceptance configured. Validated package versions and the pinned official YAM adapter are in `SOURCE.json`. From this folder:

```bash
uv pip install --python "$(command -v python)" -e .
python -m pytest -q tests
python check_assets.py --output /tmp/capsule-usd
python check_plan.py
python run.py --policy demo --device cuda:0 --physics-device cpu \
  --width 320 --height 176 --max-steps 2400 --seed 1415158438 \
  --output /tmp/capsule-native
```

Every output directory must be new. `--no-cameras --device cpu` runs development physics without a submission video. `--policy hold`, `open`, `empty` and `reload` stop at progressively later substeps. The recorded reference took 497 wall-clock seconds on an RTX A6000 with CPU PhysX and three 320×176 camera streams, including initialization. CPU-only development runs omit rendering cost. The hard limit is 2,400 actions, 120 simulated seconds at 20 Hz with 240 Hz physics. The reset is fixed, with zero object jitter. Inspect seed 0 derives direct-run trial seed 1415158438.

Registered evaluation after installing this package:

```bash
instruction=$(python -c 'from capsule_recovery.config import INSTRUCTION; print(INSTRUCTION)')
inspect-robots run --embodiment capsule_recovery --policy capsule_recovery_scripted \
  -E device=cuda:0 -E physics_device=cpu -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --instruction "$instruction" \
  --scorer task_score --max-steps 2400 --epochs 1 --seed 0 --store-frames \
  --no-prompt --grader none --no-rerun --no-rerun-save --log-dir /tmp/capsule-inspect
```

The policy reads elapsed time and ordinary measured joint positions. Nominal fixture coordinates and the standard URDF support its taught Cartesian program. Runtime object poses, contacts, scoring state and simulator handles are unavailable to it. All post-reset actuation uses the standard 14 robot controls; task joints remain passive. No runtime attachments, object pose changes, collision changes or software completion events are used. Native traces record robot actions and independent scoring measurements. The light capsules use native PhysX sleeping at a mass-normalized kinetic-energy threshold of 0.0002 m²/s². Unused contact reporting is disabled for every task body at spawn because activating reports sets sleep thresholds to zero; collision response remains enabled. Reset/final diagnostics record the native thresholds, checked after spawning.

## Score

The score uses the current outcome, with permanent invalidation for a dropped part or nonfinite state.

- **0:** fewer than all seven spent capsules are contained at rest in the recycling tub, or any invalidation occurred.
- **0.35:** all seven spent capsules are fully contained at rest in the tub.
- **0.65:** the collection condition holds and the collector is within 9 mm of its fitted pose, with its vertical-axis cosine above 0.99.
- **1:** additionally, the fresh capsule is within 3.5 mm horizontally and 3 mm vertically of its seat, with vertical-axis cosine above 0.99; the head is within 4 degrees of closed; all loose bodies move below 15 mm/s and every pair of observed poses over the preceding second differs by less than 2 mm and 2 degrees; both grippers open over 40 mm; and every robot collision body clears every moving task body by over 3 mm. These conditions persist for one second.

Containment uses transformed mesh/cube vertices and a circumscribed 128-sided envelope for round colliders (under 6 micrometres excess at the capsule flange), with a 0.5 mm contact tolerance, against the tub's inner walls, floor and rim. Merely placing a capsule centre over the tub does not count. A spent capsule must move below 15 mm/s and meet the same one-second pose-stability condition to count. Rest is checked from at least 21 distinct 20 Hz observations spanning a second, with no sample gap over 75 ms. Every pair of poses in that window is compared, so an excursion that returns to its start still fails. This pose-based measurement matches a one-second release video and rejects real translation, rotation and oscillation. Whole-body workspace bounds are x=[-0.19,0.91] m, y=[-0.65,0.65] m and z=[table top-10 mm,2.0] m, audited every physics step. Full robot clearance uses conservative collision-body bounding boxes. Ruler measurements, photos and a one-second release video establish the outcome; no instrumented brewing or pressure measurement is required.

## Setup, construction and reset

World x extends away from the robot bases, +y is the left arm's side, and table top z is 0.7475 m. `config.py` supplies all reset coordinates; `builder.py` defines every original solid, mass and native joint. `check_assets.py` generates the distributable USD assets from source. The adapter supplies the standard two YAM arms, parallel grippers, table, floor, lights and calibrated top/left-wrist/right-wrist cameras.

- Fixed machine shell: approximately 280×210×290 mm; nominal polymer housing, metal carrier and lifting fork; 2.8 kg anchored base, 0.22 kg hinged head. Hinge pivot is (0.55,0,0.9825) m, with a 0–85 degree passive range. The bar handle is 240 mm long and 18 mm in diameter. The carrier's capsule-bottom target is (0.44,0,0.957) m. The loaded head closes against the capsule rim, with the explicit 4-degree closed-position allowance.
- Collector: shallow polymer cassette, inner 120×82 mm, 65 mm walls of 2 mm thickness, 65 g; nominal root (0.485,0,0.7575) m. Its inner volume is about 0.64 litres, approximately 75% smaller than the earlier 160×110×145 mm collector. A 30×55×24 mm front pull bar is centered 95 mm in front of its root and 60 mm above it. Two lateral stems join it to the front wall. Sloped guides have 6 mm side clearance at the foot and a 25 mm lip. The arm withdraws it 165 mm, moves alongside the housing before moving over the recycling tub, and reverses that clear route for reinsertion. It returns 40 mm above the fitted height and then lowers onto the support. The arm opens, withdraws straight out and lifts away.
- Fixed lower housing: a 50×125×130 mm rear service module, two 125×30×105 mm side modules and two 20×30×115 mm carrier pillars. These rectangles remain part of the anchored machine; only the central cassette is removed. Coordinates and materials are listed in `setup-layout.json`, with exact solids in `builder.py`.
- Seven used capsules: rigid tapered cups, 24–33 mm diameter, 26 mm body height, 37 mm diameter and 1.8 mm thick flange, 12 g each. Six start in one 3×2 layer at 39 mm x pitch and 40 mm y pitch; the seventh starts in the loading carrier. Native settling determines their supported starting poses.
- One fresh gold capsule: same cup geometry, 8 g, at (0.56,0.25,0.8175) m on a fixed 70 mm pedestal with an 18 mm top and 50 mm foot diameter.
- Recycling tub: fixed polymer tray, inner 300×300 mm, 55 mm walls, 4 mm wall thickness, nominal floor reference (0.37,0.27,0.7475) m.

Rigid contact and Coulomb friction model clearing, support and closure. The capsule bodies are convex tapered cups; foil and empty walls are represented by a single rigid collision envelope. Reset restores all bodies and the head hinge, opens the arms at ready, and settles for 1.5 seconds before scoring starts. No supplies are consumed. Original geometry and code are MIT licensed; manufacturer links are procedural references and supply no assets.

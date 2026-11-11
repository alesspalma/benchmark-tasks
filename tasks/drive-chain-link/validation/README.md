# Validation evidence

The rendered native reference and the registered Inspect Robots 1.0 reference both reach **1.0 in 790 actions / 39.5 simulated seconds**. Both ran with CPU PhysX and GPU RGB rendering. The registered reference records all 791 initial/action frames from each of the three calibrated cameras.

The **single** hosted Dream-1 episode completed its **2,400-action / 120-second** budget with **score 0.0 and zero errored trials**. It returned 50 chunks in 50 request attempts. There was no second episode or automatic episode rerun. All 2,401 frames from each camera were preserved. Its three initial frames are pixel-identical to the registered reference's initial frames.

The actual setup, both chain-end insertions, keeper routing and insertion, keeper release and final hand clearance were visually reviewed. The videos contain the native top view above both native wrist views; processing only resizes/stacks and encodes H.264. The setup screenshot is an actual reset image. Media ID and reported scores are in `task.json`.

- `reference-native.json`, `reference-inspect.json`: actual full-score rendered executions, source fingerprint and report hashes.
- `reference-media.json`, `model-media.json`: frame counts, composition, codecs, file sizes and SHA-256 hashes.
- `model-native.json`: actual hosted episode settings, score, request counts, tracking and frame checks, without credentials or service session identifiers.
- `rules-check.json`, `submission-checklist.md`: current authoritative rules and submission review.
- `unit-tests.txt`: all **24** scorer and physical-slot geometry tests pass. `usd-structure.json`: all four generated assets pass schema/compliance and joint-target checks.

`cpu-validation.json` preserves earlier camera-free development evidence, including the omitted-keeper control (0.4) and the bounded upward contact probe (0.7 because the gripper stays closed). Its executable fingerprint predates a scene-description correction from “keyhole” to “forked”; physics and control logic were unchanged. The current source is validated by the rendered receipts. The probe raised the free connector about 2 mm through contact with the keeper; it is not a rated-load test.

`files.sha256.json` hashes the packaged task files. Full raw local reports and recordings are retained separately; no failed development run is represented as successful evidence. No additional Dream call is needed to reproduce or inspect these saved results.

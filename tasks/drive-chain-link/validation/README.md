# Camera-free validation

The finalized source reaches primary score **1.0 through Inspect Robots 1.0**, in **790 actions / 39.5 simulated seconds**. The registered trial completed with zero errors. Its environment ID records the executable source fingerprint in `cpu-validation.json`.

All **24** scorer and physical-geometry tests pass, and all four generated USD files pass schema/compliance and joint-target checks. An omitted-keeper native control scores **0.4**, not full success. A separate bounded upward probe leaves the keeper around both stems and raises the free connector by about **2 mm** through physical contact. The gripper remains closed during that probe, and the primary scorer correctly withholds full success.

These are actual native CPU PhysX executions with no cameras and no model requests. They establish controller/scorer integration and provide mechanical checks. They do not establish visual quality or replace the required submission video and screenshot. No Dream-1 score is claimed.

`cpu-validation.json` records the final executable fingerprint, runtime versions, numeric results and hashes of the full local receipts. `unit-tests.txt` and `usd-structure.json` contain the non-rendering check results. Earlier development failures are not included as successful evidence.

Remaining gates are a visually reviewed rendered full-score reference, the matching setup screenshot, one hosted Dream-1 evaluation, final media upload and a submission PR. Regenerate these validation records after any executable change.

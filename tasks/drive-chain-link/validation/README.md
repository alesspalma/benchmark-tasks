# Current task validation

The yellow two-pin connector starts loose on the table; its cradle starts empty. Both chain ends have bare grey rails without yellow grasp posts. All starting and final conditions are measured by native physics.

- Native CPU reference: 1.0 at 1,180 actions / 59.0 simulated seconds.
- Actual three-camera native recording: 1.0 at 1,180 actions, 14,160 audited physics steps. All 1,181 initial/action frames per view are preserved.
- Registered Inspect Robots scripted evaluation: 1.0; exact evaluation seed, derived trial seed, steps and log hashes are in `reference-inspect.json`.
- Native control omitting the keeper-locking slide: 0.4.
- 32 adversarial scorer, aperture and bare-rail/keeper-clearance tests pass. Four generated USD assets pass structure checks.

`reference-controller-equivalence.json` documents one later controller-only change: the IK position-error stop guard increased from 3mm to 5mm. The recording's worst residual was 0.0962mm, so that guard never activated. Reconstructing the recorded source reproduces its original fingerprint; every other source byte is identical. Replaying all 1,180 recorded actions with the final controller differs by at most 4.5e-12 per control value. `reference-controller.patch` contains the exact change. Scene geometry, physics, scoring tolerances, controller targets and recorded motion are unchanged. The registered evaluation uses the final controller.

The final media are spatial compositions of actual cameras, with no temporal edits, overlays or generated frames. The three-camera MP4 is H.264/yuv420p at 20fps and 640x540; screenshot is the actual initial overhead view. Their hashes and complete decode/probe checks are in `reference-media.json`.

A matching Dream-1 episode, media upload and submission remain pending. No previous paid episode or media ID is reused, and no credits were spent for this revision.

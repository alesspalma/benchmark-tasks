# Current task validation

The chain and fixed toothed former are dark grey against the light back plate. The yellow connector starts separately on the right; the blue keeper starts on the left at (0.18, 0.35, 0.7515)m. The right arm places the yellow connector, both arms join the bare chain ends, and the left arm picks up and fits the keeper while the right arm is parked.

- Actual three-camera native reference: 1.0 at 1,180 actions / 59.0 simulated seconds, with 14,160 audited physics steps and 1,181 initial/action frames per view.
- Native CPU reference: 1.0 at 1,181 actions / 59.05 simulated seconds.
- Registered Inspect Robots CPU scripted evaluation: 1.0; evaluation seed, derived trial seed, step count and log hashes are in `reference-inspect.json`.
- Native control omitting the left arm's keeper-locking slide: 0.4.
- 32 scorer, aperture and bare-rail/keeper-clearance tests pass; all four generated USD assets pass structure checks.

All evidence uses the same frozen executable-source fingerprint. Geometry, contact parameters and scoring are unchanged from the previous task revision; only colours, keeper starting side, instruction and reference arm control changed. Every primitive colour is included in the scene inventory. `reference-roles.json` records actual left/right TCP motion during keeper handling.

Final media spatially combine every actual camera frame: 640x360 overhead above two 320x180 wrist views, 20fps H.264/yuv420p, with no temporal edits, overlays or generated frames. The screenshot is the actual initial overhead view. Frame counts, full decode checks and file hashes are in `reference-media.json`.

A matching hosted Dream-1 episode, media upload and submission remain pending. No previous model episode or media ID is reused; no credits were spent for this revision.

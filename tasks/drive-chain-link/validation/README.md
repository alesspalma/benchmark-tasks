# Revised repair validation

This evidence applies to the chain-drive repair revision, not the earlier bench-only joint exercise.

- The rendered reference scores **1.0 in 773 actions / 38.65 simulated seconds**. The joint is closed and retained, both sprockets have winding magnitude 1, every scored seat is within **1.974 mm**, all parts settle and both hands clear the assembly. All 9,276 native physics samples pass the work-area audit.
- The registered Inspect Robots native run also scores **1.0 in 773 actions**, with zero errored trials.
- The native omitted-keeper control scores **0.4** and does not report a closed retained loop.
- All **24** scorer/physical-geometry tests pass, including assembled joints off the wheels, loops lifted above the wheels, a derailed wheel, crossed/backtracked loops, incomplete retention, hand contact and permanent work-area violations.
- Four generated USD files pass compliance and joint-target checks. Lint passes.

The reset, connector insertion, right-hand withdrawal, keeper insertion and final routed loop were visually reviewed. The demo preserves every actual native initial/action frame in order, showing top view above both wrists. Processing only resizes/stacks and encodes H.264. The setup screenshot is an actual reset image.

`reference-native.json`, `inspect-native.json`, `negative-control.json`, `reference-media.json`, `unit-tests.txt` and `usd-structure.json` preserve numeric results, source identity and hashes. `rules-check.json` records the current authoritative rules and task-board screen. `files.sha256.json` covers the packaged files.

**A fresh hosted-model video remains required for this changed task.** The previous Dream recording and M-96PM3V belong to the old joint-only objective and are not used here. No additional paid episode has been run. The user was asked whether to authorize one new episode after reviewing the revised demo. The current metadata is deliberately `task.draft.json`, without a media ID; no submission PR has been opened.

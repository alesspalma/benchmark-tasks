# Validation evidence

All outcome trials use the task's native scene, material law, free bodies and
scorer. The source fingerprint in each receipt identifies the executed Python
and package configuration. No hosted inference is used for these checks.

- `native-counterexamples.json`: the five final-source native outcome trials.
  No action scores 0. The same 90-degree pour with the restrainer kept away
  collects 150 mL and keeps the bar inside; it scores 0 because the explicitly
  requested magnetic restraint operation was not performed. A separately
  labelled 110-degree inversion without the tool physically loses the bar and
  scores 0. These are different motions: the inversion result does not show
  that an unrestrained 90-degree pour must lose its bar.
- A correctly restrained pour stopped while still held scores 0.7. A valid
  full sequence with a 5 mm shifted pouring position scores 1.0 at 1,561
  actions, with 150 mL collected, no spill, the bar continuously contained and
  all free apparatus released stably. The scorer imposes no taught hand or
  grasp sequence.
- `asset-compliance.json`: all five generated USD assets pass schema and
  structural checks, without starting physics. The vessels are genuinely
  hollow; bar and tool are separate free rigid bodies.
- `tests.txt`: 17 tests of magnetic energy/force and reaction invariants,
  separation dependence, orientation-independent bar/wall retention, water
  volume/mass/inertia, finite liquid flight, collision blocking, collection,
  spill, irreversible containment violations and released support.

The native full-score demonstration and registered Inspect smoke receipts are
included alongside these files. The demonstration's media are native camera
recordings, not rendered from a reconstructed state trace. Hardware field and
outflow calibration are specified in `SOURCES.md`; simulation evidence is not
represented as robot-hardware validation.

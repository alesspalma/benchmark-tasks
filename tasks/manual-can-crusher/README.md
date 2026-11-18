# Compact an aluminum can in a wall-mounted hand crusher

Prototype parked after native feasibility testing. The tool and the empty can
are full-size. The reference failed to place the can squarely in the cradle
and retain the lever grip on return. Its demonstrated score is 0.0.

Place the empty can in the open cradle. Pull the handle through the resistance
of the can, then raise it to send the compacted can into the bin. This saves
space where aluminum is already kept in a separate recycling stream.
[EPA guidance on sorting and compaction](https://www.epa.gov/recycle/how-do-i-recycle-common-recyclables)

## Run

Install Isaac Lab-Arena, Inspect Robots, and the YAM adapter according to the
Robocurve walkthrough. Install the task from this directory.

```bash
python -m pip install -e .
python run.py --device cuda:0 --physics-device cpu --output runs/demo
```

The embodiment is `can_crusher` and the native policy is
`can_crusher_scripted`. The fixed layout uses seed 1415158438. The trial allows
780 actions at 20 Hz, or 39 seconds, with native physics at 240 Hz. The
runner records a receipt and all three native camera views at 640 × 360.
Use `--policy load-only` for the ordinary load and release without pulling
the lever, or `--policy hold --no-cameras` for a stationary control.

## Setup

Mount a [Dial Easy Pull EP1026](https://dialind.com/product/easy-pull-can-crusher/)
on a rigid vertical support with the supplied mount and its ordinary discharge
space below. The full commercial machine measures 159 × 102 × 502 mm including
its handle. The representative assembly reproduces the two pivoting parallel
platens, guide rods, moving lever and passive discharge described in
[US6076455A](https://patents.google.com/patent/US6076455A/en).
All distances in the table are measured from the midpoint of the two pedestal
base mounting axes. X points from the arms toward the wall, Y to the arms'
left and Z upward.

| Item | Material and size | Initial configuration |
| --- | --- | --- |
| Supports | Two bolted metal YAM pedestals, about 160 × 160 × 833 mm, with 220 mm square feet | Y ±305 mm, arm mounting surface Z 820 mm. Hands open and clear. |
| Vertical mount | Plywood or structural panel, 32 × 580 × 1,240 mm | Front face X 492 mm. Crusher centered at Y 0. |
| Can crusher | Wall-mounted plastic body and lever, steel hinge pins and guides, 140 × 98 mm pressure faces in the simulator | Lever pivot at X 457 mm, Z 837 mm. Open cradle around X 408 mm, Y 0, Z 756 mm. The crossbar is 340 mm from its pivot and the jaw gap is 128 mm. Lever starts raised. |
| Can | Empty standard 12 fl oz aluminum beverage can with its original pull tab, about Ø66 × 122 mm, 14 g | Upright, on the side shelf at X 333 mm, Y 198 mm and Z 748 mm. |
| Side shelf | Plywood, 265 × 190 × 18 mm | Shelf top at Z 748 mm. Its center is 39 mm behind the can along X. |
| Collection bin | Ordinary 301 × 224 × 204 mm plastic bin on a rigid shelf, initially empty | Opening centered at X 342 mm, Y 0. Bin floor at Z 220 mm. |

Use one empty, drained can of the same standard format per trial. Clear the
collection bin and return the lever to its upper stop between trials. The
simulator models the can as an empty shell and ignores logos and residual
liquid. The physical scene uses the ordinary dispensing mechanism and can be
assembled with a ruler, stock mounting hardware and a level.

## Scoring

Measure the can's maximum length along its original axis between two flat
surfaces after it has been released. Use trial video to confirm where it
ended and the final position of the lever.

- **1.0** when the can is no more than 35 mm high, rests in the collection
  bin for at least 0.6 seconds, and the lever is within 12° of its open stop.
- **0.5** for a can no more than 70 mm high whose final position is in the
  cradle or bin, when the full conditions are unmet.
- **0.25** for a can seated in the cradle when the compaction conditions are
  unmet.
- **0.0** for other outcomes.

## Simulation and licenses

Robot-to-grip, platen-to-can, and bin contacts use PhysX. The lever transmits
load through its pinned slider loop. Native end bodies compress the shell
against a bounded material force. An axial spring supplies the initial load,
and plastic set follows the shortening already achieved after buckling.
The visible wall folds with the end-to-end distance. This reduced shell
model estimates the load curve and omits sheet tearing and exact crease
location. A published test of hand-lever crushers reports loads of about
22–89 N at the grip for aluminum cans.
[Hand-lever load reference](https://patents.google.com/patent/US4290354A/en)

Task geometry is generated from source under the included MIT license.
The YAM robot and Arena runtime use the upstream adapter dependencies in
`pyproject.toml`.

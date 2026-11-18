# Wring a wet yacht mop in a cone bucket

A damp mop leaves less standing water on the floor. For a cone bucket, push
the mop head into the taper and turn the shaft under load. The cone holds and
compresses the wet rayon bundles, forcing water down into the bucket. Lift the
head slightly to let it drain.

The [M2 cone bucket][pail] is a 15 L floor bucket for yacht mops. The [M2
cut-end rayon yacht mop][mop] uses a 48 inch steel handle and a 10 oz head. The
[cone-wringer method][method] uses downward pressure and a turn of the shaft.

## Run

Install Isaac Lab-Arena, Inspect Robots, and the YAM adapter from the
Robocurve walkthrough. From this folder:

```bash
python -m pip install -e .
python run.py --output runs/demo --device cuda:0 --physics-device cpu
```

The fixed layout and seed `1415158438` define each trial. The limit is
400 actions (20 seconds at 20 Hz). The run records all three native 640 × 360
views and writes one trace row per action, along with the initial state,
terminal state, and scorer receipt. Add `--no-cameras` to run physics only,
or `--policy hold` for the comparison that grasps the shaft and holds still.
Use `mop_wringer` as the Inspect Robots embodiment, `mop_wringer_scripted`
as the reference policy, and `task_score` as the scorer. The policy uses
elapsed time and joint observations. The overhead view includes both stands,
the bucket and the full shaft. The right wrist camera uses a 35 mm side
mount with a 33° downward tilt so it can see the cone during the grasp.

## Setup and reset

Use two YAM arms on rigid floor-bolted stands with open space in front of
them. Arm mounting planes are 750 mm above the floor and 610 mm apart.
Coordinate X points forward from the arm bases, Y points to the left arm.
The two steel stands use 272 × 245 mm feet, 163 × 160 mm uprights, and
211 × 231 mm top plates. Use the normal YAM mounting bolt pattern and secure
both stands to a rigid slab. The mop handle stays free to tilt and turn.

| Item | Material and size | Starting configuration |
| --- | --- | --- |
| Cone mop bucket | 15 L polypropylene pail, 360 × 360 × 293 mm. Removable tapered perforated wringer, 214 mm opening and 58 mm lower bore. Lower cone lip is 181 mm below the mouth. | Level on the floor, cone axis X 340 mm, Y 0. Carry handle lowered. Put 3 L of clean water in the pail, below the bottom of the cone. |
| Yacht mop | 1,219 mm steel shaft, 25 mm outer diameter, plastic top grip and ferrule, white 10 oz cut-end rayon head. The loose head is about 120 mm wide and extends 204 mm below the shaft. | Shaft vertical at the cone centre. Let the wet head settle lightly in the upper end of the wringer. The shaft-to-head neck begins about 367 mm above the floor. Both grippers start clear and open. |
| Water held by rayon | 200 ± 5 g above the mop's dry mass. | Soak in room-temperature water, remove excess by hand, check total mass on a scale and let residual dripping slow to under 1 g per 10 s. |

Weigh the charged bucket first with the mop out of it. Tare a dry absorbent
floor mat around the pail if small spills are difficult to see. Position the
bucket and stock mop as specified before opening each trial. Replace the
bucket's water to its baseline and reset the rayon's retained water between
trials. A bucket and mop are standard replaceable cleaning supplies. The
printed or machined mounting plate pattern follows the robot hardware.

## Scoring

Measure caught water by the bucket's mass gain after the robot finishes,
with the mop supported clear of the scale. Measure escaped water by the
absorbent mat's mass gain. View the lift from the side using the existing
floor as a height reference. Record the first frame after the mop's neck
rises to at least 405 mm above the floor. Confirm the shaft stays within
22° of vertical and the neck within 100 mm of the cone axis for 0.45 s.
A gentle lifting motion during that hold is fine. A score is valid only
when the mop head stays attached and the runoff is collected by the bucket.

- **1.0** for at least 100 g caught and the stated raised draining posture.
- **0.5** for at least 100 g caught before the draining posture is complete.
- **0.25** for 40 g or more caught and less than 100 g caught.
- **0** for all other outcomes, for more than 8 g outside the pail, for
  tipping the bucket onto its side, or for damaged or detached parts.

## Simulation and assets

A free rigid bucket rests on the floor, while both arms physically grip the
shaft. Six native spring-supported rayon bundles compress against the
rigid cone. The volume model tracks newly compressed radial travel and
relative twist while under load. Each physical change reduces the retained
water and adds it to the caught water, with spill tracked if the pail tips.
The rendered droplets illustrate that outflow under gravity. The initial
fill is included in the simulated bucket mass. The expelled amount is too
small to affect the bucket's inertia substantially. Individual loose fibres,
interstitial flow, and slosh are approximated.

The pail, wringer, mop and stands use original procedural USD geometry.
Mechanical dimensions match the ordinary method, with an estimated internal
cone profile and compliant bundle stiffness. The supplied YAM adapter
provides the arm, floor, lighting and camera rig. Task code is MIT licensed.

## Trial measurements

The scripted demonstration reached full score after 284 actions (14.20 s).
The modeled runoff was 117.8 g into the bucket with 0 g spilled. The grippers
then raised the mop from the compressed state and held it in the draining
posture. The stationary grasp comparison left the water mass unchanged.

[mop]: https://www.m2mfg.com/products/details/ym-y310-m348-rayon-detachable-yacht-mop
[pail]: https://www.m2mfg.com/products/details/15-l-bucket-with-cone-wringer
[method]: https://fgchem.com/wp-content/uploads/2025/11/FG-Digital-Catalogue-Dec-2016-low-res.pdf

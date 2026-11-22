# Tighten and lock the forefoot laces on a hiking boot

Two-zone hiking boots can hold the lower lacing at a pair of integrated clamping hooks while the ankle lacing stays free. Each lace tail crosses the boot, routes through the opposing hook, and maintains tension after release. The reference uses one arm for each tail. The task accepts either arm assignment. [Hanwag two-zone lacing](https://www.hanwag.com/uk/en-gb/find-your-shoe/lace-your-boots/) · [Hanwag Bergler clamping hooks](https://www.hanwag.com/us/en-us/men/outdoor-lifestyle/doublestitched/bergler/)

## Reproduce the starting layout

Place a 287 × 119 mm adult hiking boot upright over a rigid hardwood or polymer fitting last secured to a 97 × 91 × 86 mm rectangular wood heel support. Use a Hanwag Bergler or a secondhand leather trekking boot with three pairs of lower eyelets and open forefoot clamping hooks. Choose a size and last for which the two hooks start about 84 mm apart and draw at least 6 mm closer, to 77 mm apart or less, when the two tails are pulled snug and locked by hand. Check that fit before running trials. A too-wide last or a rigid expander across the upper prevents the required closing travel. Set the toe away from the two robots on a 1100 × 1300 mm wood worktop. On the calibrated YAM station the sole center is about 379 mm ahead of the midpoint between the arm bases and the heel support is centered 316 mm ahead of that midpoint. The empty hooks are about 110 mm back from the toe. Both arms begin open and clear of the boot.

Number the eyelet pairs 1 to 3 from the locking hooks toward the toe. Thread one 4.6 mm round polyester lace in the order L1, R2, L3, R3, L2, R1 with a straight rung at the toe and crossed runs over the tongue. Leave both ends outside their respective final eyelets and the upper ankle hooks empty. At each reset lift both tails from the clamping hooks. Set the hook centers to 84 ± 2 mm with a removable spacer. Centre the lace at the toe rung and even the two free tails to 180 ± 5 mm from eyelet 1 to the aglets. With the hooks at the reset separation, pull out large loops and set about 10 mm of pick-up at the midpoint of each crossed span and 5 mm at the toe rung, measured normal to the line between its two eyelets with a ruler. A temporary plastic spacer under the span sets each height. Remove all spacers, photograph the routing, and let both ends hang on their own sides below the sole.

One continuous lace connects the two native tails through the crisscross path. Jointed capsules with native contact, bending and free axial twist slide through the six lower guides. Each eyelet guides the same cord through an 8 × 8 mm opening. A native guide constraint keeps a short lace section in its opening with ±11 mm axial travel and velocity-dependent drag. The surrounding contact surfaces and changing free length transfer pull through the lower crossing and into the spring-backed upper. The fixed foot last supports the upper, and the tapered spring cheeks spread under lace pressure and keep the two crossed ends in place by friction. The procedural boot is original geometry. The D6 joint recipe is adapted from the MIT-licensed [overhand-knot task at commit 6567167](https://github.com/robocurve/benchmark-tasks/blob/6567167/tasks/tie-overhand-knot/rope.py) and carries its notice in [LICENSE](LICENSE).

## Scoring

Inspect a fixed view of each hook to confirm a continuous crossed tail remains in its channel. Award 0.35 for each tail seated in its opposing clamping hook. Award 1.0 if both remain seated for a continuous second with the hook centers at most 77 mm apart and at least 6 mm closer together than on the reset photo. Both grippers must stay at least 25 mm open, 105 mm from the instep center, and 20 mm clear of the lace during that second. Inspect routing and release on the video and measure the horizontal hook separation with a ruler. The simulated scorer uses the native position of the cable and spring cheek together to verify seating in the grooved channel.

## Run locally

Install the task into the same Isaac Lab-Arena environment as the YAM adapter using `pip install -e . --no-deps`. Run the native scripted reference at seed 1415158438 with the 740-action limit, 20 Hz control and 480 Hz PhysX.

```bash
OMNI_KIT_ACCEPT_EULA=YES python run.py --device cuda:0 --physics-device cpu --output /tmp/boot-lace-demo
OMNI_KIT_ACCEPT_EULA=YES python run.py --no-cameras --policy hold --output /tmp/boot-lace-idle
OMNI_KIT_ACCEPT_EULA=YES python run.py --no-cameras --policy one-side --output /tmp/boot-lace-one-side
```

The standalone runner emits raw action and native state trajectories, a step-by-step score, a receipt, and the native scene's screenshot and three camera streams when rendering is enabled. The accepted-limit composite keeps the overview above the two wrist views.

The revised reference reached 1.0 in 594 actions. The continuous cord passed through all six lower eyelets in order at reset and at the finish. Both crossed ends stayed locked while the hook gap narrowed from 83.7 to 76.4 mm. Both grippers remained open and clear for the last 20 actions. The idle and single-tail controls ended at 0.0 and 0.35, respectively, after 740 actions.

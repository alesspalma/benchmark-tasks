# Tighten and lock the forefoot laces on a hiking boot

Two-zone hiking boots can hold the lower lacing at a pair of integrated clamping hooks while the ankle lacing stays free. Each lace tail crosses the boot, routes through the opposing hook, and maintains tension after release. The reference uses one arm for each tail. The task accepts either arm assignment. [Hanwag two-zone lacing](https://www.hanwag.com/uk/en-gb/find-your-shoe/lace-your-boots/) · [Hanwag Bergler clamping hooks](https://www.hanwag.com/us/en-us/men/outdoor-lifestyle/doublestitched/bergler/)

## Reproduce the starting layout

Place a 287 × 119 mm adult hiking boot upright over a rigid hardwood or polymer fitting last sized to the boot and secured to a 97 × 91 × 86 mm rectangular wood heel support. An ordinary secondhand leather or synthetic boot with integrated open steel clamping hooks works with 4.6 mm round polyester laces. Set the toe away from the two robots on a 1100 × 1300 mm wood worktop. On the calibrated YAM station the sole center is about 379 mm ahead of the midpoint between the arm bases and the heel support is centered 316 mm ahead of that midpoint. The lower-zone eyelets are already threaded. Free working lengths from the last eyelets are about 180 mm on both sides, with their tips hanging loosely below the boot sole. The two empty hooks are about 110 mm back from the toe, initially 90 mm apart. Both arms begin open and clear of the boot. To reset, lift the tails out of the hooks, loosen the lower zone, and let each tail hang on its original side.

Each lace is a chain of 24 7.5 mm segments with native contact, bending and free axial twist. The fixed foot last gives the upper a stable fitting surface, and spring-backed lace stays approximate the motion and recovery of the upper. The generated steel clamping hooks include a tapered spring cheek. The simulated cheeks spread under lace pressure and hold it by friction. The anchored starts represent the standing lengths below the last visible lower eyelets. The procedural boot is original geometry. The D6 joint recipe is adapted from the MIT-licensed [overhand-knot task at commit 6567167](https://github.com/robocurve/benchmark-tasks/blob/6567167/tasks/tie-overhand-knot/rope.py) and carries its notice in [LICENSE](LICENSE).

## Scoring

Inspect a fixed view of each hook to confirm a continuous crossed tail remains in its channel. Award 0.35 for each tail seated in its opposing clamping hook. Award 1.0 if both remain seated for a continuous second after both grippers withdraw at least 105 mm from the instep center and the hook centers are at most 74 mm apart. Inspect routing and release on the video and measure the hook separation with a ruler. The simulated scorer uses the native position of the cable and spring cheek together to verify seating in the grooved channel.

## Run locally

Install the task into the same Isaac Lab-Arena environment as the YAM adapter using `pip install -e . --no-deps`. Run the native scripted reference at seed 1415158438 with the 740-action limit, 20 Hz control and 480 Hz PhysX.

```bash
OMNI_KIT_ACCEPT_EULA=YES python run.py --device cuda:0 --physics-device cpu --output /tmp/boot-lace-demo
OMNI_KIT_ACCEPT_EULA=YES python run.py --no-cameras --policy hold --output /tmp/boot-lace-idle
OMNI_KIT_ACCEPT_EULA=YES python run.py --no-cameras --policy one-side --output /tmp/boot-lace-one-side
```

The standalone runner emits raw action and native state trajectories, a step-by-step score, a receipt, and the native scene's screenshot and three camera streams when rendering is enabled. The accepted-limit composite keeps the overview above the two wrist views.

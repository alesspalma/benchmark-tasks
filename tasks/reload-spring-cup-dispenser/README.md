# Reload a spring-fed self-service cup dispenser

Fast food and counter workers replenish drink stations throughout service. An in-counter spring-fed dispenser loads through its serving opening. The loose stack enters mouth-first and has to stay nested as it passes the inward levers, while the follower compresses under it. The reference motion supports the lower cup with one hand and holds the outside wall of the upper cup with the other during transfer. The lower hand guides the stack into the guard and withdraws so the upper hand can push downward against the follower. The roles can be swapped. [San Jamar in-counter specifications](https://www.webstaurantstore.com/documents/pdf/in-counter_cupdispenser_mountinginformation.pdf) · [San Jamar refill instructions](https://www.webstaurantstore.com/documents/pdf/712c5450c.pdf)

O*NET identifies replenishing serving stations and cabinets as a core task for Fast Food and Counter Workers. BLS counted 3,855,900 U.S. jobs in this occupation in 2025. Drink-station restocking includes cup sleeves in front-of-house training. The Cornelius service sheet for an automated McDonald’s drink station gives another example of routine manual refilling of beverage cup tubes and explains how overfilling makes them dispense multiple cups. The San Jamar unit here is the spring-fed self-service configuration. The number is occupation-wide. [O*NET occupation and tasks](https://www.onetonline.org/link/details/35-3023.00) · [BLS employment table](https://www.bls.gov/ooh/food-preparation-and-serving/food-and-beverage-serving-and-related-workers.htm) · [Beverage-station training](https://sites.google.com/jolietcfa.com/jolietcfatraininghub/training-resources-foh) · [Cornelius cup-tube instructions](https://www.corneliusmcd.com/pdf/abs/quick-reference/ABS-ES_English.pdf)

## Physical setup and reset

Mount a San Jamar C5250C18 Euro Sentry 457 mm dispenser in the counter following its cutout template. Leave the under-counter tube accessible for resetting. Adjust the three levers to a compatible cup according to the manufacturer's fitting instructions. The 8 oz Dart Solo 378SI-0041 has a nominal 81 mm rim diameter, 51 mm base diameter and 93 mm height. Obtain six matched, undamaged paper cups from one sleeve. An 18-inch dispenser, reusable bench hole and six cups are sufficient for reproduction. [Dispenser and dimensions](https://www.webstaurantstore.com/san-jamar-c5250c18-euro-sentry-in-counter-4-24-oz-cup-dispenser-18-long/712C5250C18.html) · [Dart cup specifications](https://www.dartcontainer.com/products/foodservice-catalog/hot-coldcups/paper/solo-paper-hot-cups/378si-0041)

Use a 747.5 mm high counter and YAM hardware on 610 mm centers. Measure from the midpoint between the bases, with X forward into the counter and positive Y toward the left arm. Center the dispenser at (442, 30) mm, the loose mouth-down stock stack at (313, −110) mm and the 130 × 130 mm serving mat at (342, −226) mm. Leave the opening empty, check the follower rises when released, clear the mat and place the six cups mouth-down with their rims on a 36 mm supply riser. Return both arms to the ready pose with open grippers.

## Score

From a side-view camera use the ready cup height and count the rolled rims as all six enter and from the side as one comes out. Check that the cups are close to vertical, still nested and held at the opening without gripper support. Use a ruler with marks next to the guard set from the working height of the fitted stock. Count the cups after the trial by removing the stack from the fixture. On the mat, the full 81 mm cup rim must be inside its 130 × 130 mm boundary. In simulation, the mouth rim of the presented cup is within 13 mm below to 22 mm above the counter, corresponding to a cup base 78–113 mm above the counter, and cup axes lie within 9 mm of the dispenser axis. The six cups are free rigid bodies. The compressed follower and retaining levers respond to native PhysX contacts. Individual paper cups keep their shape under contact.

- **1.0** after verifying all six were retained hands-off, for exactly five cups remaining nested in the working position, one stable and mouth-down on the mat, and both grippers clear for at least 0.5 seconds.
- **0.65** for all six cups nested in the working position and both grippers clear for at least 0.5 seconds.
- **0.25** for a stable nested group of at least two cups at the working height and both grippers clear.
- **0.0** otherwise.

Either arm can take each role. The specified run starts with all cups outside the opening and the serving mat empty.

## Run

Use the approved Isaac Lab–Arena environment with the pinned versions in `pyproject.toml`. Install from this directory. The reset seed is **1415158438**. Limit each trial to **620 actions at 20 Hz**.

```bash
python -m pip install -e . --no-deps
python run.py --device cuda:0 --physics-device cpu --output /tmp/spring-cups-demo
python run.py --policy hold --no-cameras --output /tmp/spring-cups-hold
python run.py --policy reload_only --no-cameras --output /tmp/spring-cups-reload-only
```

The Inspect Robots embodiment is `spring_cups`. The environment exposes the three native camera views and YAM joint positions. The scripted policy uses elapsed time and joint positions. The scorer tracks physical cup poses and gripper clearance.

## Assets

Original fixtures, policies and documentation use the included MIT license. The pinned adapter supplies the YAM assets and calibrated cameras. [Robot asset terms](https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/assets/yam/LICENSE.i2rt) · [Camera asset license](https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc126418ea5e74ac567133d/LICENSE)

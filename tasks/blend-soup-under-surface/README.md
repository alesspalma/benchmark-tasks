# Blend soup with a guarded immersion blender

**Instruction** — Blend the soup in the jug, then set the blender on the rest.

Running a hand blender with the bell guard at the surface of hot soup can throw liquid toward the cook's hands or face. A spring-return power button lets the cook control the cycle without letting go. One YAM arm steadies the loose jug by its rubber-coated handle while the other places the bell below the surface, runs the motor, and releases the button without lifting. The rotation must stop before withdrawal. [Cuisinart hand-blender safety and blending instructions](https://www.cuisinart.ca/on/demandware.static/-/Sites-master-ca/en_CA/v1776775661431/Instruction%20Booklet/csb87c_en.pdf)

## Setup and reset

Use the two parallel-gripper YAM arms at 610 mm center spacing on a counter 747.5 mm above the floor. Locations are measured forward and left from the midpoint of the arm mounts. Keep the countertop dry. The jug and its filled contents weigh approximately 1.94 kg.

| Object | Physical setup |
|---|---|
| Mixing jug | Clear food-safe polypropylene, 148 mm inside diameter, 160 mm rolled rim diameter and 155 mm high. Base center at 368 mm forward, 31 mm left. Its molded U-handle extends toward the left arm. The rubber-coated vertical handhold is 27 mm wide and 21 mm thick, with its center about 122 mm from the jug axis. |
| Fill | Approximately 1.8 L of two food-safe purees of equal viscosity, one light orange and one darker orange, to a height of 113 mm from the jug base. Split the two colors roughly across a diameter at reset and remove the divider before starting. Keep the real evaluation fill cool. |
| Temperature cue | A clipped kitchen probe thermometer on the back side near the rim, with a visible inert training display set to 72 °C for the cold physical trials. |
| Immersion blender | A 315 mm hand blender with a 58 mm stainless slotted bell, a 40 mm polymer motor handle with rubber grip, a visible motor light and an 8 mm spring-return power pad. Use a handheld one-touch type and keep any cord clear of the arms. The blender starts unpowered, upright in the rest. |
| Drip rest | 110 mm glazed ceramic dish with a low rim, flat floor and shallow centering pads. Center at 374 mm forward, 166 mm right. The sitting plane is 9 mm above the counter. |

Recreate the setup photograph for each trial. Rinse the blender, return its blade fully stopped, center both objects on their marks, and put the purees side by side in the jug. Remove the divider and restore both arms to their ready pose with jaws open. Run unsafe-order controls in simulation.

## Judge the physical result

Use side or angled overhead video to see whether the entire bell guard enters the jug and is below the liquid before motor activation. Check that it remains submerged until the blade stops. The guard's top needs at least 4 mm of liquid above it, and the jug stays within 8° of upright during powered blending. A contact or status lamp on the momentary button and motor sound or rotation footage help establish the order. A start out of the liquid, an exposed rotating bell during withdrawal, or powered operation while the jug tips earns zero for the whole episode.

For the color comparison, photograph the fill from above in diffuse illumination. Use the luminance or orange color channel that best distinguishes the two starting purees. Use reference swatches of the two batches at the same illumination to map their colors to 0 and 1. Sample the fill on 48 equal areas, formed by 16 wedges and three rings of equal area. Calculate the standard deviation divided by mean of the resulting concentration values.

| Score | Outcome |
|---|---|
| 0.00 | No qualifying blend time, an unsafe event, or the jug fails the final upright position check. |
| 0.25 | At least 1 second of safely submerged blade rotation. |
| 0.50 | At least 2 seconds of safely submerged blade rotation and a concentration coefficient of variation no greater than 0.40. |
| 0.75 | At least 4 seconds of safely submerged rotation, a coefficient of variation no greater than 0.19, and the blade stopped while submerged. |
| 1.00 | The 0.75 state plus the blender upright, powered off and supported by the rest for 0.5 seconds with the hands clear. The jug is upright within 30 mm of its mark. |

Judge resting when the guard touches the bottom of the rest, its center is within 10 mm of the rest's center and the tool is within 9° of vertical. A hand is clear when its grip center has moved at least 69 mm from the blender's grip center. Use either arm for either role. Idle and unsafe controls run from the same setup.

## Run and reproduce

Use the approved Isaac Lab–Arena installation and the packages pinned in `pyproject.toml`. For the standard observations, the Inspect Robots embodiment is `soup_blend`. Its episode has one fixed initial layout, seed **1415158438**, 650 actions at 20 Hz and PhysX CPU physics at 240 Hz. The ordinary-action reference uses elapsed time and observed robot joints. It uses physical grips and lever travel at every stage.

```bash
python -m pip install -e . --no-deps
python run.py --device cuda:0 --physics-device cpu --output /tmp/soup-blend-demo
python run.py --device cpu --physics-device cpu --no-cameras --policy hold --output /tmp/soup-blend-idle
python run.py --device cpu --physics-device cpu --no-cameras --policy premature --output /tmp/soup-blend-premature
python run.py --device cpu --physics-device cpu --no-cameras --policy withdrawal --output /tmp/soup-blend-withdrawal
```

The native rotor velocity and switch travel are sampled every physics step. The solution logs the measured trajectory, safety events and score in the output directory. The visual soup is a conservative reduced circulation model. Pairwise tracer flux depends on the submerged blade position and speed. Its reduced-order flow does not simulate droplets, phase changes, solids or temperature. The displayed temperature is a starting-scene cue. The task geometry is original and MIT-licensed. [Pinned YAM assets and license](https://github.com/kairosiann/yam-isaac-lab-arena/tree/0059d02a1a91bc6834c1d0846022891df0c78730)

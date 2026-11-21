# Load two cables into a robot-axis cable carrier

**Instruction** — Lay the blue and orange cables into the robot axis's cable carrier. Snap both crossbars closed, then give the exposed end of the pair a gentle upward tug and leave the cables captured.

A linear robot axis needs a travelling carrier to keep its moving cables and hoses inside a controlled bend path. During installation the technician holds the springy service leads below the top of the carrier while closing removable crossbars. A raised or misplaced lead can be caught across the clip seat or left outside the carrier. The final tug verifies containment before the axis is operated. The insulated leads, two separately operated snap bars, and guide trough reproduce the loading step on a medium-duty axis installation. [igus E2/000 crossbars](https://www.igus.com/cable-carriers/resources/energy-chains-system-e2-000-advantages-com) · [Cable carriers for linear and robotic motion](https://www.igus.com/cable-carriers)

## Setup and reset

Use two parallel-gripper YAM arms with 610 mm center spacing at a bench 747.5 mm above the floor. The route runs left to right, with its axis 415 mm forward from the midpoint of the arm mounts. The coordinate convention is forward, left, then height above the floor. All hardware is de-energized in physical trials and the short axis is stationary.

| Object | Fixed starting layout |
|---|---|
| Robot linear-axis plate | 272 × 458 × 15 mm steel, plate center 473 mm forward and 3 mm left, supporting a pair of guides, actuator housing and stationary carriage. The guides run left to right behind the carrier. |
| Carrier and trough | Polymer carrier over a steel trough 112 × 377 mm. Six nominal 48 mm pitches are exposed. The internal width is about 74 mm, with the bed at 803 mm height. Two adjacent upper crossbars are hinged on the near side, 48 mm and 103 mm to the right of the arm midpoint. Their closed axes are 841 mm high. |
| Snap crossbars | Two 86 × 18 × 9 mm yellow polymer bars with thumb pads and snap receivers. Open both to their resting detents at about 84°. |
| Cable pair | Two 9 mm insulated leads, blue and orange, anchored in one strain relief at 415 mm forward, 180 mm left, 820 mm high. They pass over the service opening and extend about 60 mm to its right. The jackets are about 26 mm apart in the service section. Two reusable soft ties keep the pair parallel under manipulation, one by the near lay point and one near the exposed end. The setup image shows the leads above the two open bars before loading. |
| Pull witness | A short steel reference mark above the exposed service tie, outside the ends of the track. Use it to check upwards movement of approximately 16 mm during the tug. |

Before each trial, open both crossbars, lift the service cable ends outside the carrier, and let the leads and spring detents settle without either hand touching them. Both grippers start open in their ready poses. Use the same compliant cable types and strain relief. A ruler or the calibrated side view suffices for the proof movement. The reference checks the local loading step, with the service end accessible before the permanent cable connectors are mated.

## Judge the physical result

Each clip station is populated when all of both jackets are below the crossbar's seating plane and between the side plates. Judge from the setup and overhead and side recordings. A crossbar is seated when the catch has clicked home and the bar lies within 5° of flush. When both bars are seated, mark the exposed end at rest. The upwards proof pull moves that end at least 16 mm while both cables remain below both seated bars. The loaded pair must remain secured after release. Either arm can perform either role.

| Score | Outcome |
|---|---|
| 0.00 | No qualifying loaded or seated station. |
| 0.30 | Both stations hold both leads below the seating plane. |
| 0.55 | At least one seated bar retains both jackets at its station. |
| 0.80 | Both seated bars retain both jackets. |
| 1.00 | The 0.80 state survives the proof pull and a further 0.60 seconds with both hands clear. |

Use the highest satisfied milestone. Cut, missing, or pulled-out leads cannot count as contained. Scoring observes the physical objects and never requires an arm identity.

## Run and reproduce

Use the approved Isaac Lab–Arena environment and the dependencies in `pyproject.toml`. The episode uses seed **1415158438**, one fixed starting layout, 570 actions at 20 Hz, and 240 Hz CPU PhysX. The runnable Inspect Robots embodiment is `energy_chain`. The reference controller uses standard measured robot joints and a taught geometric work sequence.

```bash
python -m pip install -e . --no-deps
python run.py --device cuda:0 --physics-device cpu --output /tmp/energy-chain-demo
python run.py --device cpu --physics-device cpu --no-cameras --policy hold --output /tmp/energy-chain-idle
python run.py --device cpu --physics-device cpu --no-cameras --policy open_gate --output /tmp/energy-chain-open-gate
python run.py --device cpu --physics-device cpu --no-cameras --policy miss_cables --output /tmp/energy-chain-miss-cables
```

The crossbars and jackets are native contact bodies. The short flexible cable span uses three spring-connected segments. A bounded bistable torque models each molded snap detent while native hinges, stops and contacts determine bar movement. The installed axis is fixed during the task. The cable jackets use original geometry, and the pinned YAM dependency supplies the arm assets and their licenses.

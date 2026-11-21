# Open a roll of change and refill the cash drawer

Replenish the nickel compartment from a standard paper roll. Support both ends across the steel cash drawer edge and bend until the paper opens. Keep the crimped ends down on the way to the drawer, then tip the coins into the marked well and discard the empty paper. A cashier workflow for paper-wrapped change is described in the background of the [Telequip coin-roll opener patent](https://patents.google.com/patent/US4825738). Coin dimensions and masses are taken from the [U.S. Mint nickel specification](https://www.usmint.gov/learn/coins-and-medals/circulating-coins/nickel).

## Reset

- Use two 20 Hz YAM arms at the standard station. The bench top is at Z = 747.5 mm. Coordinates use X forward from the robot bases and Y toward the left arm. Use a steel cash drawer with a hard-plastic adjustable coin insert and a separate hard-plastic scrap tray.
- Prepare a standard 40-nickel kraft-paper roll with both ends crimped. The 78 mm coin stack is 200 g. The outside diameter of the wrapper is about 24 mm and its full modeled length is about 84 mm. Blue stripes indicate nickels.
- Set the roll across Y, centered at X = 323 mm and Y = 0 on the shallow nontipping felt rest. The drawer is fixed and already open. The exposed steel opening edge is at X = 430 mm, Y = 0, Z = 812.5 mm.
- The nickel well is X = 382–619 mm and Y = −74 to 74 mm, with another compartment on either side. Set the adjustable dividers to give the center well this clearance. The inside bottom is about Z = 759.5 mm and the side dividers rise about 67 mm. Mark the center compartment 5¢. Keep all three wells empty.
- The green open paper tray lies closer to the robot, with interior bounds X = 209–304 mm, Y = ±162 mm, floor Z = 753.5 mm and rim Z = 780.5 mm. It starts empty.
- Start the arms in their normal ready poses with fingers open. Replace the wrapper with a freshly filled intact roll and clear both trays before any retry.

## Physical scoring

Check that the paper is split, count the original coins wholly inside the blue-marked well, and check that both paper pieces lie within the scrap receptacle. The grading allows intermediate regrasping and either arm can deliver either half. An intact roll earns 0. A split roll earns `0.2 + 0.6 × (nickels in the correct well / 40)`. Score 1.0 when all 40 nickels and both paper pieces remain in their assigned receptacles for 0.6 seconds and every center moves 2 mm or less during that interval. A top photo and manual coin count verify the final arrangement.

## Simulation and execution

The forty 5 g nickels make native frictional contact with each other, the sleeve interiors, and the drawer. Six tensile connections between two rigid paper shells model the kraft seam. The calibrated extension limit is 3.6 mm at the steel opening lip and 8.5 mm away from the lip. Tearing removes the seam forces. Contacts and gravity determine how the coins separate and move.

Install this directory and run via the Isaac Lab–Arena dependencies used by the station.

```sh
python -m pip install -e .
OMNI_KIT_ACCEPT_EULA=YES python run.py --device cuda:0 --physics-device cpu --output /tmp/rolled-change-demo
OMNI_KIT_ACCEPT_EULA=YES python run.py --device cuda:0 --physics-device cpu --no-cameras --policy hold --output /tmp/rolled-change-hold
OMNI_KIT_ACCEPT_EULA=YES python run.py --device cuda:0 --physics-device cpu --no-cameras --policy opened --output /tmp/rolled-change-opened
```

The fixed seed is 1415158438. The 800-action limit is 40 seconds at 20 Hz. The recorded scripted reference reached 1.0 after 720 actions (36.0 seconds). Over 800 actions, the idle control left the roll intact and scored 0.0. The opening-only control split the paper at the steel edge, caught two coins in the well and scored 0.23. The models and task-specific geometry are generated from source under the accompanying MIT license.

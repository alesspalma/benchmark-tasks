# Vacuum-seal a filled pouch in a countertop sealer

Pulling a vacuum requires a flat pouch mouth across the heating bar and into the drip channel. The left and right hands guide the two thin edges together. One hand lowers the gasket lid while the other turns the side keeper to Operate. Either hand can do either role. After the cycle, release the lid, remove the pouch, and leave it on the counter. [FoodSaver FM2000 quick-start guide, page 1](https://newellbrands.imgix.net/34ef7d29-6a24-3d73-9c7a-60428e0c7856/34ef7d29-6a24-3d73-9c7a-60428e0c7856.pdf)

## Equipment and reset

Use an FM2000-class external suction sealer with a hinged lid and rotary keeper, standard embossed nylon/PE bags, and a dry rigid load. A commercially available FM2000 and its standard bags are the reference for the physical trial. The model is a dimensioned custom implementation of that device type. Use a fresh bag each trial and reuse the load. The setup uses no raw food or liquid.

X increases from the YAM bases over the counter towards the sealer. Positive Y is to the left arm. Dimensions below are in mm. Positions refer to a floor datum at Z = 0.

| Part | Position and specification |
| --- | --- |
| Robot and counter | Both robot bases at (0, ±305, 750). Laminate counter surface at Z = 747.5. |
| Sealer | Approximately 386 wide × 191 deep × 87 high, with a rotary keeper and gasket lid. Front deck starts at X = 394. The channel opens across Y = ±126 with the usable insertion region X = 442–490. The heater bar is at X = 423, in front of the gasket. Lid hinge axis runs through (571, 0, 797) parallel to Y. The side keeper rotates 24° and holds the lid by its outboard catch. |
| Pouch | Standard embossed food vacuum bag, cut to 197 × 266, 80–100 µm per ply. Seal its far end on the machine in advance if using a roll. Its open lip starts at X = 376 centred at Y = 0 and facing the sealer. At least 76 mm of empty film separates its contents from the opening. |
| Contents | One dry, rounded 91 × 132 × 24 block of smooth hardwood or stiff food-safe polymer approximates a dry load. Place it centrally near the closed end. Use clean rounded corners so the bag can collapse without puncturing. |

Clean the gasket, trough and strip with a dry cloth. Set the keeper to Open and raise the lid to its open stop. Insert the test load into a new pouch. Lay it with its straight open lip centred on the seal bar and 18 mm in front of the deck, using a ruler and the setup image. Keep the two loose film layers together at the mouth without forming a fold. Keep the sealer and pouch clear of both stationary robot hands at reset. Press the external controls during the cycle and allow the manufacturer's recommended cooling time between trials.

## Scoring

Use the appliance cycle indicator during the trial and inspect the pouch afterward. With the demo scene, the simulator tracks the mouth centre, the two panel joins and the two corners against the boundaries of the channel. The lock and lid joints are read independently. The cycle must keep the full opening in the channel with the lid shut and the keeper in Operate throughout. The simulation also checks alignment across the entire heater line and direct contact on every film panel.

- **0.00** if the full opening never reaches the channel and no seal has been made.
- **0.20** with the entire cut edge past the heater strip and inside the vacuum channel.
- **0.40** while the lip is still in place and the keeper and lid are in their closed operating positions.
- **0.70** for a completed, airtight seal. The film must lie tightly against the entire test load with a single continuous weld from side seam to side seam. After removal and at least one second away from the external vacuum source, a side view must show the film still flush around the load. Inspect in transmitted light for gaps, wrinkles crossing the weld or tears. To resolve an ambiguous inspection, press the free film away from the seams with a blunt finger and check that the pouch stays taut without reopening or lifting around the load.
- **1.00** when that intact vacuum-sealed pouch rests flat on the counter for one continuous second. The entire open-end edge, including both corners, is at least 15 mm in front of the front deck, and both grippers are clear of the pouch.

An incomplete weld or a leaking pouch fails the airtight criterion. A misplaced mouth may be pulled back out and positioned correctly before retrying. The score permits either hand to perform either operation.

## Run and reproduce

Use the pinned Isaac Lab-Arena environment specified in `pyproject.toml`. The fixed trial seed is **1415158438**. The episode limit is 1,000 actions at 20 Hz. For a non-rendered trial, the simulator runs rigid-body dynamics at 240 Hz on CPU.

```bash
python -m pip install -e . --no-deps
python run.py --device cuda:0 --output /tmp/pouch-demo
python run.py --no-cameras --policy hold --output /tmp/pouch-idle
python run.py --no-cameras --policy short-mouth --output /tmp/pouch-short
```

The Inspect Robots embodiment is `vacuum_pouch`. The reference policy is `vacuum_pouch_scripted` and uses standard joint commands. The hinged film model retains native contact with the heating strip and gasket. A successful simulated appliance cycle changes only the visual film contraction and weld.

`vacuum_pouch.builder.create_assets(directory)` exports meter-scale USD collision and visual geometry and a JSON inventory from a launched Isaac Lab app. The sealer, pouch, and scene solids are original MIT-licensed assets. Robot and camera geometry use the pinned YAM adapter and its assets. [YAM asset license](https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/assets/yam/LICENSE.i2rt)

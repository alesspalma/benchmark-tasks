# Remove and separate both robot mop-pad assemblies

The cell tests a household robot-mop maintenance operation. Turn over the whole cleaner with the two side handles, lift both magnetic mop assemblies off their drive mounts, peel the two microfiber pads off their rigid carriers, and sort the cloth and the plastic into their own trays.

The simulated station uses a 350 mm, 4.28 kg dual-spin cleaner, mounted in a bench-supported turnover cradle with two rubber grips. For a physical replication, clamp a fully powered-off robot between padded axle supports, leave its underside clear at the inverted stop, and use two shallow trays for separating the cloth and plastic hardware. The matte polymer plates have central mounting bosses and reusable microfiber annuli. The turnover fixture, sorting trays, and robot geometry are generated from dimensions in `robot_mop/config.py` and `robot_mop/builder.py`.

Product references

- [iRobot Plus 505 Combo size, mass, and dual mop system](https://www.irobot.com/en_US/roomba-plus-505-combo-robot-plus-autowash-dock/N185020.html)
- [iRobot procedure and photos for peeling the cloth pads](https://homesupport.irobot.com/articles/en_US/Knowledge/50520)
- [iRobot instructions for magnetic mop-pad plate engagement](https://answers.irobot.com/en-CA/knowledge/50506)

## Run and score

Use the repository's Isaac Lab Arena and bimanual YAM setup. Install this task in that environment with `python -m pip install -e . --no-deps`. From this task folder, run:

```bash
python run.py --policy demo --device cuda:0 --physics-device cpu --output runs/demo
python run.py --policy hold --no-cameras --max-steps 1400 --output runs/hold
python run.py --policy demo --no-cameras --max-steps 245 --output runs/turnover-only
```

The fixed episode uses seed `1415158438`, 20 Hz control, 240 Hz physics, and a maximum of 1,400 actions (70 seconds of simulated time). The native run writes three 640 × 360 H.264 camera streams at 20 fps, from the initial frame through the final action. The submission demonstration places the 1280 × 720 overview above the two native wrist streams, left then right, in a 1280 × 1080 video. Each trial writes `receipt.json` with the final scorer metrics and `trace.jsonl` with the per-action state and scores. The reference controller follows taught Cartesian poses using elapsed time and YAM joint positions. The turnover-only control runs the reference through inversion and stops before the first carrier grasp. The full idle control leaves both wrists in their ready poses.

The scorer checks native chassis orientation, the distance of each carrier from its magnetic mount, separation of each fabric pad from its corresponding carrier, and the final resting poses in the two trays. Both simulated halves of each intact microfiber pad have to arrive in the blue tray. Each hard carrier must be resting in the green tray. Full credit requires all four articles released and settled in their correct trays for half a second with the chassis inverted. Scoring accepts either arm for each action.

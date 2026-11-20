# Inflate a football through its needle valve to the marked pressure

A partly inflated match ball needs air before play. Stabilize it, insert the standard hollow needle straight through the rubber valve and work the hand pump while the hose remains connected. Stop at the marked pressure, pull the needle straight out and set the equipment down. The ball valves accept the narrow needle with little clearance. Inserting it at an angle can damage the valve and lose pressure.

The ball is a regulation size 5 football, 420–445 g, with a marked recommendation of 0.8 bar. The tool is a dual-action hand pump with a flexible hose, an inflation needle and a pressure gauge. Either arm may insert the needle or operate the piston. [Match ball and specified pressure](https://www.decathlon.co.uk/p/official-match-ball-pro-league-2023-2025/343208/c227m8771323) · [Dual-action hand pump with gauge and hose](https://www.kuebler-sport.com/tanga-sports-ball-pump-with-pressure-gauge-f8783.html) · [SELECT ball maker’s straight-needle inflation procedure](https://www.select-sport.com/pages/inflating-instructions)

## Physical build and reset

Use a 1,100 × 1,250 mm flat bench, 747.5 mm above the floor, with the YAM bases 610 mm apart. X points forward from the base line and positive Y points to the left arm. The coordinates below give X and Y in millimeters. The overview camera watches the ball and pump together. A camera on each wrist shows needle engagement and piston travel.

| Item | Placement |
| --- | --- |
| Match ball | Fix a rubber support ring at [348, 157]. Its aperture is Ø122 mm and its top rises 24 mm above the bench. Put the stock size 5 ball in the ring with its valve uppermost. The support ring is open and prevents the resting ball from rolling off the worktop. |
| Needle | Connect a standard Ø3 mm ball needle and its hose to the pump. Place the needle in a rubber-topped stand at [274, 282], chuck 82 mm above the bench and straight stem pointing down. A 3.3 mm socket protects the point while leaving the chuck accessible. |
| Pump | Rest a dual-action gauge pump horizontally along the Y axis. Locate its closed end at [460, 56], approximately 51 mm above the bench. The piston extends toward negative Y. Place rubber pads beneath the lower and upper ends of the barrel and leave the barrel accessible to both grippers. The pump is loose in the cradle. Use a flexible hose with at least 530 mm of available length for this layout. Fit an extension with the matching threaded coupling if the supplied hose is shorter. |
| Pressure instruments | Mark 0.790–0.825 bar on the pump face with a green label. Use an independent ball pressure gauge resolving 0.01 bar or better to reset and score, applying the same insertion method each time. The simulated dial expands the service range to make the band readable in the wrist camera. |

1. Fit the pressure gauge and adjust the ball to 0.755 ± 0.005 bar at room temperature. Remove the gauge straight from the valve. Follow the ball maker’s valve and needle lubrication instructions.
2. Place the ball valve-up in the support ring, then place the unseated needle in its stand. Leave enough slack for the straight approach above the valve. The hand pump begins with its piston all the way in.
3. Confirm that the pump, needle and ball are all clear of the open grippers in their start poses. Reset to the same pressure before each trial.

An operator can check the selected pump and ball pair in advance with a few full strokes at the reset pressure. Variation in the pump bore and the ball bladder affects the required stroke count. The success condition remains the measured pressure band. [Mechanical ball-pressure gauge](https://www.select-sport.com/products/pressure-gauge-mechanical-810067)

## Physical score

Compare the ball pressure to its reset value using the independent gauge. Observe the two wrist videos for seating, extraction and hands clear. Score 0.35 if pressure rises by at least 0.014 bar. Score 0.7 if it is between 0.790 and 0.825 bar. Score 1.0 when the ball holds that pressure, the needle is out, the ball and pump are supported and both hands are clear for one second. Any pressure above 0.85 bar during the attempt earns zero. The pump dial records pressure while connected. A disconnected stroke or a needle against the outside of the ball passes no air into the ball.

The native model uses the full-size loose ball on a rubber cradle, the steel needle and its small valve opening, the loose pump barrel and a real prismatic piston. A reduced-order pressure model integrates piston displacement through a seated and aligned needle. Partial off-axis contact leaks through the valve. Delivered air depends on the effective bore, stroke length, pressure coupling and ball volume. The model uses a rigid shell collision approximation and depicts the slack hose using a curve between its endpoints.

## Run and score

Use the accepted-license Isaac Lab Arena runtime and the pinned adapter in `pyproject.toml`. Seed is 1415158438 and the horizon is 940 actions at 20 Hz (47 seconds).

```bash
python -m pip install -e . --no-deps
python run.py --device cuda:0 --physics-device cpu --output /tmp/football-demo
python run.py --device cuda:0 --physics-device cpu --no-cameras --policy hold --output /tmp/football-hold
python run.py --device cuda:0 --physics-device cpu --no-cameras --policy air-only --output /tmp/football-air-only
```

The Inspect Robots embodiment is `football_pump`. The native reference directly commands the standard robot joints. It uses measured ball and needle poses to orient the approach and to refine alignment. Each trial records all commanded actions and its final score. The hold control leaves both hands at their reset pose. The air-only control works the real piston with the needle in its starting stand. The camera run records all initial and action frames in the three native 640 × 360 views at 20 fps.

## Assets and license

The match ball, valve, needle, pump, pressure display and workbench are original task geometry under the repository’s MIT license. The YAM geometry and camera calibration are supplied by the pinned adapter. [YAM adapter](https://github.com/kairosiann/yam-isaac-lab-arena/tree/0059d02a1a91bc6834c1d0846022891df0c78730) · [YAM asset terms](https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/assets/yam/LICENSE.i2rt) · [Camera license](https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc126418ea5e74ac567133d/LICENSE)

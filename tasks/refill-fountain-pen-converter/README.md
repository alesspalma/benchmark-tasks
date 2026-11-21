# Refill a fountain pen from bottled ink

A fountain pen with a push-button converter takes ink up through its submerged feed. The pen needs to stay at a controlled depth while the other hand presses and releases the pump. Finish with the pen upright on its protective rest so the filled reservoir can be inspected. The two arms can exchange roles. [Pilot filling and care guide](https://www.pilot.co.jp/support/manual/fountain/fountain_en.pdf) · [Pilot converter dimensions and capacity](https://webcatalog.pilot.co.jp/products/DispDetail.do?itemID=t000100002533&volumeName=00004)

## Physical setup

Use a 747.5 mm high desk and aligned YAM arms 610 mm apart. Coordinates below are in metres. X points forward from the midpoint between the robot bases, Y points left, and Z points up from the floor.

| Item | Starting arrangement |
|---|---|
| Fountain pen | Uncapped 12.6 mm polymer grip section, metal nib, and installed 80 × 9 mm push-button converter with 1.10 mL reservoir capacity. Stand it vertically nib-down on the ceramic rest, at (0.339, 0.119). The shoulder sits on the lip and the pocket leaves the nib tip clear. Start with an empty converter and its button fully extended. |
| Pen rest | A 57 × 62 mm ceramic foot and round shoulder cup with a 15.6 mm inner opening and 37 mm unobstructed tip pocket. The lip is 87 mm above the tabletop and the floor of the pocket is about 50 mm above it. A machined or printed rest of the same dimensions is usable. |
| Ink bottle | Open 64 mm glass bottle at (0.377, −0.033) with a 46 mm inside mouth diameter. Add 30 mL of washable blue fountain-pen ink to the 49 mm level line. The bottle is freestanding. Use a tall 30 mL bottle with these approximate dimensions and a narrow well, or a glass bottle with a removable filling insert with a 24 mm opening set to the same level. The insert guides the pen laterally while the hand sets its immersion depth. |
| Detached pen parts | The 57 mm cap and 93 mm barrel lie behind the pen rest and bottle on the writing mat. They stay clear of the filling area. |

Before a trial, rinse and drain the converter, wipe the pen dry, refill the bottle to its mark and restore the listed positions. Confirm the pen feed begins above the ink and the nib does not rest on the bottom of the ceramic cup. The barrel and cap are already removed as is usual for filling an exposed converter. The trial ends after return to the rest so the ink level remains visible.

## Score the outcome

Observe the filling from the side and compare the ink level with graduations on the clear reservoir, or weigh the same pen section before and after filling with a 0.01 g balance and use the ink's measured density. At full credit, the converter contains at least 0.90 mL and the pen rests unsupported and within 14° of upright, the tip within 9 mm laterally and 8 mm vertically of its starting location, and both hands at least 44 mm from the grip for 0.50 seconds. The bottle remains upright within 10° and 20 mm of its start. Any bottom strike during the trial or an out-of-bounds final bottle gives 0. Without those failures, visible fill of at least 0.12 mL, 0.50 mL and 0.90 mL earns 0.25, 0.55 and 0.80 respectively. The rest and dwell give the last 0.20.

## Run

Install from this task folder in the Isaac Lab–Arena Python 3.12 environment. Use the pinned YAM adapter declared by `pyproject.toml`. Each trial is limited to 580 actions at 20 Hz, or 29 seconds. The documented seed is 1415158438. For native physics use CPU; the renderer uses a supported local GPU.

```sh
python -m pip install -e . --no-deps
python run.py --device cuda:0 --physics-device cpu --output /tmp/fountain-ink-demo
python run.py --device cpu --physics-device cpu --no-cameras --policy hold --output /tmp/fountain-ink-hold
python run.py --device cpu --physics-device cpu --no-cameras --policy short --output /tmp/fountain-ink-short
```

At the documented seed, the full reference earned 1.00 with six strokes and 0.951 mL. Holding the initial pose earned 0.00. Returning the pen after three submerged strokes earned 0.25 with 0.476 mL.

The Inspect Robots embodiment is `fountain_ink`. Its standard model observations are RGB and joint state. The reference controller sends ordinary robot joint commands using taught poses, measured robot joint angles, and the pen and gripper poses for contact alignment. The cup, bottle, pen, grippers and converter plunger have native contact and dynamics. A conservative stroke-to-volume law transfers ink from the bottle only when the native spring-loaded plunger completes a cycle with the feed submerged and the nib safely above the floor. The model omits slosh, meniscus and individual air bubbles. The converter window exposes the resulting volume visually.

Task geometry is original and MIT-licensed. [Pinned YAM asset terms](https://github.com/kairosiann/yam-isaac-lab-arena/tree/0059d02a1a91bc6834c1d0846022891df0c78730)

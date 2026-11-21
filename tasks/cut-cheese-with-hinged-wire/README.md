# Cut and serve an even slice with a hinged cheese wire

Make a single 8 mm serving from a block of firm cheddar using a manual wire board. One hand squares the loose block to the thickness mark and holds it outside the path of the wire. The other drives the handle through the cut and raises the wire. Lift the serving away from the intact block and set it on a side plate. Either hand assignment is valid.

The tool uses the tabletop hinged wire geometry of a [Prodyne 805B cheese slicer](https://www.prodyne.com/product/thick-beechwood-cheese-slicer/), with a steel pivot and a separate tensioned wire that descends into the board slot as shown in the [manufacturer's installation diagram](https://www.prodyne.com/replacement-parts/).

## Setup

Use a clean, dry kitchen bench with a 241 × 151 × 22 mm beechwood cheese board with non-slip rubber feet. Use a hinged single-wire slicer with a 180–185 mm span and a handle beyond the side of the board. Open the wire to 52° and check that the hinge stays there when released. Apply a removable 8 mm thickness mark beside the cutting slot, with the mark on the serving side. Use a fresh, firm cheddar block approximately 72 × 54 × 31 mm, starting flat 25 mm behind the desired cut position and 24 mm to the left of the slot center. Use clean dry food-contact pads. Set an empty 134 mm plate on the left. Keep the stock hand on the part of the block that will remain on the board and withdrawn from the moving wire.

Robot bases are 610 mm apart. X points forward from the midpoint of the bases, Y points to the robot's left, and the table top is 747.5 mm high. The board surface is at Z 771.5 mm. The hinge is at X 425, Y 76 mm, with the wire slot directed toward the right. In the fixed reset, center the intact block at X 372, Y 24 mm. Center the side plate at X 345, Y 208 mm. The body of the board stays in place during the task.

Reset with the same dry board and a fresh intact block. Wipe away scraps from the wire slot, empty the plate and leave the handle raised. Start both hands open and clear.

## Physical score

| Score | Outcome |
| --- | --- |
| 0.00 | The block has yet to be aligned. |
| 0.25 | The block is flat and square and the middle of its serving face is within 4 mm of the 8 mm gauge mark. |
| 0.65 | The wire has severed one continuous serving from the block. |
| 1.00 | One intact 6–10 mm serving passes a 25 mm lift with the rest of the block remaining on the board, then rests entirely on the plate. The wire is raised at least 32° and both hands are clear for 0.5 s. |

Verify the lift and the raised wire on video. Measure the serving near both long edges with a kitchen ruler. A piece which is still attached, left on the board, broken, or lost cannot receive full credit.

## Reproduce the native run

Use the Isaac Lab–Arena and Inspect Robots environment. The Inspect Robots embodiment name is `wire_cheese` and its scripted reference uses ordinary bimanual YAM joint targets at 20 Hz and CPU physics at 240 Hz. Use seed 1415158438, fixed geometry and the 660-action limit.

```bash
python -m pip install -e . --no-deps
python run.py --device cuda:0 --physics-device cpu --output /tmp/wire-cheese-reference
python run.py --no-cameras --policy hold --output /tmp/wire-cheese-idle
python run.py --no-cameras --policy short_stroke --output /tmp/wire-cheese-short
```

The two physical pieces are native rigid bodies from reset. A finite material bond makes them behave as an intact block. Real handle and board contacts drive the native wire hinge. A seven-section material model applies cutting resistance to that hinge from the moving wire's intersection with the upright cheese cross-section, and releases the bond only after the wire does work across every section. The wire's remeshing contact with cheese is approximated by that load. The model resolves a single serving plane. The agent still has to align the block to that plane, make the full loaded stroke, and physically lift and plate the piece.

The camera-recorded reference scored 1.0 in 494 actions (24.7 seconds). The wire reached the base of the cut across all seven sections, and the robot lifted and plated the single serving. A hold-only trial scored 0.0 over all 660 actions. Stopping the wire before it reached the slot scored 0.25 after alignment and partial penetration, with the serving still attached.

The authored scene geometry is MIT-licensed. The YAM and its three-camera station come from the pinned adapter in `pyproject.toml`. The adapter links to the I2RT robot geometry and its MIT notice.

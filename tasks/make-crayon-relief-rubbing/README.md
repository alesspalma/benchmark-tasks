# Make a crayon rubbing of a raised fern

Make a clear rubbing of the raised fern with the broad side of the blue wax block. Steady the loose paper while rubbing. Leave the paper flat and put the wax block down beside it.

A sheet over a raised rubbing plate takes up more wax at the raised texture. A hand stabilizes the paper so the drag from the broad face of the crayon does not change its registration. The other hand moves the wax in overlapping strokes. The scene uses a standard classroom wax block and a supplied printable fern rubbing plate. [Classroom rubbing method](https://www.crayola.com/learning/lesson-plans/texture-collage) · [Commercial leaf rubbing plates](https://roylco.com/shop/r5815-leaf-rubbing-plates/)

## Reset the physical scene

Coordinates in millimetres are relative to the midpoint of the line joining the two arm bases. Forward is toward the work surface. Left is positive across the work surface. Height is measured from the floor.

| Equipment | Size, material and initial position |
|---|---|
| Bimanual station | Two stock YAM arms, 610 mm apart, mounted at 750 mm above the floor. Both start at their standard ready joints with open grippers. |
| Table | Light, matte laminate worktop 1080 × 1250 × 42 mm, upper face at 747.5 mm. The surface center is 300 mm forward. |
| Fern rubbing plate | Print the included geometry flat in PLA with 0.20 mm layers. The base is 126 mm forward × 200 mm wide × 3 mm thick, and the fern stands 1.2 mm proud of that base. Place two 118 × 19 × 1.2 mm silicone strips flat along the outer margins, centered 88.5 mm to either side of the centerline. These support the hand pressing the paper and sit at the same height as the fern. The underside sits over a 1.5 mm silicone sheet of the same size. Put the center 327 mm forward at the midpoint between the arms and point the stem tip forward. |
| Paper | An uncreased fresh white A5 sheet of 80 g/m² plain copier paper, flat over the relief, centered on the plate, with its 148 mm dimension pointing forward and 210 mm dimension across. The paper is loose. |
| Crayon | One unwrapped blue wax block, 41 × 23 × 12 mm, broad face down. Place its 41 mm edge forward, on a 61 × 40 × 2 mm smooth rubber rest centered at (370 forward, 174 right). A STOCKMAR ultramarine wax block is the reference consumable. |

Use millimetre ruler marks on the table to reproduce the reset within 2 mm in position and 3° in angle. The silicone under the plate keeps it stationary, and the rubber rest keeps the wax block from slipping before pickup. Replace the used paper after every trial. Wipe the clean rubber pads after a wax transfer and use standard replaceable gripper pads. There is no heating, cutting, liquid, or exposed sharp edge. [Wax-block manufacturer's dimensions](https://www.stockmar.de/en/service/publications/?file=files%2FContents%2FDownloads%2FKataloge%2FEN%2FSTOCKMAR_catalogue_product-worlds.pdf)

Generate the exact inexpensive plate model and reference silhouette used here.

```sh
python -m crayon_rubbing.plate fern.scad --overlay fern-overlay.png
openscad -o fern.stl fern.scad
```

## Score the resulting sheet

Put the finished sheet in a square overhead photo at 4 pixels/mm or higher. Align its four corners to the A5 silhouette template, with the top of the fern facing forward as at reset. Use an unmarked piece of the same paper and a solid swatch made with the same block as the blank and the fully blue reference. Blue contrast is the fraction of their luminance difference. For each 2 × 2 mm square use its center or the mean of a small patch centered there, then count it as blue at contrast of 0.34 or greater. A transparent printout of the template with a 2 mm grid also permits manual counting. Divide the number of blue squares inside the fern by the number of all fern squares to get coverage. Divide the number of blue squares outside by the number of all fern squares to get extra blue area. A second patch or a solid blue sheet cannot count as the fern alone.

- **1.0** requires at least 86% coverage and extra blue area of 15% or less. The paper is flat and face up, its center within 16 mm and its angle within 8° of its starting alignment. The block is set flat on the rubber rest or adjacent table and is fully outside the paper. Hands are at least 25 mm from the paper or above it by at least 27 mm for the last 0.45 s. Use the trial video and a ruler for these checks.
- **0.75** requires at least 70% coverage and at most 15% extra blue area.
- **0.50** requires at least 47% coverage and at most 30% extra blue area.
- **0.25** requires at least 23% coverage and at most 30% extra blue area.
- **0.0** applies to lower coverage, extra blue area greater than 30%, or a displaced or overturned sheet. The sheet placement condition applies to all tiers.

Both hands have meaningful jobs in the reference trial. The scorer accepts the same physical result with either arm holding the paper, any order of passes, and any trajectory that leaves the reproduction on the sheet.

## Run and inspect

Use the pinned Isaac Lab Arena/Inspect Robots environment and the accepted simulator license. In this folder:

```sh
python -m pip install -e .
python run.py --device cuda:0 --physics-device cpu --output /tmp/crayon-demo
python run.py --no-cameras --policy hold --output /tmp/crayon-idle
python run.py --no-cameras --policy air-pass --output /tmp/crayon-air
python run.py --no-cameras --policy no-pin --output /tmp/crayon-no-pin
```

The fixed seed is 1415158438. A trial allows 620 actions at 20 Hz. Each native camera is recorded at 640 × 360 for the demo before composition. Every run writes its score and physical contact diagnostics. The Inspect Robots embodiment is `crayon_rubbing` and the scripted policy is `crayon_rubbing_scripted`. The reference recorder uses joint positions, relative object height and the crayon's measured normal force to guide the block over the sheet. The model receives the usual three cameras, joint positions and instruction.

The recorded native reference finished in 543 actions with 98.0% fern coverage and no strongly colored cells outside it. The idle and air-pass controls deposited no wax. The no-pin control covered 24.0% of the fern and left the paper 29.2 mm out of register, scoring zero. The recorded reference measured 502 mm of sliding contact, with peak wax load 2.81 N and peak load on the paper from the bracing gripper 1.58 N.

The paper and wax are freely moving native collision bodies. The printed relief is solid geometry under the paper. The thin sheet has a planar mechanics approximation. At each physics tick, the wax transfer reads the filtered crayon–paper contact points and normal contact force. It measures sliding of the block relative to the sheet and deposits wax only at squares underneath the actual broad-face footprint. The same force deposits less wax where there is a recess under the paper, approximating the pressure concentration at the relief peaks. Repeated strokes over background can still darken it. The renderer and scorer read the same accumulated wax distribution. The simulator does not model progressive crayon wear or paper tearing.

The furniture, plate, crayon geometry and scene artwork in this folder are original and covered by its MIT license. The YAM robots and camera model are supplied by their pinned dependencies. [YAM hardware-asset terms](https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/assets/yam/LICENSE.i2rt) · [Camera asset source](https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc126418ea5e74ac567133d/LICENSE)

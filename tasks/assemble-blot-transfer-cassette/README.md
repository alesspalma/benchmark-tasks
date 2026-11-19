# Clear and close a wet-laminate cassette

A hand roller removes air gaps from the wet contact area of a laboratory cassette. One arm braces the loose layers at an exposed corner while the other rolls across them under pressure. The arms then close the cassette without shifting the layers. The 75 × 100 mm cassette reproduces that handling operation with inert layers. [Manufacturer's dimensions](https://www.bio-rad.com/sites/default/files/webroot/web/pdf/lsr/literature/Bulletin_2033C.pdf) · [Manufacturer's cassette geometry and roller use](https://www.bio-rad.com/webroot/web/pdf/lsr/literature/10019593D.pdf)

## Setup and reset

All positions below are in millimetres in the YAM station frame, with +x forward, +y to the left, and the worktop at z = 747.5. The standard arms start open and clear of the props, with bases 610 mm apart. The overview faces the loose brayer, cassette and lid. Each wrist has the standard YAM close view.

| Item | Construction and initial placement |
| --- | --- |
| Cassette tray and riser | Print the 139 × 106 × 9 ABS tray and the 237 × 163 × 78.5 riser. The tray's bottom centre is at (365, 25, 826). It sits horizontally. The guide rails run along x to support the open cover. |
| Backing and working layer | Fit a 106 × 79 × 2 backing pad in the tray. Use a 102 × 84 × 4 silicone lower layer with an ordinary exposed margin and 100 × 75 upper contact area of water-wetted clear PET. Lay it on the backing pad with the long edge along x and the edges registered. The simulated compliant layer starts with its upper face near z = 843. |
| Three visible pockets | Use a fine dropper to leave the three dry spots in the water film under the clear sheet before laying it down. Centre them at (336, 3), (377, 23) and (393, 46) in plan view, with approximate diameters of 14, 16 and 14 mm. Confirm that they are distinct in a reset photo and measure the registration border with a ruler. |
| Guided cover | Use a 121 × 92 PET lid with a rigid ABS frame and a 19 × 70 × 19 edge grip. Draw it along x through 127 mm of travel to the tray end. Reset it at the full-open stop with its centre at (492, 25, 856). The geometry in `builder.py` dimensions the rails and frame components. |
| Brayer | Stage the rubber drum with its axle parallel to x at (365, −116, 857). The drum is 106 mm wide and 28 mm in diameter, with a free axle in a cross yoke. The polymer grip is 20 × 23 × 44, with its centre 81 mm above the axle. |
| Open brayer saddle | Make two 9 × 34 ABS supports with top faces at z = 876.5, equally spaced 69 mm either side of the drum centre along x. The seats support only the outboard bridge. Lift clearance is open above the tool and shoulders. |

Discard the reset photo after the measurements are recorded. Rewet the test area and restore the three pockets, the open cover and the brayer between runs. All test material is inert. A real trial uses the same visible contact area and closure check. The reference validation uses simulation.

## Scoring

Record a photo of the clear area and registered borders after rolling and before covering. Award 0.25 for each of the three marked pockets visibly absent, up to 0.75. Award 1.0 when all three are gone, the sheet edges finish within 3 mm of their registration marks, and the lid has stayed within 4 mm of its closed stop for 0.5 s. Score 0 if no pocket is cleared. Either arm can brace, roll or close.

In simulation, a pocket clears when the rotating drum traverses at least 4 mm of its face under load and the layers stay within 4 mm of registration. The test uses native axle rotation, near-surface geometry and spring travel. Clearance is marked by the disappearance of the visible pale pocket. Spring-supported shear and compression represent the wet compliant stack. The tester can judge the physical result from the photo and ordinary rulers, without measuring actuator forces.

## Run and score

Use the repository's Isaac Lab Arena/YAM environment. The scene uses a fixed layout, seed 1415158438, 20 Hz controls and a 540-action (27 second) limit. From this directory:

```sh
python -B run.py --device cuda:0 --physics-device cpu --output /tmp/cassette-demo
python -B run.py --device cuda:0 --physics-device cpu --no-cameras --policy hold --output /tmp/cassette-hold
python -B run.py --device cuda:0 --physics-device cpu --no-cameras --policy hover --output /tmp/cassette-hover
```

Each run writes its reset snapshot, native trace and score receipt. The camera run records the initial image and each subsequent frame in three 640 × 360 streams. The no-action control spans all 540 actions. The hover control picks up the brayer and closes the lid but leaves the roller above the sheet during the sweep.

For Inspect Robots, install this directory with `python -m pip install --no-deps -e .`, then pass `blot_cassette` as the embodiment and `dream` as the policy. The hosted task's seed, instruction and action limit are the same as in `task.json`.

## Assets and terms

| Asset | Provenance and licence |
| --- | --- |
| Tray and stand | Original procedural geometry, task MIT licence |
| Backing and working sheet | Original procedural geometry, task MIT licence |
| Pocket witness graphics | Original procedural geometry, task MIT licence |
| Guided lid | Original procedural geometry, task MIT licence |
| Brayer | Original procedural geometry, task MIT licence |
| Brayer saddle | Original procedural geometry, task MIT licence |
| YAM arms and grippers | [Split I2RT geometry](https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/assets/yam/dream/SOURCE), [I2RT asset terms](https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/assets/yam/LICENSE.i2rt) |
| Bench and station | [Adapter-generated geometry](https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/scene.py), no separate asset licence declared |
| Overview camera | [MIT YAM optics calibration](https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc418ea5e74ac567133d/LICENSE) with original pose |
| Left wrist camera | [MIT YAM left wrist calibration](https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc418ea5e74ac567133d/LICENSE) |
| Right wrist camera | [MIT YAM right wrist calibration](https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc418ea5e74ac567133d/LICENSE) |

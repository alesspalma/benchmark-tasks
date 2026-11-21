# Zero a handheld optical refractometer with a water blank

Zero a 0–32 °Brix optical refractometer before measuring a new batch. A water blank places the blue-and-white boundary at zero. One hand supports the instrument horizontally while the other brings the dropper to the exposed prism, closes the daylight plate to spread the film, and uses a slotted driver to turn the calibration screw. The body can turn under driver torque without a counterhold. Handling the loose instrument, depositing a small blank on its exposed optical surface, and engaging the adjustment slot are separate physical steps. The same water-blank procedure is used to prepare handheld refractometers for analytical food and soluble-solids research. [Cole-Parmer operating instructions](https://pim-resources.coleparmer.com/instruction-manual/81150-23-to-47-manual.pdf) · [ATAGO MASTER series](https://www.atago.net/english/images/catalog/master_en.pdf)

## Equipment and setup

Use a handheld analog model with a swinging daylight plate, a top-side slotted zero adjustment, and a 0.2 °Brix graduation. Cole-Parmer 81150-36 or ATAGO MASTER-M is suitable. The analog instrument, dropper, driver, cleaning tissues, and ordinary molded tool rests can be replicated from commercial parts. The procedural scene source builds the bench and every task fixture.

The bench top is 960 × 1170 mm and 747.5 mm above the floor. Robot bases are at x = 0, y = ±305 mm, z = 750 mm. Positive x points away from the arm bases and positive y is the left arm side. Positions are specified in millimetres. Both robots start in the standard ready pose.

| Item | Hardware and fixed reset |
|---|---|
| Refractometer | Approximately 166 × 34 × 36 mm overall, 141 g, with aluminum optical tube, rubber overmold, glass prism, acrylic daylight cover and steel calibration screw. Place it in the open rubber rest with two 18 mm support pairs spaced 127 mm along the axis at (366, 27), with the eyepiece toward positive y and the prism facing upward. Open the daylight plate fully. Set the case scale near +1.2 °Brix for a water blank and wipe the prism dry. Leave the adjustment port exposed. |
| Distilled-water dropper | Standard 3 mL polyethylene transfer dropper with a squeeze bulb and a narrow outlet. Use distilled water at 20 ± 2 °C. Load four 36 µL drops (approximately 0.144 mL, checked by a 0.01 g scale). Stand it nozzle-down in the open holder at (348, −160). |
| Slotted driver | Approximately 85 mm instrument screwdriver with a 4 mm slotted blade, 22 mm handle and steel shank. Stand shaft-down in its holder at (457, −173). Start the blade turn 22° from positive x, viewed from above. A mark on the holder can set this orientation. |
| Workspace | Bench, one low instrument rest and two open tool holders. White task lighting is sufficient for a sharp eyepiece boundary. |

Set the initial bias using a wet prism and the real screw, open the cover, wipe the glass with optical tissue, and refill the dropper for each reset. A second clean dropper, two ordinary tool holders and lens tissue are the only consumable-support equipment. The dropper and screwdriver remain separate from the instrument at reset. The fixed reset seed is 1415158438.

## Scoring

Inspect the wet glass through the clear lid and read the blue–white boundary through the eyepiece facing diffuse light. A 0.01 g scale can check the blank by mass, approximately 0.072–0.144 g of water. An observer can also count the drops reaching the prism and check neighboring surfaces for stray water. Use a ruler to check that the daylight plate seats continuously against the instrument rim, then leave the instrument on its rest and maintain the result without either hand touching it for half a second. Report the score for the highest achieved tier in the final state:

- **0.25** — At least two drops (72 µL) of distilled water reached and remain on the prism, with at most one drop spilled.
- **0.45** — The water remains, the daylight plate is closed within 7° and the prism is horizontal within 13°. The illuminated eyepiece has a readable separation.
- **0.65** — All wet, closed conditions hold and the boundary is within one 0.2 °Brix division of zero. The instrument may still be held.
- **1.0** — All calibrated conditions remain as the instrument rests in the open tray and the hands are clear for 0.5 seconds.
- **0.0** otherwise.

The same scoring applies for either hand assignment, with no progress from turning a dry screw. The score describes water zeroing. The assessor makes the optical reading before opening and wiping the prism as part of reset.

## Simulation and reproduction

The plate rotates on a native hinge. The metal shaft turns only when torque on its physical slot overcomes friction. A real gripper squeeze moves a compliant insert in the water dropper. Each increment of measured bulb compression sends a fixed water drop down to the geometrically intersected surface. The liquid representation conserves the prepared volume and approximates spread while the lid is seated. The eyepiece boundary is an optical approximation calculated from water/plate state and the actual screw angle. Its magnified zero-range reticle is drawn directly in the simulated eyecup.

Use the accepted-license Isaac Lab–Arena runtime and the package pins. The maximum trial is 660 actions or 33 seconds at 20 Hz and 240 Hz physics. The scripted native reference takes only normal arm actions. It measures simulated geometry to align the taught grip, dropper nozzle, and driver tip. Dream-1 uses the standard three camera streams and robot joint readings.

```bash
python -m pip install -e .
python run.py --device cuda:0 --physics-device cpu --output /tmp/refractometer-demo
python run.py --no-cameras --policy hold --output /tmp/refractometer-idle
python run.py --no-cameras --policy dry --max-steps 516 --output /tmp/refractometer-dry
python run.py --no-cameras --policy unadjusted --output /tmp/refractometer-unadjusted
```

The script writes a receipt and step trace. Camera mode records the initial observation and every subsequent native frame. All task-specific geometry is procedural and MIT-licensed. [YAM robot asset terms](https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/assets/yam/LICENSE.i2rt) · [Camera asset terms](https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc126418ea5e74ac567133d/LICENSE)

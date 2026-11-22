# Verify an installed robot suction cup holds vacuum

A robot suction tool can show a healthy reading with its source running while a leak still limits how long it can carry a part. Test the installed bellows cup on a smooth aluminum test plate, close the supply, and verify that the trapped vacuum holds the plate without support. Regrasp the plate before venting. This prepares the end effector for a pick-and-place test with valuable or fragile workpieces. [Piab cobot tool mounting and leakage monitoring](https://www.piab.com/globalassets/productimages/0212274_rev07_picobot_ur_en.pdf) · [Schmalz contact and cup-leak troubleshooting](https://media.schmalz.com/MAM_Library/Dokumente/Bedienungsanleitung/30/3030/303001/30300102238/0d3f702b1216_BAL_30.30.01.02238_en-EN.pdf) · [Leybold isolation and pressure-rise method](https://www.leybold.com/en-uk/knowledge/vacuum-fundamentals/leak-detection/pressure-rise-and-drop-tests)

## Physical station

Coordinates in the table are millimetres in the common YAM base frame. X runs forward, Y runs towards the robot's left, and Z runs up. The arms rest 610 mm apart on bases whose datum Z is 750 mm. The left arm inserts and later supports the plate. The right arm works the isolation spool and the downstream vent. Either arm can perform either role.

| Item | Construction and placement |
| --- | --- |
| Worktop and foot | A flat 1180 × 1160 mm bench at Z 747.5 mm. Bolt a 258 × 390 × 25 mm steel fixture foot to the bench with its center at X 525, Y −68. |
| Mounted head and compliant cup | An aluminum stand holds a stationary 78 × 70 × 45 mm cobot tool body with its center at (405, 84, 1022). Mount any equivalent 52 mm elastomer bellows cup with its approximately 36 mm effective bore facing straight down. The unloaded lip plane is centered at (405, 84, 920). Keep the flexible cup and aluminum stand permanently assembled throughout the trial. |
| Pressure circuit | Connect a regulated vacuum source capable of 60 kPa below atmosphere to a two-position push-pull isolation valve. The valve outlet, gauge, cup, and normally closed atmospheric vent share the downstream circuit. The blue supply tab starts at (383, −182, 867) and opens along +X with 28 mm travel. The orange vent starts at (391, −284, 850), returns by spring, and opens on a 5–10 mm downward press. Install a 104 mm diameter gauge centered at (495, −166, 1001) with its face toward the robot. Use clean tubing of approximately 6–10 mm outside diameter. The gauge displays the vacuum magnitude in kPa, with zero at atmospheric pressure. |
| Test plate | Use a smooth, nonporous 107 × 91 × 7 mm aluminum plate of approximately 0.17–0.19 kg. Fix the 20 × 13.5 × 24 mm blue handle to a 40 × 25 × 9 mm standoff centered at X −60 and Y 0 relative to the plate center. The handle center sits at X −73, Y 0 and Z +15.5 in the same frame. The plate has no through-holes in the sealing face. Keep the full plate width under the lip during the test. |
| Plate rest | Two padded 22 × 103 × 16 mm rails lie across the bench, centered at X 301 and 367, both at Y 217. Start the plate flat across them with its center at (334, 217, 767). |

Clean the cup and plate. Bring the gauge to zero by pressing the vent with the supply isolated, then release the vent. Keep the head attached and turn off arm motion in its host machine during the test. Use the light plate and open bench beneath the cup as the test area. A camera looking side-on to the dial can record both plate clearance and the vacuum value. An ordinary stopwatch measures the two-second hold.

## Scores

- **0.00** until a sealed pickup is demonstrated at 40 kPa vacuum or above.
- **0.35** when the plate is sealed 110 mm or more above its initial center height at 40 kPa or above with supply running. The seal requires contact with the cup rim, bellows compression, and full coverage of the sealing face.
- **0.75** when both hands are clear and the isolated cup supports the plate for two continuous seconds at 40 kPa or above. A loss of vacuum or contact interrupts the interval.
- **1.00** after the preceding proof, a supported vent, and stable parking for at least half a second. Grasp or support the plate from below before opening the vent and maintain support through the release. The final gauge is below 3 kPa and the supply stays isolated. The plate is flat on both rest rails within 16 mm along X and 25 mm along Y of its start. Venting with the elevated plate unsupported keeps the score at most 0.75.

## Run and inspect

Use Python 3.12 and the standard Isaac Lab–Arena Isaac Sim environment. Install the package in this folder. The fixed scene uses no object jitter. The control rate is 20 Hz, native physics run at 240 Hz, and the 780-action limit corresponds to 39 seconds. The reference seed is `1415158438`.

```bash
pip install -e .
OMNI_KIT_ACCEPT_EULA=YES python run.py --device cuda:0 --physics-device cpu --output /tmp/vacuum-demo
OMNI_KIT_ACCEPT_EULA=YES python run.py --no-cameras --policy hold --output /tmp/vacuum-idle
OMNI_KIT_ACCEPT_EULA=YES python run.py --no-cameras --policy gap --output /tmp/vacuum-open-gap
OMNI_KIT_ACCEPT_EULA=YES python run.py --no-cameras --policy no_seal --output /tmp/vacuum-open-cup
OMNI_KIT_ACCEPT_EULA=YES python run.py --no-cameras --policy no_isolation --output /tmp/vacuum-supply-on
OMNI_KIT_ACCEPT_EULA=YES python run.py --no-cameras --policy unsupported_vent --output /tmp/vacuum-unsupported-vent
python -m unittest discover -s tests -v
```

The receipt records the score, isolated pressure, supported venting and return to the rest. The trace includes fixture joints, plate and robot poses, and the measured hand-to-plate and rim forces. The open-gap policy presents the plate below the lip with the supply running and scores 0.00. The unsupported-vent policy completes the pressure proof and then vents with the plate hand withdrawn, scoring 0.75. The idle policy scores 0.00.

The cup has twelve native rim pads, an axial spring and two limited angular joints for bellows compliance. The simulated seal requires at least 0.6 mm of axial compression, a native rim contact carrying at least 0.75 N, and a covered rim with at most 0.25 mm of local clearance to represent elastomer compliance. The air circuit models pressure and leakage and applies a pressure–area force to the plate. A partially peeled rim starts leaking immediately, with transient pneumatic load fading to zero by 0.5 mm of clearance. The pressure proof and vent check run at the native physics rate. A supported vent requires either opposed finger loads of at least 2 N per finger or a gripper contact load upward of at least half the plate's weight.

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
- **0.35** when the plate is sealed 110 mm or more above its initial center height at 40 kPa or above with supply running.
- **0.75** when both hands are clear and the isolated cup supports the plate for two continuous seconds at 40 kPa or above. A loss of vacuum or contact interrupts the interval.
- **1.00** after the preceding proof, a supported vent, and stable parking for at least half a second. The final gauge is below 3 kPa and the supply stays isolated. The plate is flat on both rest rails within 16 mm along X and 25 mm along Y of its start. Venting with the elevated plate unsupported keeps the score at most 0.75.

## Run and inspect

Use Python 3.12 and the standard Isaac Lab–Arena Isaac Sim environment. Install the package in this folder. The fixed scene uses no object jitter. The control rate is 20 Hz, native physics run at 240 Hz, and the 780-action limit corresponds to 39 seconds. The reference seed is `1415158438`.

```bash
pip install -e .
OMNI_KIT_ACCEPT_EULA=YES python run.py --device cuda:0 --physics-device cpu --output /tmp/vacuum-demo
OMNI_KIT_ACCEPT_EULA=YES python run.py --no-cameras --policy hold --output /tmp/vacuum-idle
OMNI_KIT_ACCEPT_EULA=YES python run.py --no-cameras --policy no_seal --output /tmp/vacuum-open-cup
OMNI_KIT_ACCEPT_EULA=YES python run.py --no-cameras --policy no_isolation --output /tmp/vacuum-supply-on
```

The receipt gives the score and evidence for isolated pressure, supported venting and return to the rest. The trace includes all measured fixture joints and plate/robot poses. The cup lip has twelve native rigid contact pads on an axially compliant joint. The pneumatic model checks real coverage and clearance at the rim, estimates flow with first-order response in the sealed, leaking and vented cases, and applies pressure over the bore to the native plate body. Lateral holding and plate orientation follow the contact forces. The cup skirt expresses the bellows travel, and the gauge needle tracks pressure from the circuit model.

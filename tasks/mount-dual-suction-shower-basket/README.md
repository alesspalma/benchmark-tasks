# Mount a dual-suction shower basket and load it

Mount a full-size bathroom basket to smooth glazed tile with its two cam-lock cups, then set a capped shampoo bottle inside and withdraw both grippers. The operation follows the mounting sequence for the [OXO StrongHold Suction Large Basket](https://www.oxo.com/strongholdtm-suction-large-basket.html). OXO Customer Service describes lifting the two levers, placing the basket against the surface and closing both in its [product installation answer](https://www.target.com/p/-/A-88944366).

## Physical setup

Use the commercial basket or an equivalent 27.5 × 11.2 cm drained bath basket with two separate lever-activated cups. Use a 73 × 42 cm minimum tile panel, bolted to a braced fixture, and ordinary smooth glazed ceramic tile about 30 × 27 cm. The cup and lever dimensions are defined in `suction_basket/config.py` and the original scene geometry in `suction_basket/builder.py`. Two pads roughly 52 mm in diameter, on roughly 184 mm centers, fit across two adjacent tiles with the grout joint between them. Use a refillable shampoo bottle, 4 × 5 × 11 cm and 220 g capped. Fixed ballast in a sealed bottle makes trials repeatable. Mount and release ten times by hand to check the tiles, levers and pads operate freely before running the robot. Choose a wash-off marker if adding tape marks on the counter.

Coordinates below are relative to the midpoint between the two YAM base links. The X axis points away from the robots, Y points to the left robot, and heights are above the counter. Put both grippers at the standard raised YAM home pose.

| Item | Reset |
| --- | --- |
| Tiled face | X = 54.5 cm, vertical; a grout joint centered at Y = 0 cm |
| Horizontal grout joints | Approximately 4 and 31 cm above the counter |
| Basket floor center | X = 29.3 cm, Y = 0 cm, lying upright on the counter; both levers fully open and cups facing the tile |
| Blue bottle base | X = 31.1 cm, Y = −24 cm, upright on the counter |
| Mounting band | Basket base 6–13 cm above counter; both cup footprints wholly inside tile faces |

Use clean dry glassy tile, original suction pads and a horizontal counter. Open both levers and remove the basket after each trial, then wipe the pads and tile and replace the objects. A front three-quarter camera and a ruler beside the wall show clearance above the counter and absence of sliding. The overview and wrist cameras show whether both cam levers are closed and whether each pad crosses a grout line.

## Running and scoring

Use Python 3.12 and the pinned Isaac Lab Arena/YAM environment. Install the local Inspect package.

```bash
pip install -e .
python -B run.py --device cuda:0 --physics-device cpu --output artifacts/basket-demo
python -B run.py --device cpu --physics-device cpu --no-cameras --policy hold --output artifacts/basket-idle
python -B run.py --device cpu --physics-device cpu --no-cameras --policy one-cup --output artifacts/basket-one-open
```

The default fixed reset uses seed `1415158438`. The limit is 520 steps, 26 seconds at 20 Hz. The physics step is 240 Hz. Each trial writes a receipt and a trace with measured cup and basket state. Camera runs record three 640 × 360 native views at 20 fps and compose a 1280 × 1080 H.264 MP4. The engine reads the two lever angles and lip compressions to model suction forces, including peel and shear failure. The motion reference uses fixed waypoints, elapsed time and normal robot joint observations. The policy under evaluation receives standard YAM camera and joint observations.

A raised and level basket scores 0.25 for each fully seated and locked cup. With both cups attached, a bottle resting upright in the basket brings the score to 0.75. Full score requires 0.7 seconds of stable, unassisted retention after both grippers are out of the basket area. The reset has both cups open and the bottle on the counter.

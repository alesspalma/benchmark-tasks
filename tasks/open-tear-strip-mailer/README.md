# Open a corrugated mailer by pulling its tear strip

Opening a book-mailer seal calls for a small initial pinch, sustained
off-axis tension along the tear line and a separate grasp of the freed
cover. The other hand can steady the packed carton through the tear. The
book stays in the tray while the cover is lifted.
[Mondi corrugated mailer catalogue][mailer]

## Physical setup

Use a single-wall kraft corrugated book mailer with an integrated
reinforced pull strip along the front flap and a rear crease hinge.
Mondi's EnvelopeMailer with its tear-tape option is an example. Select
an unopened, easy-open mailer of approximately 252 mm width, 165 mm
depth and 50 mm height. The reference geometry uses 2.2 mm walls,
a 232 × 10 mm tear seam and a 23 × 20 mm doubled-paper starter tab.
The seal consumes one fresh carton per trial. Pack the same paperback,
approximately 148 × 105 × 19 mm and 142 g, laid flat.

Place the flap toward the robots. In station coordinates X points
forward, Y left and Z upward. The tray datum is at X 337 mm, Y 10 mm
and Z 748 mm on a 747.5 mm high level bench. The visible starter tab
extends freely at the right-hand end of the front flap. Its center is
at X 239.5 mm, Y −136 mm and Z 776 mm before contact settles. The book
center is 9 mm toward the hinge and 5 mm to the left of the center of
the tray. Seat the tray on the bench and close the intact factory seal
with the tab accessible.

## Run

With the Robocurve Isaac Lab-Arena, Inspect Robots and YAM adapter
installed, run from this task directory.

```sh
python -m pip install -e .
python run.py --device cuda:0 --physics-device cpu --output runs/demo
python run.py --device cuda:0 --physics-device cpu --policy hold --no-cameras --output runs/hold
python run.py --device cuda:0 --physics-device cpu --policy tear-only --no-cameras --output runs/tear-only
```

The reset seed is 1415158438. A trial allows 600 actions at 20 Hz
(30 seconds), with PhysX at 240 Hz. The runner records the initial
frame, every action frame in three native 640 × 360 views, a trace and
the score receipt. Inspect Robots registers the embodiment as
`tear_mailer` and the joint-position reference as `tear_mailer_scripted`.
Policies receive the bimanual joint state and all three camera views.

The scripted demo scored 1.0 after 466 actions, with the complete strip
removed and the lid open 52.58°. The 600-action stationary control scored
0.0 and left the seal intact. The tear-only control removed the strip and
scored 0.6, with the freed lid resting at 11.48°.

## Scoring

- **1.0** for removing the entire 232 mm strip and keeping the cover
  at least 48° open for 0.4 seconds with the book inside the upright tray.
  A gripper may support the open cover.
- **0.6** for removing the entire strip with the book inside the
  upright tray when full credit has yet to be reached.
- **0.3** for tearing at least 116 mm of the seam, short of a
  complete removal, with the book inside the upright tray.
- **0.0** otherwise.

Use a ruler for the tear length and an angled or side recording for
the lid and tray attitudes. The tray must stay within 20° of upright
and its base within 25 mm of the bench. A side view distinguishes a
free cover from a partially torn flap.

## Fidelity

The native grasp and flap contact act on rigid bodies. The folded
paper leader is compliant and damped. Tension and pull direction
advance an estimated fracture front, and the crease relaxes modestly
once the tear reaches the far end. The detached ribbon is rendered
from its native leading piece. The 4 N propagation threshold is an
estimate in the force range of published tests of lighter perforated
paperboard. [Bota and Petković, Figure 4][test]

[mailer]: https://www.mondigroup.com/products-and-solutions/ecommerce/corrugated-mailers/
[test]: https://www.grid.uns.ac.rs/symposium/download/2020/32.pdf

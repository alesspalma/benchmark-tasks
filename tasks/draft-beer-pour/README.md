# Pour a draft beer with a proper foam head

Pour a pub glass of draft beer with about an inch of foam, shut off the faucet
and serve it on a coaster. A bartender first directs the flow down the tilted
glass wall, then brings the glass upright as it fills to form a foam head.
The task tests coordinating that change with a separate manual faucet. Beer
has to land in the glass throughout and the final drink has to be properly
filled. See the [Brewers Association pouring procedure][draught], pp. 55–56.

## Run

Install Isaac Lab-Arena, Inspect Robots and the YAM adapter using the task-board
walkthrough. In this directory, install the task and run the reference policy:

```bash
python -m pip install -e .
python run.py --output runs/demo --device cuda:0 --physics-device cpu
```

Run the fixed scene with seed `1415158438`. The budget is 800 actions at 20 Hz
(40 s); native rigid-body physics runs at 240 Hz on the CPU. Each of the three
RGB streams defaults to 640 × 360. The output directory must be new. It receives
reset and final states, an action and state trace, raw camera files, score and
source and generated asset SHA-256 hashes. Use `--no-cameras` for physics-only
runs and `--policy hold` or `--policy upright` for the corresponding controls.

The Inspect environment is `draft_beer` and the reference policy is
`draft_beer_scripted`. Policy observations are the three RGB images and the
14 YAM joint positions. The reference policy uses joint-position IK feedback
and elapsed time to execute the Cartesian sequence. Scoring and diagnostics
also read the native scene state.

## Fixed starting setup

Use an ordinary YAM station with a 747.5 mm table surface. X runs away from the
robot, Y runs to its left and Z runs upward. The coordinates below reference
the station frame. Heights are above the table, and distances are in mm. All
objects start upright. The faucet is closed, the glass is empty and neither
arm touches the equipment.

| Object | Material and dimensions | Initial placement |
|---|---|---|
| Single-column draft tower | Stainless steel. Simulated 62 mm OD × 265 mm column and 86 mm OD × 7 mm bolted foot | Column centre X 454, Y −40, foot bottom on table |
| Self-closing manual faucet | Stainless steel shank and body, spring-return valve, nominal 129 mm projection from column axis to spout. Spout 19 mm OD × 25 mm | Outlet X 325, Y −40, 249 mm above table, pointing vertically downward |
| Tap handle | Hardwood 26 mm OD × 83 mm handle over a stainless steel 10 mm stem. 26° fore-aft travel | Pivot X 357, Y −40, height 284 mm, handle upright when closed |
| Pub glass | Clear glass; 16 US fl oz nominal, 76 mm widest OD × 165 mm tall; 2.7 mm walls, 11 mm heel; simulation dry mass 278 g | Base X 460, Y −165, height 4 mm |
| Glass start mat | Black nonslip rubber, 100 mm OD × 4 mm | Centred under the glass |
| Drip tray | Stainless steel rectangular tray, 135 × 145 × 12 mm, 10 raised grate bars | Centre X 317, Y −40, tray bottom at table height |
| Serving coaster | Blue rubber, 94 mm OD × 5 mm | Centre X 400, Y −245 |
| Draft lager supply | Standard refrigerated beverage supply and balanced beer line behind the faucet | Operated as usual; only the outlet and external controls enter the robot workspace |

The glass follows [Libbey 14816HT][glass] capacity and exterior dimensions.
Its heel and wall thickness are engineering dimensions for the simulation.
The [Kegco stainless self-closing faucet][faucet] is a commercial spring-return
valve. The faucet and a [standard tower with compatible shank and line][tower]
were listed for about $106 combined in November 2026. Use a bar's or kegerator's
draft supply. A portable [5 L mini-keg system with a self-closing faucet][mini]
was listed for $100; install it as supplied and secure it on a shelf behind the
workspace to set the same spout position. Use pressure-rated beverage components
and a clean cold line. Bolt the tower to the counter or use the existing fixed
mount. Set the spout-to-counter clearance near 249 mm. A similar 16 oz clear
narrow pub glass may be used; verify its capacity and dimensions.

Set the line for roughly 60–65 mL/s at full opening, checking with a measuring
jug and stopwatch. Use the same ordinary refrigerated draft lager for all
trials, following the line's maintenance and pouring procedures. Return the
clean empty glass to its marked position, finish with the tap closed and empty
and dry the drip tray between trials. Each trial uses up to about one serving
of beer. For spills outside the drip tray, weigh a dry absorbent counter liner
before and after the trial.

## Scoring

Judge the final served glass from the moment the robot releases it. It must sit
upright (within 5°) with its base centre within 12 mm of the coaster centre and
base height within 4 mm of the coaster top. The faucet must be at the closed
stop (within 2° and no flow) and grippers must be at least 10 mm from the glass.
The glass must be stationary apart from at most 20 mm/s of settlement.

- **1.0** if that state, 330–425 mL of retained beer (liquid equivalent),
  16–34 mm of foam, 3–18 mm of rim clearance and at most 12 mL of spill are
  all achieved together for at least 0.6 seconds.
- **0.5** for a properly set down glass and closed faucet with at least 240 mL
  of retained beer and at most 25 mL of spill if the full condition is unmet.
- **0** otherwise.

Measure foam and distance below the rim on a side-on image with a ruler in the
glass plane. Measure retained beer by tared glass weight after scoring the foam,
or by settled liquid volume in a measuring jug. A tared drip tray plus the
mass gained by the counter liner measures spill. For a lager near 1 g/mL, the
gram measurement approximates the mL figures. The official score concerns the
condition of the served drink; standard and unconventional pours can both
succeed if the outcome is actually equivalent.

## Simulation

The glass and tap are native PhysX bodies. The glass has compound open tapered
walls, an independent heel and native gripper contacts. The tap lever has
passive physical travel limits, hinge friction and a spring that returns it
to closed when released. The nozzle opens according
to the actual lever angle. The fully open flow figure is 63.09 mL/s (one US
gallon per minute, [Brewers Association draught guideline][draught]).

The liquid model traces the nozzle stream against the glass's geometry, existing
liquid surface and gripper shapes. A stream hitting the inclined inside wall
makes less foam than a direct impact on the pool or bottom. A horizontal free
surface accounts for retained liquid, expanded foam and overflow in the actual
tilted geometry. Beer mass and centre of mass contribute to the native load.
The explicit model represents the pour method, with engineering foam coefficients
and decay. It omits splash droplets, turbulent slosh and glass breakage. It
requires ordinary robot actions to open the tap, orient the glass and serve it.

The task's generated USD geometry is built from primitives in `draft_beer/builder.py`.
The YAM and base station assets come from the provided adapter. No product artwork
is embedded.

## Reference run

The native scripted demonstration scored 1.0 in 428 actions (21.4 seconds).
It served 398 mL beer with 21.7 mm foam, 4.2 mm rim clearance and 6.7 mL spill.
The robot closed the faucet, released the upright glass on the coaster and cleared
both arms. The recorded native views use 640 × 360 at 20 fps.

[draught]: https://cdn.brewersassociation.org/wp-content/uploads/2019/03/13094643/Draught-Beer-Quality-Manual-2019.pdf
[faucet]: https://kegco.com/products/stainless-steel-self-closing-beer-faucet-with-stainless-steel-lever
[glass]: https://libbeyfoodservice.com/product/libbey/pub-straight-sided-pub-glass/14816ht
[tower]: https://kegco.com/collections/draft-beer-supplies/products/single-faucet-stainless-steel-draft-beer-tower-stainless-steel-contact
[mini]: https://www.vevor.com/category-c_10000/product-p_010987730575

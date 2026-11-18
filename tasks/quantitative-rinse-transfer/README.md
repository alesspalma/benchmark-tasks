# Recover retained beaker sample with a wash-bottle rinse

Analytical sample preparation often requires quantitative transfer. After the
bulk sample has been poured, a thin residue is still on the inside of the
source vessel. An operator rinses the inside with a directed jet from a
squeezable wash bottle, then drains the washings through a funnel into the
receiving flask. Preserving the full sample amount matters for downstream
concentration measurements. This task covers that last recovery operation.

The UMass Amherst [laboratory procedures][umass] describe a wash-bottle jet
and funnel for quantitative transfer into a volumetric flask. The University of
Toronto [analytical procedures][toronto] show the separate beaker-wall rinse
and drain through the funnel.
Dilute blue food colouring and room-temperature water provide a straightforward
repeatable analogue for a visible, water-soluble retained residue.

## Run

Install Isaac Lab-Arena, Inspect Robots and the YAM adapter as directed in the
Robocurve walkthrough. In this directory run:

```bash
python -m pip install -e .
python run.py --output runs/demo --device cuda:0 --physics-device cpu
```

The fixed starting arrangement uses seed `1415158438`. The action limit is
680 at 20 Hz (34 seconds). Physics runs at 240 Hz. The script records all three
640 × 360 cameras and a trace with the measured native pressure-wall
deformation, modelled water and solute balances, and scorer outputs.
Use `--no-cameras` for physics alone. With `--policy dry` the robot follows the
same motion with a much shallower squeeze. With `--policy hold` the arms remain
at their starting positions for the full trial.
A new run requires an unused output directory.

The Inspect Robots embodiment is `rinse_transfer`. The scripted policy is
`rinse_transfer_scripted` and the scorer is `task_score`. Model observations
are the three RGB images and fourteen native YAM joint coordinates. The
reference uses elapsed time and those joint observations. The fixed overview
lens is at X 610 mm, Y −105 mm and height 1370 mm. The left wrist lens is on
the upper side of the gripper during the beaker's side grasp, with a fixed
mount at X 0 mm, Y 58 mm and Z 73 mm in the gripper's coordinates.

## Bench and reset

Use a standard 747.5 mm-high YAM bench with arm centres 610 mm apart. In the
coordinates below X points forward from the arm mounts, Y points to the robot's
left, and all objects stand upright on the bench. Use ordinary physical objects
of the specified capacities.

| Object | Size and material | Fixed initial position |
| --- | --- | --- |
| 125 mL water wash bottle | [Nalgene 2402-0125][bottle] or equivalent, 48.3 mm cylindrical LDPE body, 119.4 mm to bend, 24 mm PP cap, fine bent LDPE spout | Base centre X 356 mm, Y 117 mm. Spout points left into the adjacent beaker. Fill with 70 mL of water. |
| 50 mL source beaker | [PYREX Griffin 1000-50][beaker] or equivalent, OD 42 mm, height about 56 mm (54 mm in the model), low-form borosilicate, one pouring lip | Base centre X 356 mm, Y 184 mm. Lip points toward the bottle. Two visible blue residue bands run around the lower inside wall. |
| 50 mL volumetric receiver | [PYREX 5640-50][flask] or equivalent, 51 mm bulb OD, 151 mm overall height (62 mm and 146 mm in the model), borosilicate with narrow neck | Base centre X 280 mm, Y 290 mm. Clean, empty and without stopper. |
| Funnel | Short-stem 55 mm polypropylene or glass lab funnel, 35 mm bowl depth, 7 mm stem OD | Stem rests in the neck with clearance for air to escape. The funnel opening lies 173 mm above the bench. |

Reset the dry beaker by marking 6 and 13 mm above the bench with
removable exterior marks. Spread 0.40 mL of dilute blue food-colour stock
around the two inside bands with a pipette, turning the beaker and letting it
drain until no free pool is visible. Weigh the dry and newly coated beaker
with a 0.01 g balance to record the retained stock aliquot. Repeat with
the same stock each trial. Use colour strong enough to see through the
glass without saturating the photography at the reference concentrations.
Set the beaker, empty receiver and funnel, and filled water bottle as shown.
The tools and glassware are reusable and the task uses only food colouring
and water.

## Score

Use a graduated cylinder or a balance to measure delivered rinse volume.
The scoring sample is the retained blue stock on the beaker wall. To measure
its recovery from the real bench, dilute the collected washings to a fixed
25 mL after recording the collected volume. Compare the transmitted colour
to 65% and 90% reference samples prepared from the known retained stock
aliquot and brought to the same 25 mL. Use identical clear vessels and a
phone photo against a white backlight with fixed exposure, or a standard
photometer. Weigh the wash bottle before and after to determine the amount of
water used. A tared absorbent sheet around the vessels allows spill
measurement without touching the work area during the trial.

- **1.0** for at least 90% of the initial dye in the receiver. Use at most
  25 mL of rinse water and spill no more than 3 mL. The bottle and beaker
  stand without gripper support, within 45 mm of their original marks and
  within 10° of upright for at least 0.4 seconds. Both fingertips clear
  the vessels by at least 50 mm, measured from about the middle of the grasp
  area or the centre of the funnel.
- **0.5** for at least 65% recovery within the same water and spill budgets
  when the full condition is unmet.
- **0** otherwise.

Only recovered dye counts towards the score. Water delivered straight into
the flask, a dry beaker pour, and residue lost on the bench do not contribute.

## Simulation and assets

The gripper deforms a freely standing, full-size wash bottle against
spring-backed native bottle panels. The panels' measured inward
displacement expels water, with a small compressible headspace and a short
hydraulic delay. A geometric gravity-corrected stream originates at the
actual bent tip. The beaker admits water through its opening. Impact and
contact with the gravity-aligned pool dissolve dye from local wall regions.
The liquid model uses the beaker's measured tilt and geometry to retain
or drain its contents, depositing runoff when the pouring lip lies over
the physical funnel. The model balances both water and dye. Clear
rendered streams and tinted pools depict the model's actual transfer.

The LDPE wall stiffness and dissolution rate are engineering estimates.
Rigid contact shapes approximate the flexible wall, thin glass and lips.
The receiver and seated funnel are held at their bench positions in the
simulation. Splash droplets, vessel breakage and thin retained films after
draining are omitted. All local props use original procedural USD geometry,
licensed under the included MIT license. The YAM adapter supplies the standard
procedural room and the I2RT Robotics [YAM arm and gripper assets][i2rt], under
the [MIT license][i2rt-license]. The adapter provides the source revision and
license as `SOURCE_COMMIT` and `LICENSE.i2rt` alongside the installed assets.

[umass]: https://www.ecs.umass.edu/cee/reckhow/courses/572/572bk11/572BK11.html
[toronto]: https://sites.chem.utoronto.ca/chemistry/coursenotes/CHM317/pdfs/CHM317Procs.pdf
[bottle]: https://www.thermofisher.com/order/catalog/product/2402-0125PK
[beaker]: https://ecatalog.corning.com/life-sciences/b2b/US/en/General-Labware/Beakers/Beakers%2C-Glass/PYREX%C2%AE-Beakers%2C-Low-Form%2C-Double-Scale%2C-Graduated/p/1000-50
[flask]: https://ecatalog.corning.com/life-sciences/b2b/CA/en/General-Labware/Flasks/Flasks%2C-Glass/PYREX%C2%AE-Class-A-Volumetric-Flasks-with-PYREX%C2%AE-Standard-Taper-Stopper/p/5640-50
[i2rt]: https://github.com/i2rt-robotics/i2rt/tree/120c3c81400171174604e503943f8d1ebc891058
[i2rt-license]: https://github.com/i2rt-robotics/i2rt/blob/120c3c81400171174604e503943f8d1ebc891058/LICENSE

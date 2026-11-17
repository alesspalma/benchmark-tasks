# Magnetic stirring-bar retention during decanting

Use the external magnetic restrainer to hold a stirring bar inside a beaker
while transferring its liquid into a second beaker. This is an ordinary wet-lab
operation: recovering a stir bar from the receiving vessel adds handling and
can contaminate or interrupt the next operation. The defining skill is
maintaining through-glass magnetic capture while the vessel tilts and pours.
It is submitted under **Other**, as a common laboratory research procedure.

## Run

Install the Isaac Lab-Arena, Inspect Robots and YAM adapter following the
task-board walkthrough, then install this directory in that environment:

```bash
python -m pip install -e .
python run.py --output runs/demo --device cuda:0 --physics-device cpu
```

`runs/demo` must not already exist. `run.py` runs the native scripted policy,
records all three RGB cameras, and writes initial/final states, a chronological
action/state trace, source/asset hashes and the actual score. Default native
camera size is 640 × 360 and control is 20 Hz. Rendering needs an NVIDIA GPU;
rigid-body physics and native mass-property updates use CPU. The 480 Hz physics
rate gives 24 physics steps per action. The action budget is 2,800 steps,
140 seconds. The demonstration's planned motion lasts about 84 seconds.

Useful controls are `--policy hold`, `--policy capture`, `--policy pour`,
`--policy no-magnet`, `--max-steps N`, `--no-cameras`, `--width W` and
`--height H`. A hold run takes no task action and starts at zero score. The
no-magnet policy keeps the tool away from the vessel while attempting the pour.
The no-magnet control uses the same 90-degree pour as the positive demonstration.
`--policy inversion-no-magnet` is a separate 110-degree stress test. These runs
use the same scene, material model and scorer.

The Inspect embodiment is `magnetic_decant`; the reference policy is
`magnetic_decant_scripted`. Model evaluation uses ordinary RGB images and YAM
joint positions. Object poses, water volumes and magnetic forces are available
only to material integration, scoring and diagnostic logs. The reference
policy follows Cartesian waypoints with joint-position IK feedback; it does not
read the bar pose or the water state.

```bash
python -m pytest -q tests
```

Numerical tests cover magnetic potential/force consistency, total magnetic
angular momentum, distance dependence, gravity-levelled fluid mass/inertia,
finite liquid flight, vessel-wall blocking, collection and spill conservation.
Native trial evidence is listed in `validation/`; tests do not replace that
evidence.

## Fixed starting scene and reconstruction

Coordinates are metres, +X forward, +Y left, +Z up, at the native YAM station.
Let T be the station table height (0.7475 m in the supplied station). All
objects below are initially upright; the tool lies flat with its stem upward.
Positions are the bottom-centre origin for beakers, centre for the bar and
magnetic face, and body placement for fixtures. No initial grasp, attachment
or magnetic capture is provided.

| Object | Construction | Initial position |
|---|---|---|
| Source beaker | 250 mL low-form borosilicate, 70 mm OD × 95 mm, 2 mm wall, 3 mm base; dry 110 g | (0.40, −0.06, T+0.008) |
| Receiver | Same empty beaker | (0.22, −0.06, T+0.008) |
| Water | 150 mL clean water in source; rendered blue for visibility | gravity-levelled inside source |
| Stirring bar | White PTFE coating, 30 × 8 mm, 5 g, magnetic axis along length | source+(0,0,0.0072), long axis X |
| External restrainer | Blue polystyrene case, 80 × 25 × 10 mm magnetic face; 16 mm diameter × 27 mm stem, 32 mm diameter × 7 mm thumb disk; 70 g | (0.25,0.24,T+0.0132), long axis X |
| Catch tray | 550 × 580 mm base, 4 mm thick, 25 mm walls, 5 mm wall thickness | centred (0.36,0.03,T) |
| Beaker mats | Two 100 × 100 × 4 mm nonslip rubber mats | each beaker XY, centre height T+0.006 |
| Tool mat | 180 × 160 × 4 mm nonslip rubber | tool XY, centre height T+0.006 |

The ordinary stem/thumb grip follows the commercial restrainer form, without
extra robot handles. See `SOURCES.md` and `reconstruction/` for dimension
provenance, the dimensioned case and primary references. The generated USD
files are the complete task geometry; native glass collision geometry is an
open cylindrical vessel composed of 64 tangent wall panels, a base and lip.
The bar and tool are independent native rigid bodies. Cosmetic liquid meshes
and graduation marks are not colliders.

## Defining physics

The magnet and bar interact through a finite-size softened dipole potential.
Each receives its native force and torque, including equal-and-opposite
reaction; nothing attaches the bar to the tool or vessel. Tool moment is
1.0 A m² through the tool face, bar moment 0.16 A m² along its length and
finite-size length 8 mm. Native PTFE/glass
static/dynamic friction is 0.10/0.08 with minimum combination to represent the
wet PTFE interface. Magnetic moments and friction are stated engineering
parameters, not hardware measurements.

Water uses a disclosed analytic material law. At the current native vessel
pose, a horizontal free surface defines volume, centre of mass and inertia.
Gravity outflow at the lowest rim uses a 0.65 discharge coefficient and 18 mm
effective lip width. Emitted liquid occupies finite-flight packets with
gravity and the vessel's translational/angular launch velocity. The receiving
vessel gains water only when a packet actually crosses its open mouth. Native
vessel walls and robot collision hulls block the stream; missed or blocked
water counts as spill. Source, receiver, in-flight and spilled volumes sum to
the original 150 mL. Native vessel mass, centre of mass and inertia change with
actual contained liquid. The model does not implement turbulent slosh, splash
droplets, wetting films or glass fracture.

## Score and practical measurement

Measure collected and spilled water with a 1 g balance or graduated cylinder;
observe the bar and restrainer through the transparent glass. The score is:

- **0** before 70 mL is collected with magnetic bar retention; permanently zero
  if the bar leaves the original vessel, an object falls off the work surface,
  state becomes invalid, or more than 10 mL is spilled.
- **0.4** for at least 70 mL collected while the external restrainer holds the
  bar at the inner wall.
- **0.7** for at least 140 mL collected with that retention.
- **1.0** for at least 140 mL with no more than 5 mL spilled, after both beakers
  are upright and independently supported, the tool rests on its mat and both
  robot arms are clear for two seconds.

Retention during transfer is an observable tool-outside/bar-at-inner-wall
relationship, not a magnetic-force threshold. The bar must remain continuously
inside the original vessel; removing it and returning it does not qualify.
Containment checks the full cylindrical bar, allowing 2 mm wall/bottom and
3 mm mouth measurement tolerance.
The instruction permits different arm assignments, grasps and motion orders.
The scorer does not prescribe the demonstration's hand sequence.

Final upright means each beaker's vertical-axis cosine exceeds 0.98. Supported
means the bottom is within 4 mm of the mat surface. Stable means linear speed
below 20 mm/s and angular speed below 0.2 rad/s. Robot collision geometry must
be at least 15 mm clear of all free objects; water still in flight must be below
0.1 mL. Exact tool-mat bounds and retention measurement tolerances are in
`magnetic_decant/scoring.py` and `materials.capture`.

Reset by returning 150 mL clean water and the bar to the source, emptying the
receiver and catch tray, drying exterior surfaces and restoring the stated
poses. No perishable or one-off material is required. Use intact lab glassware
and clean water in physical trials; replace any chipped glass. Field strength
and outflow require physical calibration before a hardware comparison; the
submitted feasibility evidence is simulation evidence.

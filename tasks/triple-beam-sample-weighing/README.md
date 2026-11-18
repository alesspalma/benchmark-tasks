# Weigh a steel specimen on a mechanical triple-beam balance

Mechanical balances give a direct, unpowered mass reading in field and research
labs. Shifting a poise changes its leverage and moves the beam supporting the
pan. The operator first finds the hundreds and tens ranges, then slides the fine
rider until the pointer aligns with zero without a hand on the beam. The final
reading is the sum of the three visible rider scales. University of Idaho
[research SOP 3505][sop] and the [Minnesota DOT laboratory manual][mn] document
this class of balance in real measurement work.

## Fixed setup

Use a level, rigid work surface. Orient the instrument with its pan on the
robot’s left and the beam and pointer extending right. These coordinates use
the YAM station axes, with X away from the robots and positive Y to their left.

| Item | Dimensions and material | Initial placement |
| --- | --- | --- |
| Standard dual YAM station | 1,100 × 1,300 mm laminated top at 747.5 mm, two arms 610 mm apart | Both arms in their usual ready position |
| Triple-beam balance | OHAUS 710-00 or equivalent 610 g, 0.1 g instrument. Cast-metal base, three metal rails and captive poises, 152 mm stainless pan. Simulated footprint 161 × 418 mm and work height to pan of 158 mm | Beam pivot projection X 353 mm, Y 43 mm. Circular pan centre X 353 mm, Y 135 mm. Pointer 244 mm to the right of the pivot |
| Steel specimen | Smooth mild-steel cylindrical blank, Ø40 × 34.2 mm, 337.0 g ± 0.2 g | Flat face down at the centre of the pan |

The middle beam is graduated from 0 to 500 g in 100 g notches and has 184 mm
of travel. The rear beam is 0 to 100 g in 10 g notches over 171 mm. The front
beam is 0 to 10 g with 0.1 g ticks over 170 mm. The rail and body details are
representative dimensions for the simulation. The commercial instrument
provides the real notches, markings, pointer, damper and zero adjustment.
Used or laboratory-stock 610 g/0.1 g instruments with the same three poise
ranges also work. The manufacturer’s [700-series data sheet][ohaus] identifies
the physical instrument. No fixture is needed.

Clean the pan and zero the empty instrument according to its manufacturer
before the trial. Set all three poises to zero and place the plain, unlabelled
sample on the pan centre. The loaded beam starts off zero. A small piece of
40 mm mild-steel round stock can be cut to 34.2 mm and deburred. Verify its
mass with an independent scale and adjust the thickness with a file if needed.
Use the same specimen each time. Between trials remove it, return the poises
to zero, verify that the empty pointer settles at zero and reload the sample.

## Scoring

When the robot is done, take a straight-on photo of the pointer against the
zero index, and a photo of the three poise readings. Use an independent bench
scale to establish the specimen mass before the trial. Use ordinary calipers
or a transparent ruler alongside the pointer if a gap needs confirmation.

- **1.0** when the sample remains on the pan, the two notched poises are in
  their marks and the sum is within 1.0 g of the specimen mass. The pointer
  must remain within 1.5 mm of zero for at least 1.2 s with both grippers at
  least 45 mm clear of the pan load, rider or pointer.
- **0.5** when the same sample, poise seating and hand clearance conditions
  hold, the reading is within 10 g and the pointer is within 10 mm, but the
  full-score condition has not been met.
- **0** otherwise.

Use the film to confirm that the robot has let go and that the pointer stays
settled. The score allows any convenient order of moving the riders.

## Simulation and running

The scene is generated from original USD primitives under the included MIT
license. The YAM robot and station use the supplied YAM Arena adapter and its
assets. Native rigid contact supports the specimen on the pan and lets the
grippers push the riders. The poises are captive on passive prismatic joints.
Their masses and travel set the relation between graduation and gravitational
torque. Native gravity rotates the limited beam hinge, whose damper represents
the instrument’s magnetic damping. A bounded force centred on the actual
nearest coarse or medium graduation represents each physical detent.
A vertical guide and bearing contact keep the pan level and transmit its live
load to the beam, representing the production instrument’s parallel-link support.

Install Isaac Lab-Arena and Inspect Robots following the simulation walkthrough
and use the compatible YAM adapter. From this task folder, run:

```bash
python -m pip install -e .
python run.py --output runs/demo --device cuda:0 --physics-device cpu
```

The fixed starting layout uses seed `1415158438` and a limit of 760 actions
(38 seconds) at 20 Hz. The underlying physics rate is 240 Hz. The native
runner records all three views at 640 × 360 and writes one JSON trace row per
command. Add `--no-cameras` for physics-only validation, `--policy hold` for
the unchanged-scene control, or `--policy coarse` to leave off the final fine
adjustment. An existing output directory is never overwritten.

Inspect Robots entry points are `triple_beam`, `triple_beam_scripted` and
`task_score`. Normal policy observations are the three RGB views and the
fourteen YAM joint coordinates. The scripted controller uses time and the
ordinary joint observations. Instrument state is read separately for scoring.

## Reference simulation result

The recorded scripted trial reached 1.0 after 403 actions (20.15 s of
simulated operation, around 34 s wall time on a local Blackwell GPU host).
The unsupported reading was 336.85 g and the pointer stayed within 0.60 mm
of zero at termination, with grippers 179 mm clear of the evaluated regions.
A same-source control stopped after the two coarse adjustments and returned
0.5 at 329.92 g. Camera files contain every frame, including reset.

[ohaus]: https://res.cloudinary.com/iwh/image/upload/q_auto,g_center/assets/1/26/Documents/Ohaus/710_00/710_00_doc_1.pdf
[sop]: https://ir4works.org/docs/sops/University%20of%20Idaho%202024%20SOPs%20Will%20Meeks.pdf#page=42
[mn]: https://www.dot.state.mn.us/materials/manuals/laboratory/2002.pdf

# Homogenize a capped sample on a touch vortex mixer

Vortex mixing homogenizes sealed liquid samples before replicate aliquots are
withdrawn for measurement. This bench operation couples a hand-held sample to
a short, fast orbital drive. The operator centres the bottom of the tube on
the rubber head and holds downward pressure while the head orbits under load.
Lifting the tube stops the motor. The [mixer manufacturer's procedure][manual]
specifies this touch sequence. A [robotic NMR sample-preparation study][nmr]
also uses vortexing to homogenize samples before analysis.

## Run

Install Isaac Lab-Arena, Inspect Robots and the YAM adapter from the Robocurve
simulation walkthrough. From this directory:

```bash
python -m pip install -e .
python run.py --output runs/demo --device cuda:0 --physics-device cpu
```

The reset is fixed (seed `1415158438`), with a trial limit of 640 actions
(32 seconds) at 20 Hz. The contact and orbital physics step at 240 Hz on the
CPU. The command records each camera natively at 640 × 360 and writes every
action, pose and score to the trace. `--no-cameras` runs the same physics
without RGB. Add `--policy no-press` for the hover comparison or
`--policy hold` for a do-nothing trial. The scripted trial takes about
one minute of wall time, including simulator startup. The right wrist
camera is angled 14° down from its station mount to see the full tube.

The Inspect embodiment is `vortex_mixer`, the reference policy is
`vortex_mixer_scripted` and the Inspect scorer is `task_score`.
The action is the standard 14-dimensional YAM command. Policy inputs are RGB
and the ordinary 14 joint-position values; the scripted demonstration uses
time and the observed joints. State used to evaluate the task is separate.

## Fixed setup and reset

Use a clean, level laboratory counter. X runs away from the robot, Y runs to
its left and heights below are measured from the counter. Neither arm touches
the equipment. The motor is off.

| Item | Material and dimensions | Starting location |
|---|---|---|
| YAM benchtop station | 1,100 × 1,300 × 60 mm laminated tabletop, 747.5 mm surface height, two arms 610 mm apart | Standard station geometry |
| Pressure-activated mixer | Benchmark Scientific Vornado BV101, 99 mm along X × 94 mm along Y × 66 mm high, ABS housing and rubber feet and cup. Ø46 mm cup, 4 mm orbit diameter, 2,800 rpm | Footprint centre X 363 mm, Y −112 mm, rubber feet flat. Cup level and unpressed |
| Capped sample tube | [Greiner CELLSTAR 210270][tube] or equivalent 50 mL, Ø30 × 115 mm transparent polypropylene skirted tube with blue sealed plastic screw cap | Base centre X 408 mm, Y −266 mm, upright on the counter |
| Benign layered sample | 35 mL combined, pale clear upper layer and darker blue lower layer made with water, food-grade glycerol and water-soluble blue food colour | Both liquids inside the capped sample tube |

Prepare the real scoring sample in a fresh tube. Use 12 mL of a 1:1
food-grade glycerol and water mixture containing ordinary blue food colour
in the bottom, then add 23 mL water slowly down the inside wall to keep the
layers visible. A two-layer tube is ready as soon as the top layer is visibly
clear. Close the cap, wipe the tube dry and set it upright on the mark.
Prepare a fresh sample between trials. Use standard reusable lab gripper
covers if the rig normally uses them. Inspect the tube and cap before each
trial, take a baseline photo and replace any leaking tube. The sample
ingredients are food grade and stay sealed inside the tube.

Use the [manufacturer's commercial mixer][mixer], listed at $233.67 when
checked. Place it on the level bench and seat its rubber feet as the manual
specifies. The commercial mixer ships as a complete standalone unit. A normal side photo uses a piece of matte white
card behind the tube. The photo card and ruler can be placed for scoring
without entering the robot workspace during manipulation.

## Scoring

The colour contrast is the absolute difference in the mean blue-minus-red
value between the upper and lower patches. Use the same camera and diffuse
lighting, and the same tube orientation against a white card, before and
after a trial. At each time use two 3 × 3 mm patches on the same visible meridian,
centred at 20% and 75% of the liquid height above the lowest visible fill.
Exclude reflections and the tube graduations. Normalize the contrast drop to
the original difference. Draw down the dye concentration if either patch is
saturated. The reduction should be reproducible with a phone photo or a view
from the trial video. Confirm from the video or motor sound that the tube
was in the cup while the mixer operated.

- **1.0** for at least 90% normalized contrast reduction, with the tube's
  base supported by the counter, within 230 mm of the mixer centre, no more
  than 12° tilted, released with 30 mm clearance from the pads, and still
  with the mixer off for at least 1 s.
- **0.5** for the same 90% contrast reduction before all placement conditions
  hold.
- **0.25** for at least 40% and less than 90% contrast reduction.
- **0** otherwise, or for losing the sample from the work surface. A physical
  tube that opens or loses visible liquid also receives zero.

Use a ruler and side video for placement and clearance. The starting tube
position is an acceptable final position. Either arm can maintain cup
engagement with any secure grasp.

## Simulation

The device is a full-size BV101-style pressure-activated mixer. A native
vertical spring joint supplies the cup's preload and travel. Slight downward
pressure depresses the switch; two force-limited motors generate the 4 mm
orbital motion from that switch state. The switch threshold (1.5 mm), spring
stiffness (1.8 N/mm) and cup contact material are engineering estimates. They
give a few newtons of pressure to start and about 11 N at the highest load of
the reference trial. The XY motor dynamics and dimensions without published
drawings are engineering estimates.

Native contacts carry both the grip and the tube-to-cup load. The
motor targets the instrument's drive joints. A short filtered window extracts the
measured physical tube-bottom displacement at the motor frequency while the
tube is contacting the activated cup. The two colour layers converge
according to the measured coupled motion. The coefficient represents mixing
over a few seconds under normal drive and is an engineering estimate.
Layers remain distinct when the tube hovers over the cup. The simulated
transparent tube is sealed and represented by a rigid contact shell. The
colour field replaces internal flow and free-surface slosh.

The scene generator produces every prop from original USD geometry. The YAM
station, robot and cameras are supplied by the adapter. The task files use the
included MIT license.

## Reference run

The native camera-recorded reference scored 1.0 in 452 steps (22.6 s).
The tube stayed physically coupled for 5.63 s, with a peak measured orbital
amplitude of 2.32 mm. Relative contrast fell by 99.87%, and the tube came to
rest upright with the mixer off.

[mixer]: https://www.benchmarkscientific.com/product/bv101/
[manual]: https://www.novatech-usa.com/pdf/Benchmark%20BV101%20Instruction%20Manual.pdf
[tube]: https://shop.gbo.com/en/england/products/bioscience/tubes-beakers/15ml-cellstar-polypropylene-tube/210270.html
[nmr]: https://journals.sagepub.com/doi/10.1016/j.jala.2006.10.007

# Reheat the lunch in the microwave

The microwave door is open and the lunch is already on the turntable in a closed, microwave-safe bowl. A closed steam vent allows pressure and hot steam to build up during heating. Opening the bowl's own vent before starting preserves the splatter cover and gives steam an exit. This is the configuration specified for Sistema's 940 mL microwave noodle bowl. [Bowl dimensions and use instructions](https://www.sistemaplastics.com/940ml-noodle-bowl)

## Reproduce the task

Use the instructions and scoring with a cold countertop fixture. A retail 940 mL Sistema Microwave Noodle Bowl and a compact left-hinged microwave chassis with 300 mm of internal width give the intended clearances. The GE 0.7 cu. ft. family is a size reference. Keep a retired chassis unplugged and its internal covers intact. Mount a low-voltage, spring-return Start switch in an external removable cap fixture at the control position. Log its event with a door contact or a side-view camera. The indicator represents a door-interlocked reheating request, and the fixture stays cold throughout every trial. [Representative microwave dimensions and layout](https://www.geappliances.com/compare/2773/3584)

Mount the two YAM arms on 610 mm centers, 750 mm above the floor. X is forward from the line of the two bases and positive Y points to the left arm. Use a 1,040 × 1,180 mm worktop at Z = 747.5 mm. Set the microwave's front door hinge at (447, 158, 879) mm. The nominal outside shell is 433 mm wide, 336 mm deep and 265 mm high, with a left hinge and right control panel. The round glass tray is 266 mm in diameter, centered at X = 591 mm and Y = 0. Place a 254 mm silicone microwave mat over the glass with its top surface at Z = 794 mm. The Start switch is at (432, −207, 863) mm, travels 6 mm, and registers after 4 mm. The window permits a view into the cavity. [Microwave mat specifications](https://tovolo.com/products/microwave-mat)

Use the 156 mm nominal body of the handled polypropylene bowl with the lid clips snapped shut and a 210 g reusable room-temperature silicone training load inside. Bowl, lid and load weigh about 340 g together. The bowl has a molded D-handle across the front-left quarter. The handle points 24° from left toward the front of the robot fixture. The reference scene uses a rocker with a visible raised position and a through-port molded into the lid. Use the real bowl's own raised steam vent for physical scoring. Start the bowl upright on the glass turntable with its base centered at (549, 0, 794) mm. The vent is toward the front and right of the bowl. In the generated fixture its hinge is 31 mm toward the robot and 21 mm to the right of the bowl center. Set the door to 130° open, snap the steam vent shut, release the Start switch and reset the two arms to the standard ready pose. The container and lid may stay assembled between trials.

## Score the trial

Use a side view of the lifted steam vent when the bowl is on the turntable. Check that the vent is in its raised open position with a visible steam exit gap. The physical bowl's vent must click into its open position. In the generated scene the flap clears the port when its forward edge has at least 4.5 mm of lift from its closed seat. The rocker travels to a 13.5° stop. For a bowl loaded on the tray, check from above that the base is flat and within the tray's usable circle. In the reference geometry the bowl center may lie up to 52 mm from the tray center. Read a door contact or inspect the fully seated front seam. The simulated door closes within 2° of its physical stop.

- **1.0** for the vented bowl on the tray, a closed door, a registered Start press, and one second of stable hands-off operation after the button is released.
- **0.5** once the vented bowl is on the tray and the door is closed, before Start or during the hands-off dwell.
- **0.0** for other states. A registered Start with the loaded bowl sealed makes the score zero for the rest of that episode. Opening the door before starting is normal. Presses with the door open cannot trigger a cycle.

The camera and switch or door contact record determine the order of the actions. Observe the vent before closing the door and in the window after the press. Either arm can operate each control. In the demonstration the left gripper holds the handle against the turntable while the right opens the vent. The left gripper then closes the door and the right presses Start.

## Run

Use the approved Isaac Lab–Arena environment and the pinned dependencies in `pyproject.toml`. The seed is **1415158438**. Run at most **740 actions** at **20 Hz** from this task directory.

```bash
python -m pip install -e . --no-deps
python -B run.py --device cuda:0 --physics-device cpu --output /tmp/lunch-vent-demo
python -B run.py --policy hold --no-cameras --device cpu --physics-device cpu --output /tmp/lunch-vent-hold
python -B run.py --policy premature --no-cameras --device cpu --physics-device cpu --output /tmp/lunch-vent-premature
```

Use Inspect Robots embodiment `lunch_vent`. Native PhysX contacts move the bowl, valve, door and Start switch. The reference policy measures the moving rocker and fingertip to guide a physical press. The coupling of the latched lid to the bowl is rigid. The vent hinge holds its position through joint resistance. The fixed turntable and indicator reproduce a cold interlocked start.

## Assets

Task geometry, reference policy and documentation are released under MIT. The pinned robot adapter supplies the YAM assets and calibrated cameras. [Robot asset terms](https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/assets/yam/LICENSE.i2rt) · [Camera asset license](https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc126418ea5e74ac567133d/LICENSE)

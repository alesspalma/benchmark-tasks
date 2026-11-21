# Prepare a wet microscope mount without large air bubbles

**Instruction** — Prepare a wet microscope mount of the printed letter using one drop of distilled water. Leave the coverslip centered over the letter with no large air bubbles.

A wet mount keeps a specimen flat and hydrated without air pockets obscuring the microscope view. The method is illustrated with a printed letter in introductory microscope protocols. Touching a lower edge of the coverslip to the drop and lowering the far edge lets the advancing wet front displace the air. One hand dispenses a measured drop, and the other handles the coverslip by its edges and seats it. The trial uses a repeatable inert printed target. [LWTech wet-mount protocol](https://openwa.pressbooks.pub/lwtechbiol160/chapter/lab-1-care-and-use-of-compound-microscopes/) · [University of Idaho laboratory guide](https://www.uidaho.edu/-/media/UIdaho-Responsive/Files/cals/departments/aelc/educators/AG-510-Botany-and-Soil-Science.pdf)

## Setup and reset

Use two parallel-gripper YAM arms at 610 mm center spacing and a level 747.5 mm bench. Coordinates below are forward and left from the midpoint of the arm mounts. The inspection surface is 42 mm above the bench.

| Object | Physical specification and placement |
|---|---|
| Inspection pad | 104 × 61 mm black ABS cradle with two 10 × 37 mm blue end rails at 42 mm height, fixed side/end stops, and millimeter scale ticks. Center at 385 mm forward, 12 mm left. |
| Slide | 75 × 25 × 1 mm glass microscope slide placed loose, flat and centered on the inspection pad with its long axis forward. Its central 6 mm lowercase letter `e` is preprinted in water-resistant black ink with a thin ink layer. The colorless slide may have a frosted end. |
| Coverslip | 22 × 22 mm No. 2 glass, nominal thickness 0.22 mm. Place it flat and loose across two 4 × 8 mm nonclamping supports at 374 mm forward and 105 mm right, with 34.5 mm of clearance below it. The supports contact the slip near its midline and keep both side edges accessible. |
| Dropper | 108 mm glass or polymer dropper with a 29 × 26 × 28 mm rubber bulb, an approximately 0.25 mm dispensing orifice, and one 25 µL dose of distilled water. A matching squeeze actuator or single-dose dropper may be used to meter 20–30 µL. Keep it upright in its slotted holder at 347 mm forward and 147 mm left. |
| Workbench | Clean dry non-slip surface with the supports and pad fixed in place. Both robot arms begin at the documented ready joint targets with their grippers open. |

Replace coverslips with visibly chipped edges and wipe the slide and bed dry between trials. Fill the dropper with one measured dose while leaving its exterior dry, restore the three loose objects to the photographed positions, and clear the inspection pad of extra tools. Use the supplied measurements to reproduce the setup.

## Score the physical result

Use the fixed side view and a perpendicular close-range photograph with a ruler or phone macro lens. Side lighting from the edge makes the wet contact footprint and air boundary visible against the dark pad. Measure the smallest circle containing the largest trapped air pocket, and estimate the fraction of wet area by superimposing a square grid on the coverslip photograph. A measured dose deposited on the slide is within 10 mm along the slide and 7 mm across the slide of the printed target, with no spill off the slide.

| Score | Observable condition |
|---|---|
| 0.00 | No valid water dose on the slide. |
| 0.25 | The dose is deposited and the coverslip is not yet seated over the letter. |
| 0.55 | A centered coverslip lies on the slide with a wetted fraction above 60 percent. |
| 0.80 | A centered coverslip lies flat on the inspection pad with at least 82 percent coverage and no air pocket exceeding 2 mm across. |
| 1.00 | The 0.80 condition holds for at least 0.5 seconds while both grippers remain at least 39 mm from the coverslip center and the slip's translation is less than 9 mm over the interval. |

The center of an accepted coverslip is within 3 mm along the slide and 2.5 mm across from the letter center. All four coverslip corners must lie within 1 mm of the slide's upper plane. The slide center must lie within 14 mm of its marked starting position and its surface must remain within 8 degrees of level. Use either arm for either role. Score the final image and stable end segment, with precedence given to the highest satisfied row.

## Run and reproduce

Use the workspace's Isaac Lab Arena runtime. Install with `python -m pip install . --no-deps` into the matching dependencies from `pyproject.toml`. The Inspect Robots embodiment is `wet_mount`. Trials start from fixed seed **1415158438** and run at 20 actions per second with 240 Hz native CPU physics. The action budget is **590 actions**. The reference uses elapsed time, robot joints and simulated object poses for tool alignment. The reference interacts with the scene through the same gripper and joint action interface as any policy.

```bash
python run.py --device cuda:0 --physics-device cpu --output /tmp/wet-mount-demo
python run.py --no-cameras --policy hold --output /tmp/wet-mount-idle
python run.py --no-cameras --policy dry --output /tmp/wet-mount-dry
python run.py --no-cameras --policy flat --output /tmp/wet-mount-flat
```

The simulator models the cap footprint and spreading from measured bulb travel, gravity, the collision geometry and the glass approach angle. A wet region and enclosed air pocket are shown as contrasting surfaces. Camera tint and thickness make the submillimeter film easier to see. The model applies opposed viscous shear forces when the two faces are almost parallel against the deposited film. It omits detailed meniscus geometry, glass fracture and dissolved air. `receipt.json` and `trace.jsonl` contain scored outcomes and the native motion measurements. Geometry and task code are MIT licensed. The original two-arm station and robot files are provided separately by the pinned YAM package. [YAM assets and license](https://github.com/kairosiann/yam-isaac-lab-arena/tree/0059d02a1a91bc6834c1d0846022891df0c78730)

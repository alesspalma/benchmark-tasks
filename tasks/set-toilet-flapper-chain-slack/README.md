# Reconnect a toilet flapper chain and test the flush

Restore the link between the external flush handle and the rubber flapper in a full-size gravity toilet tank. Hold the trip lever steady while fitting the free spring clip to the eye directly above the flapper. The chain needs enough loose travel to let the flapper seal and enough lift to open the outlet on a normal handle stroke. Release the handle and let the valve reseat after the test flush. Either arm can operate the fitting or the handle.

[Fluidmaster's two-inch flapper installation instructions](https://www.fluidmaster.com/wp-content/uploads/2024/01/4-3608_Gr2_K-400H-043_2in-PMAX-Super-Flapper-Kit_IB_ENG-SPAN_041924.pdf) specify the eye above the flapper and approximately ½ inch of slack. [Korky's trip lever installation instructions](https://www.korky.com/toilet-repair-help/install-korky-strongarm-toilet-levers) also call for positioning the arm over the flapper and checking the flush.

## Setup and reset

Use a lid-off, front-handle gravity tank about 420 mm wide, 225 mm front to back and 210 mm high, with a stock two-inch flush-valve seat, a hinged rubber flapper and a three-eye trip arm. Keep the fill valve shut off so one measured test charge drains into an empty catch pan that holds at least 5.2 L. Mount the cistern rigidly above the catch pan and leave the flush handle accessible at the front. Use a flapper chain with the lower end already clipped to the flapper. The free end carries a small spring hook with a steel pinch tab and rests on a narrow temporary sill attached to the rear wall. Keep the hook completely free of the trip lever. Select the chain length so the middle eye leaves about half an inch of slack with the flapper seated. Fill to 5.2 L with clean water. A small amount of blue food dye can improve visibility in a recording.

On the simulator bench, X points forward from the midpoint of the robot bases and Y points toward the left robot. The robot bases are 610 mm apart. The bench is at Z 747.5 mm. Tank walls run from X 350 to 562 mm and Y −208 to 208 mm. The flush valve is centered at X 442 mm and Y −26 mm. The outside handle starts at X 305 mm, Y 185 mm and Z 996 mm. The middle trip-arm eye starts at X 441 mm, Y −26 mm and Z 996 mm. It is vertically over the outlet. The chain has a free length of 161 mm. At reset, the spring hook rests on the 22 × 27 mm sill at X 465 mm, Y −41 mm and approximately Z 990 mm, about 29 mm away from the middle eye. The pinch tab measures 26 × 21 mm. Both grippers start open.

To reset, drain and empty the catch pan, let the flapper close on the clean seat, detach the upper chain hook and place it on the sill. Leave the lower chain attachment intact and the outside handle at rest. Refill the tank with the same test charge before starting the next trial.

## Physical score

| Score | Outcome |
| --- | --- |
| 0.00 | The spring clip is free or attached to the wrong eye. |
| 0.25 | The clip is retained on the eye directly above the flapper. |
| 0.50 | The clip is retained, the handle is at rest and the seated flapper has 6–19 mm of available chain slack. |
| 0.80 | A flush driven by the linked external handle discharges at least 3.7 L into the catch pan while the fitting remains attached. |
| 1.00 | The handle-driven test discharges at least 3.7 L. The clip remains on the middle eye, the handle returns to within 7° of rest, the flapper lies on its seat with 6–19 mm of chain slack, and the valve shows no continuing visible flow for 0.6 s with the hands clear. |

Measure the slack as the stretched chain length between its two attachment points minus their straight-line separation with the handle at rest and the flapper sitting flat. A ruler can reach both points from the lid opening. Use the catch pan graduations or weigh the collected water. The overview and wrist views can show the retained hook, one external handle stroke, the returning flapper and the drained water line. Lifting the flapper or taking water out by hand does not demonstrate a linked flush.

## Reproduce the native run

In the IsaacLab–Arena and Inspect Robots environment, the embodiment name is `flapper_chain`. The reference uses ordinary YAM joint targets with joint-position feedback, fixed geometry, seed 1415158438, 20 Hz control and 240 Hz CPU physics.

```bash
python -m pip install -e . --no-deps
python run.py --device cuda:0 --physics-device cpu --output /tmp/flapper-reference
python run.py --device cuda:0 --physics-device cpu --no-cameras --policy hold --output /tmp/flapper-idle
python run.py --device cuda:0 --physics-device cpu --no-cameras --policy no_clip --output /tmp/flapper-unlinked
```

The front handle, trip arm and rubber flapper move on native revolute joints. The loose spring hook is a free rigid body. Its mouth can catch a physical trip-arm pin and the spring tongue applies a bounded retention force. The chain carries tension when its live endpoints exceed its free length. Gravity discharge follows the water head and flapper opening. The visible water level and catch-pan surface follow the conserved volume. The refill valve remains shut during the test.

The camera-recorded reference scored 1.0 in 345 actions (17.25 seconds). The clip remained fitted with 16.31 mm of slack, the linked handle discharged 4.20 L, and the flapper reseated with zero continuing flow. An idle control scored 0.0 after 640 actions. A second 640-action control pressed the same exterior handle through 26.52° with the clip still free. It discharged no water and scored 0.0.

The cistern, trip lever, valve, clip, bench and catch pan geometry are original and MIT-licensed. The YAM and its three-camera station come from the pinned adapter in `pyproject.toml`, which references the I2RT robot geometry with its MIT notice.

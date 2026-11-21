# Measure a rubber coupon with a handheld Shore A durometer

Place the rubber coupon flat on the steel platen. Take five readings with the handheld Shore A durometer. Measure at least 12 mm from the edges and 6 mm apart, holding the foot flat and still for one second at each point.

The lab operation is indentation hardness measurement of rubber stock used in seals, gaskets and other engineered parts. Materials labs use hardness to identify compound variation and check whether a batch meets the part specification. In seals, hardness affects the compressive force needed to seat a part and its resistance to extrusion under pressure. A robot must support the specimen on a firm plane, grip a free handheld instrument and repeatedly seat a small pressure foot squarely under spring resistance. A reading taken with the foot raised or tilted can differ even on uniform material.

## Physical setup and reset

- Use a handheld digital Shore A durometer with a 2.5 mm indenter stroke, a truncated-cone indenter and a round pressure foot about 32 mm across. Start with a valid reference-block check following the instrument maker's procedure.
- Use a flat, uniform, single-layer 100 × 70 × 10 mm rubber specimen of nominal 60 Shore A. Keep the specimen and platen clean, dry and at an equilibrated room temperature of 23 ± 2 °C. Verify the specimen's normal readings locally with the same durometer before running the task.
- Place a 140 × 115 mm rigid steel platen at (410, 18) mm with its top 10 mm above the bench. Coordinates measure forward and left from the midpoint between arm bases. Set the loose coupon on a narrow 89 × 44 × 12 mm rest centered at (303, 148) mm, with its long dimension forward-back. The narrow rest exposes both long sides for pickup.
- Leave the durometer upright and ungrasped at (331, −162) mm in an open rest with twin 16 mm high ledges. The pressure foot rests on the ledges, with its indenter extended in the gap without touching the bench. The instrument may have a flat housing shaped for a handheld grip. Arm bases are 610 mm apart and the bench surface is 747.5 mm high.
- Reset both arms to the raised start pose with open grippers. Use the same specimen position, platform and instrument rest for every trial. A ruler and a fixed overhead camera with a scale in frame can verify edge distance and separation of the tested points.

## Physical score

Award 0.20 for each of the first five distinct readings, for a maximum of 1.00. The coupon must lie flat on the steel platen, with all four corners over its usable surface. A reading requires a stable reading of 55–65 Shore A after one continuous second of stable pressure-foot contact, with the axis within 5° of the surface normal and the center at least 12 mm from all edges. Each counted center must be at least 6 mm away from all earlier centers. A pause in air or a repeated press at the same location earns no additional credit. Read the display and check the contact duration, edge clearance and spacing from the trial video. Either arm can place the coupon or operate the durometer.

The native reference uses one arm to grasp and position the loose rubber and the other to grasp, orient, seat and lift the durometer. The joint-action interface exposes the ordinary RGB and joint observations. The reference controller reads simulator geometry to align the grasped tool and specimen.

## Simulation and reproduction

The tool has its own freely moving housing, a segmented annular pressure foot and a physically spring-loaded prismatic indenter. The annular foot uses a flat circular collision surface on the specimen. The spring's preload is 0.55 N, its rate is 3,000 N/m and its travel is 2.5 mm. The steel platen, loose coupon, foot and robot use native collision. The local rubber indentation uses a viscoelastic normal-force law at the tip applied in opposite directions to the indenter and the coupon. The coupon bulk is a rigid slab for grasping and support. The resulting stroke drives the Shore A readout. The model targets the normal response of a nominal 60 Shore A pad. Reproduce thin layers, rough surfaces or different compounds on hardware before extending the results to them.

Run a fixed-seed trial in the repository's Isaac Lab Arena runtime after installing this package. The runner uses seed 1415158438 and a fixed initial layout. It allows 820 actions at 20 Hz, or 41 seconds of simulated time. It writes reset camera views, native MP4 streams, traces and a source-pinned receipt.

```bash
python run.py --device cuda:0 --physics-device cpu --output /tmp/shore-a-demo
python run.py --device cpu --physics-device cpu --no-cameras --policy hold --output /tmp/shore-a-idle
python run.py --device cpu --physics-device cpu --no-cameras --policy repeat --output /tmp/shore-a-repeat
```

## Native evidence

The filmed scripted trial reached 1.0 after 745 actions. The tool read 58.5, 58.8, 59.1, 59.4 and 58.9 Shore A across five qualified contacts. The full-duration idle control scored 0.0. A physical test that lifted and reseated the gauge five times at the same site scored 0.2.

## References

- ASTM international defines the rubber durometer-hardness test method. [ASTM D2240 scope](https://store.astm.org/standards/d2240)
- A testing laboratory describes firm contact followed by the one-second reading and specimen preparation. [Intertek Shore hardness procedure](https://www.intertek.com/polymers-plastics/testlopedia/shore-hardness-astm-d2240/)
- The five positions and 6 mm separation are used in documented production testing. [Eland Cables laboratory method](https://www.elandcables.com/the-cable-lab/independent-cable-testing/shore-a-hardness-cable-testing)
- An off-the-shelf digital example with the same stroke costs around US$150. [PCE-DD-A handheld tester](https://www.pce-instruments.com/us/measuring-instruments/test-meters/durometer-shore-durometer-pce-instruments-durometer-pce-dd-a-shore-a-det_2131527.htm)
- The instrument mechanism and penetration scale follow commercial Shore A gauges. [Wallace handheld durometers](https://wallaceinstruments.com/products/hardness/handheld-durometers)
- The seal design reference explains how rubber hardness affects pressure resistance and compressive load. [Parker O-Ring Handbook](https://www.parker.com/literature/O-Ring%20Division%20Literature/ORD%205700%20Parker_O-Ring_Handbook.pdf)

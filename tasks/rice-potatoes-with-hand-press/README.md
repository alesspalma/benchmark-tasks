# Rice cooked potato with a hand press

Load a loose potato portion into the open basket of a handheld ricer and force it through the perforated base into the mixing bowl. The reference steadies the lower handle with one gripper while the other squeezes the upper handle. The swivel ram follows the lever and aligns as it enters the basket, and the food yields under load. The result is a bowl of short riced-potato strands and a tool at rest. [OXO Stainless Steel Potato Ricer](https://www.oxo.com/potato-ricer.html) · [GEFU FORCE ONE hand press](https://www.gefu.com/en/Potato-juice-and-speatzle-press-FORCE-ONE/13110)

## Equipment and reset

Place the two YAM arms 610 mm apart with base height 750 mm. Use a stable bench with a working plane 747.5 mm above floor level. Angles and positions below use +X forward, +Y toward the left arm, and +Z up. The origin is at ground level midway between the arms.

| Item | Dimensions and initial position |
| --- | --- |
| Clear mixing bowl | Food-safe polycarbonate or glass, approximately 224 mm inner diameter at the rim and 107 mm tall. Center at (444, −10) mm on the bench. Use a thin nonslip mat underneath. |
| Handheld ricer | Full-size lever press with stainless-steel basket, 76 mm inner diameter, 60 mm deep, and 3–4 mm perforations. Approximate closed length 375 mm including the rim rest. Basket center matches the bowl center. The lower and upper lever grips face −X, and the hinge is 88 mm behind the basket axis. Use the ricer’s integral rim rests with two food-safe silicone rim saddles to keep the ricer centered on the bowl. The rests remain removable. The upper handle starts held open about 72° by light hinge friction or the tool's open detent. |
| Side plate | 110 mm diameter ceramic saucer, centered at (359, 166) mm. |
| Cooked portion | One peeled Russet Burbank portion, 44 ± 1 g after cooking, approximately 44 mm wide and 34 mm tall. Use it at 22 ± 2 °C and center it on the saucer. Fit clean removable food-contact gripper-pad covers. |

Empty and dry the bowl with the ricer unloaded. Place the bowl on the marked mat, rest the ricer astride the rim, open the upper handle, and place a fresh potato portion on the side plate. Bring both arms to the documented YAM ready pose with fingers open. To check the real result, tare the empty bowl before reset and weigh the strands after both hands withdraw. A small tray or piece of removable paper beneath the bowl makes spills easy to collect and weigh.

### Prepare a batch of portions

Peel several Russet Burbank potatoes from the same bag. Slice across the center of each into 34 mm slabs, and cut 44 mm rounds from the slabs with a plain ring cutter. Save one extra round for checking doneness and temperature. Start the rounds together in cold unsalted water, with at least 25 mm of water above them. Bring the water to a boil and reduce to a gentle simmer. Start checking the spare after 12 minutes at a simmer. Drain the batch when a fork passes through the center of the spare without a hard core and a round can still be lifted in one piece. [Idaho Potato Commission preparation guidance](https://idahopotato.com/dr-potato/print/getting-moisture-out-of-mashed-potatoes)

Drain in a colander for two minutes, then spread the rounds in a single layer. Blot surface water and bevel the edges with a clean knife until each round is 44 ± 1 g on a kitchen scale, leaving its full central width and height. Keep portions covered after cooling. Use a probe in the center of the spare to establish the 22 ± 2 °C start temperature, and use the prepared batch within one hour of draining. Use a fresh intact portion per trial and prepare another batch for a longer session.

## Physical score

| Score | Observable outcome |
| --- | --- |
| 0 | At most incidental contact. No potato seated in the basket or extruded into the bowl, or more than 5 g of extruded food spilled. |
| 0.25 | A portion is seated on the bottom of the basket. |
| 0.50 | At least 8 g of pressed strands are in the bowl. |
| 0.75 | At least 25 g of pressed strands are in the bowl. |
| 1.0 | At least 32 g of pressed strands are in the bowl, no more than 2 g spilled, both grippers are at least 55 mm clear of the lever grips, and the upright press rests across the bowl for at least 0.4 s. |

Use the highest completed row unless the spill cap applies. Dropping intact potato into the mixing bowl does not count. Judge the shape and source of the strands from the video. A scale resolves both mass thresholds, and a ruler or setup photograph resolves handle clearance. The scoring accepts either arm assignment.

## Simulation

The flexible food uses a native compression slider with a damped reaction force and residual set. Pressed material discharges through the die when a seated portion is physically compressed by the aligned ram. Falling packets conserve mass until collected or spilled. The cooked potato skin and the die strands visualize that state. The material model approximates food fracture and the deposition footprint. The contacts, joint motion, grasping and support are native.

Use the Isaac Lab–Arena environment pinned by the project. The Inspect Robots embodiment is `potato_ricer`.

```bash
python -m pip install -e . --no-deps
python run.py --device cuda:0 --physics-device cpu --output /tmp/potato-ricer-native
python run.py --device cpu --physics-device cpu --no-cameras --policy hold --output /tmp/potato-ricer-idle
python run.py --device cpu --physics-device cpu --no-cameras --policy no_press --output /tmp/potato-ricer-no-press
```

The default fixed seed is **1415158438**. A trial allows 600 actions at 20 Hz, with 240 Hz physics. The runner writes a setup frame from each native camera, the unedited camera streams, a stepwise score trace, and the validation receipt.

The I2RT YAM arm and gripper assets, including the URDF, meshes, collision hulls and converted USD, are MIT-licensed. The pinned adapter supplies the robot and camera calibration. The bench and kitchen geometry are generated by this task and use its MIT license. [I2RT MIT license in the pinned asset bundle](https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/assets/yam/LICENSE.i2rt) · [Task MIT license](LICENSE)

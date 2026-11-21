# Replace the worn felt cap on a swivel chair glide

> Use the flat-blade tool to remove the worn felt cap from the chair foot.
> Snap on the new felt cap so it fits flush and the glide can still swivel.

A chair's felt wear cap protects the floor and is replaced as a consumable.
The underlying nylon swivel changes angle as the thin blade levers against
the service notch. One arm steadies that swivel while the other pries off the
old cap, seats a new one and withdraws. This follows the repair procedure for
a snap-fit swivel glide from a 15-inch classroom chair. [KI glide-cap repair
instructions, KI-RR-000030][repair] · [KI 15-inch frame and replacement part
diagram][parts]

## Physical setup

Set the polypropylene seat of a full-size 15-inch four-leg chair upside down
on a padded 990 × 1240 mm bench, with its backrest clear over the far edge.
The powder-coated or plated legs are 22.2 mm outer-diameter steel tubing.
Use a chair with removable felt caps on nylon swivel glides, such as KI Ruckus
with glide 10.0702.GFT.X and replacement cap 10.0129.X. The same matched
glide and cap type goes on all four feet. The test foot sits above the near
corner of the seat. Its original swivel stays attached to the tubular leg.
The fitted skirt lip, service notch and felt disc measure about 30 mm across
in the model. Use a visibly abraded cap with an intact rim for reset.
[KI replacement part diagram][parts] · [KI frame and glide specification][spec]

Use a short flat-blade screwdriver or the manufacturer removal tool 10.0138.X.
In the prescribed operation the tip engages the exposed service notch before
levering upward under the cap. Use a firm straight press for seating, or use the compact rubber-faced mallet
to tap the cap in when it needs extra force.
The parts stand presents the fresh cap felt-up and keeps its edge exposed.
[KI glide-cap repair instructions][repair]

World X runs ahead from the midpoint of the robot mounts, and positive Y is
toward the left arm. The robot bases are 610 mm apart and the table is
747.5 mm high.

| Object | Starting features and placement |
| --- | --- |
| Chair | Seat about 403 × 394 mm, centred at X 510 mm, Y +139 mm. Seat pad is 6 mm thick. The service foot centre is X 335 mm, Y −45 mm, Z 1118 mm. |
| Worn cap | Brownish worn felt, nylon snap skirt, approximately 30 mm diameter, already seated on service foot. |
| Fresh cap | Light tan felt, nylon snap skirt, same fit, felt up on stand at X 301 mm, Y −286 mm. |
| Flat-blade tool | Approximately 109 mm long, 20 mm wide moulded handle and 6.5 mm steel blade. On table at X 285 mm, Y −195 mm, tip along +Y. |
| Rubber mallet | Approximately 117 mm long with 30 mm wide and 49 mm deep rubber head. On horizontal handle stand at X 383 mm, Y −334 mm, head toward +Y. |

For reset, seat the old cap by the normal method and centre the unloaded
swivel. The new cap starts separate. Both grippers start open in the
configured ready poses. In real trials check that the cap was made for the
chosen glide and that the guide's removal notch is unobstructed.

## Physical score

- **1.0** when the old cap is off and at least 45 mm away, and the replacement
  remains seated and flush all around after both arms release for 0.6 s. With
  a side photograph or feeler gauge, check that the largest gap is at most
  1.8 mm. Gently pull the rim by hand to confirm it is captured. Rock the
  original swivel to check the cap moves with it and its motion stays free.
- **0.35** for removal when the old cap is at least 45 mm away and the
  replacement has not been retained flush.
- **0.0** otherwise.

Either hand can do either operation. The result is graded at the glide.
A cap held by one gripper against the base does not count as seated.

## Simulation and reproduction

The generated chair, blade, mallet, glide, and two caps have native PhysX
colliders. A free native ±15° swivel axis retains the original glide on the
chair. Bounded elastic forces between matching material points on four rim
sectors represent the flexible skirt and its snap fit. Detents release upon
local separation, so the cap can tilt and peel progressively. The robot
reference uses ordinary arm-joint targets and real tool contact, and does
not reposition any task object. The coefficient models cap retention and
can be compared qualitatively to a real cap through the hand pull check.

Install the task in the approved Isaac Lab-Arena runtime.

```sh
python -m pip install -e . --no-deps
python run.py --device cuda:0 --physics-device cpu --output /tmp/chair-glide-demo
python run.py --device cuda:0 --physics-device cpu --no-cameras --policy hold --output /tmp/chair-glide-hold
python run.py --device cuda:0 --physics-device cpu --no-cameras --policy remove-only --output /tmp/chair-glide-removed
```

The embodiment ID is `chair_glide`. The fixed default random seed is
1415158438. Control is 20 Hz, with 240 Hz physics, and the action limit is
760 steps (38 seconds). The demo runner records the three native 640 × 360
camera views and writes a JSON receipt with the measured score.

The task source and generated objects are under the included MIT license.
The fixture and wearable parts are independently generated dimensions.
The scene uses the pinned YAM adapter with its documented asset licenses.
[YAM asset license][yam-license]

[repair]: https://dp-assets-new.s3.amazonaws.com/assets/78934221/DoniGrazieIntellectWaveRuckusStriveandVersaGlideCapRepairandReplacementInstructions.pdf
[parts]: https://www.schoolsin.com/prod_files/20230306_KIF-Ruckus_4-Leg_Chair-Parts_Diagram.pdf
[spec]: https://bidopportunities.iowa.gov/Home/DownloadContractDocument/a2fa832b-8a9f-4f70-ba43-40e59e3b084b
[yam-license]: https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/README.md#license

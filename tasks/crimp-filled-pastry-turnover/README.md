# Crimp a filled turnover in a hinged pastry press

> Finish folding the press and squeeze it to seal the filled turnover. Reopen the press and leave the sealed turnover resting in the lower well.

The two halves of a four-inch pastry press fold a filled wrapper into a half-moon. During the squeeze, matching ridged faces compress the curved edge into a crimp that keeps the filling inside when the pastry comes out. The robot steadies one tab while folding and squeezing the other. It opens the press so the finished edge can be checked before the turnover is moved to a tray. [Choice use-and-care guide](https://www.webstaurantstore.com/documents/pdf/407mdpresswp_use_and_care_guide_bw.pdf) · [Kindral folding and crimping instructions](https://www.kindralshop.com/us/product/dumpling-press)

## Bench setup

Use two parallel-gripper YAM arms 610 mm apart on a bench with its top 747.5 mm above the floor. X points forward from the arm bases, Y to the left and Z up. Start with the grippers open in their ready poses. The lower die sits between compliant locating pads on the work board. A light hinge detent stages the loaded upper die at a gap of about 46° before the squeeze.

| Object | Construction | Starting location |
|---|---|---|
| Worktop | 990 × 1,230 × 43 mm laminate | Centre X 342 mm, Y 0 mm |
| Work board | 215 × 116 × 32 mm food-safe board with bevelled corners | Centre X 371 mm, Y 0 mm, resting on the worktop |
| Hinged pastry press | 102 mm forming diameter, moulded polypropylene, two semicircular wells, shallow ridged crimp ring and 37 × 46 mm finger tabs. Hinge pin runs along X. | Hinge at X 365 mm, Y 0 mm, 796 mm above the floor. The lower die lies level and the upper die leaves a 46° opening. Lower tab is on +Y. |
| Round wrapper and filling | One 101 mm raw pastry or dumpling wrapper, about 3 mm thick, and a level teaspoon of thick fruit filling. Moisten a narrow ring at the perimeter as needed for this pastry. | Wrapper centred on the partly folded press with filling in the +Y well, at least 8 mm inside the curved edge. |

Prepare a fresh wrapper and fill each test. Check that both leaves pivot freely and their ridged faces meet. Place the lower die between the same board marks, stage the upper die in the opening detent. Wipe the mould dry between trials and remove dough from the grooves.

## Physical scoring

Judge the highest completed outcome in the allowed run. Either arm may perform either role.

| Score | Observable result |
|---|---|
| 0.00 | The wrapper is still open or was displaced before it folded around the filling. |
| 0.25 | The wrapper has folded around the filling, but the curved edges gape or come apart as the upper die is opened. |
| 0.65 | The curved edges have formed a continuous seam, but the press is still closed or a hand is obstructing inspection. |
| 1.00 | The press is reopened within 24° of flat. The ridged seam remains continuous with the filling enclosed. The turnover rests in the lower well and both grippers are clear for 0.6 seconds. |

Inspect the fluted edge from above and from the exposed side under overhead light. A seam with a gap beyond 3 mm, a split that exposes filling, or a turnover displaced from the well cannot earn full score.

## Simulation and reproduction

The mould and dough lobes move around separate PhysX hinges. The thin shell contacts the lower well and the grippers. At the upper die, a force–displacement law represents compliance at the ridged faces and weak adhesion of moist dough. The aligned seam sets after a sustained squeeze. The resulting bounded seam resistance retains the crimp as the upper die is reopened. The figures in the simulation are calibration choices for the surrogate dough. Ingredient extrusion and small local tears are outside the material model.

Use the approved Isaac Lab–Arena and Inspect Robots runtime. The embodiment is `turnover_press`, with 20 Hz position actions and 240 Hz PhysX contact substeps. The fixed seed is 1415158438. The run limit is 720 actions, or 36 seconds. The reference uses the measured upper die pose and sends ordinary position actions. The policy observation exposes RGB and robot joint positions. The scorer uses the physical measurements to apply the same checks described above.

```sh
python -m pip install -e . --no-deps
python run.py --device cuda:0 --physics-device cpu --output /tmp/turnover-reference
python run.py --no-cameras --policy hold --output /tmp/turnover-idle
python run.py --no-cameras --policy no_crimp --output /tmp/turnover-no-crimp
```

The station solids and the turnover geometry are generated in `turnover_press/builder.py` under the accompanying MIT license. The YAM geometry and camera calibration come from the pinned adapter in `pyproject.toml`.

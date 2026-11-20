# Bleed a cold panel radiator and catch the first steady water

Vent the trapped air in a domestic panel radiator and close the vent after a small, steady water discharge. A square bleed key turns the valve. Hold a cup beside the radiator under the side outlet so the first water does not fall on the work surface. This is the normal bleed-and-close procedure for a cold, isolated radiator. [Vaillant homeowner procedure](https://www.vaillant.co.uk/advice/heating-tips/home-advice/how-to-bleed-a-radiator/) · [Viessmann procedure](https://www.viessmann.co.uk/en/heating-advice/boilers/how-to-bleed-a-radiator.html)

## What to build

The episode uses a 1,100 × 1,250 mm worktop at 747.5 mm and two YAM bases 610 mm apart. The positive X axis points forward from the base line. The positive Y axis points to the left robot. An overview and wrist camera on each arm observe the worktop.

| Item | Physical build and reset |
| --- | --- |
| Radiator | Use a 500 × 300 mm single-panel steel radiator, such as Stelrad Vita Compact K1 76301105. Confirm the actual vent port rather than locating it from the case edge. Secure the unloaded panel vertically with its original brackets to a braced frame fixed to the bench. Align the upper vent axis at X = 352 mm, Y = −34 mm, 284 mm above the tabletop. The long dimension extends in +X away from the working end. |
| Vent and key | Fit the matching ½-inch manual air vent at the upper port, with its outlet down and toward +Y and its square drive facing −X. Start closed with a 5 mm square brass butterfly bleed key fully seated under its own weight. The simulation key is 43 mm across the wings and projects approximately 38 mm from the vent axis reference. An equivalent matching key and valve pair may be used. |
| Cup | Open 250 mL clear or light polypropylene measuring cup, approximately Ø80 × 75 mm internally, with a side thumb grip, centred at [310, 187] mm. Face its grip toward +Y. Start dry, empty and on the table. Put the cup on a scale readable to 1 g for the final check. |
| Water supply | Use a 1 L open header tank with a ½-inch bottom feed and a shutoff/service tee at the radiator's lower port. Put its water surface 450 ± 25 mm above the vent outlet. Cap the two unused radiator ports. The feed and reservoir sit behind the upper side of the radiator and outside the robot workspace. |
| Spill witness | Place a fine reusable absorbent mat below the working vent and cup area. Dry and weigh it before each test. Simulated floor loss is counted at the work surface. |

The panel and valve are ordinary hardware. The trial begins at bleeding, with the loose stock key already seated as it would be just before cracking the valve. The cold free-surface tank provides hydrostatic head without a circulating pump. Use only a dedicated unheated setup with clean room-temperature water. The tabletop, structural frame and radiator stay fixed. Both grippers start open and clear. [Radiator dimensions and code](https://www.stelrad.com/wp-content/uploads/2014/09/Vita-Compact_Web-1.pdf) · [Vent and blanking plugs](https://www.screwfix.com/p/essentials-radiator-vents-blanking-plugs-nickel-2-pack/40972) · [Pipe connections](https://www.stelrad.com/wp-content/uploads/2015/04/Compact-H_Web-1.pdf) · [Stock bleed key, 2058J](https://monument-tools.com/wp-content/uploads/2018/01/Monument-Tools-Cataogue-2018.pdf)

## Physical reset

1. Fill the isolated radiator from below with the header tank. Crack the top bleed valve until air stops escaping and water flows steadily. Close it gently.
2. Close the header shutoff. Open the top valve to atmosphere and withdraw **42 ± 2 mL** of water at the bottom service tee with a graduated 60 mL syringe. Close the service tee and the vent. Reopen the header shutoff. Air compresses slightly under the 450 mm head, leaving approximately 40 mL of trapped air to vent.
3. Restore the clean catch cup and the dry spill mat. Seat the key squarely with the valve closed and its longer wing axis horizontal. Check that a gentle closing touch leaves a dry outlet and that no other connection leaks. Clean tap water gives a reusable reset. A few drops of washable food colouring in the header tank can improve the visibility of the outlet stream.

## Physical score

Use the overhead and wrist videos, a 1 g scale and the valve's closed witness line. Either arm may turn the key or hold the cup. Score zero if the spill mat gains more than 2 g, the cup contains more than 22 g of water, or the sequence never reaches continuous water after the initial air discharge. Score 0.4 for a cleared air pocket with 3–22 g in the cup and an open valve. Score 0.75 when the same discharge has been safely caught and the valve is shut. Score 1.0 when the latter condition and a dry outlet persist for 1 s, the cup is upright and supported by the bench, the key is seated or on the bench, and neither gripper is holding the cup or the key. A dry closed valve while the air pocket is still present scores zero. When shut, the close mark is within 6° of the starting line and no liquid exits.

The native contact model uses a matching square socket and square valve stem. A passive revolute valve has a seating stop. A reduced-order flow model consumes a finite air pocket using the measured valve opening and budgets water through the same orifice after the air clears. A water trajectory can enter the cup only through its actual free mouth. A tilted cup retains only the volume below the lowest point of its lip, so pouring out the captured water counts as spill. The cup and key are loose bodies. The water indicator is a visualization of the accumulated volume. The pretrial header and water are modeled by aggregate flow rather than a fluid solver.

## Run and score

Use the accepted-license Isaac Lab Arena runtime and the adapter pinned by `pyproject.toml`. Seed is `1415158438`, action rate is 20 Hz and the limit is 620 actions (31 s). Run with the project environment active:

```bash
python -m pip install -e . --no-deps
python run.py --device cuda:0 --physics-device cpu --output /tmp/radiator-demo
python run.py --device cuda:0 --physics-device cpu --no-cameras --policy hold --output /tmp/radiator-hold
python run.py --device cuda:0 --physics-device cpu --no-cameras --policy no-catch --output /tmp/radiator-no-catch
```

The radiator package uses `radiator_bleed` as its Inspect Robots embodiment and `radiator_bleed_scripted` as its local reference policy. The reference acts through standard joint commands. The hold control remains at the start pose. The no-catch control turns the real valve with the empty cup left in its starting location. Every trial writes a score receipt and a step trace. The demonstration records all initial and action frames on the three native 640 × 360 cameras at 20 fps.

## Assets and licenses

| Asset | License and provenance |
| --- | --- |
| Radiator, square valve, key, bench, legs, floor, lights, cup and water indicator | Original geometry and lighting in `radiator_bleed/builder.py` and `radiator_bleed/environment.py`. MIT, see `LICENSE`. |
| YAM robots and camera calibration | Geometry and calibration through the [pinned YAM adapter](https://github.com/kairosiann/yam-isaac-lab-arena/tree/0059d02a1a91bc6834c1d0846022891df0c78730). [YAM asset terms](https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/assets/yam/LICENSE.i2rt) · [camera calibration](https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc126418ea5e74ac567133d/LICENSE). |

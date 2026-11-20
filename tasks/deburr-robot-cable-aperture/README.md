# Deburr the cable aperture in a robot tool plate

Finish the upper edge of a robot end-effector mounting plate before inserting its cable sleeve. The opening follows two straight runs and two round ends. One gripper braces the loose plate while the other draws the S10 swivel blade clockwise through the changing contour under cutting load. [NOGA S10 swivel blade](https://www.noga.com/product/s10-blade/) · [End-effector mounting plates and cable pass-through](https://suprakgroup.com/product-category/eoat-conveyors/eoat-robot-kits)

## Physical fixture and reset

Set two YAM bases 610 mm apart on 750 mm pedestals. Positive X points away from the bases, positive Y toward the left arm. Measure plate and tool centres from the midpoint between the bases. Use a 1100 × 1220 mm hardwood workbench with a top 747.5 mm above the floor.

| Part | Dimensions and reset |
| --- | --- |
| Robot tool plate | 154 × 168 × 10 mm 6061 aluminum. The upper rim is burr-side up. The opening is 88 mm long along X, 42 mm wide along Y, and offset 40 mm toward negative Y. Its rounded ends have a 21 mm radius. The four mounting holes have 8.4 mm bores and 13 mm counterbores. Plate centre (373, −89) mm. |
| Plate rests | Pair of 20 × 76 × 38 mm steel parallels, centred at (311, −76) and (435, −76) mm. Put the plate flat on both. |
| Deburring tool | NOGA NG1000-compatible nylon handle and BS1010/S10 swivel blade. The modeled handle is 24 mm across and the shank 3.2 mm across. At reset, the cutting toe is inside the stand, handle up. |
| Tool stand | Metal or hard polymer blade guard, approximately 64 mm across, centred at (303, −365) mm. A 45 mm high collar loosely supports the tool upright and contains the hooked blade. |

Export the dimensioned [tool plate and slip-fit gauge](fixture/cable_plate.scad) to 2D DXF or STL. Use identical CNC profiles cut from the same thickness of stock. Leave the upper aperture edge as machined in the final pass. Finish the outside, lower aperture edge and mounting holes before the trial. Inspect the upper lip with a 10× loupe against a 0.15 mm feeler leaf and use a fresh coupon exhibiting a continuous raised burr approximately 0.10–0.25 mm high. Machining several coupons in one setup makes this consumable repeatable. Clean them of loose chips before setting them out. Keep the blade guarded between runs and clear of the gripper pads during a run. Brush chips out of the rests before reusing them.

## Scoring

Use a 10× loupe and draw a fresh cotton swab lightly along the upper edge. Measure the contiguous snag-free lengths on both straight runs and both round ends using a flexible printed metric strip. A raw or torn area counts as uncleared. The score is 0.8 times the fraction of the traced circumference cleared. If there is a deep tear or gouge into the wall, the score is 0.2 times that fraction. Where the visual inspection is ambiguous, the sleeve gauge printed from the supplied file must pass through and sit flush without forced insertion.

For 1.0, clear at least 98% of the circumference and leave the wall intact. Park the plate flat across both parallels and place the tool handle-up in the guard. The setup camera should show both released parts at rest and the fingers clear for three quarters of a second.

The simulation shows a warm witness lip while uncut material remains. Small sections clear when the native cutting nose travels clockwise along the rim at a working angle. Displacement of the blade's axial compliance estimates contact load. A scoring stroke requires at least 2.4 N on consecutive action samples and tangential edge motion. More than 23 N at the rim represents a heavy gouge. These thresholds estimate the resistance of a light aluminum edge cut. The edge geometry and loads act through native collisions. The witness lip displays the resulting material removal. Grade the physical trial from the edge finish and fit.

## Native evaluation

Install the task from this directory in the pinned Isaac Lab-Arena environment. The embodiment is `cable_deburr`. The reference reads the current cutting nose, plate and joint states and sends ordinary bimanual YAM joint actions. It uses overlapping clockwise strokes, lifting and reseating the blade at a different angle around the rim. Scene physics uses 240 Hz contact steps at 20 Hz control. The setup is fixed, seed 1415158438. The 2200-action limit is 110 seconds.

```bash
python -m pip install -e . --no-deps
python run.py --device cuda:0 --physics-device cpu --output /tmp/deburr-demo
python run.py --no-cameras --policy hold --output /tmp/deburr-hold
python run.py --no-cameras --policy air-pass --output /tmp/deburr-air
python run.py --no-cameras --policy partial --output /tmp/deburr-partial
```

The setup PNG and three camera feeds come from the simulation. The runner writes a chronological H.264 composition and JSON line trace with actions, rigid link state and per-step scores.

The scene geometry is original and MIT-licensed with this task. The YAM and cameras come from the pinned adapter. [YAM asset license](https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/assets/yam/LICENSE.i2rt) · [Camera asset license](https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc126418ea5e74ac567133d/LICENSE)

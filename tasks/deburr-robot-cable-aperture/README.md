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

### Make the burr-bearing plates in batches

Export the [dimensioned plate profile and sleeve gauge](fixture/cable_plate.scad). Cut identical blanks from 10 mm mill-finish 6061-T6 plate drops. Saw the outside and use one drilling template for the four mounting holes and counterbores. Cut the oblong openings on a CNC mill or aluminum-capable router using one fixture and one contour program for the batch. A CNC service such as Xometry can quote the supplied part. Specify the single edge to leave as-cut on the drawing because its standard service breaks sharp edges. [CNC stock and edge conditions](https://www.xometry.com/capabilities/cnc-machining-service/)

1. Set the intended upper face down on parallels with clearance under the complete aperture. Rough the slot through from the other face, leaving 0.20 mm of radial stock on both straight sides and both ends. With a sharp aluminum-cutting end mill, finish the full oblong in a continuous pass with the cutter extending about 0.3 mm past the unsupported exit face. Use the mill's approved 6061 speeds and feeds. The exit lip is the working burr. Keep the same tool, stock lot, workholding and program once a pilot coupon passes inspection. [Burr formation during milling of 6061-T6](https://espace2.etsmtl.ca/id/eprint/15144/)
2. Turn the plate exit-face up. Finish the lower aperture edge, the exterior and the mounting holes. Clean loose chips off the plate and look for a continuous attached raised lip all the way around the upper edge. At the midpoint and near both ends of each straight, and at the apex of each semicircle, bridge the lip with a narrow straightedge resting at the burr crest. Compare the clearance to the adjacent flat plate face with 0.10 and 0.25 mm feeler leaves. Keep a coupon only if the 0.10 mm leaf fits and the 0.25 mm leaf does not at all eight sites. Confirm continuity with a 10× loupe or a single light pass with a fresh cotton swab around the whole edge. Reject coupons with a loose flap, an existing wall gouge or a broken section of burr. A reference photo with the caliper or feeler in view records the accepted starting edge.
3. Set out one screened plate per trial with the burr side up. Use a removable chip mat under the parallels and shield the arm-base cable openings and electronics outside the working envelope. Keep the blade guarded between trials. Brush or vacuum loose chips off the plate, mat, parallels and gripper pads before the next reset.

## Scoring

Use a 10× loupe and draw a fresh cotton swab lightly along the upper edge. Measure the contiguous snag-free lengths on both straight runs and both round ends using a flexible metric strip. A raw or torn area that snags the swab counts as uncleared. The score is 0.8 times the fraction of the traced circumference cleared. Where clearance is ambiguous, the sleeve gauge printed from the supplied file must pass through and sit flush without forced insertion.

A wall gouge is a new recess at least 0.20 mm deep normal to the wall that reaches at least 0.50 mm below the upper face and extends at least 2.0 mm along the rim. Use the reset photo to distinguish old marks. Inspect suspect spots with the loupe and a millimetre rule. Hold a narrow straightedge vertically against the intact wall immediately below the damaged area, on the same line up the aperture wall. The 0.20 mm feeler must enter between the reference edge and the recessed wall while the edge still contacts the intact surface below. This works on both the straight sides and the curved ends because the aperture walls are vertical. If any new recess meets all three dimensions, multiply the fraction cleared by 0.2 and withhold full credit.

For 1.0, clear at least 98% of the circumference and leave the wall intact. Park the plate flat across both parallels and place the tool handle-up in the guard. The setup camera should show both released parts at rest and the fingers clear for three quarters of a second.

The simulation shows a warm witness lip while uncut material remains. Small sections clear when the native cutting nose travels clockwise along the rim at a working angle. Displacement of the blade's axial compliance estimates contact load. A scoring stroke requires at least 2.4 N on consecutive action samples and tangential edge motion. The simulator uses a 23 N rim-load proxy for a heavy gouge. The edge geometry and loads act through native collisions. The witness lip displays the resulting material removal. Grade the physical trial using the measured recess, edge finish and fit.

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

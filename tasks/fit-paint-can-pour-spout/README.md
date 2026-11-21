# Fit a snap-on paint-can spout and pour into a roller tray

Fit a vented pour lid onto a standard gallon metal paint can, fill the roller tray to its blue line, and return the can upright. Aligning the lid and pressing it past its seal are essential before tipping the can. The rim must remain sealed under the pour load, and the filled tray must stay beneath the spout until the stream stops. [Shur-Line paint can lids](https://www.shurline.com/products/detail/prodid/87/item/paint-can-lids) · [Handy Snap Cap](https://www.sherwin-williams.com/homeowners/products/handy-snap-cap/651352130)

## Setup and reset

- Use an open 164 mm outside diameter × 188 mm tall, one-gallon tinplate paint can with a rolled upper bead and a wire bail with a protective hand grip. Put 410 mL of washable magenta water-based paint or tinted paint of similar viscosity in the can. Fold the bail against its open stop so the grip clears the rim. Fit a removable lower support grip opposite the bail for bimanual control when tipping. The simulated loaded can starts at 640 g and the separate lid at 110 g. [Auxiliary paint-can handle](https://patents.google.com/patent/US6266849B1/en)
- Stage a matching 177 mm snap-on pour-and-store lid with its spout extended and vent open. The lid has an elastomeric skirt and a 30 mm-wide spout. Aim the spout toward the front of the bench in its rest. Lift the detached lid from the rest, seat it around the full metal rim, then tip the can toward the tray.
- Use a rigid gray roller tray of about 215 × 320 mm on the bench. Its 198 × 208 mm flat well has 34 mm of free wall above the bottom and a blue line 4.4 mm above the floor, equivalent to about 181 mL. The model has a ribbed ramp on the far side. Place a thin disposable catch sheet beneath the tray when using real paint.
- Use a 747.5 mm tabletop. The frame origin is horizontal midway between YAM arm bases, with +X toward the work area and +Y toward the left arm. Set can base center at X 315, Y −40 mm on a 210 × 205 × 6 mm rubber mat centered at X 337, Y −40 mm. The lid center is at X 350, Y 244 mm on a 12 mm-high shallow puck. Set the tray well bounds to X 24–222 mm and Y −192–16 mm. Both arms start open and above their respective sides.
- After each trial, drain and wipe the tray and spout, detach the pour lid, measure 410 mL of paint into the open can, and return the can, lid, bail and roller tray to the setup positions. Leave the vent open. Reuse the lid, tray, and washable test paint between episodes.

## Scoring

Verify the rolled bead is covered by the lid skirt at four equally spaced points. A continuous seat has no gap greater than 3 mm at those points and remains in place at the end of the trial. Film the spout and can rim while tipping. Confirm the amount in the tray with a graduated cylinder or a calibrated volume line. Collect rim leaks and paint outside the tray using a catch sheet and a scale or measuring vessel. Either arm may seat the lid or handle the can.

| Score | Bench outcome |
| --- | --- |
| 0.0 | Lid not seated, more than 8 mL outside the tray, more than 5 mL escaping the rim, or more than 210 mL in the tray. |
| 0.3 | Lid seated with less than 50 mL transferred through its mouth, within spill limits. |
| 0.6 | At least 50 mL and less than 145 mL transferred through the mouth, within limits. |
| 0.8 | 145–210 mL transferred through the mouth and no more than 210 mL in the tray, within limits. |
| 1.0 | The 0.8 conditions hold, the stream has stopped, and the sealed can stays on its mat within 7° of upright for 0.55 s after both hands clear the grips by at least 35 mm. The can's base remains within 50 mm of its start in plan view and moves at most 2 mm per 50 ms sample. |

The can and lid have native rigid-body contact. Four bounded force pairs model the pressure needed to flex the skirt over the bead and retain the lid after engagement. The same measured gap permits leaks when an incomplete seal is tipped. A horizontal gravity surface in the cylindrical interior determines hydrostatic head, and emitted paint parcels move ballistically into the tray or onto the catch sheet. The scene renders that state directly. The reference paint mass is included in the fixed can body mass. Fluid viscosity and splatter are reduced-order approximations.

## Run

```sh
python -m pip install -e . --no-deps
python run.py --device cuda:0 --physics-device cpu --output /tmp/paint-spout-demo
python run.py --policy hold --device cpu --physics-device cpu --no-cameras --output /tmp/paint-spout-hold
python run.py --policy seal_only --device cpu --physics-device cpu --no-cameras --output /tmp/paint-spout-seal-only
python run.py --policy open_pour --device cpu --physics-device cpu --no-cameras --output /tmp/paint-spout-open-pour
```

The Inspect Robots embodiment is `paint_pour`. A fixed starting layout uses seed `1415158438`. Maximum trial length is 740 actions, 37 seconds at 20 Hz. The normal observations are RGB overview and wrist cameras and joint positions. The reference runner reads can and lid poses to align the skirt and plan bimanual grasp transforms. It moves both arms through their ordinary joint and gripper commands. Native geometry is built from this task's licensed source using the documented dimensions. The YAM dependency supplies the robot and standard station assets under their [asset terms](https://github.com/kairosiann/yam-isaac-lab-arena/tree/0059d02a1a91bc6834c1d0846022891df0c78730).

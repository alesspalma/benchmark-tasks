# Prime a Büchner funnel and capture the solids

> Seat the filter paper in the Büchner funnel and wet it with the rinse water. Filter the bead suspension so the beads stay on the paper and the water collects in the flask. Return both beakers upright.

Vacuum filtration isolates a solid from a liquid in a standard laboratory workflow. The disk needs to lie flat across the perforated plate and seal against it when wetted. An open edge lets solids reach the receiving flask. The robot handles the dry disk with one arm, wets it, and meters the slurry with the other so the liquid drains without carrying the solids around the paper. [MIT Digital Lab Techniques Manual, vacuum filtration, pages 3–4](https://ocw.mit.edu/courses/res-5-0001-digital-lab-techniques-manual-spring-2007/df987730c67ad1302545ed607212c771_P-UBuAFxJiA.pdf) · [University of York laboratory procedure](https://chemtl.york.ac.uk/techniques/reaction-techniques/filtration/vacuum/buchner-filtration) · [University of Wisconsin laboratory guidance on seating and wetting](https://www2.chem.wisc.edu/deptfiles/genchem/lab/labdocs/modules/vacfilt/vacfiltplace.htm)

## Physical setup

Use the YAM bench with arm mounts 610 mm apart and worktop 747.5 mm above the floor. Coordinates are millimetres forward and to the left of the midpoint between the two bases. The paper is laid flat on a clean circular stand with 8 mm overhang to each side so that gripper jaws can reach its edges.

| Object | Construction and position |
| --- | --- |
| Funnel and plate | Porcelain Büchner funnel, 102 mm outer mouth diameter, 76 mm internal floor diameter and 39 mm bowl depth. A level plate has 3.6 mm perforations on 9.5 mm centres with outer hole centres within 30 mm of the axis. The plate is 175 mm above the worktop. Funnel axis at (345, 8). |
| Receiving flask | Transparent 250 mL thick-walled borosilicate side-arm flask, approximately 100 mm wide and 150 mm high. The empty flask sits on the stand base. A steel stand and neck clamp hold it upright. The neck carries a black rubber funnel adapter. |
| Vacuum supply | Flexible 16 mm outside-diameter rubber tube leaves the glass sidearm and runs to an active regulated laboratory vacuum at the rear of the bench. Start with a gentle continuous vacuum suitable for the chosen paper. The supply and clamp are fitted before the episode. |
| Paper | One 70 mm Grade 1 circular cellulose filter, dry and intact, centred on a clean 54 mm diameter plastic stand 29.5 mm above the worktop at (330, 193). A typical real filter is about 0.2 mm thick. |
| Rinse beaker | Small transparent polypropylene spouted beaker, 56 mm bowl diameter and 51 mm high, with a D handle facing left. Fill with 17 mL of blue-tinted water. Centred at (320, 282). |
| Suspension beaker | Transparent polypropylene spouted beaker, 56 mm bowl diameter and 51 mm high, with a D handle facing right. Fill with 55 mL of blue-tinted water and 36 readily visible 1.8–2.0 mm orange polymer beads. Use beads denser than water which flow freely from the lip when tipped. Centred at (320, −282). |

Both grippers start open in the normal ready pose. The funnel plate, receiving flask, and bench are dry. Between trials, clear the funnel and flask, recover and count the reusable beads, refill both beakers, and put a fresh dry disk on its stand. Reset the grippers and keep the funnel, seal, clamp and vacuum tubing in their starting arrangement. For the physical trial use only clean water, food colouring and inert beads. Use an intact flask rated for vacuum, a secured neck clamp, and a regulated supply.

## Physical scoring

Observe the paper from above to verify it lies flat across the perforations without a folded edge. Watch its colour darken as it is wetted and check that it stays flat while the suspension goes in. Photograph the retained orange beads and inspect the transparent flask for stray beads. For the volume check, use the flask's marks or a graduated cylinder. Note the amount in the flask after the rinse and subtract it from the final total to measure the filtrate from the suspension. A weighable absorbent pad under the fixture can measure bench spill, using 1 g per mL of water.

- **0.00** if no paper lies flat over the plate at the finish.
- **0.25** if the disk covers the perforations but is still dry, the collection flask contains one or more beads, or bench spill is over 3 mL.
- **0.50** if the disk lies flat and wet, the flask has no visible beads, and bench spill is at most 3 mL.
- **1.00** if the latter conditions hold, at least 50 mL of liquid from the suspension is in the flask, at least 34 of 36 beads are on the paper, and both beakers are upright within 50 mm of their starting marks with both grippers clear of the funnel and beakers for 0.5 s.

Award the highest applicable score. Either arm may perform either role. Rinse water alone cannot reach full score.

## Native execution

Install into the pinned Python 3.12 Isaac Lab Arena environment. Use native CPU PhysX contact on the paper, the beakers and the station. Default seed 1415158438, 240 Hz physics, and 20 Hz actions. The maximum trial is 1,200 steps or 60 seconds. A reference process emits three native 640 × 360 views, a photo from each view, an action trace and a score receipt. `--no-cameras` skips rendering for diagnostics.

```sh
python -m pip install -e . --no-deps
python run.py --device cuda:0 --physics-device cpu --output /tmp/buchner-reference
python run.py --device cuda:0 --physics-device cpu --no-cameras --policy hold --output /tmp/buchner-idle
python run.py --device cuda:0 --physics-device cpu --no-cameras --policy paper_only --output /tmp/buchner-paper-only
```

The native reference reached 1.0 at action 1,117. All 36 beads remained on the paper while 55 mL of suspension water and 17 mL of rinse collected in the flask. The complete 1,200-action idle and paper-only controls scored 0.00 and 0.25.

Inspect Robots entry point `buchner_filter`. Its observations are three native cameras and ordinary joint positions. The reference uses taught Cartesian poses, measured grasps for registration, and ordinary joint-position actions. The wet paper uses a thin rigid native collision proxy. An explicitly volume-conserving model reads each native beaker's tilt, computes overflow from its lip, and follows the liquid towards the perforated plate. Wetting closes the modeled edge bypass only while the paper is flat and centred. A steady vacuum drains the funnel to the receiving flask. Bead retention and bypass are conservative concentration transport, with visible whole-bead markers. The finite packet and wetting model represents the work of the vacuum and paper without resolving individual droplets, pores, or flexible wet fibres.

All task geometry is authored procedurally under the included MIT license. The unchanged YAM robot and collision hulls come from the pinned adapter. Attribution for the original robot and cameras is in the [YAM asset license](https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/assets/yam/LICENSE.i2rt), the [adapter station](https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/station.py), and the [YAMLAb camera calibration license](https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc126418ea5e74ac567133d/LICENSE).

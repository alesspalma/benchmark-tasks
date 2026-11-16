# Collect a granule sample with a bottom-discharge probe

The left arm holds an upright sampler. The right arm opens the **blue side shutter** at depth in a granule tub, then closes it to retain the sample. Above the receiving jar, it pulls the **orange bottom slide horizontally**: the floor moves aside and the sample falls vertically. The intake shutter remains closed throughout discharge. Both shutters close before the tool returns to its cradle. There is no suction or pumping.

The fresh native demonstration scores **1.0**, with **18.72 g of eligible sample delivered**, in **2,355 actions / 117.75 seconds**. The probe is closed, supported and released; all captured grains are delivered and none remain inside. Its matching Dream-1 evaluation remains pending. Nothing has been uploaded or submitted.

## Operating principle

Industrial quality-control staff collect samples from bulk incoming material. [Bürkle EasySampler](https://www.buerkle.de/en/easysampler-sampler-for-solids_p5360-1000) has separate capture and tip-discharge controls, allowing upright emptying directly into a vessel. [Linear spring-return samplers](https://www.buerkle.de/en/powder-sampler-mini_p5346-3010) motivate the intake shutter. This apparatus combines those principles in original enlarged rectangular geometry for coarse polymer calibration granules; it is not an OEM replica. Category: **Impact on jobs**.

The reference exposes the side port while buried and sweeps it 24 mm through the material. After releasing the blue tab it parks the right hand clear, then withdraws vertically over the source; the passive shutters seat as the head clears loose granules. Both are at their closed stops during the subsequent transfer and the intake stays closed during bottom discharge. The blue GateSlide travels 0–50 mm along +Z, with a constant 25 N/m spring, rest point −20 mm, damping 2 N·s/m and joint friction 0.03. The independent orange BottomSlide travels 0–58 mm along −X, with a constant 20 N/m spring, rest point −50 mm, identical damping and friction. Robot contact supplies all opening strokes. The native joints constrain the real sliding parts; their springs, limits, contact properties and collision geometry remain constant throughout an episode. No fixture motors, switched locks, object state writes or generated grain motions are used.

The controller reads elapsed time and ordinary measured robot joints. Body poses, grain identities and the score are exclusive to read-only measurement/scoring.

## Fixed setup

SI units. World +X extends away from the arm bases, +Y toward the left arm. Table top z=0.7475 m. The pinned YAM adapter supplies the standard calibrated arms, parallel grippers, table, floor, lights and three cameras.

- **Probe frame:** nominal aluminum and polymer, 300 mm high, 280 g. The head is 54×54×54 mm, with 48 mm clear width and a +X side port spanning local z=4–50 mm. A sliding floor has its upper face at z=4 mm and a 7 mm closing-edge bevel. A real colliding 3 mm transparent polycarbonate back wall permits inspection. A sloped roof sheds exterior granules. The hollow 18 mm square stem carries a 24×76×18 mm support grip centred at local (−10,52,255) mm; grasp point (−10,70,255) mm.
- **Blue intake assembly:** nominal aluminum/polymer, 70 g. Shutter 2.5×54×54 mm at x=28.75 mm; 6 mm square guide rod, two linear guide bearings, crossbars and 60×34×18 mm tab centred at (−50,−80,220) mm. Pulling the tab upward admits granules laterally.
- **Orange discharge assembly:** nominal aluminum/polymer, 65 g. Floor 54×54×4 mm with a 7 mm bevel on its closing edge, supported by upper/lower side rails with 1 mm running clearance; 6 mm square external rod and crossbars connect it directly to a 60×34×18 mm tab at (−105,−80,130) mm. Pulling horizontally toward −X moves the floor aside. The sample exits through the bottom rather than through the intake.
- **Storage:** probe origin (0.43,0.38,0.7535) m; polymer cradle with 116×116×6 mm foot, 30 mm walls and 100 mm clear opening.
- **Source:** fixed polymer tub, 105×105 mm inside, 145 mm walls, 3 mm thick; floor reference (0.43,0.02,0.7475) m.
- **Granules:** 1,960 separate native rigid spheres, 6.8 mm diameter, 0.18 g each, totalling 352.8 g. Reset lattice 14×14×10 at 7.1 mm pitch, then settling. Nominal static/dynamic friction 0.30/0.20, restitution zero.
- **Jar:** fixed polymer cup, 110×110 mm inside, 95 mm walls, 3 mm thick; floor reference (0.35,0.15,0.7475) m. The sampler origin is 113 mm above that reference for discharge, placing the sliding floor above the rim and the vertical stream over the opening.
- **Secondary containment:** fixed polymer tray, 520×620 mm inside, 50 mm walls and 8 mm base, centred at (0.40,0.16) m. Exterior granules caught here do not count toward delivered mass.

Metal probe contact friction is nominal 0.35/0.25; the three rubber grips use 1.20/1.00; fixtures 0.65/0.50. These are reconstruction parameters for this original apparatus. setup-layout.json records the fixed reset and calibrated station; builder.py and scene-inventory.json enumerate custom solids; SOURCE.json records package pins, licensing and primary references. The bottom slide's rails extend the lower head width to 64 mm.

## Run and reproduce

Use Python 3.12 and the official Robocurve Isaac Lab-Arena environment, with NVIDIA's license acceptance configured. From the task directory:

```bash
uv pip install --python "$(command -v python)" -e .
python -m pytest -q tests
python check_assets.py --output /tmp/sampler-assets
python check_plan.py
python run.py --policy demo --device cuda:0 --physics-device cpu \
  --width 640 --height 360 --max-steps 2800 --seed 1415158438 \
  --output /tmp/sampler-native
```

Each output directory must be new. Modes `hold`, `insert`, `open`, `retain` and `deliver` stop at intermediate substeps. Control is 20 Hz; physics 240 Hz with continuous collision detection for the granules. Budget: 2,800 actions / 140 seconds. Reset restores all parts and grains, zeros both slides, initializes READY/open grippers and settles 2.5 seconds. No layout jitter. Inspect seed 0 derives native seed 1415158438. The nominal program is 126 seconds; the demonstrated trial took 334 seconds including startup, with CPU physics and RTX PRO 6000 Blackwell rendering.

Registered evaluation: `--embodiment slot_sampler --policy slot_sampler_scripted --scorer task_score`, fixed jitter 0, 20 Hz and the instruction in config.py. Hosted evaluation substitutes `--policy dream -P model=dreamlabs/dream-1-beta -P rows_per_request=48 -P record=false`, one epoch without reruns. The direct native runner uses the same environment and controller factory.

Demo cameras render natively at **640×360**; the **1280×1080, 20 fps** composite places a 1280×720 overview above two 640×360 wrist views, left then right. Dream-1 uses native 320×176 views and a 640×528 composite. Both H.264/yuv420p recordings preserve every initial/action frame in order. The paid hosted run follows final native validation and review.

## Numeric score

A grain contributes 0.18 g only after entering the open side port from the source, remaining wholly enclosed with **both** slides ≤2 mm open, subsequently crossing the physically uncovered bottom, and settling wholly inside the jar below 15 mm/s. Holding a filled sampler above or inside the jar earns zero delivery. Side discharge does not count as bottom discharge.

Source admission requires an upright head (vertical cosine >0.97), origin below 55 mm above the source reference and within 25 mm in X/Y. Each crossing checks the actual opening and grain radius. Whole-grain chamber membership uses the walls, local z=4 mm floor height and roof with 0.5 mm tolerance. Bottom discharge requires a whole-grain downward crossing below the floor underside while its X/Y footprint fits the physically uncovered aperture. The jar's inner floor is reference+3.5 mm; its inner upper bound is reference+98.5 mm.

- **0:** less than 2 g delivered, or permanent invalidation.
- **0.4:** at least 2 g and less than 5 g delivered at rest.
- **0.7:** at least 5 g delivered at rest.
- **1.0:** at least 5 g plus all corners of the 54 mm chamber inside the 100 mm cradle opening (+0.5 mm tolerance), origin within 5 mm of support height, upright cosine >0.98, both slides ≤2 mm open, probe speeds <15 mm/s and 0.15 rad/s, both grippers >40 mm open and every robot collision body >3 mm clear, for one second.

Any granule below tabletop−5 mm permanently forces zero. Probe collision bounds must stay inside x=[−0.19,0.91] m, y=[−0.65,0.65] m, z=[tabletop−5 mm,2.0 m]. Nonfinite state forces zero. Provenance/bounds are audited at 240 Hz; scoring at 20 Hz. A balance, ruler, inspection pane and video establish corresponding physical outcomes. The simulator only measures grain identities and positions; scoring never creates, deletes or moves material.

# Collect a granule sample with a spring-closed slide probe

The probe stays upright. The left arm holds its support grip while the right arm pulls a separate blue tab straight upward to expose a side port. A permanent spring and gravity close the shutter when the tab is released. As the head lifts clear of the source bed, the spring seats the shutter past grains at the lip. The left arm then transfers the closed probe and lowers the head inside the jar and the right arm reopens the port to discharge. The closed probe then returns to its storage cradle.

The fresh native demonstration scores **1.0**, delivering **24.12 g** in **2,257 actions / 112.85 seconds**, with both hands released, the gate closed and no physics violations. The reference video contains all 2,258 initial/action frames in each of three cameras. A matching Dream-1 evaluation remains pending; nothing has been uploaded or submitted.

## Operating principle

[Spring-return sampling tools](https://www.buerkle.de/en/powder-sampler-mini_p5346-3010) use linear opening and closing. [MicroSampler instructions](https://www.buerkle.de/en/microsampler_p89) describe closed insertion, opening at depth, closed withdrawal and reopening to empty. This original enlarged rectangular head applies that operating principle to coarse polymer calibration granules. The task isolates incoming-material sampling for industrial quality control, in the **Impact on jobs** category.

The reference opens the port while the head intersects the source bed and sweeps it 24 mm through fresh material. Its two handles are on opposite sides and at different heights. The passive GateSlide joint has 0–50 mm travel, a constant 25 N/m spring with its rest point 20 mm beyond the closed stop, damping 2 N·s/m and joint friction 0.03. Robot contact supplies every opening stroke. These parameters remain constant throughout each episode. The controller reads elapsed time and ordinary measured robot joints; body poses, grain positions and score are read only by the scorer.

## Fixed setup

SI units; world +X extends away from the arm bases, +Y toward the left arm. Table top is z=0.7475 m. The pinned YAM adapter supplies the standard calibrated arms, grippers, table, floor, lights and three cameras.

- **Probe frame:** nominal aluminum and polymer, 300 mm high, 280 g. The sample head is 54×54×54 mm, with a 48 mm clear width and a front port spanning local z=4–50 mm. Its floor slopes from 20 mm at the back to 4 mm at the front. A sloping roof sheds exterior particles. A hollow 18 mm square stem carries a 24×76×18 mm support grip centred at local (−10,52,255) mm; the reference grasps at (−10,70,255) mm.
- **Shutter assembly:** nominal aluminum/polymer, 70 g. The blue shutter is 2.5×54×54 mm at local x=28.75 mm. A 6 mm square rod runs through two guide bearings. Its 60×34×18 mm pull tab is centred at local (−50,−80,190) mm, connected by two crossbars. The right gripper closes around the tab, pulls upward, releases fully and withdraws horizontally.
- **Storage:** probe origin (0.43,0.38,0.7535) m; polymer cradle with 116×116×6 mm foot, 30 mm walls and a 100 mm clear opening.
- **Source:** fixed polymer tub, 105×105 mm inner footprint, 145 mm walls, 3 mm thick; floor reference (0.43,0.02,0.7475) m.
- **Granules:** 1,960 individual native rigid spheres, 6.8 mm diameter and 0.18 g each, totalling 352.8 g. Reset lattice is 14×14×10 at 7.1 mm pitch, followed by settling. Nominal static/dynamic friction is 0.30/0.20 and restitution is zero.
- **Jar:** fixed polymer cup, 110×110 mm inside, 95 mm walls, 3 mm thick; floor reference (0.35,0.15,0.7475) m. The sampling head is lowered to an origin 12 mm above table top before its gate opens, keeping the working port inside the jar walls.
- **Secondary containment:** fixed polymer tray, 520×620 mm inside, 50 mm walls and 8 mm base, centred at (0.40,0.16) m. Loose exterior particles caught here do not contribute to the delivered sample mass.

Probe friction is nominal 0.35/0.25; fixture friction is 0.65/0.50. These are reconstruction parameters for this original apparatus. setup-layout.json records the reset and calibrated station. builder.py and scene-inventory.json specify every custom solid; SOURCE.json records package pins, licensing and primary references.

## Run and reproduce

Use Python 3.12 and the official Robocurve Isaac Lab-Arena environment, with NVIDIA's license acceptance configured. From this task folder:

```bash
uv pip install --python "$(command -v python)" -e .
python -m pytest -q tests
python check_assets.py --output /tmp/sampler-assets
python check_plan.py
python run.py --policy demo --device cuda:0 --physics-device cpu \
  --width 640 --height 360 --max-steps 2800 --seed 1415158438 \
  --output /tmp/sampler-native
```

Each output directory must be new. Modes `hold`, `insert`, `open`, `retain` and `deliver` stop after intermediate substeps. Control is 20 Hz; native physics is 240 Hz, with continuous collision detection enabled for the granules throughout. The budget is 2,800 actions / 140 seconds. Reset restores all parts and grains, zeros GateSlide, initializes READY/open grippers and settles 2.5 seconds. There is no layout jitter. Inspect seed 0 derives native seed 1415158438. The demonstrated run took 315 seconds including startup on an RTX PRO 6000 Blackwell GPU with CPU physics.

Registered evaluation uses `--embodiment slot_sampler --policy slot_sampler_scripted --scorer task_score`, fixed jitter 0, 20 Hz and the instruction in config.py. A hosted evaluation substitutes `--policy dream -P model=dreamlabs/dream-1-beta -P rows_per_request=48 -P record=false`, with one epoch and no episode reruns. The current reference uses the same environment and controller factory through the direct native runner.

Demo cameras render natively at **640×360**; the **1280×1080, 20 fps** composite places a 1280×720 overview above two 640×360 wrist views. Dream-1 uses 320×176 native views and a 640×528 composite. Both H.264 recordings preserve every initial/action frame in order. The hosted run follows final native validation and the submission checklist.

## Numeric score

A grain contributes 0.18 g only after it enters the physically open front port from the source, remains wholly inside when the gate closes to ≤2 mm, subsequently exits the reopened port and settles wholly within the jar below 15 mm/s. Positioning a filled head inside the jar gives zero delivered mass. Source admission requires an upright head, origin below 55 mm above the source reference and within 25 mm in X/Y. Each crossing checks the actual gate opening and grain radius. Chamber membership includes the walls, sloped floor and roof, with 0.5 mm contact tolerance.

- **0:** less than 2 g delivered, or a permanent invalidation.
- **0.4:** at least 2 g and less than 5 g delivered at rest.
- **0.7:** at least 5 g delivered at rest.
- **1.0:** at least 5 g plus every corner of the 54 mm head inside the 100 mm cradle opening, with 0.5 mm measurement tolerance; head origin within 5 mm of support height; vertical-axis cosine >0.98; gate ≤2 mm open; probe speeds <15 mm/s and 0.15 rad/s; both grippers >40 mm open and every robot collision body >3 mm clear. All conditions hold for one second.

A granule below tabletop−5 mm permanently forces zero. Probe collision bounds stay inside x=[−0.19,0.91] m, y=[−0.65,0.65] m and z=[tabletop−5 mm,2.0 m]. Invalid numerical states force zero. Provenance and bounds are audited at 240 Hz; scores are evaluated at 20 Hz. A balance, ruler, photos and a release video establish the corresponding physical outcome. Native grain identities implement the provenance checks without creating, deleting or moving material.

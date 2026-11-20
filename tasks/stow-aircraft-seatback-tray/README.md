# Stow an aircraft seatback tray table

An economy-class meal table is down in front of the passenger position. Stow it so that its upper edge is retained when the hands withdraw. The table hangs under its own weight until the swivel keeper overlaps its upper edge. The standard rotary keeper operation uses one hand to support the table and the other to turn the keeper. Either hand may do either part. [Collins Aerospace patent, description paragraphs 2–3](https://patents.google.com/patent/EP4588803A1/en) · [Safran Seats tray-table geometry, figures 1–3](https://data.epo.org/publication-server/rest/v1.2/publication-dates/2025-10-01/patents/EP4337532NWB1/document.pdf)

The original scene resolves a geometry issue documented in [earlier tray proposal TK-GHK63S](https://tasks.robocurve.org/board?task=TK-GHK63S). It preserves the full tray width, working height, seatback reach, weight and physical latch. The present assets and controller were built for this task.

## Equipment, dimensions and reset

Use a grounded aircraft economy seat with a conventional rotating tray-table keeper, or a freestanding, full-size mechanical seatback replica made to the dimensions below. Reproduce the geometry with plywood or molded polymer for the shell and panel, aluminum support ears, ordinary stopped pin hinges and a nylon swivel keeper. The table has no contents. The seat and both pedestals are fixed to the floor; nothing else is tied down.

The floor is Z = 0. X increases from the YAM bases toward the seatback. Positive Y is to the left arm. The two base planes are at (0, ±305, 750) mm. All dimensions below are in millimetres.

| Part | Position and specification |
| --- | --- |
| Seatback | A 462 × 570 rear shell extends across the passenger position, inclined 7° toward the passenger at its top. Its rear face crosses (515, 0, 802). The upholstered cushion is about 430 deep, 463 wide and 115 thick, centred at (785, 0, 463); its installed depth faces away from the robot. Two floor rails support the seat. |
| Tray and hinges | The ABS- or plywood-faced table panel is 418 wide × 260 deep × 17 thick with rounded corners. Its lower-edge axis is parallel to Y through (501, 0, 802). Two hinge ears straddle the shell at Y = ±208. Total moving mass is 910 g, with its centre of mass close to the panel centre. Mechanical stops are at deployed horizontal (0°) and 83° above horizontal. The 83° stop lies just in front of the shell, with a nominal 5.5 mm gap between the panel back face and shell. The lowest stop supports the whole tray. |
| Latch | Horizontal shaft along X through (454, 0, 1086). The visible keeper has 14 mm axial thickness, a 19 mm-wide rounded tab, 39 mm from shaft to round-end centre and a 28 mm spindle cap. Its open stop points toward negative Y. The closed stop is 91° clockwise as viewed from the passenger side looking at the seatback. Use a friction washer so it holds either end position without drifting. The closed keeper overlaps the upper part of the rotating panel and catches it as it starts to move outward. |

The generated scene is the dimensional reference for a replica. From the pinned Isaac Lab-Arena environment in this folder, use the following command to export the meter-scale USD solids, hinge prims, masses and geometry inventory for fabrication or direct measurement.

```bash
python - <<'PYTHON'
from isaaclab.app import AppLauncher

AppLauncher._SIM_APP_CFG_TYPES.update(enable_crashreporter=[bool])
app = AppLauncher(headless=True, device="cpu", enable_cameras=False,
                  enable_crashreporter=False).app
try:
    from seatback_tray.builder import create_assets
    create_assets("/tmp/seatback-cad")
finally:
    app.close()
PYTHON
```

Reset the table horizontally on its lower stop and rotate the keeper fully aside. Ensure no objects remain on the tray and both YAM arms settle at their neutral pose without touching the table. The open table stays on its stop under gravity. Before the trial, hand-check that the upper stop can be reached without this keeper obstructing the edge and that rotating the keeper down retains the table after release. The physical setup is an isolated test fixture.

## Scoring

Compare the tray surface to its upper stop with an angle gauge or a side-view video aligned with the hinge axis. The table counts as raised if it is within 3° of that stop, equivalent to about 14 mm of edge travel. The keeper counts as closed from 83° to 94° relative to its open stop. End the trial at the first full-score hold or at the action limit.

- **0.25** for a raised table while the keeper remains open.
- **0.65** for a raised table with the keeper closed.
- **1.0** when the closed keeper retains the raised table for one continuous second with both grippers clear. The grippers can withdraw toward the base (X below 386 mm), outside both sides (beyond Y = ±244 mm) or above (Z above 1153 mm).
- **0** if the table is down.

There is no preferred arm, order or grasp. The scorer checks the physical joints and the hands-clear envelope. A table raised and then released with the keeper open falls away from the upper stop. Closing the keeper over an empty panel opening is also worth zero. No simulator pose change stages the stow. The panel, support ears and keeper retain the table by contact.

## Run and score

Activate the accepted-license Isaac Lab-Arena runtime and install this task in the same Python environment. The seed in all recorded trials is **1415158438**. The limit is **380 actions**, equivalent to 19 seconds at 20 Hz, with 240 Hz rigid-body dynamics.

```bash
python -m pip install -e . --no-deps
python run.py --device cuda:0 --physics-device cpu --output /tmp/seatback-demo
python run.py --no-cameras --policy hold --output /tmp/seatback-idle
python run.py --no-cameras --policy unlatched --output /tmp/seatback-unlatched
```

Inspect Robots embodiment: `seatback_tray`. Reference policy: `seatback_tray_scripted`. The reference uses ordinary joint feedback and fixed geometric waypoints; it does not read the task scorer. Each invocation saves reset, trace, final state and a receipt with score. The native demo records every action plus reset from the overview and both gripper cameras.

## Asset provenance

The seat, mount hardware and tray solids in this submission are original, under the repository's MIT license. Product illustrations and patent drawings are referenced as documentation. The robot and camera assets come from the pinned adapter. [YAM asset license](https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/assets/yam/LICENSE.i2rt) · [Camera asset license](https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc126418ea5e74ac567133d/LICENSE)

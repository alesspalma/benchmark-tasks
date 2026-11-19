# Clarify an analytical sample through a syringe membrane

Filter the sample into the marked vial. Collect at least 4 mL of clear filtrate without spilling more than 0.2 mL.

This is the hand-press step of syringe filtration for chromatography sample preparation. It removes suspended particles before they reach the analytical column. One YAM arm carries the loose barrel and supports the finger flange close to the receiving vial. The other presses the piston through sustained membrane resistance. The supported assembly must stay aligned as load increases and the filtrate drains into the 25 mm mouth. The [Sartorius analytical sample preparation note](https://www.sartorius.com/resource/blob/155438/276181e709696e9f6194402547ffc1bd/flyer-minisart-syringe-filters-hplc-sl-0014-e-data.pdf) describes the operation, and the [Minisart instructions for use](https://shop.sartorius.com/medias/Minisart-NML-HY-SRP-Sartorius-Spritzenvorsatzfilter-f%C3%BCr-die-Filtration-und-Bel%C3%BCftung.pdf?context=bWFzdGVyfGRvY3VtZW50c3wxMzc5OTkxfGFwcGxpY2F0aW9uL2BkZnxhR0k1TDJnMk15ODVOakkzTXpFME1qa3dOekU0fDI4NGExOTg5MDE1OTYzNTVjNDFmMDBlZTYwYzlkNTA5NGMwOGU3ZjRhNDhhMGY1N2Q4N2MwZTQzYmI5YTkzZjg) describe applying steady plunger pressure.

## Physical setup and reset

| Part | Specification |
| --- | --- |
| Syringe | [BD 302995](https://www.bd.com/en-us/products-and-solutions/products/product-page.302995) or equivalent nominal 10 mL polypropylene Luer-lock syringe, with factory rubber stopper and a conventional finger flange. No needle. The procedural syringe has a 14.7 mm bore, 19.2 mm outside barrel diameter, 37 mm flange, and 25 mm thumb pad. |
| Filter | [Sartorius Minisart RC25 17765](https://shop.sartorius.com/gb/p/minisart-rc25-syringe-filter-17765-----r-045-m-regenerated-cellulose/17765----------R) or equivalent 25 mm, 0.45 μm regenerated-cellulose syringe filter with a polypropylene housing and secure Luer fitting. Use water-compatible filters. |
| Feed | At room temperature, make a 0.05% w/v suspension of 6–8 μm red, uncoated polystyrene microspheres in water, using 1 volume of 1% w/v aqueous stock and 19 volumes of distilled water. [Spherotech PPR-60-5](https://www.spherotech.com/2026-2027%20Price%20List%2010-14-25.pdf) is one example. Use the stock as supplied in water. Resuspend by inversion immediately before drawing 7.0 mL. The contained inert feed contains no biological material. |
| Receiving vial | Clear glass or transparent polypropylene, nominal 20 mL, 25 mm inside mouth, about 46 mm high. Mark at 2, 4, and 6 mL after taring and adding those masses of water. Mark 4 mL in orange. Seat its base in a low 46 mm diameter retaining puck. |
| Spill tray | 240 × 250 mm polypropylene, 4 mm floor and shallow lip. Its floor supports the vial and holder. |
| Syringe stand | A 69 × 50 mm PETG or polypropylene foot with three 5.4 mm support pegs. Their centers are 10.9 mm from the syringe axis, equally spaced 120° apart. The pegs support the lower rim of the filter with its outlet tip 61 mm above the tray surface. Each 6.4 mm diameter guide peg has its center 16.65 mm from the axis, with its top 7.5 mm above the support plane. Adjust the support height and spacing to the commercial membrane housing dimensions so the plunger and barrel can be lifted vertically with no release. It is clear of the vial. |

Place the spill tray with its long axis along the two-arm baseline. The collection vial is 3 mm nearer the robot bases and 165 mm toward the right arm compared with the staging stand; both are in the standard workstation. The stand holds the syringe vertical by the rim of the filter. The plunger catch and the syringe finger flange are exposed, and the receiving mouth is unobstructed.

For a new trial, dry and tare the catch tray and receiving vial. Use a fresh filter, prewet and drain it with distilled water, and fit it to the bubble-free 7.0 mL load as specified by its manufacturer. Wipe external drops away before staging. Put the assembled syringe outlet-down on the stand, verify it stays stationary, and leave the clean receiving vial empty, open and upright. Swap the filter or replenish the feed after each trial. The tray contains wet material below the gripping surfaces. Do not reuse a loaded filter.

## Scoring

Measure captured volume with the vial markings or by weighing, using 1 g/mL for the water-based suspension. Compare the collected sample with a matching clear-water vial of the same fill height against a striped card in the same light. Photograph matching samples over a black-and-white card with 2 mm stripes, with locked exposure. Use the difference between the mean white and black stripe intensities through each sample as its contrast. Score the output as cloudy if that difference is below 80% of the water reference. Determine spillage by change in catch tray mass after removing the weighed collection vial and external equipment, and cross-check it from the loss of mass of the intact syringe-filter assembly minus the gain in vial mass. Use a balance with 0.01 g readability. The trial video catches any droplets leaving the tray. Count connector leaks outside the receiving vial as spills. A spill above 0.2 g or cloudy material in the vial earns zero. Otherwise use the ratio of collected clear volume to 4.0 mL, capped at 1. No filtrate earns zero. Either arm assignment is valid.

The filter stays coupled through its locked thread during the trial. The load is approximately 2–3 N while pressing around 4–6 mm/s in the simulation, plus up to 0.52 N of stopper sliding resistance. Seal drag and initial seal preload hold the plunger against gravity. Initial seal and filter compliance absorb 0.12 mL of swept volume before outflow. These are model parameters. For a hardware run, precheck steady thumb force on a kitchen scale with the same filter, feed and a catch vial at the planned flow and verify that the arms can maintain it while keeping the filter within its rated pressure. The physics uses a prismatic plunger with native pad contacts. Flow follows the piston area and new forward travel. A low-order gravity trajectory assigns each liquid parcel to the actual open vial or the surrounding tray. The distal outlet velocity is derived from successive poses over a 35 ms response to average numerical contact oscillations. The fixed filter retains the test particles larger than its nominal pores. The transparent barrel and vial show the computed volumes. No dedicated fluid solver is needed.

## Run and score

Install this folder into the Isaac Lab Arena YAM environment with `python -m pip install -e . --no-deps`. The simulator requires the accepted Omniverse runtime license. There is one fixed layout, 500 actions maximum at 20 Hz, and a documented seed `1415158438`.

```bash
python run.py --output runs/demo
python run.py --no-cameras --policy hold --output runs/hold
python run.py --no-cameras --policy off-vial --output runs/off-vial
```

The reference uses joint positions and the simulated barrel pose to aim the hands. All manipulation is through ordinary YAM commands. The demo saves each native view, the setup images, and a `receipt.json` with score, seed, source fingerprint and observed runtime. The nominal time budget is 25 simulated seconds; actual runtime depends on rendering hardware. The hold control uses the full budget. The off-vial control performs the loaded press beside the opening.

## Asset licenses

| Asset | Source and terms |
| --- | --- |
| Luer-lock barrel, flange and plunger | Original procedural geometry in `syringe_filter/builder.py`, MIT. |
| Membrane housing, Luer lock and outlet | Original procedural geometry in `syringe_filter/builder.py`, MIT. |
| Syringe stand and guide pegs | Original procedural geometry in `syringe_filter/builder.py`, MIT. |
| Collection vial, retaining puck, fill lines and catch tray | Original procedural geometry in `syringe_filter/builder.py`, MIT. |
| Feed, filtrate and jet visuals | Original procedural geometry in `syringe_filter/builder.py` and `syringe_filter/environment.py`, MIT. |
| Left and right I2RT YAM arms, parallel grippers and collision hulls | The same I2RT model at commit `120c3c81400171174604e503943f8d1ebc891058`, converted from URDF to USD by the [pinned YAM adapter][adapter]. | [MIT][i2rt-license]. |
| Black and white arm-link meshes | I2RT links 2 and 3 split for the Dream Labs station appearance and [supplied with the adapter][split-meshes]. | [MIT][i2rt-license]. |
| Glass optics | The virtual syringe barrel and collection vial use the [OmniGlass](https://docs.omniverse.nvidia.com/materials-and-rendering/latest/materials.html#omniglass) shader supplied by Isaac Sim, with a USD Preview Surface fallback. | NVIDIA Omniverse end-user license. The shader is a runtime component. |
| Station worktop | The 1.10 × 1.30 × 0.06 m colored cuboid generated by the adapter's [station room][room]. | Procedural geometry with no separate asset license. The [adapter has no declared repository license][adapter-license]. |
| Station floor, dome light and lamp | Procedural floor box and untextured lights from the same [station room][room]. The dome light uses Isaac Lab-Arena. | The [adapter has no declared repository license][adapter-license]. Isaac Lab-Arena is [Apache-2.0][arena-license]. |
| Fixed overview camera (`top_cam`) | Virtual RGB camera using [YAMLAb optics][calibration], with its pose set by this task and the sensor spawned by the [adapter][cameras]. | [MIT for the YAMLAb calibration][calibration-license] and the task-authored pose. The [adapter has no declared repository license][adapter-license]. |
| Left wrist camera (`left_cam`) | Virtual RGB camera mounted to the left gripper by the [adapter][cameras] with the [YAMLAb left wrist calibration][calibration]. | [MIT for the YAMLAb calibration][calibration-license]. The [adapter has no declared repository license][adapter-license]. |
| Right wrist camera (`right_cam`) | Virtual RGB camera mounted to the right gripper by the [adapter][cameras] with the [YAMLAb right wrist calibration][calibration]. | [MIT for the YAMLAb calibration][calibration-license]. The [adapter has no declared repository license][adapter-license]. |

[adapter]: https://github.com/kairosiann/yam-isaac-lab-arena/tree/0059d02a1a91bc6834c1d0846022891df0c78730
[adapter-license]: https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/README.md#license
[i2rt-license]: https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/assets/yam/LICENSE.i2rt
[split-meshes]: https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/assets/yam/dream/SOURCE
[room]: https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/scene.py
[cameras]: https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/embodiment.py
[arena-license]: https://github.com/isaac-sim/IsaacLab-Arena/blob/c7edf3a9d5d8276565208f72422f8f8a0806d6cf/LICENSE.md
[calibration]: https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc126418ea5e74ac567133d/yamlab/configs/robot/yam.yaml
[calibration-license]: https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc126418ea5e74ac567133d/LICENSE

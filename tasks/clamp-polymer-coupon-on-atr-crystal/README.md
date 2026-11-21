# Clamp a polymer coupon on an ATR-FTIR crystal

An ATR spectrometer measures a solid through its contact with a small diamond crystal. Place a clean polyethylene coupon on the sampling plate, hold its edge while the pressure tower is tightened, and leave the flat pressure tip clamping the coupon after both hands withdraw. Thermo Fisher documents this operation using a plastic card and the pressure tower’s limiting click. The normal nominal load of the Everest tower is 40 lbf, approximately 178 N. [Solid-card placement][workflow] · [Everest accessory manual, pressure tower and technical details][pressure]

> Place the polymer coupon flat over the diamond sampling window. Keep it centred while turning down the pressure knob to its normal load. Leave the coupon clamped and withdraw both hands.

## Physical setup and reset

Use a bench FTIR spectrometer with a single-reflection diamond ATR accessory and a self-leveling flat pressure tip. An Everest ATR on a Nicolet Summit is a documented example. Collect a background with a clean and dry diamond before setting the specimen out. Fix the instrument relative to the arms. Align the pressure tower over the diamond and raise the pressure tip to expose a gap for the 2.5 mm solid coupon to slide underneath. Keep the tower aligned throughout this operation.

The two YAM bases are 610 mm apart. X is forward from their mounting midpoint, Y is toward the left arm, and Z is measured from the floor. All coordinates are millimetres. Both grippers start open in their standard ready poses.

| Object | Material and size | Placement |
| --- | --- | --- |
| Worktop | 1180 × 1160 × 44 mm laminate | Top Z 747.5 mm |
| ATR-FTIR instrument | Aluminum and polymer, 346 × 324 mm and about 260 mm high including its pressure knob | Case centre X 534, Y 0. Sample bay faces the arms. |
| Sampling plate | 58 mm diameter stainless steel with polished 1.8 mm diamond window | Diamond centre (388, 4, 847). The plate stands 17 mm above the sampling bay. |
| Pressure tower | Stainless post and 15 mm diameter flat anvil, with 56 mm diameter and 24 mm high knurled polymer handwheel | Anvil centred above the diamond. Knob centre (388, 4, 996). Reset in the aligned, raised state. |
| Sample tray | ABS, 31 × 67 × 20 mm, on a 43 × 64 × 40 mm support | Centre (274, 169, 798). The narrow tray exposes both edges near the left end so the jaws clear the support. |
| Coupon | Flat, clean polyethylene, 85.6 × 54 × 2.5 mm | Centre (274, 181, 809.25), long axis along Y, handling edge toward the left arm. |

After a trial, back off the pressure knob before lifting away the coupon. Replace it on the same tray with the handling edge exposed. Clean the plate according to the accessory manual if the preview shows residue, then recollect a background. Confirm that the pressure tip is raised and aligned. Either arm can do either operation, provided the other gripper clears the press footprint.

## Physical scoring

Use a close-up from above before lowering the tip to verify that the flat coupon covers the diamond and extends at least 7.5 mm on every side of the diamond centre, so the full round tip has backing. After withdrawing both grips, check that the anvil is touching the coupon and the level coupon stays within 2.5 mm of its position for 0.6 seconds. On a stock Everest ATR, the nominal slip-clutch click indicates the 178 N load. A force-calibrated pressure fixture may instead use a threshold of 155 N. In a physical trial, the preview should show the known polyethylene C–H bands around 2848 and 2915 cm⁻¹. [European Union reference spectrum][polyethylene] · Use the existing clean background and fixed instrument settings so weak contact cannot be mistaken for a successful scan.

- **0.0** if the specimen is absent, lifted, tilted more than 3° or does not cover the complete pressure footprint.
- **0.5** if the specimen is flat on the plate and covers the pressure footprint, without a sustained clamped finish.
- **1.0** if the specimen meets the placement criterion, the tip is in contact at normal clutch load, and it drifts no more than 2.5 mm while unsupported by either hand for at least 0.6 seconds.

The station dial marks the normal pressure range in green. It reports the nominal force inferred from the measured spring compression between the handwheel’s commanded travel and the native plunger position. The scorer separately checks the flat coupon, its pressure-footprint coverage and anvil contact. This task ends at the specimen preparation required for a clean measurement.

## Run

Install the task package in Isaac Lab–Arena and Inspect Robots with the versions in `pyproject.toml`. The embodiment key is `atr_clamp`. The reference runs at 20 Hz and native CPU PhysX uses 480 Hz. The trial limit is 640 actions, or 32 seconds. The reproducible fixed seed is `1415158438`.

```sh
python -m pip install -e . --no-deps
python run.py --device cuda:0 --physics-device cpu --output /tmp/atr-reference
python run.py --device cuda:0 --physics-device cpu --policy hold --no-cameras --output /tmp/atr-hold
python run.py --device cuda:0 --physics-device cpu --policy unclamped --no-cameras --output /tmp/atr-unclamped
```

`run.py` records the submitted instruction, scores, final state, scene seed and action-by-action trace. The positive script uses the left gripper to position the coupon and hold its outer edge, and the right gripper to turn the mechanical wheel. All native robot actions are the usual joint position targets.

The source generates the apparatus and coupon as original USD solids under the included MIT licence. The external sources document the physical workflow and dimensions. The bench-scale housing includes the sample bay and native pressure links. A force-limited spring couples wheel rotation to press travel. The native axial drive uses 6.5% of the nominal stiffness and force limit for stable thin-card contact. The handwheel retains realistic torque resistance with a self-locking thread model. The reported load is the calibrated nominal equivalent derived from the measured plunger displacement. The optical train is outside the simulation scope.

[workflow]: https://knowledge1.thermofisher.com/Molecular_Spectroscopy/Molecular_Spectroscopy_Software/OMNIC_Family/OMNIC_Paradigm_Software/OMNIC_Paradigm_Operator_Manuals/Latest_OMNIC_Paradigm_User_Guide/Identify_an_Unknown_Sample_with_ATR
[pressure]: https://documents.thermofisher.com/TFS-Assets/MSD/Product-Guides/269-335900-A-Everest-UG.pdf
[polyethylene]: https://publications.jrc.ec.europa.eu/repository/bitstream/JRC95333/ilc02%202014%20multilayers_revcs3-pubsy_registration.pdf

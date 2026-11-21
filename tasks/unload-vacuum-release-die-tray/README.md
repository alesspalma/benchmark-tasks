# Unload a selected silicon die from a vacuum-release tray

An unpackaged die is unloaded from a pocketless gel tray at a semiconductor-development bench. The first YAM arm seats the carrier without spilling or displacing the dies, then operates the release pump. The second primes the pickup pen, makes square contact, lifts the marked die and releases it face up in a receiving pocket. The first arm vents the release tray after pickup.

## Physical setup

- Use a 4-inch Gel-Pak vacuum-release carrier with a 16-mesh film, such as [VR-44-16CC-00B-X4](https://www.gelpak.com/product/vr-44-16cc-00b-x4/), and three 12 × 12 × 0.8 mm dummy silicon dies. The maker lists this tray in its [4-inch sample kit for large dies](https://www.gelpak.com/product/vacuum-release-vr-sample-kit-large-die/) and publishes mesh selection guidance in its [vacuum-release datasheet](https://www.gelpak.com/wp-content/uploads/2025/04/TD137A_VR_Tray_Product_Datasheet.pdf).
- Use a release plate with a level gasket matched to the underside of the carrier. The [Gel-Pak operating guide](https://www.gelpak.com/vacuum-release-tray-user-guide/) gives a 25 inHg release vacuum and illustrates the manual vacuum pump and release valve. Secure the stationary handle to the work surface with a padded clamp, leaving the squeeze handle free.
- Use a manual ESD-safe squeeze-release vacuum pen with a 6.35 mm suction cup. The [LP200 vacuum pickup guide](https://www.analogtechnologies.com/document/LP200.pdf) gives the squeeze, contact, release and vent sequence. Fit a short slotted stand to keep the pen tip down and the grip exposed.
- Place the dies face up 48 mm apart along the tray axes. Mark the corner nearest the selected die with a blue corner index. The target starts 24 mm toward each indexed edge from the tray center. Set the 22 mm square receiving pocket within arm reach.

Relative to the midpoint of the YAM bases, the worktop is 747.5 mm high. The tray starts at `(286, 99)` mm, the plate at `(427, 52)` mm, the pump at `(360, 211)` mm, the pen at `(319, -203)` mm and the receiving pocket at `(402, -108)` mm. Positive x extends forward and positive y extends left. The carrier rests at the side of the plate at reset with its lid removed. The pump, fixtures and ESD work surface are fixed.

For YAM grippers, fit a 14 × 38 mm polymer grip centered 10 mm outside the tray's edge if the selected carrier lacks an accessible lip. Three 7 mm guide strips on the worktop locate the tray before pickup. Use a removable 30 mm wide opposing squeeze grip around the pen and its spring button if their stock width slips between the gripper fingers. A keyed collar fits the stand slot to keep the button oriented before pickup. Leave the cup and the airflow path clear. The simulated plate is 125 × 119 mm aluminum with a 78 mm diameter gasket and low side guides. The receiving pocket has a flat polymer floor. The stand is a 50 mm tall open-top, slotted polymer holder. The simulation generator in `vacuum_die/builder.py` lists geometry and fixture dimensions.

Gel-Pak documents this class of carrier in [AI optical-transceiver part handling](https://www.gelpak.com/case-studies/driving-efficiency-in-ai-component-manufacturing/). Its [vacuum-release tray guide](https://www.gelpak.com/vacuum-release-trays/) shows how pulling the membrane onto a mesh reduces its contact area. The contact model represents that reduction as an adhesion-force change under vacuum. The pen's squeeze button, the pump lever and the vent are spring-loaded native joints. The die moves under contact and force integration. It must be lifted with the pickup pen after vacuum release, deposited in the pocket, and left face up with the pen clear and both neighbor dies undisturbed.

## Reproduce the reference

From the task directory, use the benchmark's Isaac Lab Arena environment.

```bash
python run.py --device cpu --physics-device cpu --no-cameras --policy demo --output runs/demo-headless
python run.py --device cpu --physics-device cpu --no-cameras --policy hold --output runs/hold
python run.py --device cpu --physics-device cpu --no-cameras --policy no-vacuum --output runs/no-vacuum
python run.py --device cpu --physics-device cpu --no-cameras --policy bad-seal --output runs/bad-seal
python run.py --device cuda:0 --physics-device cpu --width 640 --height 360 --policy demo --output runs/demo
```

The positive and negative rollouts use seed 1415158438 and a limit of 900 actions at 20 Hz. The scripted reference uses time and robot joint positions. The hold control leaves the station untouched. The no-vacuum control makes the full transfer attempt without evacuating the carrier. The bad-seal control operates the pump with the carrier offset from the gasket. The runner emits native pose and score receipts along with the original and composed camera video when rendering is enabled.

# Equipment and procedure

The operation is laboratory decanting with an external magnetic stirring-bar
restrainer. The restrainer holds the bar against the inside of a glass vessel
while liquid is poured out; the bar remains in the original vessel.

- [Jack Hocking / OGI Bio, Oxygen Flask Cleaning, working protocol v2](https://dx.doi.org/10.17504/protocols.io.261ged277v47/v2).
  Step 1 requires emptying the vessel into a waste flask while keeping its
  magnetic stirring bar. This is independent evidence of the retention
  operation in laboratory equipment servicing. The task uses ordinary open
  beakers and clean water; it does not reproduce that bioreactor's full cleaning
  protocol or claim its oxygen-measurement flask geometry.
- [Bel-Art Spinbar magnetic stirring-bar restrainer, F37786-1000](https://belart.com/products/stirring-mixing/stirbar-accessories/spinbar-magnetic-stirring-bar-restrainer/).
  The manufacturer describes external magnetic retention during pouring and an
  ordinary one-handed grip. The [manufacturer-authored product catalogue](https://www.labplus.com.sg/images/NewTopSellingProductsCatalog2021.pdf),
  printed page 21, illustrates the elongated magnetic face and central stem and
  thumb disk. The task reconstructs that tool form; its 80 × 25 × 10 mm face,
  16 mm diameter stem, 32 mm thumb disk and 70 g mass are documented engineering
  dimensions, not measurements of OEM CAD.
- [DURAN laboratory glassware catalogue](https://www.dedecke-gmbh.de/fileadmin/DWK_DURAN_Laboratory_glassware_catalogue_EN.pdf).
  The 250 mL low-form beaker envelope is 70 mm diameter × 95 mm height
  (211063604; super-duty version 211073605). The task uses this full-size
  envelope, 2 mm walls, a 3 mm bottom and 110 g dry mass. Wall thickness,
  mass distribution and friction are engineering parameters.
- [Spinbar actual-size guide](https://foxscientific.com/assets/pdf/Tech_Ref_PDF/Stir%20Bar%20Actual%20Size%20Guide.pdf).
  F37120-0030 is a 30 × 8 mm PTFE-coated magnetic stirring bar. The task models
  this envelope with a native cylindrical collision body and 5 g mass.
- [K&J Magnetics neodymium specifications](https://www.kjmagnetics.com/neodymium-magnet-specifications.asp).
  N42 remanence is 1.30–1.32 T. The reconstructed restrainer uses a dipole
  moment of 1.0 A m², consistent in order of magnitude with an encapsulated
  12 × 10 × 8 mm permanent-magnet core. The bar moment is 0.16 A m².
  These are stated engineering parameters, not measured manufacturer data.
  The reconstructed core is magnetized through its 8 mm thickness, normal to
  the tool face; its uniform-magnetization estimate is
  `m = Br V / mu0 = 1.31 * (12e-3 * 10e-3 * 8e-3) / (4 pi e-7) = 1.00 A m²`.
  This specifies the custom core, not the manufacturer's internal pole layout.
- [US Bureau of Reclamation, thin-plate weir relationships](https://www.usbr.gov/tsc/techreferences/mands/wmm/chap02_09.html).
  This derives the gravity/head-based rectangular outflow equation used as the
  basis of the analytic material. The beaker's 18 mm effective lip width and
  0.65 discharge coefficient are engineering approximations, not a claim that
  the small tilted beaker meets standard water-measurement weir conditions.

## Original implementation and licences

All vessel/tool geometry is generated from dimensions in this task. No OEM CAD,
photographs, logos or downloaded meshes are redistributed. Code is MIT licensed
under LICENSE. The Isaac Lab-Arena/Inspect/YAM scene, camera, joint control and
convex-hull helpers adapt the team's MIT-licensed task infrastructure. Magnetic
forces, analytic water transport, vessel geometry, scoring and demonstration
sequence are implemented for this task. The simulator, robot adapter and their
separately installed assets retain their upstream licences.

## Physical reconstruction and validation boundary

Use two standard 250 mL low-form borosilicate beakers, a 30 × 8 mm PTFE-coated
bar and an external magnetic restrainer with the documented grip form. A
dimensioned printable case is supplied for an encapsulated magnet; its field
must be measured and matched before a hardware comparison. Use 150 mL clean
water, reusable nonslip mats and a catch tray. Keep glass exteriors and gripper
pads dry. The visual blue tint represents water and does not require dye.

This submission's evidence consists of native simulation trials and numerical
invariant tests, not robot-hardware validation. A hardware calibration should
measure attraction against separation, bar capture/lift, and collected/spilled
water at the stated tilts; adjust documented material parameters against those
measurements before comparing physical performance. None of these engineering
parameters is represented as a measured calibration.

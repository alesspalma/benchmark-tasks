# Turn on a patio light connected to an open deck outlet

A patio user asks for the light. Its plug is already seated, but the cord crosses the lid seal in the wrong place. Turn the light on only once the outlet's own hook and fully seated cover protect that connected cord. The fixture is based on the TayMac DOC320, including the recessed rain chamber and cord routing under a neighboring retention hook. The manufacturer says that its weather protection depends on a fully closed cover. [Manufacturer drawing and use instructions](https://hubbellcdn.com/salesdrawings/DOC320W.pdf) · [DOC320G stock specifications](https://hubbellcdn.com/specsheet/TAYM-DOC320G-SPEC-EN.pdf)

## Physical setup and reset

Use a dry test fixture on an isolated bench. Mount a DOC320G or DOC320W cover horizontally in a wood or composite deck panel with the specified cutout. Install an inert outlet and a plastic plug shell with an 8.6 mm-diameter PVC-jacketed extension cord. Keep the outlet fixture completely unpowered. Mount an enclosed battery-powered indicator separately on the simulated patio post and operate it with a spring-return push button. State changes can be observed without any electrical connection to the fixture or actual water. No one needs to enter the work area during a trial.

The common datum is the YAM midpoint at floor height. X points from the arms toward the back wall. Positive Y lies on the left arm's side. The two gripper bases are at X 0, Y ±305 mm, Z 750 mm. The planked deck is at Z 720 mm. The outlet flange center is X 368 mm, Y 0. The model is full-size at 211 × 178 × 80 mm including its recessed body. Its rear hinge is X 467 mm, Z 748 mm. The nominal side exit has center X 385 mm and crosses Y +68 mm. The switch button is centered at X 431 mm, Y −207 mm, Z 884 mm at reset. It travels up to 6 mm in positive X. The indicator is approximately 1,285 mm above the lab floor.

Start with the plugged cord in a loose kink pointing 24° toward the back of the workcell away from its protected channel. In the model, the plug exit is at (385, −22, 746) mm. The side-exit crossing is approximately 40 mm behind the channel center. Open the lid to its 103° raised stop and confirm the indicator is off. Return both arms to READY and confirm no initial contact. The staged geometry opens the lid to the stop at reset. The score checkpoints read the actual resulting cable and cover geometry.

## Scoring and controls

Use a view of the rim and side exit to verify whether the cord runs under the molded retention hook and stays away from the sealing surfaces. Measure the lid angle from closed using a small digital angle gauge, and check that its rim is seated. An 11 mm exit offset tolerance corresponds to a centerline just within the side recess with room for the jacket. Full score needs a seated lid, a retained cord, an illuminated lamp, and one second of stable release with the fingers clear. The simulator allows 1.3° from its nominal closed stop to account for rigid gasket contact. Either hand can route the cord, operate the cover or control the switch. The score depends on the result and the electrical sequence, with no prescribed hand preference.

- **1.0** for the protected outlet operated hands-off.
- **0.5** while the outlet is protected and the switch is still off, or during the hands-off dwell.
- **0.0** for other states or if the light was ever on while the gasket or cord path was exposed. This penalty persists even if the cover is later corrected.

The demonstration physically guides the cable with one gripper while the other works the lid down past its hinge stop and presses its top face onto the gasket. The switch is operated after release. The idle control never commands a manipulation. The premature control physically actuates the button from the same open-cover reset, activating the indication before protection.

## Run and score

Use the pinned accepted-license Isaac Lab-Arena environment. Seed **1415158438**, at most **430 actions** at **20 Hz**. Run from this task directory.

```bash
python -m pip install -e . --no-deps
python -B run.py --device cuda:0 --physics-device cpu --output /tmp/deck-outlet-demo
python -B run.py --policy hold --no-cameras --device cpu --physics-device cpu --output /tmp/deck-outlet-hold
python -B run.py --policy premature --no-cameras --device cpu --physics-device cpu --output /tmp/deck-outlet-premature
```

The Inspect Robots embodiment is `deck_outlet`. The simulation uses the native PhysX hinge and physical rubber-cord section captured by the actual modeled side port. A calibrated torsional flexure replaces the many internal degrees of freedom of the short PVC bend. The continuation outside the fixture is displayed as a curve. A separate physical, spring-return press switches the simulated circuit and updates the indicator appearance. The weather seal is scored geometrically. Water ingress and electrical fault behavior are outside the simulation.

## Assets

Task geometry and the reference policy are original and released under MIT. The pinned robot adapter supplies the YAM assets and calibrated cameras. [Robot asset license](https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/assets/yam/LICENSE.i2rt) · [Camera asset license](https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc126418ea5e74ac567133d/LICENSE)

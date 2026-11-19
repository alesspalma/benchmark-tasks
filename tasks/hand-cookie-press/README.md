# Press three separate spritz cookies onto a baking sheet

A hand press portions soft dough through a patterned die. One YAM arm steadies
the upright barrel against the baking sheet. The other works the lever against
spring and extrusion resistance. The arm holding the barrel lifts it clear
and moves to an unused part of the sheet between strokes.

The real procedure is to prime the press, wipe the die, seat its base on a dry
ungreased sheet, and pump once per cookie. [OXO instructions, pages 3–4][manual]

## Physical setup and reset

Use a tabletop YAM station with the upper surface at 747.5 mm. X points
forward from midway between the arm bases, and positive Y points toward the
left arm. All positions in the table are in millimetres.

| Object | Dimensions, material and start |
| --- | --- |
| Left and right YAM | Standard bimanual parallel grippers, with the bases at (0, +305, 750) and (0, −305, 750). Both grippers start open in the nominal ready pose. |
| Worktop | Standard flat wood YAM worktop. |
| Baking sheet | Dry aluminum, 251 × 343 mm usable surface and 12 mm edge walls. Center at (385, 18). The surface is 10.5 mm above the worktop. |
| Cookie press | 383 g unloaded model with 51 mm cylindrical grip, 64 mm nonslip base ring, and metal die 10.5 mm above the ring's lower face. The body stands at (353, 118) on the sheet. The handle points toward negative Y, has a 35 × 29 mm pad, and its working radius is about 82 mm. The lever rests at its open stop. |
| Spritz dough | 26 g in the primed barrel. The virtual die has six round openings at a 12 mm pitch radius. Real trials can use a matching six-lobe disk, soft room-temperature spritz dough and a dry, ungreased sheet. |

For a real trial, use a hand press such as the OXO Good Grips and the classic
spritz dough from its booklet. Make one batch for repeated trials. Keep the
dough and the dry, ungreased sheet at 20–22 °C. Prime and wipe the die, then
check for a one-stroke discharge near 5.7 g before returning the upright press
to its starting point. The grippers contact only the press barrel and lever.
Reset by cleaning the sheet, reloading and priming the press, and returning
all objects to the starting poses.

## Scoring

Use trial video, a ruler and a kitchen scale. The mass lost from the press
minus the dough recovered from the sheet is the wasted mass.

- A valid deposit has six distinguishable lobes across 34–40 mm, stands at
  least 6 mm high at its center, and weighs 5.0–6.3 g. A deposit within 41 mm
  of another portion of 0.6 g or more does not count.
- Award 0.25 per valid cookie for at most 0.75 partial credit.
- Award 1.0 for exactly three valid cookies with all centres at least 43 mm
  apart and the press clear of them by 47 mm horizontally for 0.5 s. There
  must be less than 0.75 g of dough wasted, and there must be three deposits
  of at least 0.6 g in total.
- A stationary policy cannot dispense dough. A stroke held above the sheet
  spends its dough without producing a valid portion.

Valid deposit positions and arm assignments are interchangeable.

## Simulation

The two arms, free barrel, hand lever and sheet use native PhysX contacts.
The measured lever motion drives a ratchet and meters one rated portion
over approximately 26° of working travel. The lever's spring, rate loss
and dough load give an estimated resisting torque of about 0.6–0.9 N·m
during a quick loaded stroke. This corresponds to roughly 7–11 N at the pad
and is an engineering setting for the simulator.

The reduced material model conserves mass, grows the six-lobe profile at the
actual die position and accounts for unsupported flow as waste. It requires
a seated ring and detects smearing when the rim sweeps over a deposit and
when repeated strokes overlap. An unsupported stroke consumes the same dough
mass as a seated stroke, with the loss accounted for as waste. The scripted
policy uses the ordinary YAM action interface, time and joint positions.

## Run

Install from this task directory in the standard Robocurve native environment.

```bash
python -m pip install -e .
python run.py --device cuda:0 --physics-device cpu --output runs/demo
python run.py --device cuda:0 --physics-device cpu --no-cameras --policy hold --output runs/hold
python run.py --device cuda:0 --physics-device cpu --no-cameras --policy same-spot --output runs/same-spot
```

The fixed scene uses seed 1415158438. The limit is 600 actions at 20 Hz with
physics at 240 Hz. The runner records all native camera views and native
scoring state. The registered Inspect Robots embodiment is `cookie_press`
and the policy is `cookie_press_scripted`.

The reference scores 1.0 in 550 actions (27.5 s) with three 5.7 g portions
and zero waste. The stationary control and the control that attempts repeated
strokes at one location each score 0.0 across 600 actions.

## Asset licenses

The baking sheet, press geometry, die pattern and modelled dough are
task-specific procedural assets under the [MIT license](LICENSE). The runner
and procedural geometry writer reuse the MIT-licensed helpers from
`core-an-apple` in the team's Robocurve source. No manufacturer images or CAD
are included.

| Asset | Source and license |
| --- | --- |
| Left and right YAM arm, parallel grippers, and meshes | [I2RT YAM source][i2rt] at 120c3c81400171174604e503943f8d1ebc891058, [MIT notice][i2rt-license]. The pinned adapter supplies the USD conversion. |
| Black and white YAM split meshes | [Pinned adapter mesh provenance][split-links], [I2RT MIT notice][i2rt-license]. |
| Table, floor and untextured lights | Procedural station fixtures through the [pinned YAM adapter][adapter]. The adapter has [no declared repository license][adapter-license]. [Isaac Lab-Arena][arena] is Apache-2.0. |
| Overhead virtual camera | The task specifies its view pose. The other parameters come from the [YAMLAb calibration][calibration] under its [MIT license][calibration-license]. The sensor is spawned by the pinned adapter. |
| Left wrist virtual camera | [YAMLAb left wrist calibration][calibration], [MIT][calibration-license], through the pinned adapter. |
| Right wrist virtual camera | [YAMLAb right wrist calibration][calibration], [MIT][calibration-license], through the pinned adapter. |

[manual]: https://www.oxo.com/media/wysiwyg/PDF_Files/1243_1257580%20OXO%20Good%20Grips%20Cookie%20Press%20Instruction%20Booklet.pdf
[i2rt]: https://github.com/i2rt-robotics/i2rt/tree/120c3c81400171174604e503943f8d1ebc891058
[i2rt-license]: https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/assets/yam/LICENSE.i2rt
[split-links]: https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/assets/yam/dream/SOURCE
[adapter]: https://github.com/kairosiann/yam-isaac-lab-arena/tree/0059d02a1a91bc6834c1d0846022891df0c78730
[adapter-license]: https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/README.md#license
[arena]: https://github.com/isaac-sim/IsaacLab-Arena/blob/c7edf3a9d5d8276565208f72422f8f8a0806d6cf/LICENSE.md
[calibration]: https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc126418ea5e74ac567133d/yamlab/configs/robot/yam.yaml
[calibration-license]: https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc126418ea5e74ac567133d/LICENSE

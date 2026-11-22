# Load the film in a manual 35 mm camera

> Pull the 35 mm film leader straight to the orange mark, close the back, and work the advance lever until the rewind knob turns.

A reusable 35 mm camera can close and its advance lever can move even if the film leader missed the take-up mechanism. The moving rewind knob verifies that the cartridge is paying out film. This task starts with the cartridge in the well, ready for leader alignment and capture in the manual winding procedure documented in the [Pentax 17 instruction manual, pages 16–18](https://www.ricoh-imaging.co.jp/english/support/man-pdf/pentax17_om_en_web.pdf).

## Physical setup

Use a Pentax 17 with a CR2 battery, a lens cap and the mode dial in flash-off mode. Its orange film-end indicator, rear pressure plate, manual advance lever and rewind fork are the reference for this trial. Use one reusable 35 mm training cartridge loaded with already exposed scrap film and the factory-trimmed narrow leader. Set the ISO dial to the cartridge label. Keep the camera stationary in a padded nonskid cradle with the back up and the controls accessible. Place the cartridge with its shaft protrusion toward the camera bottom and leader facing the gate. Leave the back at its open stop. Set the initial exposed leader so its tip rests 46 ± 2 mm short of the orange index. If needed, use a film leader retriever to reset the same cartridge. A rental is an alternative to buying a camera. [Pentax 17 loading and rewind instructions, pp. 13–20](https://www.ricoh-imaging.co.jp/english/support/man-pdf/pentax17_om_en_web.pdf) · [Example Pentax 17 rental source](https://perfectimagecamera.com/rental/pentax-17-film-camera/)

Set the rewind fork in the cartridge spool and leave the crank in its rest position. If the knob stands proud after closing the physical camera, the cover hand pushes it home before advancing. Turn the camera on during reset and verify the advance lever is available for a loading stroke. One arm draws the leader onto the rails, keeps the film flat at the index, and later works the advance lever. The other arm closes the hinged back over the film. A trial covers the first loading stroke, with the later shutter and counter sequence handled separately.

Use two parallel-gripper YAM arms at 610 mm center spacing on a 747.5 mm bench. The simulated camera is 139 × 85 mm across the back. Place the real or simulated camera with its center about 392 mm forward from the midpoint of the arm mounts and 14 mm to the right. The 25 mm diameter cartridge rests in the open well, with the loading rails running toward the right arm. Both arms start in their normal ready poses with fingers open.

## Judge the physical result

Score the highest milestone that still holds at the finish. Either arm may perform any operation.

| Score | Physical outcome |
|---|---|
| 0.00 | The leader has yet to reach the index with the cartridge seated. |
| 0.50 | The seated cartridge's film leader lies along the film rails and its tip reaches the orange index. |
| 0.75 | The correctly aligned film is captured, with the back fully closed and latched. |
| 1.00 | The advance lever pulls at least 13 mm of film from the cartridge with the back latched, and the rewind knob turns at least 40° during the proof stroke. Verify the resulting state after the sprung lever returns and both grippers clear the controls. |

Before the trial, mark a row of film perforations with fine dry ink on the training strip's margin. Take a dimensioned overhead image of the felt mouth and film at the last open frame before the cover latches. Record the material coordinate of the mouth using the numbered perforations and a millimetre scale in the film plane. Record the closed and hands-clear finish and read the rewind angle from a removable witness. After a timed advance, unlatch by lifting the rewind knob straight up without turning it, hold the cartridge in its well and photograph the felt mouth again before touching the strip. The difference in film coordinates at the mouth is the measured payout. Award 1.0 only if at least 13 mm moved during the timed trial and the other full-score conditions hold.

To distinguish 0.75 from a latched but uncaptured leader, a timed advance showing measured film travel is evidence of capture. If the trial ended with no winding, the grader may perform one diagnostic advance while the back is still latched and without reseating the film. Confirm synchronous rewind-knob motion and measure at least one perforation pitch of actual film travel with the same before-and-after images. This post-trial check can confirm 0.75 only. Its extra film travel is excluded from full credit. If the camera can advance when the film is out of its transport, the lever motion alone earns zero capture credit.

## Reproduce the reference

Use the approved Isaac Lab–Arena environment and the dependencies in `pyproject.toml`. The Inspect Robots embodiment is `film_camera`. The reference uses standard YAM observations and position actions at 20 Hz, with 120 Hz CPU PhysX. The asset geometry is original. The tapered leader is a semirigid contact strip with independent native payout, transverse movement and in-plane skew. A render mesh connects its rear edge to the cartridge slot. The model approximates film bending and sprocket-to-perforation engagement. Both ends of the leader must fit along the loading rails before a bounded clutch engages at closure. A lever stroke then moves the strip under the pressure plate. Native angles and the increase in measured payout after capture determine the score.

```bash
python -m pip install -e . --no-deps
python run.py --device cuda:0 --physics-device cpu --output /tmp/film-reference
python run.py --no-cameras --policy hold --output /tmp/film-idle
python run.py --no-cameras --policy no_advance --output /tmp/film-no-advance
python run.py --no-cameras --policy short_leader --output /tmp/film-short-leader
python run.py --no-cameras --policy misaligned --output /tmp/film-misaligned
```

The misaligned control draws enough film to reach the longitudinal index range while leaving the leader outside the rails laterally. Closing the back and stroking the lever in that state do not engage the film transport.

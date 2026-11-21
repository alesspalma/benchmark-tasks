# Load the film in a manual 35 mm camera

> Pull the 35 mm film leader straight to the orange mark, close the back, and work the advance lever until the rewind knob turns.

A reusable 35 mm camera can close and its advance lever can move even if the film leader missed the take-up mechanism. The moving rewind knob verifies that the cartridge is paying out film. This task starts with the cartridge in the well, ready for leader alignment and capture in the manual winding procedure documented in the [Pentax 17 instruction manual, pages 16–18](https://www.ricoh-imaging.co.jp/english/support/man-pdf/pentax17_om_en_web.pdf).

## Physical setup

Use an empty, reusable manual 35 mm camera with a rear cover, leader index, advance lever and observable film rewind knob. Use a cheap reusable training cartridge with a standard pre-cut photographic film leader. Keep the camera stationary in a soft nonskid cradle with its back facing up and its controls accessible. Place the cartridge in its well, with its leader facing the gate and the film still retracted to its short exposed lead. Seat the cartridge ends under the camera’s retainers. One arm draws the leader to the index and advances the lever. The other arm swings the camera back through its detent and latches it after the leader is aligned. Start with the rear cover held at its open stop. Reset by rewinding the training film, opening the back and restoring the shallow cartridge seat and exposed leader. Use the ISO appropriate for the training film if the camera displays that setting. Work in subdued light with an unexposed roll.

Use two parallel-gripper YAM arms at 610 mm center spacing on a 747.5 mm bench. The camera is 139 × 85 mm across the back. Its center is about 392 mm forward from the midpoint of the arm mounts and 14 mm to the right. The 25 mm diameter cartridge rests in the open well centered 392 mm forward and 44 mm to the left. The orange indexes flank the end of the rear gate. Both arms start in their normal ready poses with fingers open.

## Judge the physical result

Use the highest completed milestone. Either arm may perform any operation.

| Score | Physical outcome |
|---|---|
| 0.00 | The leader has yet to reach the index with the cartridge seated. |
| 0.50 | The seated cartridge's film leader lies along the film rails and its tip reaches the orange index. |
| 0.75 | The correctly aligned film is captured, with the back fully closed and latched. |
| 1.00 | The advance lever pulls at least 13 mm of film from the cartridge with the back latched, and the rewind knob turns at least 40° during the proof stroke. Verify the resulting state after the sprung lever returns and both grippers clear the controls. |

A physical grader can put a removable witness mark on the rewind knob, measure the motion on video, and examine the leader position on a loaded-but-open rear side before scoring closure. Use the intact closed back and moving witness together to confirm transport. The simulated scoring also measures the native film slide distance independently. The lever starts clear of the robot and its movement alone earns no credit.

## Reproduce the reference

Use the approved Isaac Lab–Arena environment and the dependencies in `pyproject.toml`. The Inspect Robots embodiment is `film_camera`. The reference uses standard YAM observations and position actions at 20 Hz, with 120 Hz CPU PhysX. The asset geometry is original. The cartridge and cover are native collision bodies, and the length of film dispensed is a prismatic native travel coordinate. The thin leader is represented as a semirigid strip. Low contact-equivalent forces represent initial registration of the leader near the sprocket teeth. A bounded transport clutch takes up the film once the correctly aligned leader sits under the latched cover and the lever is advanced. Native hinge angles, the payout slide and winding knob are the outcome measurements.

```bash
python -m pip install -e . --no-deps
python run.py --device cuda:0 --physics-device cpu --output /tmp/film-reference
python run.py --no-cameras --policy hold --output /tmp/film-idle
python run.py --no-cameras --policy no_advance --output /tmp/film-no-advance
python run.py --no-cameras --policy short_leader --output /tmp/film-short-leader
```

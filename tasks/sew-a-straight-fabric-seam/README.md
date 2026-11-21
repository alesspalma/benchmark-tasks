# Sew a straight lap seam in canvas

Join two initially separate canvas swatches on a domestic sewing machine. Each hand positions a layer, then the right hand operates the presser-foot lifter and start/stop switch. The narrow overlap has to pass straight across the feed dogs so the needle catches both pieces and leaves a usable seam allowance. Lift only the orange piece to show that the teal piece is attached.

This is a basic overlapping seam used to join canvas panels for covers and upholstery. The guide line and separate swatches make the edge placement and the machine's physical work visible. [Sailrite overlapping seam guide](https://www.sailrite.com/Learning-to-Sew-Part-3-How-to-Sew-Basic-Seams) · [Brother straight-stitch controls](https://help.brother-usa.com/app/answers/detail/a_id/150689/p/7578) · [Singer feed-dog guidance](https://ukhelp.singer.com/en-US/troubleshooting-tips-594637)

## Setup and reset

Use a domestic straight-stitch machine with a hand-actuated start/stop switch and raised presser foot, such as a Brother NS1850D or equivalent with alternate controls clear of the approach to the fabric. Select straight stitch with approximately 4 mm stitch length and the low speed setting. Prethread the needle and bobbin with light upholstery thread compatible with the chosen machine and cloth. Fit the usual presser foot and an ordinary finger guard for a live needle. The starting needle is up and the machine is stopped. Mark a 6.5 mm seam guide on the needle plate with removable tape. Use a Brother-style thread-cutter control or its normal manual cutter to free the finished swatches before lifting. Keep hands and grippers clear of the needle slot during stitching.

Place the machine on a rigid table with its working face and needle plate toward the two arms. Use machine bed about 445 × 435 mm and about 40 mm above the bench surface. The raised presser foot allows approximately 38 mm of access to start the two loose swatches. Coordinate directions are forward from the arm bases and left as seen from behind the robots. The centerline under the needle is 365 mm forward of the arm midpoint, at lateral zero. See `straight_seam/config.py` for tool centers, CAD spacings and reference coordinates.

Cut two flat canvas swatches, each 145 × 140 mm, roughly 1–1.6 mm thick, from ordinary woven medium-weight fabric. Use orange for the upper layer and teal for the underlay. Fit a small 18 × 25 mm quilting clip on the outer portion of each piece, well away from the stitching area. These give the parallel jaws an accessible way to handle the flat textile without pinning both layers together. Place both pieces separately on the machine bed, their long edges pointing forward. The orange center is 244 mm forward and 154 mm left, and the teal center is 239 mm forward and 108 mm right. The two local clip offsets from their centers are about 59 mm toward the arms and 60 mm outward. Neither swatch starts in the presser foot, and the needle starts fully raised. The arm base spacing is approximately 610 mm. Initialize from `yam_arena.constants.READY` with open fingers.

Reset with fresh swatches cut using a ruler or the same template. Move the clips onto the new pieces, clear the needle area and remove any cut thread tails. Restore the measured starting locations, lift the presser foot, and check the machine is stopped. Reference seed `1415158438`. Nothing in the simulation requires a different fixture between trials.

## Judge the finished seam

Start with the two fabric pieces unjoined. Use a ruler and an overhead photo to measure the longest stitched run that goes through both pieces. Use the trial video to see both pieces clear the bed and only the orange piece supported. The allowed maximum difference from the specified 6.5 mm seam allowance is 4.5 mm. A stitch is only credited if it penetrates the actual overlap.

| Score | Outcome |
| --- | --- |
| 0.00 | No stitch catches both layers. |
| 0.25 | At least one stitch catches both layers. |
| 0.50 | At least 34 mm of joined seam with no interval between successive stitches above 12 mm. |
| 0.80 | At least 73 mm of joined seam, no gap above 9 mm, and each stitch meets the seam allowance tolerance. |
| 1.00 | The 0.80 outcome plus a stopped machine, raised foot, and the orange and teal centers at least 24 mm and 21 mm above the bed respectively, with both swatches clear for 0.4 seconds while only the orange swatch is held. |

Either arm can handle either swatch. The result is the stitched material and the unsupported lift, regardless of the tool sequence or hand assignments used to get it.

## Run the simulation

The task is registered as `straight_seam` in Inspect Robots. It uses the standard two YAM RGB wrist views, fixed overview and joint inputs. The scripted demonstration uses elapsed time, observed robot joints and the measured cloth pose to select its final grasp. The fixed tutorial episode lasts at most 850 actions at 20 Hz. Physics runs at 240 Hz.

```sh
python -m pip install -e . --no-deps
python run.py --device cuda:0 --physics-device cpu --output /tmp/straight-seam-demo
python run.py --no-cameras --policy hold --output /tmp/straight-seam-idle
python run.py --no-cameras --policy foot_up --output /tmp/straight-seam-foot-up
python run.py --no-cameras --policy misaligned --output /tmp/straight-seam-offset
```

The machine, presser foot, lever, needle, switch and swatch contacts are native PhysX. The reduced-order feed transfers bounded traction during the measured raised-needle phase only while the lowered foot covers fabric. A physical needle descent through both layers records each stitch at its realized location. The swatches have stiff in-plane geometry and move independently until bounded, compliant thread forces join recorded penetrations. Wrinkle mechanics, bobbin threading, thread tails and the cutter are outside the model. The thin stitch rendering corresponds to these measured locations.

Task geometry and source are original and MIT licensed. The pinned YAM station, cameras and robot meshes come from the existing [YAM Isaac Lab Arena adaptation](https://github.com/kairosiann/yam-isaac-lab-arena/tree/0059d02a1a91bc6834c1d0846022891df0c78730), MIT with robot asset terms in its repository.

# Reload a push-paddle roll towel dispenser

**Instruction** — Reload the open paper towel dispenser with the new roll. Tuck the free edge between the feed rollers and work the paddle until a usable tail protrudes from the bottom slot. Close the cover securely, leaving the tail exposed.

Manual roll towel dispensers use a feed nip that has to catch the loose paper edge before a paddle stroke can thread it. The operator seats the roll, controls the loose edge while working the paddle and latches the dispenser for public use. Building custodians regularly service restroom supplies in schools, offices, healthcare facilities and other establishments. O*NET lists servicing, cleaning or supplying restrooms as a core duty. BLS records 2,432,600 U.S. janitor and building-cleaner jobs in 2025. [O*NET occupational tasks](https://www.onetonline.org/link/details/37-2011.00) · [BLS employment](https://www.bls.gov/OOH/building-and-grounds-cleaning/janitors-and-building-cleaners.htm)

Source the dispenser using the Georgia-Pacific GP PRO 54338A universal push-paddle model or a matching manual push-paddle unit that accepts an approximately 200 mm wide hardwound roll. This is the manufacturer's bottom-feed procedure for an empty dispenser. The same installation works with paper of another colour and compatible core size. [Manufacturer specification](https://www.gppro.com/gp/gppro/p/54338A) · [Manufacturer loading instructions](https://hdsupplysolutions.com/wcsstore/ExtendedSitesCatalogAssetStore/product/fm/additional/11/117289_UseandCareManual.pdf)

## Physical setup and reset

Hang the roll dispenser on a rigid tiled wall, with the bottom outlet lip 754 mm above the floor. Its plastic case is about 320 mm wide, 365 mm high and 265 mm deep, with an open front cover, two rubber feed rollers and an outward-facing spring-returned paddle. Place a service trolley to the dispenser's left, with its open, shallow roll saddle the same height as the dispenser's permanent holders. The saddle axes are parallel and centred 263 mm apart. The robot platform is 748 mm above the floor and its bases are 610 mm apart.

Use an unopened 200 mm wide, 184 mm diameter hardwound paper roll with a 51 mm cardboard core. Break its factory seal and free 110–125 mm of leader from the lower face. Put the roll on the cart, axis horizontal, with the leader hanging down in front of the holders and nowhere in the feed nip. Open the cover fully to the service stop. Before the next trial, remove the installed roll, wind up the paid-out paper or tear it off to leave a fresh clean edge and restore that setup. Use the same specified compatible roll dimensions and reset image each time. The task begins with the previous roll already spent and removed.

One arm supports and tucks the loose edge above the rollers while the other depresses the paddle. Loading from the cart leaves both roll end faces accessible and lets either arm take the guiding role. The paper is free at reset, so seating the roll and operating the lever alone leaves the outlet empty.

## Physical scoring

Measure the exposed tail vertically from the bottom outlet lip to the lower edge of the web. A tail counts only when it is visibly continuous through the outlet to the loaded roll. The roll counts as seated when it rests in the holders, its centre within 25 mm front to back, 19 mm axially and 22 mm vertically of the centre of the seats and its axis within 14° of theirs. The front cover is closed when flush to the jamb within 2° and retained after the hands are clear. Calibrate its closed angle once using the installed face. The evaluator records the highest satisfied row at the end of the trial.

| Score | Physical outcome |
| --- | --- |
| 0.00 | The roll is outside or unsupported by the holders. |
| 0.25 | The roll is seated and less than 25 mm of attached paper protrudes. |
| 0.50 | The roll is seated and 25–69 mm of attached paper protrudes. |
| 0.75 | The roll is seated and at least 70 mm of paper protrudes, with the cover still open or unsecured. |
| 1.00 | The roll is seated, at least 70 mm of attached paper protrudes and the cover stays securely latched with the work area clear for 0.5 seconds. |

## Simulation and runs

The scene uses native rigid bodies for the roll and housing, the over-centre cover's revolute joint and the paddle's spring-returned stroke. The paper model approximates the free edge with a compliant hinged segment on a payout joint. Paddle travel can advance the paper only after its actual leading edge enters the nip. This models the rollers' grip and one-way transmission. The paper web's rigid proxy contacts the grippers and enclosure. Its interactions with the rubber drums and the moving outlet use the reduced-order nip model to represent compliant passage. The same guided 87 mm path from nip to outlet maps actual simulated web travel to the exposed edge in both the rendered scene and the scorer. The paper approximation omits wrinkles, tearing and the complete wound length. The scorer measures the resulting roll, outlet and cover state from the simulation. All task geometry is original and covered by the included license. The YAM station assets and their terms ship with the pinned [YAM Isaac Lab Arena dependency](https://github.com/kairosiann/yam-isaac-lab-arena/tree/0059d02a1a91bc6834c1d0846022891df0c78730).

Use the benchmark's [simulation walkthrough](https://tasks.robocurve.org/walkthrough) to set up Isaac Lab Arena, Python 3.12 and the NVIDIA renderer. In that environment, from this task folder, run the following:

```bash
python -m pip install -e .
python run.py --device cpu --physics-device cpu --no-cameras --policy demo --output /tmp/towel-demo
python run.py --device cpu --physics-device cpu --no-cameras --policy hold --output /tmp/towel-hold
python run.py --device cpu --physics-device cpu --no-cameras --policy no_feed --output /tmp/towel-no-feed
python run.py --device cpu --physics-device cpu --no-cameras --policy no_thread --output /tmp/towel-no-thread
python run.py --device cuda:0 --physics-device cpu --policy demo --width 640 --height 360 --output /tmp/towel-demo-rgb
```

Each run starts in the same layout. The default seed is 1415158438. The run limit is 800 action steps at 20 Hz, or 40 seconds of simulated time. The RGB reference run writes the native overview and two wrist video streams, reset PNGs, geometry hashes, and per-step measurements. `receipt.json` reports the score and termination. The idle control leaves the roll at stock. The two incomplete controls measure loading without a paddle stroke and applying a paddle stroke with an unthreaded edge.

The Inspect Robots embodiment name is `paddle_towels`. It uses the stock YAM RGB and joint observations at 20 Hz. The reference uses elapsed time, robot joint positions and geometric measurements of the visible roll, free paper edge and cover angle. The fixed start and reset seed above are shared by the reference and model runs.

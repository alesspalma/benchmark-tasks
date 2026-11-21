# Snap the loose corner of a shower liner back onto its rod

**Instruction** — Reattach the loose corner of the shower liner to the rod using the open ring on the counter. Snap the ring shut and leave the corner hanging from it.

One corner has slipped off the shower rod. Raise that corner by the reinforced top hem and hold its hole upright. Guide a loose, open ring over the rod, thread its tip through the free eyelet, then press the clasp home. The replacement supports the loose corner once both hands leave. The closed clasp remains visible in the overview and wrist views. [Moen's installed-rod clasp sequence](https://assets.moen.com/shared/docs/product-specifications/sr2100sp.pdf) · [Snap-closure shower rings](https://amazerbath.com/products/plastic-shower-curtain-rings-snap-closure)

## Physical setup and scoring

Use a standard 25.4 mm diameter stainless rod 1.84 m above the floor and across a 1.54 m tile opening. Leave the rest of a 1.73 m long PEVA liner installed. The detached, reinforced outer eyelet has a 21.6 mm bore and a 32 mm outer diameter. The supported neighbouring eyelet is 165 mm away across the upper hem. Use a snap-closure hook compatible with the rod and eyelet. The demonstrated plastic ring measures 87 mm wide by 105 mm tall when closed, with a 9 mm section and an over-centre clasp. Place the ring flat on a shallow 184 × 158 mm tray on the counter ahead of the liner. Mount the trial at heights reachable by the bimanual YAM bench. Mount both arms on the elevated service platform 1.55 m above the bathroom floor and 610 mm apart.

For the fixed simulation reset, X points forward from the arms, Y toward the left arm, and Z vertically upward. The rod runs along Y, centred at X=399 mm and Z=1039 mm, with end flanges at Y=−416 and 1125 mm. The neighbouring eyelet supports the loose corner at (404, 138, 992) mm. The loose corner sags to the lower folded stop. The bathroom floor lies at Z=−800 mm. The service worktop lies at Z=747.5 mm, immediately in front of the liner. Put the replacement ring flat at (268, −170) mm with the spine toward the arms and open clasp toward +X. Let the ring and liner settle and return both arms to the ready pose.

Judge the ring, rod and corner from the end photos and use the video to verify that the hands withdrew for 0.70 s. The ring surrounds the rod when the rod centreline lies inside its loop in the ring's own cross-section. Check whether ring stock goes through the eyelet by viewing the two sides of the reinforced sheet. A clasp within 4° of its shut seat and with a seam no wider than 4.75 mm counts as closed. The full result supports the detached eyelet at least 35 mm above its initial position, with neither hand closer than 47 mm to the ring, clasp or eyelet. The stationary rod and previously loaded rings stay in place. The two hands are interchangeable.

| Score | Outcome |
| --- | --- |
| 0.0 | The replacement does not enclose the rod. |
| 0.32 | The ring surrounds the rod and its clasp and detached eyelet are still open or separate. |
| 0.56 | The ring surrounds the rod and either the clasp is closed or the ring is threaded through the loose eyelet. |
| 0.82 | The ring surrounds the rod, passes through the eyelet, and closes. |
| 1.0 | The raised corner hangs securely from the closed ring for 0.70 s after both hands withdraw. |

## Simulation and replay

The rail, replacement hook, clasp and open eyelet throat use PhysX contacts. The clasp is a native hinge with a bounded over-centre spring. The liner uses the measured corner geometry and a gravity-loaded fold at the adjacent eyelet. The rest of the cloth stays visual and supported. This captures the loose corner's displacement, weight, and eyelet orientation with rigid reinforcement.

Install the package from this task folder into the Isaac Lab–Arena environment. The Inspect Robots embodiment is `snap_liner`. The reference uses ordinary YAM joint commands with measured eyelet, ring and clasp poses for geometric alignment. The fixed seed is `1415158438`, and the trial time limit is 560 commands at 20 Hz.

```sh
python -m pip install -e . --no-deps
python run.py --device cuda:0 --physics-device cpu --output /tmp/reattach-liner-demo
python run.py --device cpu --physics-device cpu --no-cameras --policy hold --output /tmp/reattach-liner-idle
python run.py --device cpu --physics-device cpu --no-cameras --policy ring-only --output /tmp/reattach-liner-control
python run.py --device cpu --physics-device cpu --no-cameras --policy open-only --output /tmp/reattach-liner-open
```

The task geometry and code use the included MIT license. Robot meshes, kinematics and camera calibration come from the pinned dependencies. [Robot asset terms](https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/assets/yam/LICENSE.i2rt) · [Camera asset license](https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc126418ea5e74ac567133d/LICENSE)

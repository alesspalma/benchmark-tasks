# Fit a strip-clamp flat mop pad

A flat mop uses two hinged end clamps to keep a reusable microfibre pad stretched under its head. A strip left out of a clamp can pull free and bunch underneath during a stroke. Fold both loose fixation strips around the ends, close both clamps, and check that the cloth stays fitted when the mop is lifted and set down.

The physical setup uses the 40 cm **Vileda Professional CombiSpeed Pro holder, 143573**, and a compatible 40 cm **MicroLite pocket-and-strip refill, 143243**. Use the strips and clamps on this combination. The maker's strip fixation procedure is to open the clamp, fold the strip in and close the clamp. The folding head remains open and flat throughout. Both parts are stocked as single units. [Single holder](https://www.avodesch.nl/Vileda-CombiSpeed-Pro-Mopframe-40-cm) · [Single pad](https://www.karcher-inte.cz/vileda-microlite-kapsovy-pasky-40-cm-1-ks) · [Frame and part numbers](https://export.vileda-professional.com/floorcleaning/mop-systems/combispeed-pro-system/combispeed-pro) · [Illustrated strip attachment](https://www.koalaproducts.nl/mpattachment/file/download/id/984/) · [Compatible pad](https://assets.vileda-professional.com/int/products/catalogues/vp-product-catalogue/latest/37/)

## Setup and reset

Coordinates are metres in the standard YAM work surface frame. The head runs parallel with Y. Use a dry pad with the cleaning face down and the woven strips up. Rest both loose end strips against their short guides, outside the opened clamps. The pad is loose and can slip before being secured.

| Object | Layout and material |
| --- | --- |
| Bench | 1.18 by 1.25 m laminate-topped workbench, with its top at z=0.7475. The centre is x=0.300, y=0. |
| Lift-out tile | The service tile is 158 by 414 by 43 mm. Its centre is x=0.360, y=0. Two cross-side stops and four small end stops locate the head and leave room to fold the end strips. The head lifts straight out. Make the tile and stops from plywood or HDPE. |
| Strip guides | Use two 43 by 13 by 41 mm blocks. Their plan centres are x=0.3815 and y=±0.222, with their bottoms level with the tile. The loose strips lean on the blocks at the reset so their corners are accessible. |
| Flat mop holder | Use the 400 mm polypropylene holder with glass-reinforced nylon strip clamps and its swivel socket. Place the lower face on top of the loose cloth, centred on the tile. Open both strip clamps to their open stops. Leave the frame extended and its central folding catch engaged. |
| Mop pad | Fit the loose woven pad under the head with its stripes parallel with the long edge and its two fixation strips outside the head. The model uses a 404 by 127 mm central cloth, with a 60 mm long and 43 mm wide strip at each end. Lean one loose strip against each guide. |
| YAM arms | The two bases lie at y = ±0.305. Start both arms in the station ready position with their fingers open. |

Cut the side stops 14 × 156 × 15 mm and centre them at x = 0.360 ± 0.081, y = 0. The four end stops are 22 × 15 × 14 mm, centred at x = 0.360 ± 0.044 and y = ±0.223. The stop heights are measured above the service tile. Mount both side assemblies over 122 × 39 × 43 mm feet, centred along X at the tile centre and at y = ±0.223. Use countersunk screws or reversible adhesive to secure the fixture pieces. The normal handle can be removed from the universal socket for bench service, leaving the swivel socket and short fitting accessible. Use a piece of 23 mm aluminium tube if a short service handle is needed.

Between trials, open both clamps fully, spread the dry pad, set the head back into the lift-out tile, and lean the two loose strips against their guides. Keep the clamps clear of the ends, and check that the head is level and the woven pad symmetric to within 5 mm. Use the same pad after each run unless its strips are torn.

## Score a trial

A strip earns credit when its free end lies under its closed clamp, folds back around the end of the mop, and remains in place after the fingers have left. Inspect the visible strip at the mouth and look from the end of the clamp. A strip against a closed lever on the outside gives no credit. The cleaning face must cover the plate, lie flat and be centred to within 10 mm. With a ruler and two close photos, check that neither strip projects from its bite by more than 10 mm and that both folded ends lie along the head's end walls.

- 0.0 if the pad is displaced, if neither strip is captured, or if the tool is still supported in a hand at the end.
- 0.4 if one strip is retained.
- 0.8 if both strips are retained and the pad lies flat.
- 1.0 if both strips remain retained during a deliberate lift at least 31 mm above the fixture, held for 0.25 s, then the mop is set down within 7° of level and both hands clear it for at least 0.5 s.

In simulation, the score samples the native cloth and clamp poses. A strip must turn inward under the clamp, overlap the bite, and occupy the space between the jaw and its seat. The clamp must be within 8.6° of its closed stop. Lifting after both have closed proves that the native contacts retain the pad rather than the bench supporting it. In a physical trial, use a ruler placed against the tile and the overview recording to observe the lift. Use either hand for either strip.

## Simulation

The scene uses original procedural assets, the licensed YAM robot model, and a 20 Hz action rate with 240 Hz PhysX substeps. The cleaning face is a compact contact proxy with two jointed fabric extensions at each end. Passive springs reproduce the clip detents, and the mounted parts move via native contacts. The simulated handle is a short service fitting on one free swivel axis. The head's two folding halves are latched flat for this service operation. Nylon clamp torque, cloth bending and friction are approximate. Bench evidence comes from simulation.

From the task directory in a YAM environment:

```bash
python -m pip install --no-deps -e .
python -B run.py --device cuda:0 --physics-device cpu --output /tmp/strip-mop-demo
python -B run.py --device cuda:0 --physics-device cpu --no-cameras --policy hold --output /tmp/strip-mop-hold
python -B run.py --device cuda:0 --physics-device cpu --no-cameras --policy one-clamp --output /tmp/strip-mop-one-clip
```

The fixed seed is 1415158438 and the horizon is 680 actions (34 s). The full-score demo uses ordinary gripper pose commands and joint feedback. `run.py` records three native 640 × 360 streams and an initial scene image. The Inspect Robots entry point is `sweeper_sheet`.

## Assets and terms

The workroom, table, fixture, tool, cloth and native geometry are original procedural work, MIT licensed. [I2RT YAM meshes and collision assets](https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/assets/yam/LICENSE.i2rt) retain their own license. Wrist optics use [MIT YAM calibration](https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc126418ea5e74ac567133d/LICENSE) via the pinned adapter. The overview pose and room lights are original.

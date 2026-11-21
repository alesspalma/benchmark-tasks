# Swap a garden-hose nozzle at its water-stop connector

Change the watering accessory on a live garden hose from a round spray head to a fan nozzle. One hand retracts the water-stop connector's collar while the other pulls the old head free. The spring valve closes the exposed hose end during the exchange. The replacement's grooved plug enters axially, depresses the poppet, and is captured when the collar returns. An insertion that opens the valve can still fail to latch. The finished nozzle keeps spraying into the tray with both hands clear. [GARDENA water-stop connector](https://us.gardena.com/products/1-2-quick-repair-with-water-stop) · [Continuous-spray watering nozzle](https://www.gardena.com/uk/products/watering/nozzles-sprayers/classic-water-sprayer/967307001.html)

## Physical setup and reset

The interchangeable accessory coupling is the 13–15 mm hose-end water stop, such as GARDENA 18213-80, mounted on green 13 mm garden hose. Use two compatible snap-in heads, a round spray head and a broad spray head, with their outlets set to continuous flow and pointing downward. The demonstrated fittings are unbranded polypropylene bodies with elastomer grip bands and brass-colored plug inserts. Their grasp sections are 38 × 66 × 38 mm. The stud insert has a 20.4 mm retention bead, 14 mm groove diameter, 6 mm groove width and a tapered valve pilot. The simulated collar has a 43 mm body, 49.5 mm across its grip flats, and moves up to 11 mm along the hose axis. The poppet's working travel is 7.5 mm. Use a plug and water stop from the same sold system when rebuilding, since incomplete combinations can mismatch the latching profile.

Use the bimanual YAM station on 610 mm arm centers and a bench at 747.5 mm height. Measure X forward from the midpoint of the robot mounts, Y toward the left arm and Z up. Set the socket axis along Y at (359, 56, 817) mm, with its opening facing the right arm. A low hose saddle supports the upstream hose about 110 mm behind the socket while allowing the connector end to yield axially. Use a regulated low-flow water supply upstream, adjusted until the attached old head produces about 4–4.5 L/min. The hose and heads should feed a spill tray throughout setup. The gray head starts locked at the socket, with its grasp section centered near (359, −2, 817) mm. The loose blue head rests level at (421, −177, 817) mm on a 29 × 102 mm pedestal, parallel to the gray head with its plug pointing toward positive Y.

Use a plastic catch tray about 509 × 121 mm centered at (310, −62) mm. Put an empty parts tray on a second 29 × 102 mm pedestal centered near (220, −164, 817) mm. The trays and pedestal footprints are in `builder.py`. Use gray and blue removable color bands or naturally distinguishable head bodies. Before each trial, drain and clear the catch tray and parts tray, verify the collar returns on release, reset both nozzles, restore the water feed and return both arms to their ready poses with jaws open. The water feed stays on throughout the episode.

## Physical scoring

Use the side-view video to check the release, the exposed socket and the final latch. The old spray head must leave the connector and the exposed, empty socket must produce no sustained water discharge for at least 0.60 seconds. The old head is parked hands-off on the marked pedestal in the parts tray, within 32 mm laterally and 40 mm axially of the nominal center. The replacement must be captured with the collar within 2 mm of its initial resting position. A pull on the accessory body with the collar released distinguishes a captured plug from an accessory merely pressed into the socket. Running water must discharge through the replacement into the catch tray while neither part is supported by the grippers. Check at least 100 mL in a 2.0-second hands-off window with a graduated jug or digital scale (100 g water), or measure the same rate from the video and scale logging. The socket itself should show no continuous jet.

- **1.0** after the valve shutoff and parked old head, with the replacement seated and locked and at least 100 mL caught during 2.0 seconds of stable hands-off flow.
- **0.60** for the verified shutoff and parked old head.
- **0.30** for the verified shutoff after removal.
- **0.0** otherwise.

Either hand can fill each role. The label of the moving arm is absent from the scorer. A stationary robot leaves the gray nozzle spraying and scores zero.

## Run

Use the approved Isaac Lab–Arena environment and versions pinned in `pyproject.toml`. The fixed reset seed is **1415158438**, with up to **620 actions at 20 Hz**. The scene records three native RGB views. The reference controller uses time, observed joint positions, and simulator measurements of the hose and replacement nozzle for axial alignment. It completes the swap with ordinary arm and gripper actions. The physical state and measured flow enter the scorer. Signed collar, radial locking-dog and poppet positions are native prismatic joint states.

```bash
python -m pip install -e . --no-deps
python run.py --device cuda:0 --physics-device cpu --output /tmp/water-stop-demo
python run.py --policy hold --no-cameras --output /tmp/water-stop-idle
python run.py --policy shallow_insert --no-cameras --output /tmp/water-stop-shallow
```

The Inspect Robots embodiment is `water_stop`. The volume model integrates flow proportional to poppet lift at regulated feed pressure, up to 75 mL/s. A rendered spray is displayed at an outlet when the native valve supplies that seated nozzle. Visible ripples mark the calculated point where the water hits the catch tray. The simulation resolves spring-loaded valve motion from plug contact, plug guidance, dog contact with the grooved plug and hose-end axial stiffness. The collar cam drives the dogs through bounded radial forces determined by the actual sleeve position. Water turbulence, droplet dynamics and long-hose deformation are represented by steady flow, a simple visible spray and lumped compliance.

## Assets

The original scenes, mechanisms and controller use the included MIT license. The pin in `pyproject.toml` supplies the YAM meshes, URDF and calibrated cameras. [Robot asset terms](https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/assets/yam/LICENSE.i2rt) · [Camera asset license](https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc126418ea5e74ac567133d/LICENSE)

# Set the height and tension of a table-tennis net

The table is full size with a clipped post at each end of the net. The far
end is set to playing height. The top cord is slack and the near guide is
too high. The reference turns the winding knob under load with one hand and
presses the near guide through its friction fit with the other. Either
hand assignment and any order are valid.

> Set the table-tennis net to playing height and tension, then let go.

Before play, the net has to hold its shape without a hand on the hardware.
A tight cord can still leave the net tilted. Setting the height alone
leaves the centre slack. Standard posts provide both height and tension
adjustments. ITTF equipment guidance describes an axle winder as one
mechanical way to take up cord. The fixture combines the clip mount and
separate guide and winding adjustments documented for net assemblies.
[STIGA clip net and post][stiga] · [ITTF Net Assembly guide, pp. 65–66][ittf]

## Setup and reset

The y-axis runs along the length of the table and the net spans the x-axis.
The robot bases face +x, away from the working edge. Coordinates below
are in metres in the YAM base frame.

| Object | Size, material and reset pose |
| --- | --- |
| Table | Standard 2.740 × 1.525 m green playing surface, 25 mm wood top, 0.760 m high, 20 mm white outer lines and a 3 mm centreline. The tabletop occupies x = 0.5775 to 2.1025 and y = −1.340 to 1.400. Four 48 mm square steel legs support the table. |
| Near and far clamp posts | Steel clip brackets around the table edge, with rubber pads and upright steel post bodies. Heads at x = 0.425 and 2.255, y = 0.030. Both clips begin firmly mounted. The far groove is 152.5 mm above the deck. |
| Near height guide | Friction-fit steel slider and 32 × 34 × 19 mm polymer cap with the cord groove. Begin with groove height 166.5 mm. It can descend 14 mm within the upright post. |
| Cord and net | 1.830 m cotton-blend net with white 10 mm top tape and a 2.4 mm cord. The cord is captured at both post guides. The lower edge stays approximately 2 mm above the table and the net hangs slack at reset. |
| Near winding adjuster | Steel reel with an 8 mm winding radius, 32 mm handle radius and a rubber-covered 21 mm diameter × 26 mm high knob. The shaft centre is (0.378, −0.055, 0.837). The handle starts at −100° from +x as viewed from above. The 21-tooth ratchet has 17.14° pitch, equivalent to 2.39 mm of cord. The reel begins unwound. |
| Robot | Two pedestal-mounted YAM arms with parallel-jaw fingers, base tops at (0, ±0.305, 0.750). Joint targets [0, 1, 1, −0.6, 0, 0] rad with hands open. |

For a physical trial, fit a clamped 1.83 m net to a regulation table.
Retain both clips during reset. Use a reel-type near adjuster or reproduce
the dimensioned assembly in the native geometry generator. Set the guide
14 mm above playing height and release the winding load. The modelled
cord has 0.75 mm slack before it starts to stretch. Each tooth winds
2.39 mm of cord. Use a scale to check the winding load at the knob and
select a cord that gives the documented gauge readings at playing
tension. Start both hands clear.

## Scoring

Measure the net 300 mm inward from each post using a light 152.5 mm net
height gauge. At the midpoint hang a separate 100 g tension gauge and
read the loaded height. The target is 142.5 mm. A ruler and a small
spring scale can reproduce the observations. Keep hands clear while
reading the gauges.

- **1.0** when both light-gauge heights and the loaded midpoint are within
  4 mm of their targets. The reel remains held by the ratchet and the
  reel and guide stay steady, with both hands at least 64 mm from the
  reel axis and the guide head for 0.75 seconds. The rotation and guide
  speed limits are 0.18 rad/s and 8 mm/s.
- **0.5** when the midpoint passes the tension check after release and
  the released reel and guide both stay steady while the full conditions
  are incomplete.
- **0.25** for a ratchet-retained intermediate take-up after release,
  with cord tension of at least 20 N as read with an inline spring scale.
- **0.0** otherwise.

## Run

Use Robocurve Isaac Lab-Arena and Inspect Robots. Install the task from
its own folder using its pyproject file. Use a new output path for each run.

    python -m pip install -e .
    python run.py --device cuda:0 --physics-device cpu --output runs/demo
    python run.py --device cuda:0 --physics-device cpu --no-cameras --policy hold --output runs/hold
    python run.py --device cuda:0 --physics-device cpu --no-cameras --policy height-only --output runs/height-only
    python run.py --device cuda:0 --physics-device cpu --no-cameras --policy tension-only --output runs/tension-only
    inspect-robots run --embodiment tennis_net --policy tennis_net_scripted \
      --instruction "Set the table-tennis net to playing height and tension, then let go." \
      --max-steps 460 --seed 1415158438 --scorer task_score -E cam_width=640 -E cam_height=360

The fixed seed is 1415158438. Physics runs at 480 Hz and control at
20 Hz. Each trial permits 460 actions over 23 seconds. The native
runner writes the three 640 × 360 camera streams, their initial frames,
a per-action trace, a receipt and the final measurements. The reference
uses the ordinary action interface.

The documented reference reaches 1.0 after 350 actions (17.5 seconds).
Released readings were 152.3 mm and 151.3 mm at the two light gauges,
and 139.0 mm at the loaded midpoint. The reel held 38.8 N on its third
tooth. Holding the reset pose for all 460 actions scores 0.0. Winding the
same reel and releasing it without setting the guide scores 0.5, with a
near gauge reading of 162.6 mm.

## Simulation and fidelity

The hands contact the actual knob and guide head. The reel and guide
have native joints and their loads oppose motion through those joints.
The reel wraps at an 8 mm radius with a 6,200 N/m effective axial cord
stiffness. Winding resistance at the third tooth is about 0.32 N·m.
A load-limited retaining drive transfers the reverse load through
each tooth after physical rotation. A friction shoe opposes guide
movement with up to 4 N. These values specify the modelled fixture.

The cord curve uses an elastic and small-sag span model with a uniform
effective vertical load of 0.24 N/m. The net visual weave and top tape
follow the measured endpoints and calculated sag. The near tail runs
from groove to reel. The primary contacts are between the hands and
the rigid hardware. Separate fabric contact and a suspended gauge are
represented by the span model.

[stiga]: https://www.stigaus.com/products/stiga-premium-clipper-net-post-set
[ittf]: https://documents.ittf.sport/sites/default/files/public/2022-02/2020_BoD_Minutes_20201119_on-line.pdf#page=65

## Third-party assets and licenses

The table, clips, net, cord and reel are generated from this task's source
under the included MIT license. The assets below are provided by runtime
dependencies.

| Asset | Source and use | Applicable license |
| --- | --- | --- |
| Left and right I2RT YAM arms, parallel grippers and collision hulls | The same I2RT model at commit `120c3c81400171174604e503943f8d1ebc891058`, converted from URDF to USD by the [pinned YAM adapter][adapter]. | [MIT][i2rt-license]. |
| Black and white arm-link meshes | I2RT links 2 and 3 split for the Dream Labs station appearance and [supplied with the adapter][split-meshes]. | [MIT][i2rt-license]. |
| Station floor, dome light and lamp | Procedural floor box and untextured lights from the adapter's [station room][room]. The dome light uses Isaac Lab-Arena. | The [adapter has no declared repository license][adapter-license]. Isaac Lab-Arena is [Apache-2.0][arena-license]. |
| Fixed overview camera (`top_cam`) | Virtual RGB camera using [YAMLAb optics][calibration], with its pose set by this task and the sensor spawned by the [adapter][cameras]. | [MIT for the YAMLAb calibration][calibration-license] and the task-authored pose. The [adapter has no declared repository license][adapter-license]. |
| Left wrist camera (`left_cam`) | Virtual RGB camera mounted to the left gripper by the [adapter][cameras] with the [YAMLAb left wrist calibration][calibration]. | [MIT for the YAMLAb calibration][calibration-license]. The [adapter has no declared repository license][adapter-license]. |
| Right wrist camera (`right_cam`) | Virtual RGB camera mounted to the right gripper by the [adapter][cameras] with the [YAMLAb right wrist calibration][calibration]. | [MIT for the YAMLAb calibration][calibration-license]. The [adapter has no declared repository license][adapter-license]. |

[adapter]: https://github.com/kairosiann/yam-isaac-lab-arena/tree/0059d02a1a91bc6834c1d0846022891df0c78730
[adapter-license]: https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/README.md#license
[i2rt-license]: https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/assets/yam/LICENSE.i2rt
[split-meshes]: https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/assets/yam/dream/SOURCE
[room]: https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/scene.py
[cameras]: https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/embodiment.py
[arena-license]: https://github.com/isaac-sim/IsaacLab-Arena/blob/c7edf3a9d5d8276565208f72422f8f8a0806d6cf/LICENSE.md
[calibration]: https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc126418ea5e74ac567133d/yamlab/configs/robot/yam.yaml
[calibration-license]: https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc126418ea5e74ac567133d/LICENSE

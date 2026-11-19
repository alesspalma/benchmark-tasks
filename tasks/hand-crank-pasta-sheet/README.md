# Roll a pasta sheet with a hand-crank roller

This prototype models a bench-clamped home pasta machine. One hand presents a loose pre-flattened dough sheet squarely to the smooth roller nip. The other turns a shaft crank while the rollers draw the dough to the next thickness setting. The [Marcato manual](https://www.marcato.it/sites/default/files/node/documentation/files/2023-08/Atlas%20Ampia_instruction%20manual_1.pdf) specifies successively reduced thickness settings, with nominal 4.8 mm and 3.7 mm for the first two settings. [Marcato's egg-pasta method](https://www.marcato.it/blogs/recipes/recipe-egg-pasta) is a repeatable source for the stock.

The closest board matches found were TK-ZU7VYH (align and feed a pouch into a powered laminator) and TK-YAGTHB (apply rolling-pin pressure over dough). The pasta machine requires manual torque while the second arm presents a pliable sheet to a steel nip. The nip sets product thickness and its rollers drive the sheet in tension.

## Calibration layout

The machine is full size, with two native 150 mm steel rolls of 34 mm diameter at a 3.7 mm gap and a nominal 72 mm radius crank. The wooden stock surface holds one loose sheet of 92 × 86 × 4.8 mm. It overhangs the edge of the cutting board by 32 mm on the left. The catcher is a 170 × 155 mm board below the rolls. The existing station table is 747.5 mm above the floor. The roller center lies 172.5 mm above the table and 445 mm forward of the robot bases. The grip is a 37 mm long, 22 mm diameter knob. The 610 mm bimanual YAM station and three camera locations are defined in `pasta_sheet/environment.py`. The machine is fixed to the table in simulation, representing the manual's clamp.

## Reproduce the native probe

In the pinned YAM Isaac Lab Arena environment, run the reference command from this folder. The probe defaults to 600 actions, a 30 s budget at 20 Hz, and fixed seed `1415158438`. Native PhysX steps at 480 Hz. The runner writes a JSON receipt and numbered action trace under a new output path.

```sh
python -B run.py --no-cameras --policy demo --output /tmp/pasta-native-probe
```

`crank-only`, `feed-only` and `hold` remove the corresponding reference action. A future complete validation needs the continuous released output to survive independent measurements at the lead, midpoint and tail at 3.7 mm class, and should log the full no-op horizon and a non-feed control. Every outcome is scored 0 in this feasibility prototype, even if the crank moves or the sheet leaves its board. The code gives no credit for an approach, applied torque or an apparent paper-thin output alone.

## Material and asset scope

The machine housing, rollers, crank, boards and dough skins are procedural team-authored colliders. Native contacts supply all robot handling forces. A passive torque pair couples the two roller shafts in opposite directions. Two native dough skins with a compressive prismatic spring test the reachable loaded nip. The skins can narrow under contact, then spring back. Permanent deformation, longitudinal elongation and flexible output are not implemented by this contact probe.

| Asset | Source and license |
| --- | --- |
| Machine housing, rollers, crank and grip | Original procedural geometry, project MIT. |
| Stock board and output board | Original procedural geometry, project MIT. |
| Loose dough skin bodies | Original procedural geometry, project MIT. |
| YAM arm meshes and gripper splits | [i2rt YAM asset license](https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/assets/yam/LICENSE.i2rt) and [mesh provenance](https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/assets/yam/dream/SOURCE). |
| Worktop and station room | Procedural room from the [pinned YAM adapter](https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/scene.py). The adapter declares no repository license. |
| Overview camera | [MIT YAMLAb calibration](https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc126418ea5e74ac567133d/LICENSE) and task-authored extrinsics. |
| Left wrist camera | [MIT YAMLAb calibration](https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc126418ea5e74ac567133d/LICENSE). |
| Right wrist camera | [MIT YAMLAb calibration](https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc126418ea5e74ac567133d/LICENSE). |

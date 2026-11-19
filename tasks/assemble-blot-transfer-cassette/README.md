# Clear and close an inert transfer cassette

This prototype reproduces the contact work of holding a full-size wet laminate registered while rolling out trapped air, then closing its cassette. Pocket alignment and air elimination are mechanically distinct from feeding a packaged item into a powered laminator. The trainer uses a 75 × 100 mm working area and a hand brayer with a free 106 mm rubber drum. The operating dimensions and the use of a hand roller to clear gaps are documented by the equipment maker. Internal links in this prototype use original geometry. [Bio-Rad size table](https://www.bio-rad.com/sites/default/files/webroot/web/pdf/lsr/literature/Bulletin_2033C.pdf) · [Bio-Rad cassette documentation](https://www.bio-rad.com/webroot/web/pdf/lsr/literature/10019593D.pdf)

## Setup and score

The setup comprises a fixed ABS cassette with a gray 139 × 106 × 9 mm tray, a registered blue backing patch, a teal 100 × 75 × 4 mm upper sheet, a clear 121 × 92 mm guided cover with an orange lip, and a loose rubber brayer in a shoulder cradle. The tray origin is at (365, 25, 826) mm in the robot station frame, with +x forward and +y toward the left arm. The cover starts 127 mm forward of closed. The brayer axle starts at (365, −116, 857) mm, parallel to x. The upper sheet can shear laterally by 12 mm or compress vertically by 5 mm. Three simulated air gaps are visible through the upper contact face.

A physical training replica can use a transparent PET witness sheet over a smooth silicone pad wetted with plain water. A dimensional photo shows the edges and the visible air pockets. A quarter credit is available for each of the three initial pockets cleared. Complete the score by removing all three pockets, keeping the layer edges aligned within 3 mm, and closing the cover to within 4 mm of its stop for at least half a second. Reset the sheet and cover and restore all three gaps before each trial. A physical specimen has yet to be built or calibrated.

The simulated drum, sheet, cradle and cover make real rigid contacts. The sheet has spring-supported compression and limited lateral travel, and the roller has a free revolute axle. An air pocket clears only after loaded, aligned drum travel and native axle rotation cross it. The colored pocket witness represents an aggregate wet-interface response.

## Run the prototype

Use the repository's Isaac Lab Arena/YAM environment. The scene uses a fixed layout, seed 1415158438, 20 Hz controls and a limit of 540 actions (27 s). From this folder:

```sh
python -B run.py --device cuda:0 --physics-device cpu --no-cameras --output /tmp/cassette-native-prototype
```

The native run writes a reset snapshot, a final snapshot, a contact trace and a score receipt. The prototype presently fails the brayer transport, so the native reference never clears a pocket. The branch is an experimental record.

## Assets and terms

| Asset | Provenance and licence |
| --- | --- |
| Tray and stand | Original procedural geometry, repository task MIT licence |
| Backing, working sheet and gap markers | Original procedural geometry, repository task MIT licence |
| Guided lid and orange grip | Original procedural geometry, repository task MIT licence |
| Brayer handle, supports and rotating roller | Original procedural geometry, repository task MIT licence |
| Cradle | Original procedural geometry, repository task MIT licence |
| Bimanual YAM | [Split I2RT geometry](https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/assets/yam/dream/SOURCE), [I2RT asset terms](https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/assets/yam/LICENSE.i2rt) |
| Station bench and lighting | [Adapter-generated geometry](https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/scene.py), no separate asset licence declared |
| Overview camera | [MIT YAM optics calibration](https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc126418ea5e74ac567133d/LICENSE) with original pose |
| Left wrist camera | [MIT YAM left wrist calibration](https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc126418ea5e74ac567133d/LICENSE) |
| Right wrist camera | [MIT YAM right wrist calibration](https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc126418ea5e74ac567133d/LICENSE) |

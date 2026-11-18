# Core a whole apple with a tube corer

Brace a whole apple, work the ring blade down its stem axis, and lift the cut
plug in the corer. The intact cored fruit can then be stuffed or sliced. A
full-size hand corer with a soft handle, such as the [OXO 20181][corer], is
suitable for the physical trial.

## Setup and reset

Use a firm medium green apple, about 74 mm in diameter, 66 mm high and 155 g,
at room temperature. Stand it stem-up on a 248 × 172 × 10 mm polyethylene
board. Use a 200 mm stainless-steel tube corer with a nominal 21 mm cutting
diameter and a soft handle about 25 mm across. The simulation uses these
working dimensions and an open channel above the blade.

Place the board and corer on the standard YAM worktop, 747.5 mm above the
floor. In the YAM station frame, positive X points forward and positive Y
points left. Place the board centre at X 385 mm, Y 17 mm. Centre the apple
at X 386 mm, Y 29 mm. Lay the corer flat on the worktop with its blade at
X 289 mm, Y −218 mm and its handle toward positive X. Start with both
grippers open and the tool unattached. For another physical trial, remove
the cut plug from the corer, wipe the tool and board, and replace the fruit
with a fresh apple of matching dimensions.

## Run and score

Use the Robocurve YAM simulation environment and run from this directory.

```sh
python -m pip install -e .
python run.py --device cuda:0 --physics-device cpu --output runs/demo
python run.py --device cuda:0 --physics-device cpu --output runs/hold --policy hold --no-cameras
python run.py --device cuda:0 --physics-device cpu --output runs/shallow --policy shallow --no-cameras
```

The reference uses seed 1415158438, 240 Hz physics and 20 Hz control. A
trial is limited to 680 actions (34 seconds). The runner records the setup,
three native 640 × 360 views and per-action checks in `trace.jsonl`. It
reports the score in `receipt.json`. The scripted policy uses the corer and
apple poses for alignment and sends the same form of joint commands as a
learned policy.

- **1.0** when the central plug has been cut free and the apple is intact,
  supported only by the board, upright within 22° of vertical and within
  45 mm of its start. Both grippers clear the fruit, and the corer retains
  the plug with its bottom at least 10 mm above the apple's top for 0.4 s.
- **0.5** when the core has been cut free and the apple is upright on the
  board within 45 mm of its start, while the full condition is incomplete.
- **0.0** otherwise. A cut free plug can be checked by gently lifting
  it after the trial. Use a ruler and side video to judge lift clearance.

## Simulation fidelity

The tool, apple shell and plug are native rigid bodies. The cut interface
uses inset contact meshes to approximate pulp compliance. An aligned blade
must physically travel through the fruit against a rate-dependent axial
reaction before the tissue link releases. The modeled high-speed reaction
is about 9–11 N. This engineering estimate uses [published penetration
measurements][texture] as context. The cited tests use solid probes and
different loading geometries. Limited equal-and-opposite forces represent
the cut pulp wedging in the corer's bore. The model represents the stem
axis cut and retention with geometry and forces, without modeling juice
or individual tissue fibres.

[corer]: https://www.oxo.com/shop/kitchenware/tools-gadgets/fruit-vegetable-tools/corer-475.html
[texture]: https://www.jstage.jst.go.jp/article/hortj/90/3/90_UTD-256/_html/-char/en

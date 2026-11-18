# Core a whole apple

This feasibility prototype models a cook holding a medium Gala apple on a cutting board while working a hand corer down its stem axis. The [OXO 20181](https://www.oxo.com/shop/kitchenware/tools-gadgets/fruit-vegetable-tools/corer-475.html) is a 203 mm stainless-steel corer with a soft handle. The native trials did not verify a cut or extraction.

The apple is 74 mm across and 66 mm high. Its mass including the core is 155 g. The brown polyethylene-equivalent cutting board is 248 × 172 × 10 mm. The setup uses metres and the YAM station coordinate frame. The board is centred at X 0.385 m, Y 0.017 m from the centreline. The apple is at X 0.386 m, Y 0.029 m, stem up. The corer lies flat at X 0.289 m, Y −0.218 m measured at its cutter tip, handle pointing in the positive X direction.

The fruit shell, detached core and tool are native rigid bodies. The model uses a force surrogate to represent tissue cutting and limited contact retention of the cut core. It applies equal and opposite reaction forces from measured geometry and velocities. The script reads actual state only to aim the corer and controls the arms through their joint inputs. The nominal 9–11 N cutting reaction is an engineering estimate. Instrumental apple puncture results give an order of magnitude for tissue loading. They use a different probe and test geometry from a manual tube blade. [Apple flesh puncture study](https://www.jstage.jst.go.jp/article/hortj/90/3/90_UTD-256/_html/-char/en)

Run from this task directory in the benchmark simulator environment.

```sh
python run.py --device cuda:0 --physics-device cpu --output runs/native
```

The native run uses fixed seed 1415158438, 240 Hz physics and 20 Hz controls. The runner writes measured traces and exits on success or at 520 actions. Three native 640 × 360 cameras record the setup and actions. Use `--no-cameras` for a physics trace alone.

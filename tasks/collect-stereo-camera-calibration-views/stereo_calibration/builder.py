"""Original checkerboard and the fixed bench fixtures."""

from . import config as C
from .usd_scene import Writer

FRAME = (.20, .28, .32)
BLACK = (.018, .022, .026)
MARGIN = (.88, .87, .81)
BLUE = (.08, .25, .40)
BENCH = (.57, .60, .55)


def bench(w):
    s = w.stage("bench")
    w.box(s, "/Asset/Table", (1.02, 1.18, .045), (.35, 0., C.TABLE-.0225), BENCH)
    w.box(s, "/Asset/Floor", (7., 7., .02), (0., 0., -.01), (.36, .43, .46))
    for i, x in enumerate((-.11, .79)):
        for j, y in enumerate((-.53, .53)):
            w.box(s, f"/Asset/Leg{i}{j}", (.045, .045, C.TABLE-.045),
                  (x, y, (C.TABLE-.045)/2), FRAME)

    # An open two-slot easel locates the board by its lower edge.
    x = C.BOARD[0]
    w.box(s, "/Asset/StandFoot", (.094, .281, .008), (x, 0., C.TABLE+.004), BLUE)
    for i, y in enumerate((-.090, .090)):
        for j, dx in enumerate((-.022, .022)):
            w.box(s, f"/Asset/StandRail{i}{j}", (.013, .042, .064),
                  (x+dx, y, C.STAND_TOP+.032), BLUE)

    # Synchronized machine-vision heads on one rigid rail and a status strip.
    cx, cz = C.CAMERA_X, C.CAMERA_Z
    w.box(s, "/Asset/RigFoot", (.092, .251, .009), (cx+.018, 0., C.TABLE+.0045), FRAME)
    w.box(s, "/Asset/RigColumn", (.020, .020, .161), (cx+.034, 0., C.TABLE+.087), FRAME)
    w.box(s, "/Asset/RigRail", (.020, .190, .019), (cx+.034, 0., cz), FRAME)
    for i, (_, y, z) in enumerate(C.CAMERAS):
        w.box(s, f"/Asset/Camera{i}", (.033, .041, .034),
              (cx+.019, y, z), BLACK)
        w.cylinder(s, f"/Asset/Lens{i}", .012, .005, (cx+.0031, y, z),
                   (.11, .23, .31), axis="X", collide=False)
    w.box(s, "/Asset/Status", (.026, .206, .031),
          (cx+.020, 0., cz+.041), FRAME, collide=False)
    for i in range(C.ACCEPTED_PAIRS):
        w.box(s, f"/Asset/Signal{i}", (.001, .014, .014),
              (cx+.0066, .024*(i-3.5), cz+.041), BLACK, collide=False)
    return s


def checkerboard(w):
    s = w.stage("checkerboard")
    body = w.body(s, "/Asset/Board", .112, ccd=True)
    w.box(s, body+"/Backing", C.SIZE, (0., 0., 0.), MARGIN)
    cols, rows = (C.GRID[0]+1, C.GRID[1]+1)
    for row in range(rows):
        for col in range(cols):
            center = (
                C.PATTERN_X,
                (col-(cols-1)/2)*C.SQUARE,
                ((rows-1)/2-row)*C.SQUARE,
            )
            color = BLACK if (row+col) % 2 == 0 else (.94, .94, .89)
            w.box(s, f"{body}/Square{row}_{col}", (.0004, C.SQUARE, C.SQUARE),
                  center, color, collide=False)
    return s


def create_assets(directory):
    w = Writer(directory)
    assets = [
        dict(name="bench", type="base", file=w.save(bench(w))),
        dict(name="checkerboard", type="rigid", file=w.save(checkerboard(w)),
             position=C.BOARD.tolist()),
    ]
    w.inventory()
    return {"assets": assets, "settle_actions": 12}


def update_signals(stage, count):
    from pxr import Gf, UsdGeom

    for i in range(C.ACCEPTED_PAIRS):
        path = f"/World/envs/env_0/bench/Signal{i}"
        prim = stage.GetPrimAtPath(path)
        if prim.IsValid():
            color = (.085, .88, .38) if i < count else BLACK
            UsdGeom.Gprim(prim).GetDisplayColorAttr().Set([Gf.Vec3f(*color)])

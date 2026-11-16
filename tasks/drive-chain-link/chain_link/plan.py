"""Nominal fixture program; execution uses time and measured robot joints only."""

import numpy as np

from . import config as C
from .motion import SIDE_X, SIDE_Y, Program


def waypoints(mode="demo"):
    if mode == "hold":
        return {}
    p = Program()
    precision = {
        "reach the central backing bridge",
        "align pins above their bushes",
        "withdraw right fingers from the connecting link",
        "reach cover bridge",
        "position support fingers above backing",
        "reach applicator handle",
        "lower applicator outside the chain",
        "carry loaded applicator under fixture",
        "align loaded applicator beneath chain",
        "align clip planes with the grooves",
        "withdraw left fingers from cover",
    }

    def add(dt, label, **kw):
        fine = label in {
            "align pins above their bushes",
            "reach cover bridge",
            "reach applicator handle",
            "lower applicator outside the chain",
            "align loaded applicator beneath chain",
            "align clip planes with the grooves",
        }
        return p.add(
            dt,
            label,
            settle=label in precision,
            tolerance=(
                0.003
                if label == "reach applicator handle"
                else 0.0015
                if fine
                else 0.002
            ),
            **kw,
        )

    right_park = (np.array([0.32, -0.30, 1.08]), SIDE_X, 1)
    left_park = (np.array([0.32, 0.30, 1.08]), SIDE_X, 1)
    pick = C.CONNECTOR_PICKUP_POSITION + C.CONNECTOR_GRASP
    # Fixed gripper-to-link calibration measured during a free-space teach.
    # Execution still observes only time and ordinary robot joints.
    held_tcp = np.array(
        [0.010697954360406863, 0.0007371936537540761, -0.00042231392260101946]
    )
    held_rotation = np.array(
        [
            [-0.0005541169132320944, -0.012499490835931484, -0.9999217247781392],
            [0.00039449887109565575, 0.999921797747936, -0.012499710363697101],
            [0.9999997686625167, -0.00040139429253202314, -0.0005491425450176446],
        ]
    )
    target = C.CONNECTOR_POSITION + C.CONNECTOR_ROTATION @ held_tcp
    target_rotation = C.CONNECTOR_ROTATION @ held_rotation
    add(
        3,
        "clear the chain before pickup",
        right=(np.array([0.30, -0.47, 1.14]), SIDE_X, 1),
    )
    add(
        3,
        "approach beside the inverted link",
        right=(pick + [-0.07, 0, 0.12], SIDE_X, 1),
    )
    add(3, "lower beside the connector", right=(pick + [-0.07, 0, 0], SIDE_X, 1))
    add(2, "reach the central backing bridge", right=(pick, SIDE_X, 1))
    add(2, "grasp the connector backing", right=(pick, SIDE_X, 0))
    add(
        4,
        "lift pins clear of the loading guides",
        right=(pick + [0, 0, 0.07], SIDE_X, 0),
    )
    add(3, "lift pins above the chain", right=(pick + [0, 0, 0.20], SIDE_X, 0))
    add(
        4,
        "carry above the clamped bushes",
        right=(target + [0, 0, 0.145], target_rotation, 0),
    )
    add(
        3,
        "align pins above their bushes",
        right=(target + [0, 0, 0.065], target_rotation, 0),
    )
    entry = target + [0, 0, 0.0515]
    add(
        3,
        "lower connecting-link pilots to the bore mouths",
        right=(entry, target_rotation, 0),
    )
    add(1, "engage both pilots before releasing the link")
    seated = target + [0, 0, 0.0003]
    add(
        6,
        "seat the connecting pins in both bushes",
        right=(seated, target_rotation, 0.048),
    )
    add(2, "allow the connecting link to settle")
    add(2, "release connector into the bushes", right=(seated, target_rotation, 0.12))
    add(
        2,
        "withdraw right fingers from the connecting link",
        right=(seated + [-0.04, 0, 0], target_rotation, 0.12),
    )
    add(
        1,
        "open the clear right hand",
        right=(seated + [-0.04, 0, 0], target_rotation, 1),
    )
    add(3, "allow the connector to seat under gravity")
    add(
        3,
        "raise empty right hand above the chain",
        right=(target + [-0.04, 0, 0.14], target_rotation, 1),
    )
    add(3, "park right hand before cover fitting", right=right_park)
    if mode in ("connector", "ends"):
        return p.moves
    pick_plate = C.KEEPER_POSITION + C.KEEPER_GRASP
    # Support the fitted cover against the chain underside, leaving the pin
    # grooves accessible. It drops onto the retaining clips on release.
    seat = (
        C.CONNECTOR_POSITION
        + C.KEEPER_GRASP
        + np.array([0, 0, -C.COVER_FIT_Z + 0.0032])
    )
    entry = seat + np.array([-0.045, 0, -0.018])
    add(
        3,
        "clear the left arm before pickup",
        left=(np.array([0.30, 0.46, 1.14]), SIDE_X, 1),
    )
    add(
        3,
        "approach beside the blue cover",
        left=(pick_plate + [-0.07, 0, 0.10], SIDE_X, 1),
    )
    add(3, "lower beside the cover", left=(pick_plate + [-0.07, 0, 0], SIDE_X, 1))
    add(2, "reach cover bridge", left=(pick_plate, SIDE_X, 1))
    add(2, "pinch cover plate", left=(pick_plate, SIDE_X, 0))
    add(
        2,
        "lift cover clear of its guides",
        left=(pick_plate + [0, 0, 0.015], SIDE_X, 0),
    )
    add(3, "lift cover above the fixture", left=(pick_plate + [0, 0, 0.18], SIDE_X, 0))
    add(
        4,
        "carry cover above front opening",
        left=(np.array([entry[0], 0, 1.08]), SIDE_X, 0),
    )
    add(3, "lower cover before service opening", left=(entry, SIDE_X, 0))
    # A 4.5 mm commanded gap on the 5 mm plate lets the pin leads centre it
    # while the lower finger supports its weight.
    cover_grip = 0.048
    add(
        3,
        "align cover bores below pin tips",
        left=(seat + [0, 0, -0.018], SIDE_X, cover_grip),
    )
    support = C.CONNECTOR_POSITION + [-0.005, -0.045, 0.01698]
    support_rotation = SIDE_Y
    add(
        3,
        "position support fingers above backing",
        right=(support + [0, 0, 0.10], support_rotation, 0),
    )
    add(
        2,
        "hold the backing down during cover fitting",
        right=(support, support_rotation, 0),
    )
    # Contact strokes include their timed hold; a hard stop can prevent the
    # unloaded encoder target being reached even after the part is seated.
    add(4, "raise cover over both pins", left=(seat, SIDE_X, cover_grip))
    add(2, "hold seated cover under the chain")
    if mode == "unlocked":
        return p.moves
    # Once both bores are guided on the pins, support the front edge of the
    # cover so the lower finger leaves room for the clip applicator pockets.
    cover_hold = seat + [-0.014, 0, 0]
    add(2, "ease cover grip for applicator clearance", left=(seat, SIDE_X, 0.058))
    add(
        3, "shift cover support toward its front edge", left=(cover_hold, SIDE_X, 0.058)
    )
    add(2, "support cover at its front edge", left=(cover_hold, SIDE_X, cover_grip))
    add(1, "settle cover support before removing backing support")
    # The thin cover needs normal gripping pressure before levelling;
    # the earlier loose support permits it to slip between the finger pads.
    add(2, "firmly grasp cover before levelling", left=(cover_hold, SIDE_X, 0.0))
    cover_grip = 0.0
    # Fixed grasp teach at the guided front edge. Level the plate against the
    # chain underside, leaving room for the retaining clips and guide caps.
    cover_held_tcp = np.array(
        [-0.023603520312218785, 0.0012899017449940764, 0.000503118055184068]
    )
    cover_rotation = np.array(
        [
            [0.04875646460480981, -0.005409812680806723, 0.9987960457901286],
            [0.009127723713963115, 0.9999459880893937, 0.004970469165582423],
            [-0.9987689882145119, 0.008874391848630137, 0.0488032104504969],
        ]
    )
    cover_hold = C.CONNECTOR_POSITION + [0, 0, -C.COVER_FIT_Z + 0.0007] + cover_held_tcp
    add(
        3,
        "level cover and clear retaining grooves",
        left=(cover_hold, cover_rotation, cover_grip),
    )
    add(2, "settle level cover before removing backing support")
    add(
        3,
        "lift support fingers clear of the joint",
        right=(support + [0, 0, 0.10], support_rotation, 0),
    )
    add(3, "park support hand before applicator pickup", right=right_park)
    tool_pick = C.TOOL_POSITION + C.LOADING_ROTATION @ C.TOOL_GRASP
    # The bench guide keeps height/orientation fixed. Ordinary robot contact
    # slides the passive carriage to its physical stops; no tool-pose feedback.
    aligned = tool_pick + [-C.PRESS_X_TRAVEL - 0.001, 0, 0]
    pressed = aligned + [0, -C.PRESS_Y_TRAVEL - 0.002, 0]
    withdrawn = aligned + [0, 0.001, 0]
    returned = tool_pick + [0.001, 0.001, 0]
    # A 32.9 mm opening clears the 24 mm handle while keeping the upper
    # finger below the chain. Fully open fingers hit the service cradle.
    handle_open = 0.35
    add(
        3,
        "approach mounted applicator handle",
        right=(tool_pick + [-0.07, 0, 0.10], SIDE_X, handle_open),
    )
    add(
        3,
        "lower beside mounted applicator",
        right=(tool_pick + [-0.07, 0, 0], SIDE_X, handle_open),
    )
    add(2, "reach applicator handle", right=(tool_pick, SIDE_X, handle_open))
    add(2, "grasp mounted applicator handle", right=(tool_pick, SIDE_X, 0.03))
    add(4, "align guided applicator beneath pins", right=(aligned, SIDE_X, 0.03))
    add(2, "settle guided alignment")
    add(5, "press spring clips into grooves", right=(pressed, SIDE_X, 0.03))
    add(3, "settle clips into the grooves")
    add(2, "allow positive clip capture")
    add(3, "withdraw empty applicator", right=(withdrawn, SIDE_X, 0.03))
    add(3, "return guided applicator to its start", right=(returned, SIDE_X, 0.03))
    add(2, "release mounted applicator", right=(returned, SIDE_X, handle_open))
    add(2, "withdraw from applicator", right=(returned + [-0.07, 0, 0], SIDE_X, handle_open))
    add(3, "park right arm", right=right_park)
    cover_hold = cover_hold + [0, 0, -0.001]
    add(
        3,
        "lower cover gently onto its retaining clips",
        left=(cover_hold, cover_rotation, cover_grip),
    )
    add(1, "settle cover on the retaining clips")
    add(2, "release retained lower cover", left=(cover_hold, cover_rotation, 0.12))
    cover_exit = cover_hold + [-0.04, 0, 0]
    add(
        2,
        "withdraw left fingers from cover",
        left=(cover_exit, SIDE_X, 0.12),
    )
    add(1, "open the clear left hand", left=(cover_exit, SIDE_X, 1))
    add(
        3,
        "raise empty left hand above the chain",
        left=(cover_exit + [0, 0, 0.16], SIDE_X, 1),
    )
    add(3, "park left arm", left=left_park)
    add(3, "settle retained chain repair")
    return p.moves

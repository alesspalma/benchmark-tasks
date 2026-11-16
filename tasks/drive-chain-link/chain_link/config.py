"""Representative 100 mm-pitch conveyor-chain joint; SI units, XYZW rotations."""

import numpy as np

CONTROL_HZ, PHYSICS_HZ, MAX_STEPS = 20, 1200, 4200
TABLE_TOP = 0.7475
DECK_TOP = 0.927
PITCH = 0.100
PIN_DIAMETER = 0.012
BORE_DIAMETER = 0.0122
PLATE_THICKNESS = 0.005
INNER_WIDTH = 0.028
CHAIN_Z = 0.0217
COVER_FIT_Z = 0.0439  # Nominal cover-fitting stroke; before release onto clips.
COVER_Z = 0.0443  # Centre of allowed cover end-float, including its retained seat.
GROOVE_Z = 0.0478
GROOVE_DIAMETER = 0.0106
GROOVE_WIDTH = 0.0014
CLIP_THICKNESS = 0.0010
CLIP_LEAF_OUTER_RADIUS = 0.0061
CLIP_STIFFNESS = 0.8  # N m/rad; permanent native passive spring, never retargeted.
CHAIN_POSITION = np.array([0.4516025403784439, 0.0, DECK_TOP + 0.0242])
CONNECTOR_POSITION = np.array([0.265, 0.0, DECK_TOP + 0.0459])
CONNECTOR_PICKUP_POSITION = np.array([0.40, -0.35, TABLE_TOP + 0.150])
CONNECTOR_ROTATION = np.diag([-1.0, 1.0, -1.0])
CONNECTOR_QUATERNION = np.array([0.0, 1.0, 0.0, 0.0])
CONNECTOR_GRASP = np.array([-0.010, 0.0, 0.0])
CHAIN_GRASP_HEIGHT = 0.0165
KEEPER_POSITION = np.array([0.50, 0.35, TABLE_TOP + 0.150])
KEEPER_QUATERNION = np.array([0.0, 0.0, 0.0, 1.0])
KEEPER_GRASP = np.array([-0.010, 0.0, 0.0])
PRESS_X_TRAVEL, PRESS_Y_TRAVEL = 0.045, 0.0306
TOOL_POSITION = CONNECTOR_POSITION + np.array(
    [PRESS_X_TRAVEL, 0.030, -GROOVE_Z - 0.0002]
)
TOOL_QUATERNION = np.array([0.0, 0.0, 1.0, 0.0])
CLIP_QUATERNION = np.array([0.0, 0.0, -np.sqrt(0.5), np.sqrt(0.5)])
LOADING_ROTATION = np.diag([-1.0, -1.0, 1.0])
TOOL_INSTALL_ROTATION = np.diag([-1.0, -1.0, 1.0])
TOOL_GRASP = np.array([-0.020, 0.175, -0.032])
CLIP_POSITIONS = {
    f"clip{k}": TOOL_POSITION + LOADING_ROTATION @ np.array([0, y, 0])
    for k, y in enumerate((-PITCH / 2, PITCH / 2))
}
RADIUS = PITCH / (2 * np.sin(np.pi / 12))
ANGLES = np.deg2rad(np.arange(195, 526, 30))
NODES = np.c_[RADIUS * np.cos(ANGLES), RADIUS * np.sin(ANGLES), np.zeros(12)]
CLOSED_NODES = NODES.copy()
# The unloaded ends rest in a real locating fixture. The missing connecting
# link is inserted from above; its cover and clips are fitted from below.
END_NODES = {"right": NODES[0], "left": NODES[-1]}
END_BODIES = {"right": "Link00", "left": "Link10"}
GRIPS = {"right": (NODES[0] + NODES[1]) / 2, "left": (NODES[-1] + NODES[-2]) / 2}
ROOT_CENTER = (NODES[0] + NODES[1]) / 2
ROOT_YAW = np.arctan2(*(NODES[1] - NODES[0])[1::-1])
ROOT_POSITION = CHAIN_POSITION + ROOT_CENTER
ROOT_QUATERNION = np.array([0.0, 0.0, np.sin(ROOT_YAW / 2), np.cos(ROOT_YAW / 2)])
CHAIN_SHIFT = CONNECTOR_POSITION[0] - (CHAIN_POSITION + NODES[0])[0]
INSTRUCTION = (
    "Join and retain the two chain ends using the yellow connecting "
    "link, blue cover plate and the two spring clips in the bench-mounted applicator. Leave the assembled joint in its service "
    "cradle, the chain seated on its guide, and both hands released."
)

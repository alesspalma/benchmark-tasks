"""Fixed bench-scale training geometry, SI units and XYZW rotations."""

import numpy as np

CONTROL_HZ, PHYSICS_HZ, MAX_STEPS = 20, 240, 2400
TABLE_TOP = 0.7475
DECK_TOP = 0.837
PITCH = 0.100
CHAIN_POSITION = np.array([0.465, 0.0, 0.845])
CONNECTOR_POSITION = np.array([0.195, 0.0, 0.840])
KEEPER_POSITION = np.array([0.18, -0.35, TABLE_TOP + 0.004])
RADIUS = PITCH / (2 * np.sin(np.pi / 12))
ANGLES = np.deg2rad(np.arange(195, 526, 30))
NODES = np.c_[RADIUS * np.cos(ANGLES), RADIUS * np.sin(ANGLES), np.zeros(12)]
END_NODES = {"right": NODES[0], "left": NODES[-1]}
END_BODIES = {"right": "Link00", "left": "Link10"}
GRIPS = {"right": (NODES[0] + NODES[1]) / 2, "left": (NODES[-1] + NODES[-2]) / 2}
ROOT_CENTER = (NODES[0] + NODES[1]) / 2
ROOT_YAW = np.arctan2(*(NODES[1] - NODES[0])[1::-1])
ROOT_POSITION = CHAIN_POSITION + ROOT_CENTER
ROOT_QUATERNION = np.array([0.0, 0.0, np.sin(ROOT_YAW / 2), np.cos(ROOT_YAW / 2)])
CHAIN_SHIFT = CONNECTOR_POSITION[0] - (CHAIN_POSITION + NODES[0])[0]
INSTRUCTION = (
    "Join the two ends of the open chain on the connecting link. Slide the blue forked retaining plate "
    "sideways under both pin heads toward the chain, until both pin stems reach the closed ends of its slots. "
    "Leave the retained joint in its service cradle and release both hands. This is an unpowered training assembly."
)

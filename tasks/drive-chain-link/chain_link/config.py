"""Fixed unpowered chain-drive service rig, metres and XYZW rotations."""

import numpy as np

CONTROL_HZ, PHYSICS_HZ, MAX_STEPS = 20, 240, 2400
TABLE_TOP, DECK_TOP, PITCH = 0.7475, 0.837, 0.100
CHAIN_Z = DECK_TOP + 0.021
SPROCKET_CENTERS = np.array([[0.30, 0.0], [0.60, 0.0]])
PITCH_RADIUS = PITCH
FRONT_X = 0.30 - np.sqrt(3) * PITCH / 2
NODES = np.array(
    [
        [FRONT_X, -0.05, 0],
        [0.30, -0.10, 0],
        [0.40, -0.10, 0],
        [0.50, -0.10, 0],
        [0.60, -0.10, 0],
        [0.60 + np.sqrt(3) * 0.05, -0.05, 0],
        [0.60 + np.sqrt(3) * 0.05, 0.05, 0],
        [0.60, 0.10, 0],
        [0.50, 0.10, 0],
        [0.40, 0.10, 0],
        [0.30, 0.10, 0],
        [FRONT_X, 0.05, 0],
    ],
    dtype=float,
)
CHAIN_POSITION = np.array([0.0, 0.0, CHAIN_Z])
ROOT_CENTER = (NODES[0] + NODES[1]) / 2
ROOT_YAW = np.arctan2(*(NODES[1] - NODES[0])[1::-1])
ROOT_POSITION = CHAIN_POSITION + ROOT_CENTER
ROOT_QUATERNION = np.array([0.0, 0.0, np.sin(ROOT_YAW / 2), np.cos(ROOT_YAW / 2)])
CONNECTOR_TARGET = np.array([FRONT_X, 0.0, CHAIN_Z + 0.015])
CONNECTOR_POSITION = np.array([0.175, -0.32, CONNECTOR_TARGET[2]])
CONNECTOR_GRASP = np.array([0.0, 0.0, 0.018])
KEEPER_POSITION = np.array([0.18, 0.16, DECK_TOP + 0.006])
KEEPER_GRASP = np.array([0.0, 0.140, 0.055])
KEEPER_INSERTION = np.array([0.014, 0.0, -0.032])
SEAT_POINTS = []
for center, angles in zip(SPROCKET_CENTERS, ((90, 150, 210, 270), (-90, -30, 30, 90))):
    t = np.deg2rad(angles)
    SEAT_POINTS.append(center + PITCH_RADIUS * np.c_[np.cos(t), np.sin(t)])
SEAT_POINTS = np.array(SEAT_POINTS)
INSTRUCTION = (
    "Reconnect the open drive chain around the two sprockets. Insert the orange connecting link "
    "through both chain-end holes and secure its pins with the blue retaining plate. "
    "Leave a closed, retained loop correctly seated on both sprockets, and release both hands. "
    "The drive is unpowered."
)

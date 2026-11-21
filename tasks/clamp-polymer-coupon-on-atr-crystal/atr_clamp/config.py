"""Dimensions and nominal loads for the ATR sample station."""

import numpy as np
from yam_arena.constants import READY
from yam_arena.station import BASES

STOWED = dict.fromkeys(BASES, READY)
KEY = "clamp-polymer-coupon-on-atr-crystal"
INSTRUCTION = (
    "Place the polymer coupon flat over the diamond sampling window. "
    "Keep it centred while turning down the pressure knob to its normal load. "
    "Leave the coupon clamped and withdraw both hands."
)
DESCRIPTION = (
    "The FTIR spectrometer has a circular sampling plate with a small central diamond, "
    "a vertical pressure tip and a knurled pressure knob. The polymer card starts in the sample tray."
)
PHYSICS_HZ = 480
MAX_STEPS = 640
CHECK_DT = 0.05
TABLE_Z = 0.7475
CRYSTAL = np.array([0.388, 0.004, 0.847])
PLATE_RADIUS = 0.029
DIAMOND_R = 0.0009
COUPON_SIZE = np.array([0.0540, 0.0856, 0.0025])
PICK = np.array([0.274, 0.181, 0.80925])
TIP_BOTTOM = CRYSTAL[2] + 0.00385
KNOB = np.array([CRYSTAL[0], CRYSTAL[1], 0.996])
KNOB_R = 0.028
LEAD = 0.0100
CUP_STIFFNESS = 100_000.0
NATIVE_LOAD_SCALE = 0.065
NOMINAL_LOAD = 178.0
FULL_LOAD = 155.0
DIAL = (0.358, -0.093, 0.817)

"""Real-scale overhead earmuff, medium 145 mm adult fit envelope."""
import numpy as np
from yam_arena import station
KEY='earmuff_fit'
TITLE='Fit and adjust hearing protectors on an adult headform'
INSTRUCTION='Put the hearing protectors on the manikin and adjust both cups so they sit properly around its ears. Leave both hands clear when finished.'
DESCRIPTION='A full-size adult headform and overhead twin-wire hearing protector are on the table. Spread the passive sprung headband, place it straight across the crown, enclose each ear inside its cushion opening, adjust the height and angle of both cups, then release. The cups must stay seated with an unbroken cushion contact around both ears. No acoustic attenuation rating is scored.'
MAX_STEPS=2400
PHYSICS_HZ=480
TOP=station.TABLE_TOP
HEAD=np.array([0.48,0.,TOP])
CROWN_HEIGHT=.335
EAR_HEIGHT=.205
HALF_HEAD_WIDTH=.0725
RESET=np.array([.26,0.,TOP+.220])
ARM_ANCHOR=.032
ARM_END=np.array([0.,.026,-.110])
CUP_OUTER=(.040,.052)
CUP_INNER=(.022,.035)
CUSHION_OUTER=(.038,.049)
CUSHION_DEPTH=.012
CUSHION_FACE=.024
PAD_COUNT=16
PAD_TRAVEL=.006
PAD_K=300.0
BAND_K=6.1 # N m/rad estimate to be calibrated against 11.8 N source target.
BAND_DAMPING=.08
SLIDE_TRAVEL=.045
SLIDE_FRICTION=0.0 # Native contact pads supply preload/friction, not a joint coefficient.
GRIP_DEPTH=.004

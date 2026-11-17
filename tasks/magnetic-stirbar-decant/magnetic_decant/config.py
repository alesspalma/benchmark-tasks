"""Fixed ordinary wet-lab bench, SI units; actual-size low-form glassware."""
import numpy as np
from yam_arena import station

KEY = 'magnetic_decant'
TITLE = 'Decant a laboratory liquid while retaining its magnetic stirring bar'
INSTRUCTION = 'Use the magnetic stir-bar restrainer to pour at least 140 mL into the empty beaker, keeping the stirring bar inside its original beaker. Leave both beakers upright and the restrainer on the mat, with both hands released.'
DESCRIPTION = 'Two low-form glass beakers, a blue magnetic stirring-bar restrainer with its central stem grip, and a catch tray are on the bench. The source holds 150 mL clean water and a white 30 mm magnetic stirring bar. Place the restrainer outside the glass to hold the bar against the inner wall while decanting. Both beakers are initially upright, and the tool lies flat on the left mat.'
TOP = station.TABLE_TOP
MAX_STEPS = 2800
PHYSICS_HZ = 480
R_OUT, R_IN, HEIGHT, FLOOR = .035, .033, .095, .003
DRY_MASS = .110
DRY_COM = np.array([0.,0.,.043])
DRY_INERTIA = np.diag([.000148,.000148,.000128])
SOURCE = np.array([.40,-.06,TOP+.008])
RECEIVER = np.array([.22,-.06,TOP+.008])
TOOL = np.array([.25,.24,TOP+.0132])
BAR_LENGTH, BAR_RADIUS, BAR_MASS = .030,.004,.005
BAR = SOURCE+np.array([0.,0.,FLOOR+BAR_RADIUS+.0002])
VOLUME = .000150
TOOL_MOMENT, BAR_MOMENT = 1.0,.16
MAG_SOFTENING = .008
MAGNET_OFFSET = np.array([0.,0.,0.])
DOWN = np.diag([1.,-1.,-1.])
SIDE = np.array([[1.,0.,0.],[0.,0.,1.],[0.,-1.,0.]])
TOOL_SIDE = np.array([[0.,1.,0.],[0.,0.,1.],[1.,0.,0.]])
TOOL_GRASP = np.array([0.,0.,.030])
BEAKER_GRASP = np.array([0.,0.,.055])
TOOL_REL = np.array([0.,.040,.012])

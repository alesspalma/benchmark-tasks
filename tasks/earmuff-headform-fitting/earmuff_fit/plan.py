"""Bimanual cup grasp, elastic spreading and passive height adjustment."""
import numpy as np
from . import config as C
from .motion import SIDE_Y,Program,rotation

def waypoints(mode='demo'):
    if mode=='hold':return {}
    p=Program();orient={'left':rotation('Z',np.pi)@SIDE_Y,'right':SIDE_Y}
    x=C.RESET[0];z=C.RESET[2]-.110;closed=.55
    def both(x,y,z,grip):
        return {n:(np.array([x,s*y,z]),orient[n],grip) for n,s in [('left',1),('right',-1)]}
    p.add(3,'approach outside the cups',**both(x,.12,z+.08,1.))
    p.add(4,'reach ordinary cup sidewalls',**both(x,.058+C.GRIP_DEPTH,z+.002,1.))
    p.add(1,'settle outside cup surfaces')
    p.add(2,'grasp both rigid cups',**both(x,.058+C.GRIP_DEPTH,z+.002,closed))
    p.add(2,'lift the hearing protector',**both(x,.058+C.GRIP_DEPTH,z+.10,closed))
    if mode=='grasp':return p.moves
    spread=.125+C.GRIP_DEPTH
    p.add(3,'spread passive sprung headband',**both(x,spread,z+.10,closed))
    p.add(2,'hold against headband spring')
    if mode=='spread':return p.moves
    orient={'left':rotation('Z',np.pi)@SIDE_Y,'right':SIDE_Y}
    high=C.HEAD[2]+C.CROWN_HEIGHT+.065
    p.add(3,'raise cups clear of adult crown',**both(x,spread,high,closed))
    p.add(4,'guide open protector over the head',**both(C.HEAD[0]+.004,spread,high,closed))
    p.add(4,'lower the open cups to ear height',**both(C.HEAD[0]+.004,spread,C.HEAD[2]+C.EAR_HEIGHT+.002,closed))
    p.add(3,'bring cushions around both pinnae',**both(C.HEAD[0]+.004,.096+C.GRIP_DEPTH,C.HEAD[2]+C.EAR_HEIGHT+.002,closed))
    if mode=='high':return p.moves
    p.add(4,'centre both cups around the ears',**both(C.HEAD[0]+.004,.096+C.GRIP_DEPTH,C.HEAD[2]+C.EAR_HEIGHT+.002,closed))
    p.add(3,'settle even cushion contact',**both(C.HEAD[0]+.004,.094+C.GRIP_DEPTH,C.HEAD[2]+C.EAR_HEIGHT+.002,closed))
    from .motion import SIDE_X
    front=rotation('Z',-np.pi/2)@SIDE_Y
    ear=C.HEAD[2]+C.EAR_HEIGHT+.002
    crown=C.HEAD[2]+C.CROWN_HEIGHT
    rx=C.HEAD[0]+.004
    p.add(2,'release both seated cups before band adjustment',left=(np.array([rx,.094+C.GRIP_DEPTH,ear]),orient['left'],1.),right=(np.array([rx,-(.094+C.GRIP_DEPTH),ear]),orient['right'],1.))
    p.add(1,'clear both cups laterally',left=(np.array([rx,.125,ear+.02]),orient['left'],1.),right=(np.array([rx,-.125,ear+.02]),orient['right'],1.))
    p.add(3,'withdraw hands for crown adjustment',left=(np.array([.30,.24,1.04]),front,1.),right=(np.array([.30,-.24,1.04]),front,1.))
    p.add(3,'approach crown with one open fingertip',right=(np.array([.36,-.12,crown+.09]),front,1.))
    return p.moves

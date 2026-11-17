"""Coordinated native-contact capture and pour with ordinary hand/tool grips."""
import numpy as np
from . import config as C
from .motion import Program,rotation


def waypoints(mode='demo'):
    if mode=='hold':return {}
    separated=mode in {'no-magnet','inversion-no-magnet'}
    p=Program();right=C.SOURCE+C.BEAKER_GRASP
    left=C.TOOL+C.TOOL_GRASP
    side=C.SIDE;tool_side=C.TOOL_SIDE;left_side=tool_side@C.DOWN
    p.add(3,'approach ordinary tool stem and source glass',left=(left+[0,0,.12],C.DOWN,1),right=(right+[0,-.04,.12],side,1))
    p.add(3,'lower around stem / approach free beaker body',left=(left,C.DOWN,1),right=(right,side,1))
    p.add(2,'grasp ordinary tool and glass body',left=(left,C.DOWN,.10),right=(right,side,.72))
    lift=np.array([0,0,.15]);high_source=C.SOURCE+lift
    p.add(3,'lift both free objects',left=(left+lift,C.DOWN,.10),right=(right+lift,side,.72))
    tc=high_source+C.TOOL_REL;tool_grasp=tc+tool_side@C.TOOL_GRASP
    p.add(4,'orient restrainer outside beaker',left=(tool_grasp+[0,.08,0],left_side,.10))
    if not separated:
        p.add(3,'bring magnetic face to glass exterior',left=(tool_grasp,left_side,.10))
        p.add(6,'capture free stirring bar through glass')
        high_tool=tool_grasp+[0,0,.035]
        p.add(6,'raise retained bar along glass wall',left=(high_tool,left_side,.10))
        p.add(3,'settle internal bar retention')
    else:high_tool=tool_grasp+[0,.09,.035]
    if mode=='capture':return p.moves
    pour_tcp=np.array([.29,-.06,C.TOP+.18]);origin=pour_tcp-C.BEAKER_GRASP
    if mode=='alternate':
        pour_tcp[0]+=.005
        origin=pour_tcp-C.BEAKER_GRASP
    tool_rel=C.TOOL_REL+[0,0,.035]+tool_side@C.TOOL_GRASP
    lp=origin+tool_rel
    if separated:lp+=np.array([0,.09,0])
    p.add(4,'carry coordinated beaker and restrainer over receiver',left=(lp,left_side,.10),right=(pour_tcp,side,.72))
    angles=(-35,-65,-90,-110) if mode=='inversion-no-magnet' else (-35,-65,-90)
    for angle in angles:
        R=rotation('Y',np.deg2rad(angle));tool_pos=pour_tcp+R@(tool_rel-C.BEAKER_GRASP)
        if separated:tool_pos+=np.array([0,.09,0])
        p.add(3,'tilt while maintaining external magnetic hold',left=(tool_pos,R@left_side,.10),right=(pour_tcp,R@side,.72))
    p.add(4,'allow liquid stream to clear before returning upright')
    if mode in {'pour','no-magnet','inversion-no-magnet'}:return p.moves
    p.add(5,'return original beaker upright',left=(lp,left_side,.10),right=(pour_tcp,side,.72))
    p.add(4,'return beaker to its support',left=(high_tool,left_side,.10),right=(right+lift,side,.72))
    p.add(3,'separate external restrainer',left=(high_tool+[0,.08,.04],left_side,.10))
    p.add(3,'lower original beaker onto mat',right=(right,side,.72))
    p.add(2,'release supported beaker',right=(right,side,1))
    p.add(3,'withdraw right gripper',right=(right+[0,-.04,.15],side,1),left=(left+lift,C.DOWN,.10))
    p.add(3,'return restrainer flat on mat',left=(left,C.DOWN,.10))
    p.add(2,'release tool',left=(left,C.DOWN,1))
    p.add(3,'withdraw open hands',left=(left+[0,0,.15],C.DOWN,1))
    p.add(4,'leave completed apparatus independently supported')
    return p.moves

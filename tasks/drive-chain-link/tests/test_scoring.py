"""Failure gates and compact-fork geometry only; no physical feasibility claim."""
import copy
import json
from pathlib import Path
import unittest
import numpy as np
from chain_link.config import CHAIN_POSITION,NODES,CONNECTOR_TARGET,KEEPER_INSERTION,CONNECTOR_POSITION,KEEPER_POSITION
from chain_link.scoring import Scorer,simple_loop


def part(p):
    return dict(pose=[*p,0.,0.,0.,1.],velocity=[0.]*6)


def assembly():
    bodies={}
    for i,(a,b) in enumerate(zip(NODES[:-1],NODES[1:])):
        yaw=np.arctan2(b[1]-a[1],b[0]-a[0])
        p=CHAIN_POSITION+(a+b)/2
        bodies[f'Link{i:02}']=dict(pose=[*p,0.,0.,np.sin(yaw/2),np.cos(yaw/2)],velocity=[0.]*6)
    return dict(time_s=0.,bodies=bodies,connector=part(CONNECTOR_TARGET),
                keeper=part(CONNECTOR_TARGET+KEEPER_INSERTION),
                wheel_angles={'wheel0':0.,'wheel1':0.},
                hands={n:dict(opening_m=.09) for n in ('left_robot','right_robot')},
                robot_clearance_m=.04)


class FunctionalScoreGates(unittest.TestCase):
    def test_stationary_repair_is_only_partial_even_after_two_minutes(self):
        s,sc=assembly(),Scorer()
        for i in range(480):
            s['time_s']=i/4
            sc.observe_motion(s)
        m=sc.evaluate(s)
        self.assertTrue(m['closed_loop'])
        self.assertEqual(m['score'],.7)
        self.assertFalse(m['drive_test_passed'])
        self.assertFalse(m['success'])

    def test_reset_layout_is_zero(self):
        s=assembly();s['connector']=part(CONNECTOR_POSITION);s['keeper']=part(KEEPER_POSITION)
        self.assertEqual(Scorer().evaluate(s)['score'],0)

    def test_missing_keeper_is_only_partial(self):
        s=assembly();s['keeper']=part(KEEPER_POSITION)
        self.assertEqual(Scorer().evaluate(s)['score'],.4)

    def test_either_missing_end_is_zero(self):
        for name in ('Link00','Link10'):
            s=assembly();s['bodies'][name]['pose'][0]+=.03
            self.assertEqual(Scorer().evaluate(s)['score'],0)

    def test_keeper_in_slot_mouth_is_not_retained(self):
        s=assembly();s['keeper']['pose'][0]=CONNECTOR_TARGET[0]-.005
        m=Scorer().evaluate(s)
        self.assertTrue(m['plate_seated']);self.assertFalse(m['retained'])
        self.assertEqual(m['score'],.7)

    def test_closed_joint_off_wheels_is_zero(self):
        s=assembly()
        for p in [s['connector'],s['keeper'],*s['bodies'].values()]:p['pose'][1]+=.25
        self.assertEqual(Scorer().evaluate(s)['score'],0)

    def test_spinning_axles_without_chain_travel_never_completes(self):
        sc,s=Scorer(),assembly()
        for theta in np.linspace(0,4*np.pi,1000):
            s['wheel_angles']={n:float(theta) for n in ('wheel0','wheel1')}
            sc.observe_motion(s)
        m=sc.evaluate(s)
        self.assertFalse(m['drive_test_passed']);self.assertFalse(m['success'])

    def test_disconnected_motion_does_not_count(self):
        sc,s=Scorer(),assembly();s['keeper']=part(KEEPER_POSITION)
        for theta in np.linspace(0,4*np.pi,1000):
            s['wheel_angles']={n:float(theta) for n in ('wheel0','wheel1')}
            sc.observe_motion(s)
        self.assertEqual(sc.motion_samples,0)
        self.assertEqual(sc.link_travel,0)

    def test_reset_clears_movement_history(self):
        sc,s=Scorer(),assembly();sc.observe_motion(s);sc.observe_motion(s)
        self.assertGreater(sc.motion_samples,0)
        sc.reset();self.assertEqual(sc.motion_samples,0);self.assertIsNone(sc.motion_previous)

    def test_break_discards_previous_movement_history(self):
        sc,s=Scorer(),assembly();sc.observe_motion(s);sc.observe_motion(s)
        s['keeper']=part(KEEPER_POSITION);sc.observe_motion(s)
        self.assertEqual(sc.motion_samples,0);self.assertIsNone(sc.motion_previous)

    def test_invalid_state_permanently_fails(self):
        sc,s=Scorer(),assembly();s['connector']['pose'][0]=np.nan
        self.assertEqual(sc.evaluate(s)['score'],0)
        self.assertEqual(sc.evaluate(assembly())['score'],0)

    def test_escape_permanently_fails(self):
        sc=Scorer();sc.observe_physics({'escaped':np.array([[.2,.66,.8]])})
        self.assertEqual(sc.evaluate(assembly())['score'],0)

    def test_crossing_and_backtracking_are_rejected(self):
        self.assertFalse(simple_loop(np.array([[0.,0],[1,1],[0,1],[1,0]])))
        self.assertFalse(simple_loop(np.array([[0.,0],[1,0],[.5,0],[.5,1]])))

    def test_evaluation_does_not_mutate_input(self):
        s=assembly();before=copy.deepcopy(s);Scorer().evaluate(s)
        self.assertEqual(s,before)


class CompactKeeperGeometry(unittest.TestCase):
    def test_stems_fit_and_heads_are_blocked_at_both_closed_ends(self):
        records=json.loads((Path(__file__).resolve().parents[1]/'scene-inventory.json').read_text())
        cells=[r for r in records if r.get('asset')=='keeper' and r.get('kind')=='box']
        def occupied(p):
            return any(np.all(abs(np.asarray(p)-r['center_m'])<np.asarray(r['size_m'])/2-1e-9) for r in cells)
        def hits(x,y,radius):
            return any(occupied([x+radius*np.cos(a),y+radius*np.sin(a),0]) for a in np.linspace(0,2*np.pi,180))
        for y in (-.05,.05):
            self.assertFalse(hits(-.010,y,.010))
            self.assertTrue(hits(-.010,y,.014))


if __name__=='__main__':unittest.main()

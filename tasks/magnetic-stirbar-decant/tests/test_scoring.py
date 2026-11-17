"""Scoring outcomes independent of the demonstration's prescribed motion."""
import copy
import numpy as np
from magnetic_decant import config as C
from magnetic_decant.scoring import Scorer


def completed():
 q=[0.,0.,0.,1.]
 objects={n:dict(pose=[*p,*q],velocity=[0.]*6) for n,p in [('source',C.SOURCE),('receiver',C.RECEIVER),('bar',C.BAR),('restrainer',C.TOOL)]}
 return dict(time_s=0.,objects=objects,bounds={'restrainer:Body':np.array([[.21,.2275,C.TOP+.0082],[.29,.2525,C.TOP+.0522]])},material=dict(receiver_ml=145.,captured_delivered_ml=145.,source_ml=3.,spilled_ml=2.,in_flight_ml=0.,mass_balance_ml=150.),robot_clearance_m=.05)


def test_full_transfer_while_held_is_partial_then_released_support_is_full():
 s=completed();s['objects']['source']['pose'][2]+=.15;s['objects']['bar']['pose'][2]+=.15;a=Scorer();assert a.evaluate(s)['score']==.7
 s=completed();assert a.evaluate(s)['score']==.7;s['time_s']=2.05;assert a.evaluate(s)['score']==1.


def test_no_action_native_layout_is_zero():
 s=completed();s['material'].update(receiver_ml=0.,captured_delivered_ml=0.,source_ml=150.,spilled_ml=0.);assert Scorer().evaluate(s)['score']==0.


def test_plain_pour_without_external_retention_cannot_claim_magnetic_work():
 s=completed();s['material']['captured_delivered_ml']=0.;assert Scorer().evaluate(s)['score']==0.


def test_bar_removed_then_returned_latches_zero():
 s=completed();s['objects']['bar']['pose'][2]+=C.HEIGHT;a=Scorer();assert a.evaluate(s)['score']==0.
 s=completed();s['time_s']=10.;assert a.evaluate(s)['score']==0.;assert 'bar_left_original_vessel' in a.violations


def test_bar_end_crossing_mouth_counts_as_leaving_even_if_centre_is_inside():
 s=completed();s['objects']['bar']['pose']=[*(C.SOURCE+[0,0,C.HEIGHT-.005]),0.,-np.sqrt(.5),0.,np.sqrt(.5)]
 assert 'bar_left_original_vessel' in Scorer().evaluate(s)['violations']


def test_spill_and_object_drop_remain_invalid_after_safe_recovery():
 for mutation in ('spill','drop'):
  a=Scorer();s=completed()
  if mutation=='spill':s['material'].update(spilled_ml=11.,receiver_ml=139.,captured_delivered_ml=139.)
  else:s['objects']['restrainer']['pose'][2]=C.TOP-.1
  assert a.evaluate(s)['score']==0.;s=completed();s['time_s']=30.;assert a.evaluate(s)['score']==0.


def test_ordinary_alternative_supported_poses_and_tool_rotation_are_valid():
 s=completed();s['objects']['source']['pose'][0]+=.005;s['objects']['restrainer']['pose'][3:]=[0,0,1,0]
 a=Scorer();assert a.evaluate(s)['score']==.7;s['time_s']=2.1;assert a.evaluate(s)['score']==1.

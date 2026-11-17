"""Counterexamples to mechanical fitting, using independent dimensioned outcomes."""
from copy import deepcopy
import numpy as np
from earmuff_fit import config as C
from earmuff_fit.scoring import Scorer


def physical_fit():
    # Medium145mm head,12N ring load at2.5mm foam compression. All posesXYZW.
    links={'Crown':[*list(C.HEAD+[0,0,.335]),0,0,0,1]}
    for sign,name in [(1,'Left'),(-1,'Right')]:
        center=np.array([.484,sign*.094,.9525])
        links[name+'Cup']=[*center,0,0,0,1]
        for k in range(16):links[f'{name}Pad{k:02}']=[*list(center+[0,sign*.0025,0]),0,0,0,1]
    return {'crown_contact_force_N':1.,'robot_clearance_m':.010,'time_s':0.,'objects':{'protector':{'links':links,'joints':{},'velocity':[0.]*6}},'hands':{'left':{'tcp':[.25,.30,1.]},'right':{'tcp':[.25,-.30,1.]}}}


def score_after_dwell(state):
    s=Scorer();s.evaluate(state);state['time_s']=1.1;return s.evaluate(state)


def test_complete_released_fit():
    r=score_after_dwell(physical_fit());assert r['score']==1 and r['success']


def test_held_final_state_is_incomplete():
    state=physical_fit();state['hands']['left']['tcp']=[.484,.10,.9525]
    r=score_after_dwell(state);assert r['score']==.7 and not r['success']


def test_one_cushion_gap_loses_that_cup():
    state=physical_fit();state['objects']['protector']['links']['RightPad07'][1]-=.002
    r=score_after_dwell(state);assert r['score']==.3 and not r['right']['fit']


def test_pinna_trapped_under_upper_cushion():
    state=physical_fit();links=state['objects']['protector']['links']
    for name,p in links.items():
        if name.startswith('Left'):p[2]-=.020
    r=score_after_dwell(state);assert r['score']==.3 and not r['left']['fit']


def test_unsupported_crown_is_not_full_fit():
    state=physical_fit();state['objects']['protector']['links']['Crown'][2]+=.012
    r=score_after_dwell(state);assert r['score']==0


def test_different_small_valid_fore_aft_fit_is_accepted():
    state=physical_fit()
    for name,p in state['objects']['protector']['links'].items():
        if name!='Crown':p[0]+=.001
    assert score_after_dwell(state)['score']==1


def test_transient_release_needs_full_dwell():
    state=physical_fit();s=Scorer();assert s.evaluate(state)['score']==.7
    state['time_s']=.5;assert not s.evaluate(state)['success']
    state['time_s']=.6;state['hands']['right']['tcp']=[.484,-.10,.9525];s.evaluate(state)
    state['hands']['right']['tcp']=[.25,-.30,1.];state['time_s']=1.5;assert not s.evaluate(state)['success']
    state['time_s']=2.6;assert s.evaluate(state)['success']


def test_in_plane_rotated_cup_does_not_rotate_the_ear_with_it():
    state=physical_fit()
    #90degrees about cup normal: real60mm ear height cannot fit44mm width.
    pose=state['objects']['protector']['links']['LeftCup']
    pose[3:]=[0.,2**-.5,0.,2**-.5]
    assert not score_after_dwell(state)['left']['fit']


def test_ear_sealed_hovering_band_is_not_supported():
    state=physical_fit();state['objects']['protector']['links']['Crown'][2]+=.004
    state['crown_contact_force_N']=0.
    r=score_after_dwell(state);assert r['left']['fit'] and r['right']['fit'] and r['score']==0


def test_near_surface_with_zero_force_is_valid_within_feeler_tolerance():
    state=physical_fit();state['objects']['protector']['links']['Crown'][2]+=.0001
    state['crown_contact_force_N']=0.
    assert score_after_dwell(state)['score']==1


def test_crown_feeler_boundary():
    good=physical_fit();good['objects']['protector']['links']['Crown'][2]+=.00049
    good['crown_contact_force_N']=0.
    assert score_after_dwell(good)['score']==1
    bad=physical_fit();bad['objects']['protector']['links']['Crown'][2]+=.00051
    assert score_after_dwell(bad)['score']==0


def test_unstable_released_fit_is_not_full_score():
    state=physical_fit();state['objects']['protector']['velocity'][0]=.03
    assert score_after_dwell(state)['score']==.7


def test_valid_tilt_uses_face_gap_instead_of_root_height_proxy():
    from earmuff_fit.geometry import crown_surface_gap
    from earmuff_fit.motion import rotation
    from yam_arena import ik
    state=physical_fit();pose=state['objects']['protector']['links']['Crown']
    pose[0]=C.HEAD[0]+.0149
    pose[3:]=ik.matrix_to_quat_xyzw(rotation('Y',.24)).tolist()
    pose[2]+=.0001-crown_surface_gap(pose)
    state['crown_contact_force_N']=0.
    assert pose[2]-(C.HEAD[2]+C.CROWN_HEIGHT)<-.0015
    assert score_after_dwell(state)['score']==1

"""Practical volume, retained bar and independently supported final apparatus."""
import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix
from . import config as C
from .materials import bar_envelope


class Scorer:
    def __init__(self):self.reset()

    def reset(self):self.violations=set();self.clear_since=None

    def observe(self,state):
        source=state['objects']['source']['pose'];bar=state['objects']['bar']['pose']
        b,low,high,radial=bar_envelope(source,bar)
        if high>C.HEIGHT+.003 or low<C.FLOOR-.002 or radial>C.R_IN+.002:
            self.violations.add('bar_left_original_vessel')
        for name,obj in state['objects'].items():
            if not np.all(np.isfinite(obj['pose'])) or not np.all(np.isfinite(obj['velocity'])):self.violations.add('nonfinite_state')
            if obj['pose'][2]<C.TOP-.05:self.violations.add(name+'_off_work_surface')
        if state['material']['spilled_ml']>10.0001:self.violations.add('more_than_10mL_spilled')

    def evaluate(self,state):
        self.observe(state);m=state['material'];delivered=min(m['receiver_ml'],m['captured_delivered_ml'])
        score=0.7 if delivered>=140 else (0.4 if delivered>=70 else 0.)
        stable=True;upright=True;supported=True
        for name in ('source','receiver'):
            o=state['objects'][name];r=quat_xyzw_to_matrix(np.asarray(o['pose'][3:]));p=np.asarray(o['pose'][:3])
            stable&=bool(np.linalg.norm(o['velocity'][:3])<.02 and np.linalg.norm(o['velocity'][3:])<.2)
            upright&=bool(r[2,2]>.98)
            supported&=bool(abs(p[2]-(C.TOP+.008))<.004)
        tool=state['objects']['restrainer'];bounds=np.asarray(state['bounds']['restrainer:Body'])
        tool_supported=bool(bounds[0,2]>=C.TOP+.006 and bounds[0,2]<=C.TOP+.011 and
            bounds[0,0]>=.16 and bounds[1,0]<=.34 and bounds[0,1]>=.16 and bounds[1,1]<=.32 and
            np.linalg.norm(tool['velocity'][:3])<.02 and np.linalg.norm(tool['velocity'][3:])<.2)
        clear=state.get('robot_clearance_m',0.)>=.015
        good=delivered>=140 and m['spilled_ml']<=5 and m['in_flight_ml']<.1 and stable and upright and supported and tool_supported and clear and not self.violations
        if good:
            if self.clear_since is None:self.clear_since=state['time_s']
        else:self.clear_since=None
        seconds=0 if self.clear_since is None else state['time_s']-self.clear_since
        if seconds>=2:score=1.
        if self.violations:score=0.
        return dict(score=score,task_score=score,success=score==1,delivered_eligible_ml=delivered,
            receiver_ml=m['receiver_ml'],spilled_ml=m['spilled_ml'],bar_retained=not bool('bar_left_original_vessel' in self.violations),
            upright=bool(upright),stable=bool(stable),supported=bool(supported),tool_supported=tool_supported,
            hands_clear=clear,released_stable_seconds=seconds,violations=sorted(self.violations),mass_balance_ml=m['mass_balance_ml'])

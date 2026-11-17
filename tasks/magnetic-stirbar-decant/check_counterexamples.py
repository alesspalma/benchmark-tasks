"""Unpaid native outcome trials with unchanged scene, materials and scorer."""
import argparse,json,hashlib
from pathlib import Path
from datetime import UTC,datetime
from inspect_robots import Scene
from inspect_robots.types import Action
from magnetic_decant import config as C
from magnetic_decant.controller import ScriptedPolicy
from magnetic_decant.environment import Environment,source_fingerprint
from run import plain

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);p.add_argument('--device',default='cuda:0');p.add_argument('--modes',nargs='+',default=['hold','no-magnet','inversion-no-magnet','pour','alternate']);a=p.parse_args()
 a.output.mkdir(parents=True,exist_ok=False);source=source_fingerprint();results=[]
 env=Environment(device=a.device,physics_device='cpu',render_images=False)
 try:
  for mode in a.modes:
   scene=Scene(id=C.KEY,instruction=C.INSTRUCTION);obs=env.reset(scene,seed=1415158438);policy=ScriptedPolicy("capture" if mode.startswith("capture-") else mode);initial=plain(env.diagnostics())
   with (a.output/(mode+'-trace.jsonl')).open('w') as trace:
    for k in range(min(C.MAX_STEPS,int(policy.duration*20)+2)):
     result=env.step(Action(data=policy.action_at((k+1)/20,obs.state['joint_pos'])));obs=result.observation
     trace.write(json.dumps(dict(step=k+1,phase=policy.phase,metrics=plain(env.metrics),material=plain(env.material.snapshot()),objects=plain(env.state(light=True)['objects'])))+'\n')
     if k%100==0:print(json.dumps(dict(mode=mode,step=k+1,score=env.metrics['score'])),flush=True)
     if result.terminated:break
   expected=0. if mode.startswith('capture-') else {'hold':0.,'no-magnet':0.,'inversion-no-magnet':0.,'pour':.7,'alternate':1.}[mode]
   record=dict(mode=mode,steps=k+1,initial_metrics=initial.get('material'),final_metrics=plain(env.metrics),final=plain(env.diagnostics()),expected_score=expected,passed=env.metrics['score']==expected)
   if mode.startswith('capture-'):
    from magnetic_decant.materials import capture
    from yam_arena.urdf import quat_xyzw_to_matrix
    import numpy as np
    state=env.diagnostics();sp=state['objects']['source']['pose'];bp=state['objects']['bar']['pose'];bar_height=(quat_xyzw_to_matrix(np.array(sp[3:])).T@(np.array(bp[:3])-sp[:3]))[2]
    record['capture_at_inner_wall']=capture(state);record['bar_height_m']=float(bar_height);record['passed'] &= capture(state) and bar_height>.035 and not env.metrics['violations']
   if mode in {'no-magnet','inversion-no-magnet'}:
    record['bar_lost_physically']='bar_left_original_vessel' in env.metrics['violations']
    record['zero_reason']='bar_left_original_vessel' if record['bar_lost_physically'] else 'no_liquid_collected_with_external_magnetic_restraint'
    record['tilt_degrees']=110 if mode=='inversion-no-magnet' else 90
    if mode=='inversion-no-magnet':record['passed'] &=record['bar_lost_physically']
   results.append(record);(a.output/(mode+'-receipt.json')).write_text(json.dumps(record,indent=2)+'\n');print(json.dumps({k:record[k] for k in ('mode','steps','expected_score','passed','final_metrics')}),flush=True)
 finally:
  env.close();summary=dict(source_sha256=source,source_unchanged=source_fingerprint()==source,dream_api_used=False,paid_episodes=0,completed_at_utc=datetime.now(UTC).isoformat(),results=results,passed=len(results)==len(a.modes) and all(r['passed'] for r in results));(a.output/'receipt.json').write_text(json.dumps(summary,indent=2)+'\n')
 if not summary['passed']:raise SystemExit(1)
if __name__=='__main__':main()

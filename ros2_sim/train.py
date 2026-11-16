#!/usr/bin/env python3
"""Train the waypoint-controller gain by black-box search on real Gazebo rollouts.

This is controller calibration, NOT AirVLA/pi0 training. Training trajectories are
short navigation exercises; full missions and blocked gates are held out for eval.
"""
import argparse
import copy
import json
import os
import signal
import subprocess
import time
from pathlib import Path
from uuid import uuid4
from drone_sim.mission import Executor,ROOT,distance,utc


def train(output):
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    base=json.loads((ROOT/'config/world.json').read_text());trials=[]
    from drone_sim.transport import GazeboTransport
    for gain in [.5,.8,1.1]:
        os.environ['GZ_PARTITION']='drone_train_'+uuid4().hex
        cfg=copy.deepcopy(base);cfg['controller']['kp']=gain
        log=(output/f'gain_{gain}.log').open('w')
        proc=subprocess.Popen(['gz','sim','-s','-r',str(ROOT/'worlds/scenario_1_headless.sdf'),'-v','2'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        ex=None
        try:
            t=GazeboTransport();deadline=time.monotonic()+20
            while t.snapshot()['position'] is None and time.monotonic()<deadline:time.sleep(.1)
            ex=Executor(t,cfg);ex.prepare();ex.active=True;ex.status='CALIBRATING';ex.started=time.monotonic();ex.run_id='calibration_'+str(gain);ex.revision=1
            ex.facts={};ex.session={'events':[],'metrics':{'distance_m':0.,'collision':False,'tracking_samples':[]}}
            began=time.monotonic();waypoints=[[.85,.65,1.6],[1.8,.65,1.6],[1.8,1.4,1.6],[.85,.65,1.6]];errors=[]
            for p in waypoints:
                ex.goto(p,timeout=35);errors.append(distance(t.snapshot()['position'],p))
            duration=time.monotonic()-began;score=duration+30*sum(errors)
            trials.append({'gain':gain,'score':score,'duration_seconds':duration,'terminal_errors_m':errors,'passed':True,'samples':ex.session['metrics']['tracking_samples']})
        except Exception as e:trials.append({'gain':gain,'score':1e6,'passed':False,'error':str(e)})
        finally:
            if ex:ex.close()
            os.killpg(proc.pid,signal.SIGINT)
            try:proc.wait(timeout=8)
            except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
            log.close()
        print('Calibration trial',gain,trials[-1].get('score'),flush=True)
    valid=[t for t in trials if t['passed']]
    report={'method':'black_box_controller_gain_search','neural_policy_training':False,'transport':'gazebo_native','created_at':utc(),
        'training_waypoints':'takeoff + short rectangle near home; no package/gate/delivery examples','trials':trials}
    if valid:
        best=min(valid,key=lambda x:x['score']);base['controller']['kp']=best['gain'];report['selected_gain']=best['gain']
        (output/'trained_config.json').write_text(json.dumps(base,indent=2))
    (output/'training_report.json').write_text(json.dumps(report,indent=2))
    if not valid:raise RuntimeError('No stable controller found; no trained config produced')
    return report

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',default=str(ROOT/'results/calibration'));a=p.parse_args();train(a.output)

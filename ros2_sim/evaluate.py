#!/usr/bin/env python3
"""Run real Gazebo dynamics; no mock backend. Optional learned RGB-D package mode.

Native mode launches an isolated world. ROS mode connects to an already launched
ROS simulator. Every episode writes its full event log and an independent report.
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
from drone_sim.mission import Executor, ROOT, distance


def request(policy='abort',views=2):
    req=json.loads((ROOT/'config/default_mission.json').read_text());spec=req['mission_spec']
    existing=[g for g in spec['goals'] if g['predicate']=='inspected']
    if views!=2:
        spec['goals']=[g for g in spec['goals'] if g['predicate']!='inspected']
        for i in range(views):
            g=copy.deepcopy(existing[0]);g['id']='INSPECT_'+str(i);g['args'][1]='view_'+str(i);spec['goals'].append(g)
        first=next(i for i,a in enumerate(req['robot_plan']) if a['skill']=='INSPECT')
        req['robot_plan']=[a for a in req['robot_plan'] if a['skill']!='INSPECT']
        req['robot_plan'][first:first]=[{'skill':'INSPECT','args':['red_package','view_'+str(i)]} for i in range(views)]
        # Rebuild coherent milestone ordering for the generated benchmark mission.
        chain=[next(g['id'] for g in spec['goals'] if g['predicate']=='found')]+['INSPECT_'+str(i) for i in range(views)]+[g['id'] for g in spec['goals'] if g['predicate'] in ['holding','traversed','placed','at']]
        spec['ordering']=[{'before':a,'after':b,'source':'TASK_SEMANTICS'} for a,b in zip(chain,chain[1:])]
        spec['user_goal']=spec['user_goal'].replace('two viewpoints',str(views)+' viewpoints')
    spec['contingencies']=[{'trigger':'gate_not_clear','action':'BYPASS_GATE' if policy=='bypass' else 'ABORT_AND_RETURN_HOME',
        'authorization':'USER' if policy=='bypass' else 'SYSTEM_POLICY','fallback':'ABORT_AND_RETURN_HOME',
        'source_text':'If the gate is blocked, bypass it.' if policy=='bypass' else 'Default: abort and return home.'}]
    if policy=='bypass':spec['user_goal']+=' If the gate is blocked, bypass it.'
    return req


class TruthEvaluator:
    """Ground-truth subscriber for assessment only; never read by Executor."""
    def __init__(self):
        from gz.transport13 import Node
        from gz.msgs10.pose_v_pb2 import Pose_V
        self.node=Node();self.models={};self.samples=[]
        self.node.subscribe(Pose_V,'/world/mission/pose/info',self.receive)
    def receive(self,m):
        for p in m.pose:
            if p.name in ['drone','red_package']:
                self.models[p.name]=[p.position.x,p.position.y,p.position.z]
        if 'drone' in self.models:self.samples.append(list(self.models['drone']))
    def report(self,outcome,views):
        drone=self.models.get('drone');package=self.models.get('red_package')
        home=drone is not None and distance(drone,[.85,.65,1.6])<.2
        placed=package is not None and abs(package[0]-7.6)<.35 and abs(package[1]-4.15)<.17 and abs(package[2]-.62)<.08
        return {'source':'gazebo_world_pose_evaluation_only','drone_pose':drone,'package_pose':package,
                'home_verified':home,'placement_verified':placed,'samples':len(self.samples),
                'reported_outcome':outcome,'requested_views':views}


def run_episode(scenario=1,policy='abort',views=2,transport_kind='gazebo',config=None,timeout=360,output=None,perception='ideal'):
    out=Path(output or ROOT/'results'/('episode_'+uuid4().hex[:8]));out.mkdir(parents=True,exist_ok=True)
    os.environ['DRONE_SCENARIO']=str(scenario)
    process=None;executor=None;log=None;perception_process=None
    if perception=='rgbd':os.environ['DRONE_PACKAGE_PERCEPTION']='rgbd'
    else:os.environ.pop('DRONE_PACKAGE_PERCEPTION',None)
    try:
        if transport_kind=='gazebo':
            os.environ['GZ_PARTITION']='drone_eval_'+uuid4().hex
            log=(out/'gazebo.log').open('w')
            process=subprocess.Popen(['gz','sim','-s','-r']+(['--headless-rendering'] if perception=='rgbd' else [])+[str(ROOT/'worlds'/f"scenario_{scenario}{'' if perception=='rgbd' else '_headless'}.sdf"),'-v','2'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            from drone_sim.transport import GazeboTransport
            transport=GazeboTransport();truth=TruthEvaluator()
            if perception=='rgbd':perception_process=subprocess.Popen(['/usr/bin/python3',str(ROOT/'rgbd/live_bridge.py')],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        else:
            from drone_sim.transport import RosTransport
            transport=RosTransport();truth=None
        deadline=time.monotonic()+20
        while transport.snapshot()['position'] is None and time.monotonic()<deadline:
            if process and process.poll() is not None:raise RuntimeError('Gazebo exited; inspect '+str(out/'gazebo.log'))
            time.sleep(.1)
        executor=Executor(transport,copy.deepcopy(config));executor.start(request(policy,views));deadline=time.monotonic()+timeout;last_sequence=0
        while executor.active and time.monotonic()<deadline:
            state=executor.state()
            for event in state['session']['events']:
                if event['sequence']>last_sequence and event['message_type']!='OBSERVATION':
                    print(event['message_type'],(event.get('action',{}).get('skill','') if isinstance(event.get('action',{}),dict) else event['action']),event.get('reason',event.get('outcome','')),flush=True);last_sequence=event['sequence']
            (out/'live.json').write_text(json.dumps({'status':state['status'],'sensors':state['sensors'],'facts':state['facts']},indent=2))
            time.sleep(.5)
        if executor.active:executor.cancel();executor.thread.join(timeout=5)
        state=executor.state();session=state['session'];outcome=state['status']
        report=truth.report(outcome,views) if truth else {'source':'ROS_sensor_evidence_only','reported_outcome':outcome}
        expected='MISSION_COMPLETED' if scenario in [1,3] else 'MISSION_COMPLETED_WITH_AUTHORIZED_BYPASS' if policy=='bypass' else 'MISSION_ABORTED_SAFE_RETURN_COMPLETED'
        facts=state['facts']
        report.update(expected_outcome=expected,actual_outcome=outcome,backend=transport.kind,scenario=scenario,policy=policy,
            observed_views=len(facts['inspected']),collision=session['metrics']['collision'],duration_seconds=session['metrics']['duration_seconds'],
            perception_mode=executor.config['perception_mode'],controller=executor.config['controller'])
        report['passed']=outcome==expected and not report['collision'] and (scenario in [2,4] and policy=='abort' or len(facts['inspected'])==views)
        if truth:report['passed']=report['passed'] and report['home_verified'] and (scenario in [2,4] and policy=='abort' or report['placement_verified'])
        (out/'session.json').write_text(json.dumps(session,indent=2));(out/'report.json').write_text(json.dumps(report,indent=2))
        print(json.dumps(report,indent=2),flush=True);return report
    finally:
        if executor:executor.close()
        if perception_process:
            os.killpg(perception_process.pid,signal.SIGINT)
            try:perception_process.wait(timeout=8)
            except subprocess.TimeoutExpired:os.killpg(perception_process.pid,signal.SIGKILL);perception_process.wait()
        if process:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=8)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
        if log:log.close()


def main():
    p=argparse.ArgumentParser();p.add_argument('--scenario',type=int,choices=[1,2,3,4],default=1);p.add_argument('--policy',choices=['abort','bypass'],default='abort');p.add_argument('--views',type=int,choices=range(1,5),default=2);p.add_argument('--transport',choices=['gazebo','ros2'],default='gazebo');p.add_argument('--output');p.add_argument('--config');p.add_argument('--timeout',type=int,default=360)
    p.add_argument('--perception',choices=['ideal','rgbd'],default='ideal')
    a=p.parse_args();cfg=json.loads(Path(a.config).read_text()) if a.config else None
    r=run_episode(a.scenario,a.policy,a.views,a.transport,cfg,a.timeout,a.output,a.perception)
    raise SystemExit(0 if r['passed'] else 1)

if __name__=='__main__':main()

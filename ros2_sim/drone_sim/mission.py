"""Evidence-based skill executor shared by Gazebo and ROS 2 transports."""
from drone_sim.runtime_status import contract_status
from drone_sim.behavior_tree import build_tree, nominal_actions, execute_tree, validate_generated_tree
import copy
import base64
import io
import heapq
import json
import math
import os
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from uuid import uuid4

ROOT=Path(__file__).resolve().parents[1]
DEFAULT_GOAL='Find the red package, inspect it from two viewpoints, pick it up, fly through the gate, place it on the blue platform, and return home.'
SKILLS={'SEARCH','INSPECT','NAVIGATE','PICK','TRAVERSE_GATE','PLACE','RETURN_HOME','LAND','HOVER'}


def distance(a,b): return math.sqrt(sum((x-y)**2 for x,y in zip(a,b)))
def utc(): return datetime.now(timezone.utc).isoformat()


def intersects(a,b,lo,hi):
    t0,t1=0.,1.
    for start,end,mn,mx in zip(a,b,lo,hi):
        delta=end-start
        if abs(delta)<1e-9:
            if start<mn or start>mx: return False
        else:
            x,y=sorted(((mn-start)/delta,(mx-start)/delta));t0=max(t0,x);t1=min(t1,y)
            if t0>t1:return False
    return True


def route(start,end,boxes,clearance=.35):
    """Visibility-graph routing in horizontal cruise plane, with inflated geometry."""
    z=end[2]; rects=[]
    for b in boxes:
        if b['min'][2]-clearance<=z<=b['max'][2]+clearance:
            rects.append(([b['min'][0]-clearance,b['min'][1]-clearance],
                          [b['max'][0]+clearance,b['max'][1]+clearance]))
    def safe(a,b): return not any(intersects(a[:2],b[:2],lo,hi) for lo,hi in rects)
    nodes=[list(start[:2]),list(end[:2])]
    for lo,hi in rects:
        nodes.extend([[x,y] for x in (lo[0]-.03,hi[0]+.03) for y in (lo[1]-.03,hi[1]+.03) if 0.35<x<8.65 and .35<y<4.85])
    costs=[float('inf')]*len(nodes); costs[0]=0; prev={}; queue=[(0,0)]
    while queue:
        cost,u=heapq.heappop(queue)
        if u==1: break
        if cost!=costs[u]:continue
        for v in range(len(nodes)):
            if u==v or not safe(nodes[u],nodes[v]):continue
            new=cost+distance(nodes[u],nodes[v])
            if new<costs[v]:costs[v]=new;prev[v]=u;heapq.heappush(queue,(new,v))
    if not math.isfinite(costs[1]):raise RuntimeError('NO_SAFE_ROUTE')
    out=[];v=1
    while v: out.insert(0,nodes[v]+[z]);v=prev[v]
    return out


def validate_request(req):
    if not isinstance(req,dict):raise ValueError('JSON object required')
    spec=req.get('mission_spec'); plan=req.get('robot_plan'); supplied_tree=plan if isinstance(plan,dict) else None
    if supplied_tree:plan=nominal_actions(supplied_tree)
    if not isinstance(spec,dict) or not isinstance(plan,list) or not 1<=len(plan)<=80:raise ValueError('mission_spec and 1–80 robot_plan steps required')
    if len(json.dumps(req))>500000:raise ValueError('Mission is too large')
    if spec.get('unresolved') or spec.get('assumptions'):raise ValueError('Resolve unknowns and review assumptions before external execution')
    if any(c.get('recognition_status')=='NEEDS_CLARIFICATION' for c in spec.get('contingencies',[])):raise ValueError('Resolve unrecognized gate alternate before execution')
    goals=spec.get('goals',[])
    if not goals or any(not isinstance(g,dict) or not isinstance(g.get('args'),list) for g in goals):raise ValueError('Typed mission goals required')
    allowed={'found','inspected','holding','traversed','placed','at','landed'}
    if any(g.get('predicate') not in allowed for g in goals):raise ValueError('Unsupported mission predicate')
    objects={'found':'red_package','inspected':'red_package','holding':'red_package','placed':'red_package','traversed':'gate_1'}
    for g in goals:
        if g['predicate'] in objects and (not g['args'] or g['args'][0]!=objects[g['predicate']]):raise ValueError('World supports red_package and gate_1 only')
        if g['predicate']=='placed' and g['args'] not in [['red_package','blue_platform'],['red_package','red_platform']]:raise ValueError('Unsupported delivery platform')
        if g['predicate']=='placed' and g['args'][1]=='red_platform' and os.environ.get('DRONE_SCENARIO') not in {'3','4'}:raise ValueError('Red platform requires Scenario 3 or 4')
        if g['predicate']=='at' and g['args'] not in (['drone','home'],['home']):raise ValueError('Only home terminal location is supported')
    if any(g['predicate']=='landed' for g in goals) or any(a.get('skill')=='LAND' for a in plan):raise ValueError('LAND is not commissioned: touchdown sensing is not implemented in this version')
    views=[g['args'][1] for g in goals if g['predicate']=='inspected' and len(g['args'])==2]
    if len(views)>12 or len(views)!=len(set(views)):raise ValueError('Inspection viewpoints must be unique, at most 12')
    for a in plan:
        if not isinstance(a,dict) or a.get('skill') not in SKILLS or not isinstance(a.get('args'),list):raise ValueError('Unsupported robot action')
        args=a['args'];skill=a['skill']
        valid=(skill in {'RETURN_HOME','HOVER','LAND'} and not args or
               skill in {'SEARCH','PICK'} and args==['red_package'] or
               skill=='INSPECT' and len(args)==2 and args[0]=='red_package' and any(g['predicate']=='inspected' and g['args']==args for g in goals) or
               skill=='NAVIGATE' and args in [['target_loc'],['gate_loc'],['blue_platform'],['red_platform'],['home']] or
               skill=='TRAVERSE_GATE' and args==['gate_1'] or
               skill=='PLACE' and args in [['red_package','blue_platform'],['red_package','red_platform']])
        if not valid:raise ValueError('Invalid action arguments: '+str(a))
    ids=[g.get('id') for g in goals]
    if any(not isinstance(i,str) or len(i)>100 for i in ids) or len(ids)!=len(set(ids)):raise ValueError('Unique goal IDs required')
    # Require an executable action for every requested milestone. Never silently drop views.
    mapping={'found':'SEARCH','inspected':'INSPECT','holding':'PICK','traversed':'TRAVERSE_GATE','placed':'PLACE','at':'RETURN_HOME','landed':'LAND'}
    for g in goals:
        if not any(a['skill']==mapping[g['predicate']] and (g['predicate'] in {'at','landed'} or a['args']==g['args']) for a in plan):raise ValueError('Plan omits goal '+g['id'])
    positions={}
    for g in goals:
        positions[g['id']]=next(i for i,a in enumerate(plan) if a['skill']==mapping[g['predicate']] and (g['predicate'] in {'at','landed'} or a['args']==g['args']))
    for edge in spec.get('ordering',[]):
        if edge.get('before') not in positions or edge.get('after') not in positions or positions[edge['before']]>=positions[edge['after']]:raise ValueError('Robot plan violates mission ordering')
    for c in spec.get('constraints',[]):
        if c.get('name')!='minimum_view_separation_deg' or not 0<float(c.get('params',{}).get('degrees',0))<=90:raise ValueError('Unsupported constraint; external executor must not silently ignore it')
    if supplied_tree:
        try:validate_generated_tree(supplied_tree,spec,('blue_platform','red_platform') if os.environ.get('DRONE_SCENARIO') in {'3','4'} else ('blue_platform',))
        except ValueError as error:raise ValueError('Invalid behavior tree or authorized alternates: '+str(error))
    return copy.deepcopy(spec),copy.deepcopy(plan)


class Executor:
    def __init__(self,transport,config=None):
        self.transport=transport; self.config=config or json.loads((ROOT/'config/world.json').read_text())
        self.config=copy.deepcopy(self.config)
        if getattr(transport,'learned_package',False):self.config['perception_mode']='learned_rgbd_package; logical_camera_scene'
        self.base_config=copy.deepcopy(self.config)
        self.lock=threading.RLock(); self.stop=threading.Event();self.decision=threading.Event()
        self.thread=None;self.session=None;self.status='IDLE';self.run_id=None;self.active=False;self.heading=0.
        self.watchdog=threading.Thread(target=self._watchdog,daemon=True);self.closed=False;self.watchdog.start()
        self.last_command=time.monotonic();self.resetting=False;self.reset_failed=False;self.archived_sessions=[]

    def _watchdog(self):
        while not self.closed:
            if self.active and time.monotonic()-getattr(self,'last_command',time.monotonic())>.6:
                self.transport.velocity([0,0,0])
            anchor=getattr(self,'hold_anchor',None)
            if not self.active and anchor is not None:
                s=self.transport.snapshot()
                if s['position'] and time.monotonic()-s['pose_received']<self.config['sensor_timeout']:
                    v=[max(-.3,min(.3,self.config['controller']['kp']*(d-p))) for d,p in zip(anchor,s['position'])]
                    if s['gripper']=='attached':v[2]+=self.config.get('payload_vertical_feedforward',.16)
                    yaw=s['yaw'];body=[math.cos(yaw)*v[0]+math.sin(yaw)*v[1],-math.sin(yaw)*v[0]+math.cos(yaw)*v[1],v[2]]
                    self.transport.velocity(body)
                else:self.transport.velocity([0,0,0])
            time.sleep(.1)

    def camera_evidence(self):
        """Capture latest *past* frames at a milestone; never substitute live URLs."""
        frames=[];now=time.monotonic();wall=time.time()
        with self.transport.lock:
            camera_state=copy.deepcopy(self.transport.state.get('cameras',{}))
            images={c:self.transport.images.get(c) for c in ['front','down']}
        for camera in ['front','down']:
            meta=camera_state.get(camera,{})
            frame={'camera':camera,'label':('Gazebo ' if self.transport.kind=='gazebo_native' else 'ROS2 ')+('forward' if camera=='front' else 'downward')+' camera','source':'event_time_'+self.transport.kind+'_camera'}
            received=meta.get('received',0);age=now-received if received else None
            if age is not None:
                frame.update(age_seconds=round(age,3),captured_at=datetime.fromtimestamp(wall-age,timezone.utc).isoformat())
            if not images[camera]:frame['reason']='No camera frame received'
            elif age is None or age<0 or age>1.5:frame['reason']='Camera frame is stale; not shown as current evidence'
            else:
                try:
                    from PIL import Image
                    image=Image.open(io.BytesIO(images[camera]));image.thumbnail((240,160));out=io.BytesIO();image.convert('RGB').save(out,format='JPEG',quality=65)
                    frame['data_url']='data:image/jpeg;base64,'+base64.b64encode(out.getvalue()).decode('ascii')
                except ImportError:frame['reason']='Thumbnail encoder unavailable (Pillow); live camera remains available'
                except Exception:frame['reason']='Camera frame could not be decoded'
            frames.append(frame)
        return frames

    def event(self,kind,**data):
        if kind=='MISSION_RESULT' or kind=='ACTION_RESULT' and data.get('result')=='SUCCEEDED' and data.get('action',{}).get('skill') in {'SEARCH','INSPECT','PICK','TRAVERSE_GATE','PLACE','RETURN_HOME'}:
            data['camera_evidence']=self.camera_evidence()
        e={'schema_version':'1.0','message_type':kind,'run_id':self.run_id,'mission_revision':getattr(self,'revision',0),'time':utc(),**data}
        with self.lock:
            if self.session is not None:
                e['sequence']=len(self.session['events'])+1;self.session['events'].append(e)
        self.transport.emit('/drone/mission/status',e)
        return e

    def state(self):
        s=self.transport.snapshot(); now=time.monotonic()
        with self.lock:
            return {'schema_version':'1.0','scenario':int(os.environ.get('DRONE_SCENARIO','1')),'transport':self.transport.kind,'connected':s['position'] is not None and now-s['pose_received']<self.config['sensor_timeout'],
                'runtime_contract':contract_status(s,self.session,self.active,self.config),
                'battery':{'available':False,'percent':None,'source':'not_modeled','reserve_verified':False},
                'completion_behavior':'Hover at home at 1.6 m. LAND is not commissioned.',
                'status':self.status,'run_id':self.run_id,'active':self.active,'sensors':s,'reset_supported':callable(getattr(self.transport,'reset_world',None)),
                'pose_age_seconds':now-s['pose_received'] if s['pose_received'] else None,
                'perception_mode':self.config['perception_mode'],'pose_source':self.config['pose_source'],
                'facts':copy.deepcopy(getattr(self,'facts',{})),'session':copy.deepcopy(self.session)}

    def prepare(self):
        # Harmonic's DetachableJoint starts attached (initial_attach is ignored
        # in older releases). Require an observed detach acknowledgement before
        # flying; repeat requests while transport discovery completes.
        end=time.monotonic()+8
        while time.monotonic()<end:
            self.transport.grip(False)
            self.transport.velocity([0,0,0])
            if self.transport.snapshot()['gripper']=='detached':return
            time.sleep(.1)
        raise ValueError('Gripper startup detach was not acknowledged')

    def start(self,req):
        spec,plan=validate_request(req)
        with self.lock:
            if self.resetting or self.reset_failed:raise ValueError('Complete a verified world reset before starting a mission')
            if self.active:raise ValueError('A mission is already running')
            if self.session is not None:raise ValueError('Restart the simulator and gateway for a fresh episode before running another mission')
            if not self.state()['connected']:raise ValueError('No fresh simulator odometry. Start Gazebo and the ROS bridge first.')
            self.config=copy.deepcopy(self.base_config);self.heading=0.
            self.prepare()
            self.stop.clear();self.decision.clear();self.run_id=uuid4().hex;self.revision=1;self.status='EXECUTING';self.active=True
            self.spec=spec;self.plan=plan;self.execution_tree=copy.deepcopy(req['robot_plan']) if isinstance(req['robot_plan'],dict) else build_tree(spec,plan);self.original_pddl=req.get('pddl','');self.original_validation=req.get('validation');self.initial_position=self.transport.snapshot()['position'];self.started=time.monotonic();self.last_command=self.started
            self.facts={'found':False,'inspected':{},'holding':False,'picked':False,'placed':False,'traversed':False,'at_home':False,'landed':False}
            self.session={'session_id':self.run_id,'started_at':utc(),'backend':self.transport.kind,'perception_mode':self.config['perception_mode'],
                'events':[],'versions':[],'interrupts':[],'human_decisions':[],'outcome':'RUNNING','metrics':{'distance_m':0.,'collision':False,'replans':0,'tracking_samples':[]}}
            self.version('INITIAL',spec,plan)
            self.thread=threading.Thread(target=self.run,daemon=True);self.thread.start()
            return {'run_id':self.run_id}

    def version(self,reason,spec,plan):
        self.session['versions'].append({'version':self.revision,'time':utc(),'reason':reason,'mission_spec':copy.deepcopy(spec),
            'validation':{'status':'ACCEPTED','html_validation':getattr(self,'original_validation',None),'scope':'supported arguments, goal coverage, runtime geometry and observations'},'robot_plan':copy.deepcopy(self.execution_tree) if reason=='INITIAL' else build_tree(spec,plan),'nominal_plan':copy.deepcopy(plan),
            'pddl':self.original_pddl if self.revision==1 else '; Recovery executed by policy controller; no solver-generated recovery PDDL.\n','state':self.transport.snapshot()})

    def reset_world(self,scenario=None):
        if scenario is not None and (type(scenario) is not int or scenario not in {1,2,3,4}):raise ValueError('Scenario must be 1, 2, 3 or 4')
        with self.lock:
            if self.active or self.resetting:raise ValueError('Stop the mission before resetting the world')
            if not callable(getattr(self.transport,'reset_world',None)):raise ValueError('This transport does not support world reset')
            self.resetting=True;self.status='RESETTING';self.hold_anchor=None
        try:
            if self.thread:self.thread.join(timeout=3)
            if self.thread and self.thread.is_alive():raise RuntimeError('Execution thread has not stopped')
            self.transport.velocity([0,0,0]);self.transport.clear_observations(preserve_feedback=True)
            evidence=self.transport.reset_world() if scenario is None else self.transport.reset_world(scenario=scenario)
            if scenario is not None:os.environ['DRONE_SCENARIO']=str(scenario)
            state=self.transport.snapshot()
            if not state['pose_received'] or time.monotonic()-state['pose_received']>self.config['sensor_timeout']:raise RuntimeError('Fresh odometry missing after reset')
            with self.lock:
                if self.session is not None:self.archived_sessions.append(copy.deepcopy(self.session))
                self.session=None;self.run_id=None;self.thread=None;self.status='IDLE';self.reset_failed=False
                self.config=copy.deepcopy(self.base_config);self.heading=0.;self.stop.clear();self.decision.clear()
                self.facts={'found':False,'inspected':{},'holding':False,'picked':False,'placed':False,'traversed':False,'at_home':False,'landed':False}
                return {'status':'IDLE','reset_evidence':evidence,'archived_sessions':len(self.archived_sessions)}
        except Exception:
            self.status='RESET_FAILED';self.reset_failed=True;raise
        finally:self.resetting=False

    def cancel(self):
        self.stop.set();self.decision.set();self.transport.velocity([0,0,0]);return {'status':'STOP_REQUESTED'}

    def human(self,action):
        if self.status!='WAITING_FOR_HUMAN':raise ValueError('No pending human decision')
        if action not in {'BYPASS_GATE','ABORT_AND_RETURN_HOME'}:raise ValueError('Invalid recovery action')
        self.human_action=action;self.session['human_decisions'].append({'time':utc(),'action':action});self.decision.set()

    def observed(self,name,max_age=None):
        s=self.sense();o=s['objects'].get(name)
        if not o or time.monotonic()-o['received']>(max_age or self.config['sensor_timeout']):raise RuntimeError('OBJECT_NOT_OBSERVED: '+name)
        return o

    def sense(self):
        if self.stop.is_set():raise RuntimeError('CANCELLED')
        s=self.transport.snapshot()
        if not s['position'] or time.monotonic()-s['pose_received']>self.config['sensor_timeout']:
            self.transport.velocity([0,0,0]);raise RuntimeError('STALE_ODOMETRY')
        if getattr(self,'facts',{}).get('holding') and not getattr(self,'releasing',False) and s['gripper']!='attached':raise RuntimeError('PAYLOAD_LOST')
        if any(any(k in name for k in ['obstacle','gate_','blue_platform','red_platform']) for pair in s['contacts'] for name in pair):
            self.session['metrics']['collision']=True;raise RuntimeError('CONTACT_COLLISION')
        x,y,z=s['position']
        if not(-.1<x<9.1 and -.1<y<5.3 and .15<z<3.2):raise RuntimeError('GEOFENCE')
        return s

    def command(self,v,yaw_rate=0):
        self.last_command=time.monotonic();self.transport.velocity(v,yaw_rate)

    def hold(self,seconds):
        pos=self.sense()['position'];self.goto(pos,timeout=seconds+5,settle=seconds)

    def boxes(self):
        return self.config['obstacles']+[
            {'min':[4.84,3.57,0],'max':[4.96,3.73,2.5]},
            {'min':[6.14,3.57,0],'max':[6.26,3.73,2.5]}]

    def move(self,dest):
        s=self.sense();start=s['position']
        # First change altitude at current XY, then route at destination altitude.
        if abs(start[2]-dest[2])>.15:self.goto([start[0],start[1],dest[2]])
        for waypoint in route(self.sense()['position'],dest,self.boxes(),self.config['clearance']):self.goto(waypoint)

    def goto(self,dest,timeout=None,settle=None):
        cfg=self.config['controller'];end=time.monotonic()+(timeout or self.config['action_timeout']);stable=None;last=None
        settle=cfg['settle_seconds'] if settle is None else settle
        while time.monotonic()<end:
            s=self.sense();p=s['position'];err=[d-x for d,x in zip(dest,p)]
            if last:self.session['metrics']['distance_m']+=distance(p,last)
            last=p
            self.session['metrics']['tracking_samples'].append(distance(p,dest))
            yaw_error=math.atan2(math.sin(self.heading-s['yaw']),math.cos(self.heading-s['yaw']))
            if abs(yaw_error)<.08 and distance(p,dest)<cfg['tolerance'] and distance(s['velocity'],[0,0,0])<.16:
                stable=stable or time.monotonic()
                if time.monotonic()-stable>=settle:self.command([0,0,0]);return
            else:stable=None
            v=[max(-cfg['max_speed'],min(cfg['max_speed'],cfg['kp']*e)) for e in err]
            if s['gripper']=='attached':v[2]+=self.config.get('payload_vertical_feedforward',0.16)
            yaw=s['yaw'];body=[math.cos(yaw)*v[0]+math.sin(yaw)*v[1],-math.sin(yaw)*v[0]+math.cos(yaw)*v[1],v[2]]
            self.command(body,max(-.7,min(.7,1.5*yaw_error)))
            time.sleep(.05)
        self.command([0,0,0]);raise RuntimeError('WAYPOINT_TIMEOUT: '+str(dest))

    def survey_gate(self):
        # A generated tree may test a gate condition before an explicit NAVIGATE.
        # Obtain evidence at the inspection pose instead of testing from pickup.
        self.event('GATE_SURVEY_STARTED',survey_pose=[5.55,2.8,1.6])
        self.heading=math.pi/2
        self.move([5.55,2.8,1.6]);self.hold(.6)
        deadline=time.monotonic()+3
        while True:
            try:
                clear=self.gate_clear()
                self.event('GATE_SURVEY_RESULT',gate_clear=clear,status=self.transport.snapshot())
                return clear
            except RuntimeError as error:
                if str(error) not in {'GATE_CLEAR_UNKNOWN','STALE_GATE_SENSOR'} or time.monotonic()>=deadline:raise
                self.hold(.2)

    def gate_clear(self):
        s=self.sense();now=time.monotonic()
        if any(now-s['semantic_received'].get(c,0)>self.config['sensor_timeout'] for c in ['down','front']):raise RuntimeError('STALE_GATE_SENSOR')
        blocker=s['objects'].get('gate_blocker')
        if blocker and now-blocker['received']<2:return False
        # Both posts must actually be visible to declare a surveyed gate clear.
        if not all(n in s['objects'] and now-s['objects'][n]['received']<2 for n in ['gate_left','gate_right']):raise RuntimeError('GATE_CLEAR_UNKNOWN')
        return True

    def skill(self,a):
        k=a['skill'];args=a['args'];cfg=self.config
        if k=='SEARCH':
            self.move(cfg['target_search']);o=self.observed('red_package');self.facts['found']=True;self.target=copy.deepcopy(o['position'])
            self.event('OBSERVATION',object_id='red_package',pose=o['position'],source=o['source'])
        elif k=='INSPECT':
            views=[g for g in self.spec['goals'] if g['predicate']=='inspected'];idx=next(i for i,g in enumerate(views) if g['args']==args)
            target=self.observed('red_package',10)['position'];angle=math.pi+idx*2*math.pi/len(views)
            self.move([target[0]+.65*math.cos(angle),target[1]+.65*math.sin(angle),1.6]);self.hold(.4)
            o=self.observed('red_package');pos=self.sense()['position']
            if any(distance(pos,p)<.3 for p in self.facts['inspected'].values()):raise RuntimeError('INSPECTION_VIEW_NOT_DISTINCT')
            v1=[pos[i]-target[i] for i in range(2)]
            minimum=max([float(c['params']['degrees']) for c in self.spec.get('constraints',[]) if c['name']=='minimum_view_separation_deg'] or [30])
            for prior in self.facts['inspected'].values():
                v2=[prior[i]-target[i] for i in range(2)];den=distance(v1,[0,0])*distance(v2,[0,0])
                angle=math.degrees(math.acos(max(-1,min(1,sum(a*b for a,b in zip(v1,v2))/max(den,1e-9)))))
                if angle<minimum:raise RuntimeError('INSPECTION_ANGLE_TOO_SMALL')
            self.facts['inspected'][args[1]]=list(pos);self.event('INSPECTION_EVIDENCE',viewpoint=args[1],pose=pos,object_pose=o['position'],source=o['source'])
        elif k=='NAVIGATE':
            loc=args[0]
            if loc=='target_loc':
                p=self.observed('red_package',10)['position'];self.move([p[0],p[1],1.6])
            elif loc=='gate_loc':
                self.survey_gate()
            elif loc in ['blue_platform','red_platform']:self.move([7.6,4.15 if loc=='blue_platform' else 2.65,1.6])
            elif loc=='home':self.move(cfg['home'])
        elif k=='PICK' and getattr(self.transport,'learned_package',False):
            self.pick_rgbd()
        elif k=='PICK':
            o=self.observed('red_package');p=o['position'];self.target=list(p)
            self.move([p[0],p[1],p[2]+cfg['grasp']['offset']]);self.hold(.5)
            p=self.observed('red_package')['position'];d=self.sense()['position'];offset=cfg['grasp']['offset']
            if distance(d[:2],p[:2])>cfg['grasp']['xy_tolerance'] or abs(d[2]-p[2]-offset)>cfg['grasp']['z_tolerance']:raise RuntimeError('GRASP_ALIGNMENT_FAILED')
            # Simulated fingers use an attachment joint, never teleport the package.
            self.transport.grip(True);self.wait_gripper('attached');self.goto([d[0],d[1],d[2]+cfg['grasp']['test_lift']]);self.hold(.5)
            lifted=self.observed('red_package')['position']
            if lifted[2]<self.target[2]+.18:raise RuntimeError('TEST_LIFT_FAILED')
            self.facts['holding']=True;self.facts['picked']=True;self.event('GRASP_EVIDENCE',attachment='attached',before=self.target,after=lifted)
            self.move([d[0],d[1],1.6])
        elif k=='TRAVERSE_GATE':
            if not self.survey_gate():raise RuntimeError('GATE_BLOCKED')
            self.move([5.55,2.95,1.6]);before=self.sense()['position'];self.goto([5.55,4.35,1.6]);after=self.sense()['position']
            if not(before[1]<3.3 and after[1]>4 and abs(after[0]-5.55)<.2):raise RuntimeError('TRAVERSAL_NOT_VERIFIED')
            self.facts['traversed']=True;self.event('TRAVERSAL_EVIDENCE',before=before,after=after)
        elif k=='PLACE':
            if not self.facts['holding']:raise RuntimeError('NOT_HOLDING')
            platform_y=4.15 if args[1]=='blue_platform' else 2.65
            self.move([7.6,platform_y,1.6]);self.goto([7.6,platform_y,.5+.12+cfg['grasp']['offset']]);self.hold(.4)
            self.releasing=True;self.transport.grip(False);self.wait_gripper('detached');self.facts['holding']=False;self.releasing=False
            self.goto([7.6,platform_y,1.6]);self.hold(1.)
            o=self.observed('red_package');p=o['position']
            if abs(p[0]-7.6)>.35 or abs(p[1]-platform_y)>.17 or abs(p[2]-.62)>.08:raise RuntimeError('PLACEMENT_NOT_VERIFIED')
            self.facts['placed']=True;self.facts['placed_on']=args[1];self.event('PLACEMENT_EVIDENCE',pose=p,attachment='detached',settled_seconds=1)
        elif k=='RETURN_HOME':
            self.move(cfg['home']);self.hold(.5)
            self.facts['at_home']=distance(self.sense()['position'],cfg['home'])<.15
            if not self.facts['at_home']:raise RuntimeError('HOME_NOT_VERIFIED')
        elif k=='HOVER':self.hold(1)
        elif k=='LAND':
            # Land only at home, with ground clearance for the underside gripper.
            self.move(cfg['home']);self.goto([.85,.65,.43]);self.hold(1);self.facts['landed']=True
        else:raise RuntimeError('UNSUPPORTED_SKILL')

    def pick_rgbd(self):
        cfg=self.config;o=self.observed('red_package');p=o['position']
        # Align with a view wide enough to see the package before descending.
        self.move([p[0],p[1],max(.95,p[2]+.8)]);self.hold(.6)
        o=self.observed('red_package');p=list(o['position']);self.target=list(p)
        self.event('GRASP_LOCALIZATION',pose=p,source=o['source'],stamp_ns=o['stamp_ns'],quality=o['quality'])
        self.goto([p[0],p[1],p[2]+cfg['grasp']['offset']]);self.hold(.4)
        d=self.sense()['position']
        if distance(d[:2],p[:2])>cfg['grasp']['xy_tolerance'] or abs(d[2]-p[2]-cfg['grasp']['offset'])>cfg['grasp']['z_tolerance']:raise RuntimeError('GRASP_ALIGNMENT_FAILED')
        # A joint acknowledgement alone does not prove the payload is attached.
        # Bound the test lift and independently observe the package rising.
        self.transport.grip(True);self.wait_gripper('attached');before=list(self.sense()['position'])
        try:
            self.goto([before[0],before[1],before[2]+cfg['grasp']['test_lift']],timeout=8)
            self.hold(.5);after=self.sense()['position']
            if self.sense()['gripper']!='attached' or after[2]-before[2]<.18:raise RuntimeError('TEST_LIFT_FAILED')
            lifted=self.verify_payload_lift(p,before)
        except RuntimeError:
            self.transport.grip(False)
            # Remove unloaded payload feedforward before safe recovery. Never set
            # holding/picked when the package has not passed the lift check.
            if not self.stop.is_set():self.wait_gripper('detached')
            raise
        self.facts['holding']=True;self.facts['picked']=True
        self.event('GRASP_EVIDENCE',attachment='attached',localization_source='learned_rgbd',verification_source='learned_rgbd_payload_motion_and_attachment_feedback',drone_before=before,drone_after=after,package_pregrasp=p,package_after_lift=lifted)
        self.move([after[0],after[1],1.6])

    def verify_payload_lift(self,package_before,drone_before,timeout=3):
        barrier=time.monotonic();deadline=barrier+timeout
        while time.monotonic()<deadline:
            state=self.sense();obj=state['objects'].get('red_package')
            if obj and obj.get('source')=='learned_rgbd' and obj['received']>barrier:
                p=obj['position'];d=state['position'];offset=drone_before[2]-package_before[2]
                if state['gripper']=='attached' and p[2]-package_before[2]>=.18 and distance(d[:2],p[:2])<.12 and abs(d[2]-p[2]-offset)<.12:
                    return list(p)
            self.hold(.15)
        raise RuntimeError('GRASP_PAYLOAD_NOT_VERIFIED: no fresh RGB-D evidence that package rose with drone')

    def wait_gripper(self,expected):
        end=time.monotonic()+4
        while time.monotonic()<end:
            self.sense()
            if self.transport.snapshot()['gripper']==expected:return
            self.command([0,0,0]);time.sleep(.05)
        raise RuntimeError('GRIPPER_STATE_TIMEOUT: '+expected)

    def satisfied(self,g):
        p=g['predicate'];args=g['args'];f=self.facts
        return {'found':lambda:f['found'],'inspected':lambda:args[1] in f['inspected'],'holding':lambda:f['picked'],
            'traversed':lambda:f['traversed'],'placed':lambda:f['placed'] and f.get('placed_on',args[1])==args[1],'at':lambda:f['at_home'],'landed':lambda:f['landed']}[p]()

    def authorize_gate_bypass(self):
        self.sense()
        policy=next((p for p in self.spec.get('contingencies',[]) if p.get('trigger')=='gate_not_clear'),{})
        if policy.get('action')!='BYPASS_GATE' or policy.get('authorization')!='USER':raise RuntimeError('UNAUTHORIZED_GATE_BYPASS')
        if self.gate_clear():raise RuntimeError('GATE_NOT_BLOCKED')
        if not self.facts['holding']:raise RuntimeError('NOT_HOLDING')
        self.bypass=True
        blocker={'min':[5.1,3.525,0],'max':[6.,3.775,2.2]}
        if blocker not in self.config['obstacles']:self.config['obstacles'].append(blocker)
        destination=policy.get('destination','blue_platform')
        waived=[g['id'] for g in self.spec['goals'] if g['predicate']=='traversed' or g['predicate']=='placed' and g['args'][1]!=destination]
        self.alternate_destination=destination
        self.session['waived_goals']=waived
        self.event('BT_ALTERNATE_SELECTED',trigger='gate_blocked',action='BYPASS_GATE',authorization='USER',waived_goals=waived,destination=destination)

    def recovery(self,a,error):
        self.heading=0.;self.command([0,0,0]);s=self.transport.snapshot();self.session['metrics']['replans']+=1
        interrupt=self.event('REPLAN_INTERRUPT',failed_action=a,reason=str(error),status=s,completed_goals=[g['id'] for g in self.spec['goals'] if self.satisfied(g)])
        self.session['interrupts'].append(interrupt);self.transport.emit('/drone/mission/interrupt',interrupt)
        # Only gate blockage may waive traversal. Sensor failures/collisions never trigger bypass.
        policy=next((p for p in self.spec.get('contingencies',[]) if p.get('trigger')=='gate_not_clear'),{}) if str(error)=='GATE_BLOCKED' else {}
        action=policy.get('action','ABORT_AND_RETURN_HOME')
        if action=='BYPASS_GATE' and policy.get('authorization')!='USER':action='ABORT_AND_RETURN_HOME'
        if action=='ASK_HUMAN':
            self.status='WAITING_FOR_HUMAN';self.decision.clear();self.human_action='ABORT_AND_RETURN_HOME';deadline=time.monotonic()+30
            anchor=self.sense()['position']
            while not self.decision.is_set() and time.monotonic()<deadline:
                self.goto(anchor,timeout=3,settle=.1)
            if self.stop.is_set():raise RuntimeError('CANCELLED')
            action=self.human_action;self.status='EXECUTING'
        if action=='BYPASS_GATE':
            # Use explicit route round the right gate post, not through the blocked aperture.
            self.bypass=True
            plan=[{'skill':'NAVIGATE','args':['blue_platform']},{'skill':'PLACE','args':['red_package','blue_platform']},{'skill':'RETURN_HOME','args':[]}]
            if not self.facts['holding']:action='ABORT_AND_RETURN_HOME'
        if action!='BYPASS_GATE':
            self.bypass=False;action='ABORT_AND_RETURN_HOME';plan=[{'skill':'RETURN_HOME','args':[]}]
        self.revision+=1;self.recovery_action=action
        revised=copy.deepcopy(self.spec)
        revised['revision_context']={'decision':action,'parent_version':self.revision-1,'interrupt_sequence':interrupt['sequence'],'state':s,
            'waived_goals':[g['id'] for g in self.spec['goals'] if not self.satisfied(g) and (action!='BYPASS_GATE' or g['predicate']=='traversed')]}
        if action=='BYPASS_GATE':
            # Add blocker to known geometry for every subsequent route.
            self.config=copy.deepcopy(self.config);self.config['obstacles'].append({'min':[5.1,3.525,0],'max':[6.,3.775,2.2]})
        if action=='ABORT_AND_RETURN_HOME':
            revised['goals']=[{'id':'RECOVERY_HOME','predicate':'at','args':['home'],'source':'SYSTEM_POLICY','source_text':'Abort remaining mission and return home'}]
            revised['ordering']=[];revised['requested_capabilities']=['RETURN_HOME','NAVIGATE']
        else:
            revised['goals']=[g for g in revised['goals'] if g['predicate']!='traversed' and not self.satisfied(g)]
            retained={g['id'] for g in revised['goals']}
            revised['ordering']=[o for o in revised.get('ordering',[]) if o['before'] in retained and o['after'] in retained]
            revised['requested_capabilities']=['NAVIGATE','PLACE','RETURN_HOME']
        self.version('RECOVERY',revised,plan);self.event('REPLAN_INSTALLED',action=action,robot_plan=plan)
        return plan

    def run(self):
        plan=copy.deepcopy(self.plan);self.recovery_action=None;self.bypass=False;recoveries=0
        try:
            self.transport.grip(False)
            self.move([*self.initial_position[:2],1.6])
            while True:
                failed_action=[{'skill':'HOVER','args':[]}]
                def act(a):
                    failed_action[0]=a;self.event('ACTION_STARTED',action=a)
                    try:
                        if a['skill']=='BYPASS_GATE':self.authorize_gate_bypass()
                        else:self.skill(a)
                        self.event('ACTION_RESULT',action=a,result='SUCCEEDED',facts=copy.deepcopy(self.facts),status=self.transport.snapshot())
                    except RuntimeError as error:
                        self.event('ACTION_RESULT',action=a,result='FAILED',reason=str(error));raise
                def condition(predicate):
                    failed_action[0]={'skill':'TRAVERSE_GATE','args':['gate_1']}
                    clear=self.survey_gate()
                    return clear if predicate=='gate_clear' else not clear
                try:
                    if not execute_tree(getattr(self,'execution_tree',None) or build_tree(self.spec,plan),act,condition,self.event):raise RuntimeError('GATE_BLOCKED')
                    break
                except RuntimeError as error:
                    if self.stop.is_set() or recoveries>=1:raise
                    recoveries+=1;plan=self.recovery(failed_action[0],error);self.execution_tree=build_tree(self.spec,plan)
            if self.recovery_action=='ABORT_AND_RETURN_HOME':
                if not self.facts['at_home']:raise RuntimeError('RECOVERY_HOME_NOT_VERIFIED')
                outcome='MISSION_ABORTED_SAFE_RETURN_COMPLETED'
            else:
                missing=[g['id'] for g in self.spec['goals'] if not self.satisfied(g) and not(self.bypass and g['id'] in self.session.get('waived_goals',[]))]
                if self.bypass and self.facts.get('placed_on')!=getattr(self,'alternate_destination','blue_platform'):missing.append('alternate_delivery')
                if missing:raise RuntimeError('UNFULFILLED_GOALS: '+','.join(missing))
                if self.session['metrics']['collision']:raise RuntimeError('COLLISION_INVALIDATES_SUCCESS')
                outcome='MISSION_COMPLETED_WITH_AUTHORIZED_BYPASS' if self.bypass else 'MISSION_COMPLETED'
            self.status=outcome
        except Exception as error:
            self.status='CANCELLED_HOLD' if self.stop.is_set() else 'MISSION_INCOMPLETE_HALTED';self.event('EXECUTION_HALTED',reason=str(error))
        finally:
            self.command([0,0,0]);self.session['outcome']=self.status;self.session['finished_at']=utc()
            self.session['metrics']['duration_seconds']=time.monotonic()-self.started;self.session['final_facts']=copy.deepcopy(self.facts)
            self.event('MISSION_RESULT',outcome=self.status,facts=self.facts)
            self.hold_anchor=self.transport.snapshot()['position']
            with self.lock:self.active=False
            # Hold zero velocity; this simulation controller has no PX4 failsafe. Not hardware-ready.

    def close(self):
        self.cancel()
        if self.thread:self.thread.join(timeout=3)
        self.closed=True;self.transport.close()

"""Read-only runtime evidence for the UI; never fabricates battery or skill success."""
import time

def contract_status(s,session,active,config):
    now=time.monotonic();events=(session or {}).get('events',[])
    started=next((e for e in reversed(events) if e['message_type']=='ACTION_STARTED'),None)
    if not started:return {'action':None,'status':'IDLE','checks':[],'completion_behavior':'Return home holds at 1.6 m; landing is not commissioned.'}
    result=next((e for e in reversed(events) if e['message_type']=='ACTION_RESULT' and e['sequence']>started['sequence']),None)
    action=started['action'];skill=action['skill'];checks=[]
    def add(name,ok,detail):checks.append({'name':name,'status':'pass' if ok else 'fail','detail':detail})
    age=now-s.get('pose_received',0);add('Fresh odometry',s.get('position') is not None and age<config['sensor_timeout'],f'Age {age:.2f} s; limit {config["sensor_timeout"]:.1f} s')
    p=s.get('position')
    if p:add('Inside geofence',-.1<p[0]<9.1 and -.1<p[1]<5.3 and .15<p[2]<3.2,'Current measured drone position')
    collided=(session or {}).get('metrics',{}).get('collision',False)
    add('No recorded collision',not collided,'Session collision monitor')
    if skill in {'SEARCH','INSPECT','PICK','PLACE'}:
        obj=s.get('objects',{}).get('red_package');fresh=obj is not None and now-obj['received']<config['sensor_timeout']
        checks.append({'name':'Package observation','status':'pass' if fresh else 'unknown','detail':obj['source'] if fresh else 'No fresh estimate; may be occluded during grasp/carry'})
    if skill in {'PICK','PLACE'}:checks.append({'name':'Gripper feedback','status':'info','detail':s.get('gripper','unknown')})
    checks.append({'name':'Battery reserve','status':'unknown','detail':'No battery sensor/model in this Gazebo world; reserve is not verified'})
    if skill=='RETURN_HOME' and p:add('At home hover position',sum((x-y)**2 for x,y in zip(p,config['home']))**.5<.15,'Home target: '+str(config['home'])+' m; no touchdown commanded')
    status=result['result'] if result else 'EXECUTING' if active else 'INTERRUPTED'
    return {'action':action,'status':status,'reason':result.get('reason') if result else None,'checks':checks,'checks_scope':'Current telemetry; skill success is taken only from ACTION_RESULT','completion_behavior':'Return home holds at 1.6 m; landing is not commissioned.'}

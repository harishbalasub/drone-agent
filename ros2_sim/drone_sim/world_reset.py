"""Verified Gazebo episode reset. Simulator truth here is reset evidence only."""
import os
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path


def initial_poses():
    scenario=os.environ.get('DRONE_SCENARIO','1')
    if scenario not in {'1','2','3'}:raise RuntimeError('Invalid reset scenario')
    world=ET.parse(Path(__file__).resolve().parents[1]/'worlds'/('scenario_'+scenario+'.sdf')).getroot().find('world')
    return {name:[float(v) for v in world.find("model[@name='"+name+"']/pose").text.split()[:3]] for name in ['drone','red_package']}


def reset_gazebo(transport,timeout=10):
    from gz.transport13 import Node
    from gz.msgs10.world_control_pb2 import WorldControl
    from gz.msgs10.boolean_pb2 import Boolean
    from gz.msgs10.pose_pb2 import Pose
    from gz.msgs10.pose_v_pb2 import Pose_V
    node=Node();poses={};lock=threading.Lock()
    def observe(message):
        with lock:
            for p in message.pose:
                if p.name in {'drone','red_package'}:poses[p.name]=([p.position.x,p.position.y,p.position.z],time.monotonic())
    topic='/world/mission/dynamic_pose/info'
    if not node.subscribe(Pose_V,topic,observe):raise RuntimeError('Could not subscribe to reset pose evidence')
    def request(service,message,kind):
        # A new transport node may not have discovered every service yet.
        deadline=time.monotonic()+5
        while time.monotonic()<deadline:
            ok,reply=node.request(service,message,kind,Boolean,1000)
            if ok and reply.data:return
            time.sleep(.1)
        raise RuntimeError('Gazebo reset service did not acknowledge '+service)
    try:
        transport.velocity([0,0,0]);transport.grip(False)
        control=WorldControl();control.reset.all=True
        request('/world/mission/control',control,WorldControl)
        # DetachableJoint may start attached after reconstruction. Require a fresh
        # detach acknowledgement before restoring the independent model poses.
        barrier=time.monotonic();deadline=barrier+timeout
        while time.monotonic()<deadline:
            transport.velocity([0,0,0]);transport.grip(False);state=transport.snapshot()
            if state['gripper']=='detached' and state['gripper_received']>barrier:break
            time.sleep(.05)
        else:raise RuntimeError('Gripper detach not acknowledged after world reset')
        expected=initial_poses()
        for name,position in expected.items():
            pose=Pose();pose.name=name;pose.position.x,pose.position.y,pose.position.z=position;pose.orientation.w=1.
            request('/world/mission/set_pose',pose,Pose)
        barrier=time.monotonic();deadline=barrier+timeout
        while time.monotonic()<deadline:
            transport.velocity([0,0,0])
            with lock:observed=dict(poses)
            if all(name in observed and observed[name][1]>barrier and sum((x-y)**2 for x,y in zip(observed[name][0],position))**.5<.2 for name,position in expected.items()):
                return {'source':'gazebo_reset_pose_verification','expected':expected,'observed':{k:v[0] for k,v in observed.items()},'package_localized_for_mission':False}
            time.sleep(.05)
        raise RuntimeError('Reset acknowledged but initial drone/package poses were not verified')
    finally:node.unsubscribe(topic)

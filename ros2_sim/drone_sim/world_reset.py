"""Verified Gazebo episode reset. Simulator truth here is reset evidence only."""
import os
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path


def initial_poses(scenario=None):
    scenario=str(scenario or os.environ.get('DRONE_SCENARIO','1'))
    if scenario not in {'1','2','3','4'}:raise RuntimeError('Invalid reset scenario')
    world=ET.parse(Path(__file__).resolve().parents[1]/'worlds'/('scenario_'+scenario+'.sdf')).getroot().find('world')
    return {name:[float(v) for v in world.find("model[@name='"+name+"']/pose").text.split()[:3]] for name in ['drone','red_package']}


def reset_gazebo(transport,timeout=10,scenario=None):
    from gz.transport13 import Node
    from gz.msgs10.boolean_pb2 import Boolean
    from gz.msgs10.pose_pb2 import Pose
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.msgs10.entity_factory_pb2 import EntityFactory
    from gz.msgs10.entity_pb2 import Entity
    from gz.msgs10.empty_pb2 import Empty
    from gz.msgs10.scene_pb2 import Scene
    scenario=int(scenario or os.environ.get('DRONE_SCENARIO','1'))
    if scenario not in {1,2,3,4}:raise ValueError('Invalid scenario')
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
        # Keep the physics entity IDs and joint plugin alive across episodes.
        # A global reset can leave DetachableJoint reporting attached for a joint
        # that no longer exists in physics. Reset only the movable episode poses.
        already_detached=transport.snapshot()['gripper']=='detached'
        barrier=time.monotonic();deadline=barrier+timeout
        while time.monotonic()<deadline:
            transport.velocity([0,0,0]);transport.grip(False);state=transport.snapshot()
            if state['gripper']=='detached' and (already_detached or state['gripper_received']>barrier):break
            time.sleep(.05)
        else:raise RuntimeError('Gripper detach not acknowledged after world reset')
        # Synchronize the physical models, including after a normal reset.
        # Read Gazebo's scene graph to verify create/remove acknowledgements.
        def models():
            deadline=time.monotonic()+5
            while time.monotonic()<deadline:
                ok,scene=node.request('/world/mission/scene/info',Empty(),Empty,Scene,1000)
                if ok:return {m.name for m in scene.model}
                time.sleep(.1)
            raise RuntimeError('Gazebo scene inventory unavailable')
        names=models()
        world=ET.parse(Path(__file__).resolve().parents[1]/'worlds'/f'scenario_{scenario}.sdf').getroot().find('world')
        desired={'gate_blocker':scenario in [2,4],'red_platform':scenario in [3,4]}
        for name,present in desired.items():
            if name in names and not present:
                # Let the sensor renderer consume a pose update before deletion.
                # Gazebo can otherwise retain a deleted static model in RGB views.
                parked=Pose();parked.name=name;parked.position.z=-100.;parked.orientation.w=1.
                request('/world/mission/set_pose',parked,Pose)
                barrier=time.monotonic();deadline=barrier+3
                while time.monotonic()<deadline:
                    cameras=transport.snapshot().get('cameras',{})
                    if time.monotonic()-barrier>=.6 and all(c.get('received',0)>barrier+.3 for c in cameras.values()):break
                    time.sleep(.05)
                entity=Entity();entity.name=name;entity.type=Entity.MODEL
                request('/world/mission/remove',entity,Entity)
            elif name not in names and present:
                factory=EntityFactory();factory.sdf='<sdf version="1.9">'+ET.tostring(world.find(f"model[@name='{name}']"),encoding='unicode')+'</sdf>'
                request('/world/mission/create',factory,EntityFactory)
        deadline=time.monotonic()+timeout
        while True:
            names=models()
            if all((name in names)==present for name,present in desired.items()):break
            if time.monotonic()>deadline:raise RuntimeError('Scenario geometry was not verified')
            time.sleep(.1)
        expected=initial_poses(scenario)
        for name,position in expected.items():
            pose=Pose();pose.name=name;pose.position.x,pose.position.y,pose.position.z=position;pose.orientation.w=1.
            request('/world/mission/set_pose',pose,Pose)
        barrier=time.monotonic();deadline=barrier+timeout
        while time.monotonic()<deadline:
            transport.velocity([0,0,0])
            with lock:observed=dict(poses)
            if all(name in observed and observed[name][1]>barrier and sum((x-y)**2 for x,y in zip(observed[name][0],position))**.5<.2 for name,position in expected.items()):
                return {'source':'gazebo_reset_pose_verification','expected':expected,'observed':{k:v[0] for k,v in observed.items()},'package_localized_for_mission':False,'scenario':scenario,'models':sorted(names),'reset_mode':'episode_pose_reset','simulation_clock_reset':False}
            time.sleep(.05)
        raise RuntimeError('Reset acknowledged but initial drone/package poses were not verified')
    finally:node.unsubscribe(topic)

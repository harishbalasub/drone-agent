#!/usr/bin/env python3
"""Learned package observation from synchronized RGB-D + timestamped drone odometry.

No logical-camera subscriptions or world object poses. Other scene semantics stay
in the existing ideal sensor. Inference runs in a separate project-local Python.
"""
import base64,json,math,os,select,subprocess,sys,threading,time
from collections import deque
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from rgbd.data import Synchronizer,decode_image,stamp
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image
from gz.msgs10.odometry_pb2 import Odometry
from gz.msgs10.stringmsg_pb2 import StringMsg
ROOT=Path(__file__).resolve().parents[1]

def camera_matrix(p,q):
    x,y,z,w=q;rot=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    m=np.eye(4);m[:3,:3]=rot@np.array([[0,0,1],[0,1,0],[-1,0,0]]);m[:3,3]=np.asarray(p)+rot@np.array([0,0,-.08]);return m

def main():
    env=dict(os.environ);env.pop('PYTHONPATH',None)
    worker=subprocess.Popen([str(ROOT/'.runtime/rgbd_venv/bin/python'),str(ROOT/'rgbd/inference_worker.py'),'--checkpoint',str(ROOT/'results/rgbd_spark_cuda_v1/best.pt')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True,env=env)
    if not select.select([worker.stdout],[],[],40)[0] or not json.loads(worker.stdout.readline()).get('ready'):raise RuntimeError('RGB-D model failed to initialize')
    node=Node();pub=node.advertise('/drone/perception/package',StringMsg);sync=Synchronizer();sync.buffers={k:{} for k in ['rgb','depth']};poses=deque(maxlen=300);lock=threading.Lock()
    epoch=[0];previous_stamp=[None];joint_state=['unknown']
    grasp_offset=json.loads((ROOT/'config/world.json').read_text())['grasp']['offset']
    def gripper(message):joint_state[0]=message.data
    node.subscribe(StringMsg,'/drone/gripper/state',gripper)
    def odom(m):
        p=m.pose.position;q=m.pose.orientation;ns=stamp(m)
        with lock:
            if previous_stamp[0] is not None and ns<previous_stamp[0]-1_000_000_000:
                poses.clear();epoch[0]+=1
                with sync.lock:
                    for buffer in sync.buffers.values():buffer.clear()
            previous_stamp[0]=ns;poses.append((ns,[p.x,p.y,p.z],[q.x,q.y,q.z,q.w]))
    node.subscribe(Odometry,'/drone/odometry',odom)
    node.subscribe(Image,'/drone/camera/down/image',lambda m:sync.add('rgb',m))
    node.subscribe(Image,'/drone/camera/down/depth_image',lambda m:sync.add('depth',m))
    last=0;current_epoch=epoch[0]
    print('Learned RGB-D package bridge ready (Spark checkpoint, local CPU inference)',flush=True)
    try:
        while True:
            if current_epoch!=epoch[0]:last=0;current_epoch=epoch[0]
            pair=sync.matched(last)
            if not pair:time.sleep(.015);continue
            ns,messages=pair
            with lock:pose=min(poses,key=lambda p:abs(p[0]-ns)) if poses else None
            # Wait for matching odometry, never use a current pose for an old frame.
            if not pose or abs(pose[0]-ns)>50_000_000:time.sleep(.01);continue
            last=ns;rgb=decode_image(messages['rgb']);depth=decode_image(messages['depth'],1,np.dtype('<f4'));h,w=depth.shape
            f=w/(2*math.tan(1.3/2));req={'shape':[h,w],'rgb':base64.b64encode(rgb.tobytes()).decode(),'depth':base64.b64encode(depth.tobytes()).decode(),'intrinsics':{'fx':f,'fy':f,'cx':(w-1)/2,'cy':(h-1)/2},'world_from_camera':camera_matrix(pose[1],pose[2]).tolist(),'grasp_candidate':joint_state[0]=='attached','expected_payload_z':pose[1][2]-grasp_offset}
            began=time.monotonic();worker.stdin.write(json.dumps(req)+'\n');worker.stdin.flush()
            if not select.select([worker.stdout],[],[],3)[0]:raise RuntimeError('RGB-D inference timeout')
            result=json.loads(worker.stdout.readline())
            if current_epoch!=epoch[0]:continue  # Discard inference from the previous episode.
            payload={'schema_version':'1.0','frame':'map','source':'learned_rgbd','stamp_ns':ns,'pose_skew_ns':abs(pose[0]-ns),'inference_seconds':time.monotonic()-began,'valid':result['valid'],'estimate':result,'checkpoint':'rgbd_spark_cuda_v1/best.pt','perception_mode':'grasp_candidate' if req['grasp_candidate'] else 'supported_surface'}
            msg=StringMsg();msg.data=json.dumps(payload);pub.publish(msg)
    except KeyboardInterrupt:pass
    finally:worker.terminate();worker.wait(timeout=5)
if __name__=='__main__':main()

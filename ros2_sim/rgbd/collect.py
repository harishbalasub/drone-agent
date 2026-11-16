#!/usr/bin/env python3
"""Render labeled RGB-D data from randomized static camera rigs in Gazebo.

Ground-truth masks and package poses are dataset labels only. This script never
feeds these labels into the mission executor or claims learned-policy results.
"""
import argparse
import json
import math
import os
import random
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path
from uuid import uuid4
import numpy as np
from PIL import Image as PILImage
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from rgbd.world import sample_scene,write_world,camera_transform
from rgbd.data import Synchronizer,decode_image


def episode(output,seed,views):
    from gz.transport13 import Node
    from gz.msgs10.image_pb2 import Image
    from gz.msgs10.pose_pb2 import Pose
    from gz.msgs10.boolean_pb2 import Boolean
    scene=sample_scene(seed);folder=output/f'episode_{seed:06d}'
    if folder.exists():raise ValueError('Refusing to overwrite episode '+str(folder))
    folder.mkdir(parents=True);world=write_world(scene,folder/'world.sdf')
    log=(folder/'gazebo.log').open('w');process=subprocess.Popen(['gz','sim','-s','-r','--headless-rendering',str(world),'-v','2'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    node=Node();sync=Synchronizer()
    for kind,topic in [('rgb','/dataset/rgb'),('depth','/dataset/depth'),('mask','/dataset/mask/labels_map')]:
        node.subscribe(Image,topic,lambda m,k=kind:sync.add(k,m))
    rng=random.Random(seed+9900);records=[];last_stamp=0
    try:
        deadline=time.monotonic()+30
        while sync.matched() is None and time.monotonic()<deadline:
            if process.poll() is not None:raise RuntimeError('Gazebo exited; inspect '+str(folder/'gazebo.log'))
            time.sleep(.1)
        if sync.matched() is None:raise RuntimeError('No timestamp-matched RGB/depth/segmentation frames')
        for view in range(views):
            px,py,_=scene['package_position'];alt=rng.uniform(.8,2.5)
            # Include off-center and absent views rather than keeping target centered.
            pos=[px+rng.uniform(-.9,.9),py+rng.uniform(-.7,.7),alt];yaw=rng.uniform(-math.pi,math.pi)
            before=sync.matched();pose_boundary=before[0] if before else last_stamp
            req=Pose();req.name='rgbd_rig';req.position.x,req.position.y,req.position.z=pos
            req.orientation.z=math.sin(yaw/2);req.orientation.w=math.cos(yaw/2)
            acknowledged=False
            for attempt in range(5):
                result,reply=node.request('/world/rgbd_dataset/set_pose',req,Pose,Boolean,3000)
                if result and reply.data:acknowledged=True;break
                time.sleep(.2)
            if not acknowledged:raise RuntimeError('Camera pose request failed')
            # Rendering runs asynchronously. Wait for three sensor periods in
            # SIMULATION time, not wall time, after the acknowledged pose change.
            deadline=time.monotonic()+25;matched=None
            while time.monotonic()<deadline:
                matched=sync.matched(max(last_stamp,pose_boundary+600000000))
                if matched is not None:break
                time.sleep(.05)
            if matched is None:raise RuntimeError('Sensor synchronization failed')
            timestamp,frames=matched;last_stamp=timestamp
            rgb=decode_image(frames['rgb']);depth=decode_image(frames['depth'],1,np.dtype('<f4'))
            label=decode_image(frames['mask'])
            # Gazebo semantic labels are repeated across RGB channels; validate,
            # never infer masks from the package's visible color.
            if not np.array_equal(label[:,:,0],label[:,:,1]) or not np.array_equal(label[:,:,0],label[:,:,2]):raise RuntimeError('Unexpected semantic label encoding')
            mask=(label[:,:,0]==1).astype(np.uint8)
            prefix=f'{view:04d}';PILImage.fromarray(rgb).save(folder/(prefix+'_rgb.png'));PILImage.fromarray(mask*255).save(folder/(prefix+'_mask.png'));np.save(folder/(prefix+'_depth.npy'),depth)
            height,width=depth.shape;f=width/(2*math.tan(1.3/2))
            record={'schema_version':'1.0','episode_seed':seed,'view':view,'split':scene['split'],'stamp_ns':timestamp,
                'rgb':str((folder/(prefix+'_rgb.png')).relative_to(output)),'mask':str((folder/(prefix+'_mask.png')).relative_to(output)),'depth':str((folder/(prefix+'_depth.npy')).relative_to(output)),
                'intrinsics':{'fx':f,'fy':f,'cx':width/2,'cy':height/2,'width':width,'height':height,'source':'matching SDF pinhole calibration'},
                'world_from_camera':camera_transform(pos,yaw).tolist(),'package_center_truth':scene['package_position'] if scene['target_present'] else None,
                'package_height_m':.24,'visible_mask_pixels':int(mask.sum()),'ground_truth_for_training_only':True,'scene':scene,'sampler':'static_camera_rig_not_flight'}
            records.append(record)
        (folder/'samples.json').write_text(json.dumps(records,indent=2))
        return records
    finally:
        os.killpg(process.pid,signal.SIGINT)
        try:process.wait(timeout=8)
        except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
        log.close()


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--start-seed',type=int,default=0);p.add_argument('--episodes',type=int,default=5);p.add_argument('--views',type=int,default=20)
    a=p.parse_args();output=Path(a.output).resolve();output.mkdir(parents=True,exist_ok=True)
    records=[]
    # Each episode is a subprocess: gz transport resolves its default partition
    # once per process, so changing GZ_PARTITION inside one process isn't enough.
    for seed in range(a.start_seed,a.start_seed+a.episodes):
        env=os.environ.copy();env['GZ_PARTITION']='rgbd_'+uuid4().hex
        if (output/f'episode_{seed:06d}/samples.json').exists():continue
        subprocess.run([sys.executable,str(Path(__file__).resolve()),'--episode-worker',str(output),str(seed),str(a.views)],env=env,check=True)
        records+=json.loads((output/f'episode_{seed:06d}/samples.json').read_text())
        all_records=[r for f in sorted(output.glob('episode_*/samples.json')) for r in json.loads(f.read_text())]
        (output/'manifest.json').write_text(json.dumps({'schema_version':'1.0','samples':all_records,'label_source':'Gazebo segmentation sensor; not runtime perception'},indent=2))
    all_records=[r for f in sorted(output.glob('episode_*/samples.json')) for r in json.loads(f.read_text())]
    (output/'manifest.json').write_text(json.dumps({'schema_version':'1.0','samples':all_records,'label_source':'Gazebo segmentation sensor; not runtime perception'},indent=2))
    print('Saved',len(records),'new rendered RGB-D samples in',output)

if __name__=='__main__':
    if len(sys.argv)>1 and sys.argv[1]=='--episode-worker':episode(Path(sys.argv[2]),int(sys.argv[3]),int(sys.argv[4]))
    else:main()

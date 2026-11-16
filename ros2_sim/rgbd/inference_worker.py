#!/usr/bin/env python3
"""Persistent model worker; JSON lines keep Torch separate from ROS's Python ABI."""
import argparse, base64, json, sys
from pathlib import Path
import numpy as np
import torch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from rgbd.model import load_model
from rgbd.geometry import localize_package
p=argparse.ArgumentParser();p.add_argument('--checkpoint',required=True);a=p.parse_args()
torch.set_num_threads(2);model=load_model(a.checkpoint,'cpu')
print(json.dumps({'ready':True}),flush=True)
for line in sys.stdin:
    try:
        r=json.loads(line);h,w=r['shape'];rgb=np.frombuffer(base64.b64decode(r['rgb']),np.uint8).reshape(h,w,3).astype(np.float32)/255
        depth=np.frombuffer(base64.b64decode(r['depth']),dtype='<f4').reshape(h,w)
        x=torch.from_numpy(rgb.transpose(2,0,1).copy());x=(x-torch.tensor([.485,.456,.406])[:,None,None])/torch.tensor([.229,.224,.225])[:,None,None]
        with torch.inference_mode():mask=(model(x[None])['out'].softmax(1)[0,1]>.7).numpy()
        # The drone's own landing legs/gripper were absent from static-rig training.
        # Exclude near-camera self geometry; grasp alignment is performed above .95 m.
        mask &= np.isfinite(depth) & (depth>.35)
        result=localize_package(depth,mask,r['intrinsics'],r['world_from_camera'])
        # Supported mission supports: ground or the 0.5 m delivery platform.
        if min(abs(result['position'][2]-z) for z in [.12,.62])>.08:raise ValueError('UNSUPPORTED_PACKAGE_SUPPORT_HEIGHT')
        print(json.dumps({'valid':True,**result}),flush=True)
    except (ValueError,KeyError) as e:print(json.dumps({'valid':False,'reason':str(e)}),flush=True)

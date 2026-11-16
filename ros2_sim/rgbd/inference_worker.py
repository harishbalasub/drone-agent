#!/usr/bin/env python3
"""Persistent model worker; JSON lines keep Torch separate from ROS's Python ABI."""
import argparse, base64, json, sys
from pathlib import Path
import numpy as np
import torch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from rgbd.model import load_model
from rgbd.geometry import localize_package
from rgbd.observation_filter import filter_depth_mask,validate_height
p=argparse.ArgumentParser();p.add_argument('--checkpoint',required=True);a=p.parse_args()
torch.set_num_threads(2);model=load_model(a.checkpoint,'cpu')
print(json.dumps({'ready':True}),flush=True)
for line in sys.stdin:
    try:
        r=json.loads(line);h,w=r['shape'];rgb=np.frombuffer(base64.b64decode(r['rgb']),np.uint8).reshape(h,w,3).astype(np.float32)/255
        depth=np.frombuffer(base64.b64decode(r['depth']),dtype='<f4').reshape(h,w)
        x=torch.from_numpy(rgb.transpose(2,0,1).copy());x=(x-torch.tensor([.485,.456,.406])[:,None,None])/torch.tensor([.229,.224,.225])[:,None,None]
        with torch.inference_mode():mask=(model(x[None])['out'].softmax(1)[0,1]>.7).numpy()
        # Surface localization excludes near-camera self geometry. Once the joint
        # reports attachment, allow the package's close-up top surface as a grasp
        # candidate; this mode is not itself proof of a successful pickup.
        candidate=bool(r.get('grasp_candidate',False))
        mask=filter_depth_mask(depth,mask,candidate)
        result=localize_package(depth,mask,r['intrinsics'],r['world_from_camera'])
        # Supported mission supports: ground or the 0.5 m delivery platform.
        validate_height(result['position'],candidate,r.get('expected_payload_z'))
        print(json.dumps({'valid':True,**result}),flush=True)
    except (ValueError,KeyError) as e:print(json.dumps({'valid':False,'reason':str(e)}),flush=True)

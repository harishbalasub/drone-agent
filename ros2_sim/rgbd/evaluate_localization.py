#!/usr/bin/env python3
"""Held-out mask and metric localization evaluation, or explicitly labeled mask-oracle diagnostic."""
import argparse
import json
import sys
import time
from pathlib import Path
import numpy as np
from PIL import Image
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from rgbd.geometry import localize_package


def evaluate(root,split,checkpoint=None,device='cuda',oracle=False):
    root=Path(root);samples=[s for s in json.loads((root/'manifest.json').read_text())['samples'] if s['split']==split]
    if not samples:raise ValueError('No samples in split '+split)
    if not oracle and not checkpoint:raise ValueError('A trained checkpoint is required unless --oracle-mask is explicitly selected')
    if checkpoint:
        import torch
        from rgbd.model import load_model
        model=load_model(checkpoint,device)
    errors=[];xy_errors=[];latencies=[];fp=neg=visible=accepted=0;intersection=union=0
    for s in samples:
        gt=np.asarray(Image.open(root/s['mask']))>0;truth_visible=int(gt.sum())>=25
        began=time.perf_counter()
        if oracle:mask=gt
        else:
            rgb=np.asarray(Image.open(root/s['rgb']).convert('RGB'),dtype=np.float32)/255
            x=torch.from_numpy(rgb.transpose(2,0,1).copy()).to(device)
            x=(x-torch.tensor([.485,.456,.406],device=device)[:,None,None])/torch.tensor([.229,.224,.225],device=device)[:,None,None]
            with torch.inference_mode():mask=(model(x[None])['out'].softmax(1)[0,1]>.7).cpu().numpy()
        intersection+=int((gt&mask).sum());union+=int((gt|mask).sum())
        try:
            estimate=localize_package(np.load(root/s['depth']),mask,s['intrinsics'],s['world_from_camera'],s['package_height_m'])
            valid=True
        except ValueError:valid=False
        latencies.append((time.perf_counter()-began)*1000)
        if truth_visible:
            visible+=1
            if valid:
                accepted+=1;delta=np.array(estimate['position'])-np.array(s['package_center_truth']);errors.append(float(np.linalg.norm(delta)));xy_errors.append(float(np.linalg.norm(delta[:2])))
        else:neg+=1;fp+=int(valid)
    return {'mode':'oracle_mask_geometry_diagnostic_NOT_LEARNED' if oracle else 'learned_rgb_segmentation_plus_depth_geometry','split':split,'samples':len(samples),
        'visible_targets':visible,'accepted_visible':accepted,'visible_recall':accepted/max(1,visible),'negative_frames':neg,'false_accepts':fp,'false_accept_rate':fp/max(1,neg),
        'mask_iou':intersection/max(1,union),'median_xy_error_m':float(np.median(xy_errors)) if xy_errors else None,'p95_xy_error_m':float(np.percentile(xy_errors,95)) if xy_errors else None,
        'median_3d_error_m':float(np.median(errors)) if errors else None,'median_latency_ms':float(np.median(latencies)),
        'assumptions':['upright package of known 0.24 m height','simulated motion-capture camera pose','registered optical-axis depth'],'checkpoint':str(checkpoint) if checkpoint else None}

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--dataset',required=True);p.add_argument('--split',choices=['train','validation','test'],default='test');p.add_argument('--checkpoint');p.add_argument('--oracle-mask',action='store_true');p.add_argument('--device',default='cuda');p.add_argument('--output',required=True);p.add_argument('--threads',type=int,default=2);a=p.parse_args()
    if not a.oracle_mask:
        import torch
        torch.set_num_threads(a.threads)
    if a.oracle_mask and a.checkpoint:raise SystemExit('Choose either learned checkpoint or oracle diagnostic')
    r=evaluate(a.dataset,a.split,a.checkpoint,a.device,a.oracle_mask);Path(a.output).write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))

#!/usr/bin/env python3
"""Fine-tune a two-class package segmenter. Requires a working PyTorch environment.

Training input contains RGB/masks only. Depth, object truth and transforms are
not passed to the neural network. Test episodes are never used for optimization.
"""
import argparse
import json
import random
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
from PIL import Image
from rgbd.model import build_model


def validate_splits(samples):
    groups={k:[s for s in samples if s['split']==k] for k in ['train','validation','test']}
    if any(not g for g in groups.values()):raise ValueError('Separate train, validation and test episodes are required')
    seeds={k:{s['episode_seed'] for s in group} for k,group in groups.items()}
    if any(seeds[a]&seeds[b] for a,b in [('train','validation'),('train','test'),('validation','test')]):raise ValueError('Episode leakage between splits')
    return groups


def main():
    p=argparse.ArgumentParser();p.add_argument('--dataset',required=True);p.add_argument('--output',required=True);p.add_argument('--epochs',type=int,default=30);p.add_argument('--batch-size',type=int,default=16);p.add_argument('--device',default='cuda');p.add_argument('--workers',type=int,default=2);p.add_argument('--threads',type=int,default=2);a=p.parse_args()
    import torch
    torch.set_num_threads(a.threads)
    import torch.nn.functional as F
    from torch.utils.data import Dataset,DataLoader
    if a.device.startswith('cuda') and not torch.cuda.is_available():raise SystemExit('CUDA is unavailable. Verify the DGX PyTorch environment; no silent CPU fallback.')
    random.seed(42);np.random.seed(42);torch.manual_seed(42)
    root=Path(a.dataset).resolve();samples=json.loads((root/'manifest.json').read_text())['samples'];groups=validate_splits(samples)
    out=Path(a.output);out.mkdir(parents=True,exist_ok=True)
    if (out/'best.pt').exists():raise SystemExit('Output already has a checkpoint. Use a new run directory.')
    class Frames(Dataset):
        def __init__(self,split):self.rows=groups[split];self.augment=split=='train'
        def __len__(self):return len(self.rows)
        def __getitem__(self,i):
            row=self.rows[i];rgb=np.asarray(Image.open(root/row['rgb']).convert('RGB'),dtype=np.float32)/255
            mask=(np.asarray(Image.open(root/row['mask']))>0).astype(np.int64)
            if self.augment:rgb=np.clip(rgb*random.uniform(.8,1.2)+random.uniform(-.04,.04),0,1)
            x=torch.from_numpy(rgb.transpose(2,0,1).copy());mean=torch.tensor([.485,.456,.406])[:,None,None];std=torch.tensor([.229,.224,.225])[:,None,None]
            return (x-mean)/std,torch.from_numpy(mask)
    train=DataLoader(Frames('train'),batch_size=a.batch_size,shuffle=True,num_workers=a.workers)
    val=DataLoader(Frames('validation'),batch_size=a.batch_size,shuffle=False,num_workers=a.workers)
    model=build_model(True).to(a.device);optimizer=torch.optim.AdamW(model.parameters(),lr=1e-4,weight_decay=1e-4)
    weights=torch.tensor([1.,8.],device=a.device);history=[];best=-1.
    for epoch in range(a.epochs):
        model.train();total=0.
        for x,y in train:
            x=x.to(a.device);y=y.to(a.device);optimizer.zero_grad(set_to_none=True);logits=model(x)['out']
            ce=F.cross_entropy(logits,y,weight=weights);prob=logits.softmax(1)[:,1];truth=(y==1).float()
            dice=1-(2*(prob*truth).sum()+1)/(prob.sum()+truth.sum()+1);loss=ce+dice
            loss.backward();optimizer.step();total+=float(loss.detach())
        model.eval();intersection=union=0;false_accepts=absent=0
        with torch.inference_mode():
            for x,y in val:
                prediction=model(x.to(a.device))['out'].softmax(1)[:,1].cpu()>.7;truth=y==1
                intersection+=int((prediction&truth).sum());union+=int((prediction|truth).sum())
                negatives=truth.flatten(1).sum(1)<25;absent+=int(negatives.sum());false_accepts+=int(((prediction.flatten(1).sum(1)>=25)&negatives).sum())
        iou=intersection/max(1,union);record={'epoch':epoch+1,'train_loss':total/max(1,len(train)),'validation_mask_iou':iou,'validation_negative_frames':absent,'validation_mask_false_accepts':false_accepts}
        history.append(record);print(json.dumps(record),flush=True)
        if iou>best:
            best=iou;torch.save({'architecture':'lraspp_mobilenet_v3_large_2class','model':model.state_dict(),'epoch':epoch+1,'validation_iou':iou,'probability_threshold':.7},out/'best.pt')
        (out/'training.json').write_text(json.dumps({'method':'supervised RGB segmentation + separate calibrated depth geometry','trained_on':'train split only','selected_by':'validation mask IoU','test_used_for_training':False,'history':history,'split_counts':{k:len(v) for k,v in groups.items()},'torch_version':str(torch.__version__),'device':a.device,'threads':a.threads,'batch_size':a.batch_size,'dataset':str(root)},indent=2))
    print('Checkpoint saved. Run held-out localization evaluation before enabling learned perception.')

if __name__=='__main__':main()

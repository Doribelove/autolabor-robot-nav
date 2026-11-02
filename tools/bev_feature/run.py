#!/usr/bin/env python3
"""Float training/export/smoke; never labels random weights as a trash detector."""
import argparse,json,sys,time,platform
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src/application/autolabor_fod_vision/src'))
import numpy as np
import torch
from torch.nn import functional as F
from autolabor_fod_vision.bev_feature.geometry import DEFAULT,sampling,decode
from autolabor_fod_vision.bev_feature.model import TrashBEV
from autolabor_fod_vision.bev_feature.data import validate,images,targets

def main():
 p=argparse.ArgumentParser();p.add_argument('mode',choices=['smoke','train','export','infer']);p.add_argument('--dataset');p.add_argument('--checkpoint');p.add_argument('--output',required=True);p.add_argument('--epochs',type=int,default=1);p.add_argument('--position-std-m',type=float);p.add_argument('--device',default='cuda' if torch.cuda.is_available() else 'cpu');a=p.parse_args()
 if a.mode=='train' and platform.machine()!='x86_64':raise ValueError('Formal training belongs on x86_64 Linux; this host is '+platform.machine())
 torch.set_num_threads(4);torch.manual_seed(7);out=Path(a.output);out.mkdir(parents=True,exist_ok=True)
 spec=dict(DEFAULT);rig=json.loads((ROOT/'config/four_camera_bev.json').read_text());trained=False;ck=None
 if a.checkpoint:
  ck=torch.load(a.checkpoint,map_location='cpu',weights_only=True);spec=ck['spec'];rig=ck['rig'];trained=ck['trained_on_metric_labels']
 geometry=sampling(rig,spec);model=TrashBEV(geometry['shape'],depth_bins=spec['depth_bins']).to(a.device)
 if ck:model.load_state_dict(ck['model'])
 if a.mode=='train':
  if not a.dataset:raise ValueError('--dataset required; no invented metric labels')
  if a.epochs<1:raise ValueError('--epochs must be positive')
  manifest=Path(a.dataset);samples=[json.loads(x) for x in manifest.read_text().splitlines() if x.strip()]
  if not samples:raise ValueError('empty dataset')
  for s in samples:validate(s,manifest.parent)
  opt=torch.optim.Adam(model.parameters(),lr=1e-4);model.train();losses=[]
  for epoch in range(a.epochs):
   for s in samples:
    sample_rig=json.loads((manifest.parent/s['calibration']).read_text());g=sampling(sample_rig,spec,s['rig_to_base_xyyaw']);inputs=[torch.from_numpy(images(s,manifest.parent,sample_rig,spec)).to(a.device)]+[torch.from_numpy(g[k]).to(a.device) for k in ('feature_grid','depth_grid','valid')]
    heat,offset,mask=[torch.from_numpy(x)[None].to(a.device) for x in targets(s['objects'],spec)]
    logits,pred=model(*inputs);prob=logits.sigmoid().clamp(1e-6,1-1e-6)
    positive=heat.eq(1);negative=~positive
    focal=-(torch.log(prob)*(1-prob)**2*positive+torch.log(1-prob)*prob**2*(1-heat)**4*negative).sum()/positive.sum().clamp_min(1)
    loss=focal+((pred-offset).abs()*mask).sum()/mask.sum().clamp_min(1)
    opt.zero_grad();loss.backward();opt.step();losses.append(float(loss.detach()))
  torch.save(dict(model=model.state_dict(),spec=spec,rig=rig,trained_on_metric_labels=True,origin='independent_float_prototype_not_HEAL',epochs=a.epochs),out/'checkpoint.pt');(out/'losses.json').write_text(json.dumps(losses));return
 model.eval()
 if a.mode=='infer':
  if not trained or not a.dataset:raise ValueError('inference requires trained checkpoint and synchronized dataset')
  if a.position_std_m is None or not np.isfinite(a.position_std_m) or a.position_std_m<=0:raise ValueError('--position-std-m required from localization residual validation')
  manifest=Path(a.dataset)
  with (out/'decoded.jsonl').open('w') as output,torch.no_grad():
   for line in manifest.read_text().splitlines():
    if not line.strip():continue
    s=json.loads(line);validate(s,manifest.parent,require_labels=False);sample_rig=json.loads((manifest.parent/s['calibration']).read_text())
    g=sampling(sample_rig,spec,s['rig_to_base_xyyaw']);inputs=[torch.from_numpy(images(s,manifest.parent,sample_rig,spec)).to(a.device)]+[torch.from_numpy(g[k]).to(a.device) for k in ('feature_grid','depth_grid','valid')]
    if a.device.startswith('cuda'):torch.cuda.synchronize()
    start=time.perf_counter();heat,offset=model(*inputs)
    if a.device.startswith('cuda'):torch.cuda.synchronize()
    ms=(time.perf_counter()-start)*1000
    centers=decode(heat.sigmoid().cpu().numpy(),offset.cpu().numpy(),spec)
    output.write(json.dumps(dict(stamp_ns=s['stamp_ns'],source_stamps_ns={n:c['capture_ns'] for n,c in s['cameras'].items()},clock_verified=True,clock_domain='ros_epoch_ns',frame_id='base_link',position_std_m=a.position_std_m,centers=centers,model_bpu_ms=None,float_model_ms=ms,backend='independent_float_prototype'))+'\n')
  return
 geometry=sampling(rig,spec);args=[torch.randn(4,3,*spec['input_hw'],device=a.device)]+[torch.from_numpy(geometry[k]).to(a.device) for k in ('feature_grid','depth_grid','valid')]
 if a.mode=='export':
  if not trained:raise ValueError('export deployment graph requires checkpoint trained with verified metric labels')
  torch.onnx.export(model,tuple(args),str(out/'trash_bev.onnx'),opset_version=16,input_names=['images','feature_grid','depth_grid','valid'],output_names=['center_logits','offset_xy'])
  (out/'contract.json').write_text(json.dumps(dict(spec=spec,input_shapes=[list(x.shape) for x in args],backend='uncompiled_float_graph',bpu_placement_verified=False),indent=2));return
 with torch.no_grad():
  for _ in range(2):model(*args)
  if a.device=='cuda':torch.cuda.synchronize()
  begin=time.perf_counter()
  for _ in range(3):result=model(*args)
  if a.device=='cuda':torch.cuda.synchronize()
  ms=(time.perf_counter()-begin)*1000/3
 # Exercise real four-view fusion separately; each camera must contribute gradient.
 context=torch.ones(4,1,16,28,device=a.device,requires_grad=True);depth=torch.ones(4,40,16,28,device=a.device)/40
 fusion=model.lss(context,depth,*args[1:]);fusion.sum().backward();grad=context.grad.abs().sum((1,2,3)).tolist();assert all(v>0 for v in grad)
 report=dict(stage='random_weight_shape_and_feature_fusion_smoke',trained_trash_detector=False,HEAL_source_port=False,BPU_inference=False,device=a.device,model_ms=ms,output_shapes=[list(x.shape) for x in result],per_camera_fusion_gradient=grad)
 (out/'smoke.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
if __name__=='__main__':main()

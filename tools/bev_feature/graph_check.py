#!/usr/bin/env python3
"""Real-image four-view forward/backward and ONNX graph check, UNTRAINED.
No detector checkpoint is produced. This is not calibration or accuracy data.
"""
import argparse
import json
import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src/application/autolabor_fod_vision/src'))
import numpy as np
import torch
import onnx
from autolabor_fod_vision.bev_feature.model import TrashBEV
from autolabor_fod_vision.bev_feature.geometry import sampling,DEFAULT
from autolabor_fod_vision.bev_feature.data import images

def main():
    p=argparse.ArgumentParser();p.add_argument('--sample',required=True);p.add_argument('--output',required=True);a=p.parse_args()
    torch.set_num_threads(4);torch.manual_seed(7)
    path=Path(a.sample);sample=json.loads(path.read_text().splitlines()[0])
    rig=json.loads((path.parent/sample['calibration']).read_text());g=sampling(rig)
    model=TrashBEV(g['shape']).cuda().train()
    x=torch.from_numpy(images(sample,path.parent,rig,DEFAULT)).cuda().requires_grad_()
    args=(x,)+tuple(torch.from_numpy(g[k]).cuda() for k in ('feature_grid','depth_grid','valid'))
    heat,offset=model(*args)
    # Synthetic scalar loss exercises the complete graph; no pseudo-labels saved.
    loss=heat.square().mean()+offset.square().mean();loss.backward()
    gradient=x.grad.abs().sum((1,2,3)).tolist()
    assert all(np.isfinite(gradient)) and all(v>0 for v in gradient)
    assert model.heatmap.weight.grad.abs().sum()>0 and model.offset.weight.grad.abs().sum()>0
    out=Path(a.output);out.mkdir(parents=True,exist_ok=True)
    model.eval()
    target=out/'UNTRAINED_graph_smoke.onnx'
    torch.onnx.export(model,args,str(target),opset_version=16,
        input_names=['images','feature_grid','depth_grid','valid'],output_names=['center_logits','offset_xy'])
    graph=onnx.load(str(target));onnx.checker.check_model(graph)
    ops={n.op_type for n in graph.graph.node}
    assert {'GridSample','Mul','ReduceSum','Conv'}<=ops
    (out/'graph_check.json').write_text(json.dumps(dict(
        trained_detector=False,BPU_compiled=False,real_images_used=True,clocks_synchronized=False,
        complete_graph_per_camera_input_gradient=gradient,onnx_checker='passed',
        onnx_ops=sorted(ops),coordinate_frame=rig['frame_id']),indent=2))

if __name__=='__main__':main()

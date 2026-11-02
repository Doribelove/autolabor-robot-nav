#!/usr/bin/env python3
"""Prepare coupled four-view float inputs for calibration/export parity checks.
Does not quantize or compile a model. Geometry is not copied from nuScenes.
"""
import argparse
import json
import sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src/application/autolabor_fod_vision/src'))
from autolabor_fod_vision.bev_feature.geometry import DEFAULT,sampling
from autolabor_fod_vision.bev_feature.data import validate,images

def main():
    p=argparse.ArgumentParser();p.add_argument('--dataset',required=True);p.add_argument('--output',required=True);a=p.parse_args()
    source=Path(a.dataset);out=Path(a.output);out.mkdir(parents=True,exist_ok=True);records=[]
    for line in source.read_text().splitlines():
        if not line.strip():continue
        s=json.loads(line);validate(s,source.parent,require_labels=False)
        rig=json.loads((source.parent/s['calibration']).read_text());g=sampling(rig,DEFAULT,s['rig_to_base_xyyaw'])
        arrays={k:g[k] for k in ('feature_grid','depth_grid','valid')}
        arrays['images']=images(s,source.parent,rig,DEFAULT)
        name='%06d.npz'%len(records)
        np.savez(out/name,**arrays)
        records.append(dict(file=name,stamp_ns=s['stamp_ns'],frame_id=s['frame_id'],
            source_stamps_ns={k:v['capture_ns'] for k,v in s['cameras'].items()},
            input_shapes={k:list(v.shape) for k,v in arrays.items()}))
    if not records:raise ValueError('empty calibration dataset')
    (out/'manifest.json').write_text(json.dumps(dict(spec=DEFAULT,format='independent_float_prototype_inputs',
        quantization_executed=False,HEAL_adapter_implemented=False,samples=records),indent=2))

if __name__=='__main__':main()

#!/usr/bin/env python3
"""Evaluate annotated replay frames, not image-box-derived pseudo truth.
JSONL row: stamp_ns, truth [{id,x,y,seam}], predictions [{x,y,track_id}],
all x/y in map metres. Optional model_bpu_ms and capture_to_map_publish_ms.
truth IDs must identify the same stationary object across frames.
"""
import argparse
import json
from collections import defaultdict
from pathlib import Path
import numpy as np
from scipy.optimize import linear_sum_assignment

def evaluate(rows, gate=.3):
    errors=[]; by_id=defaultdict(list); ids=defaultdict(set)
    missed=false=duplicates=seam_duplicates=seam_observations=0
    seen=set(); truth_positions={}
    for frame in rows:
        stamp=frame['stamp_ns']
        if stamp in seen:raise ValueError('duplicate frame stamp')
        seen.add(stamp)
        truth=frame['truth']; pred=frame['predictions']
        names=[t['id'] for t in truth]
        if len(set(names))!=len(names):raise ValueError('duplicate truth object in frame')
        seam_observations+=sum(bool(t.get('seam')) for t in truth)
        for t in truth:
            xy=np.array([t['x'],t['y']],float)
            if not np.isfinite(xy).all():raise ValueError('nonfinite truth')
            if t['id'] in truth_positions and not np.allclose(xy,truth_positions[t['id']],atol=1e-6,rtol=0):
                raise ValueError('evaluation requires stationary ground truth')
            truth_positions[t['id']]=xy
        if not truth or not pred:
            missed+=len(truth);false+=len(pred);continue
        txy=np.array([[t['x'],t['y']] for t in truth]);pxy=np.array([[p['x'],p['y']] for p in pred])
        if not np.isfinite(pxy).all():raise ValueError('nonfinite prediction')
        dist=np.linalg.norm(txy[:,None,:]-pxy[None,:,:],axis=2)
        # Maximize cardinality within gate, then minimize metric error.
        ii,jj=linear_sum_assignment(np.where(dist<=gate,dist,1e6))
        used_t=set();used_p=set()
        for i,j in zip(ii,jj):
            if dist[i,j]>gate:continue
            used_t.add(i);used_p.add(j);errors.append(float(dist[i,j]))
            name=truth[i]['id'];by_id[name].append(pxy[j])
            if pred[j].get('track_id') is not None:ids[name].add(pred[j]['track_id'])
        missed+=len(truth)-len(used_t)
        for j in set(range(len(pred)))-used_p:
            nearest=int(np.argmin(dist[:,j]))
            if nearest in used_t and dist[nearest,j]<=gate:
                duplicates+=1;seam_duplicates+=bool(truth[nearest].get('seam'))
            else:false+=1
    def stats(values):
        return dict(count=len(values),mean=float(np.mean(values)),median=float(np.median(values)),p95=float(np.percentile(values,95)),max=float(np.max(values))) if len(values) else None
    stability={}
    for name,points in by_id.items():
        a=np.asarray(points);mean=a.mean(axis=0)
        stability[name]=dict(observations=len(a),mean_xy=mean.tolist(),radial_spread_m=stats(np.linalg.norm(a-mean,axis=1)),track_ids=sorted(ids[name]))
    return dict(frames=len(rows),matching_gate_m=gate,error_m=stats(errors),missed=missed,
        false_positive=false,extra_duplicate_points=duplicates,seam_extra_duplicates=seam_duplicates,
        seam_truth_observations=seam_observations,stationary_map_stability=stability,
        model_bpu_ms=stats([r['model_bpu_ms'] for r in rows if r.get('model_bpu_ms') is not None]),
        capture_to_map_publish_ms=stats([r['capture_to_map_publish_ms'] for r in rows if r.get('capture_to_map_publish_ms') is not None]))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--frames',required=True);p.add_argument('--output',required=True);p.add_argument('--gate',type=float,default=.3);a=p.parse_args()
    rows=[json.loads(x) for x in Path(a.frames).read_text().splitlines() if x.strip()]
    Path(a.output).write_text(json.dumps(evaluate(rows,a.gate),indent=2))

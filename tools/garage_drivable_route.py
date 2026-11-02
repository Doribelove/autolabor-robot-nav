#!/usr/bin/env python3
"""Connect contour sweeps with verified nonholonomic paths, entirely offline."""
import argparse
import concurrent.futures
import json
import math
from pathlib import Path
import time

import cv2
import numpy as np

from garage_route_validation import GarageMap,Hybrid

ROOT=Path('validation/garage_drivable_coverage_20261101')
PREVIOUS=Path('validation/garage_full_navigation1_20261101')
WORKER=None


def init_worker():
    global WORKER
    WORKER=Hybrid(GarageMap())


def connect_worker(job):
    return WORKER.connect(*job)


def line_path(line):
    a,b=np.asarray(line,dtype=float);d=b-a;length=np.linalg.norm(d)
    yaw=math.atan2(d[1],d[0]);xy=a+np.linspace(0,1,int(math.ceil(length/.05))+1)[:,None]*d
    return np.column_stack([xy,np.full(len(xy),yaw)])


def paint_brush(mask,path,grid):
    p=np.asarray(path);c=np.cos(p[:,2]);s=np.sin(p[:,2])
    center=p[:,:2]+1.22*np.column_stack([c,s])
    side=.6*np.column_stack([-s,c]);left=center+side;right=center-side
    l=np.round((left-grid.origin)/grid.resolution-.5).astype(np.int32)
    r=np.round((right-grid.origin)/grid.resolution-.5).astype(np.int32)
    for i in range(1,len(p)):
        cv2.fillConvexPoly(mask,np.array([l[i-1],l[i],r[i],r[i-1]]),1)


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--limit',type=int,default=0);ap.add_argument('--timeout',type=float,default=3.)
    ap.add_argument('--resume',action='store_true');ap.add_argument('--workers',type=int,default=4)
    args=ap.parse_args();ROOT.mkdir(exist_ok=True);grid=GarageMap();hybrid=Hybrid(grid)
    previous=json.loads((PREVIOUS/'contour_plan.json').read_text())
    measured=json.loads((PREVIOUS/'latest_live_status.json').read_text()) if (PREVIOUS/'latest_live_status.json').exists() else {}
    pose=json.loads((PREVIOUS/'pose_before_restart.json').read_text());q=pose['orientation']
    yaw=math.atan2(2*(q[3]*q[2]+q[0]*q[1]),1-2*(q[1]*q[1]+q[2]*q[2]))
    current=np.array([*pose['position'][:2],yaw]);start_pose=current.copy()
    candidates=[];rejected=[]
    for index,line in enumerate(previous['swaths']):
        p=line_path(line);ok,quality=grid.verify_path(p)
        if ok:candidates.append(dict(source_index=index,line=line,path=p,validation=quality))
        else:rejected.append(dict(source_index=index,phase='sweep',validation=quality))
    # Preserve the circular sequence, choosing its entry near the measured pose.
    first=min(range(len(candidates)),key=lambda i:np.linalg.norm(candidates[i]['path'][0,:2]-current[:2])+
              1.35*abs(math.atan2(math.sin(candidates[i]['path'][0,2]-yaw),math.cos(candidates[i]['path'][0,2]-yaw))))
    candidates=candidates[first:]+candidates[:first]
    if args.limit:candidates=candidates[:args.limit]
    covered=np.zeros(grid.cells.shape,np.uint8);steps=[];begin=time.monotonic();attempts=[];seen=set()
    if args.resume:
        saved=json.loads((ROOT/'route_checkpoint.json').read_text());steps=saved['steps'];rejected=saved['rejected']
        seen={s['source_index'] for s in steps}|{s['source_index'] for s in rejected if s.get('phase')=='connector'}
        for step in steps:paint_brush(covered,step['path'],grid)
        if steps:current=np.array(steps[-1]['path'][-1])
    pool=concurrent.futures.ProcessPoolExecutor(max_workers=args.workers,initializer=init_worker)
    futures={};expected={}
    for n,item in enumerate(candidates):
        if item['source_index'] in seen:continue
        expected[n]=candidates[n-1]['path'][-1].copy() if n else start_pose.copy()
        futures[n]=pool.submit(connect_worker,(expected[n],item['path'][0],args.timeout))
    for n,item in enumerate(candidates):
        if item['source_index'] in seen:continue
        if np.allclose(current,expected[n],rtol=0,atol=1e-6):path,metrics=futures[n].result()
        else:path,metrics=hybrid.connect(current,item['path'][0],args.timeout)
        if path is None and steps:
            path,metrics=hybrid.retreat_connect(np.vstack([s['path'] for s in steps[-2:]]),item['path'][0],args.timeout)
        attempt=dict(source_index=item['source_index'],phase='connector',**metrics)
        attempts.append(attempt)
        if path is None:
            rejected.append(attempt)
        else:
            for kind,p,check in [('transit',path,metrics['independent_validation']),('sweep',item['path'],item['validation'])]:
                steps.append(dict(kind=kind,source_index=item['source_index'],path=p.tolist(),validation=check))
                paint_brush(covered,p,grid)
            current=item['path'][-1].copy()
        area=float(np.count_nonzero(covered&(grid.cells==0))*grid.resolution**2)
        progress=dict(done=n+1,total=len(candidates),accepted_sweeps=len(steps)//2,rejected=len(rejected),
            brush_area_m2=area,ratio_all_known_free=area/(np.count_nonzero(grid.cells==0)*grid.resolution**2),
            elapsed_s=time.monotonic()-begin,last_reason=metrics['reason'])
        (ROOT/'progress.json').write_text(json.dumps(progress,indent=2))
        print(json.dumps(progress),flush=True)
        if (n+1)%10==0:
            (ROOT/'route_checkpoint.json').write_text(json.dumps(dict(start=start_pose.tolist(),steps=steps,rejected=rejected)))
    result=dict(start=start_pose.tolist(),grid_digest=grid.digest,footprint=dict(front=1.32,rear=.62,half_width=.5),
        minimum_turning_radius_m=1.35,operation_width_m=1.2,brush_front_offset_m=1.22,
        target_coverage_ratio=.8,known_free_area_m2=float(np.count_nonzero(grid.cells==0)*grid.resolution**2),
        covered_area_m2=area,coverage_ratio=progress['ratio_all_known_free'],steps=steps,rejected=rejected,
        attempts=attempts,elapsed_s=time.monotonic()-begin,
        coverage_assumption='Geometric union of the 1.2 m front brush bar along both sweeps and transitions; assumes active cleaning along the entire route. No actuator control or physical cleaning tested.')
    (ROOT/'route.json').write_text(json.dumps(result,ensure_ascii=False))
    pool.shutdown()
    np.save(ROOT/'covered.npy',covered)
    print('FINAL',json.dumps({k:v for k,v in result.items() if k not in ['steps','rejected','attempts']},ensure_ascii=False),flush=True)


if __name__=='__main__':main()

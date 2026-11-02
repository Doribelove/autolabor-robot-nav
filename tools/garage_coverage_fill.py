#!/usr/bin/env python3
"""Fill residual open floor using short, verified connections; no robot access."""
import argparse,json,math,time
from pathlib import Path
import cv2
import numpy as np
from garage_route_validation import GarageMap,Hybrid
from garage_drivable_route import ROOT,paint_brush,line_path


def candidate_lines(grid):
    free=(grid.cells==0).astype(np.uint8)
    yy,xx=np.nonzero(free)
    bbox=np.array([[xx.min(),yy.min()],[xx.max(),yy.min()],[xx.max(),yy.max()],[xx.min(),yy.max()]])
    world=grid.origin+(bbox+.5)*grid.resolution
    lines=[]
    for angle in [0,math.pi/2,-math.pi/6,math.pi/3,math.pi,3*math.pi/2,5*math.pi/6,4*math.pi/3]:
        u=np.array([math.cos(angle),math.sin(angle)]);v=np.array([-u[1],u[0]])
        corners=np.array([[1.37,.55],[1.37,-.55],[-.67,-.55],[-.67,.55]])
        kernel=np.zeros((37,37),np.uint8)
        cv2.fillConvexPoly(kernel,np.round((corners[:,0,None]*u+corners[:,1,None]*v)/grid.resolution).astype(np.int32)+18,1)
        valid=cv2.erode(free,kernel,anchor=(18,18),borderType=cv2.BORDER_CONSTANT,borderValue=0)
        along=world@u;across=world@v;ts=np.arange(along.min(),along.max(),grid.resolution*.5)
        for offset in np.arange(across.min()+.5,across.max(),1.02):
            points=ts[:,None]*u+offset*v
            ij=np.floor((points-grid.origin)/grid.resolution).astype(int)
            inside=(ij[:,0]>=0)&(ij[:,0]<grid.width)&(ij[:,1]>=0)&(ij[:,1]<grid.height)
            ok=inside & valid[np.clip(ij[:,1],0,grid.height-1),np.clip(ij[:,0],0,grid.width-1)].astype(bool)
            edges=np.diff(np.r_[False,ok,False].astype(np.int8));starts=np.where(edges==1)[0];ends=np.where(edges==-1)[0]-1
            for a,b in zip(starts,ends):
                a+=2;b-=2
                if b-a<40:continue
                path=line_path([points[a],points[b]])
                mask=np.zeros_like(free);paint_brush(mask,path,grid)
                indices=np.flatnonzero(mask&free)
                lines.append(dict(path=path,indices=indices,length=float(np.linalg.norm(points[b]-points[a]))))
    return lines


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--seconds',type=float,default=100);args=ap.parse_args()
    grid=GarageMap();hybrid=Hybrid(grid);route=json.loads((ROOT/'route.json').read_text())
    if not (ROOT/'contour_connected_route.json').exists():
        (ROOT/'contour_connected_route.json').write_text(json.dumps(route))
    covered=np.load(ROOT/'covered.npy');flat=covered.ravel();free_count=np.count_nonzero(grid.cells==0)
    current=np.asarray(route['steps'][-1]['path'][-1]);began=time.monotonic();items=candidate_lines(grid);accepted=0;failed=0
    print('CANDIDATES',len(items),flush=True)
    while items and time.monotonic()-began<args.seconds and np.count_nonzero(covered&(grid.cells==0))/free_count<.8:
        gains=np.array([np.count_nonzero(flat[i['indices']]==0)*grid.resolution**2 for i in items])
        scores=gains/np.array([5+.08*np.linalg.norm(i['path'][0,:2]-current[:2])+.1*i['length'] for i in items])
        scores[gains<2.]=-1
        index=int(np.argmax(scores))
        if scores[index]<0:break
        item=items.pop(index);path=item['path'];ok,check=grid.verify_path(path)
        if not ok:failed+=1;continue
        transit,quality=hybrid.connect(current,path[0],.1)
        if transit is None:
            previous=np.vstack([s['path'] for s in route['steps'][-2:]])
            transit,quality=hybrid.retreat_connect(previous,path[0],.15)
        if transit is None:failed+=1;continue
        for kind,p,validation in [('transit',transit,quality['independent_validation']),('sweep',path,check)]:
            route['steps'].append(dict(kind=kind,source_index=10000+accepted,path=p.tolist(),validation=validation))
            paint_brush(covered,p,grid)
        current=path[-1].copy();accepted+=1
        area=np.count_nonzero(covered&(grid.cells==0))*grid.resolution**2
        print(json.dumps(dict(added=accepted,failed=failed,coverage=area/route['known_free_area_m2'],elapsed=time.monotonic()-began)),flush=True)
    area=float(np.count_nonzero(covered&(grid.cells==0))*grid.resolution**2)
    route.update(covered_area_m2=area,coverage_ratio=area/route['known_free_area_m2'],
                 residual_fill=dict(accepted=accepted,failed=failed,elapsed_s=time.monotonic()-began))
    (ROOT/'route.json').write_text(json.dumps(route));np.save(ROOT/'covered.npy',covered)
    print('FINAL',route['coverage_ratio'],flush=True)

if __name__=='__main__':main()

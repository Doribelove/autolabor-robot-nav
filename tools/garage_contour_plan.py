#!/usr/bin/env python3
"""Generate ordered inward contour sweeps, clipping every directed vehicle footprint."""
import json
import math
from pathlib import Path
import time

import cv2
import numpy as np
from PIL import Image
from scipy import ndimage
import yaml
from autolabor_coverage.coverage_geometry import (GridMap,Point,CoveragePlanner,
    CoverageTimeParameters,estimate_route_time,Swath,sample_path,minimum_area_rectangle)

ROOT=Path('validation/garage_full_navigation1_20261101')
SOURCE=Path('runtime/garage_navigation1_map/map_fused_2d')


def valid_samples(points, heading, grid, cells):
    longitudinal=np.linspace(-.62,1.32,int(math.ceil(1.94/max(grid.resolution,.1)))+1)
    lateral=np.linspace(-.5,.5,int(math.ceil(1./max(grid.resolution,.1)))+1)
    xx,yy=np.meshgrid(longitudinal,lateral)
    c,s=math.cos(heading),math.sin(heading)
    dx=(xx*c-yy*s).ravel();dy=(xx*s+yy*c).ravel()
    ix=np.floor((points[:,0,None]+dx-grid.origin_x)/grid.resolution).astype(np.int32)
    iy=np.floor((points[:,1,None]+dy-grid.origin_y)/grid.resolution).astype(np.int32)
    inside=(ix>=0)&(iy>=0)&(ix<grid.width)&(iy<grid.height)
    safe=cells[np.clip(iy,0,grid.height-1),np.clip(ix,0,grid.width-1)]==0
    return np.all(inside&safe,axis=1)


def segments_for_ring(points, grid, cells, reserve_corners=True):
    result=[];count=len(points)
    for i in range(count):
        a=np.array(points[i],float);b=np.array(points[(i+1)%count],float)
        before=a-np.array(points[(i-1)%count]);after=np.array(points[(i+2)%count])-b
        edge=b-a;length=np.linalg.norm(edge)
        if length<1.21:continue
        heading=math.atan2(edge[1],edge[0])
        def trim(v):
            if not reserve_corners:return 0.
            if np.linalg.norm(v)<1e-6:return 0.
            turn=abs(math.atan2(math.sin(math.atan2(v[1],v[0])-heading),
                                math.cos(math.atan2(v[1],v[0])-heading)))
            # Reserve turning space near polygon corners. Actual connectors
            # still use collision-checked nonholonomic Hybrid A*, never a
            # straight link through the corner or an in-place rotation.
            return min(1.35*math.tan(min(turn,math.pi-.1)/2),length*.4)
        start=trim(before);end=length-trim(after)
        if end-start<1.21:continue
        n=int(math.ceil((end-start)/grid.resolution))
        samples=a+np.linspace(start,end,n+1)[:,None]*edge/length
        valid=valid_samples(samples,heading,grid,cells)
        begin=None
        for j,ok in enumerate(valid):
            if ok and begin is None:begin=j
            if begin is not None and (not ok or j==len(valid)-1):
                last=j if ok else j-1
                if np.linalg.norm(samples[last]-samples[begin])>=1.21:
                    result.append([samples[begin].tolist(),samples[last].tolist()])
                begin=None
    return result


def main():
    began=time.monotonic();config=yaml.safe_load((SOURCE/'map.yaml').read_text())
    raster=np.asarray(Image.open(SOURCE/config['image']));pixels=np.flipud(raster)
    height,width=pixels.shape;resolution=float(np.float32(config['resolution']));ox,oy,_=config['origin']
    cells=np.full(pixels.shape,-1,np.int8);cells[pixels>=250]=0;cells[pixels<=65]=100
    grid=GridMap(width,height,resolution,ox,oy,cells.ravel())
    pose=json.loads((ROOT/'pose_before_restart.json').read_text());cursor=np.array(pose['position'][:2])
    labels,_=ndimage.label(cells==0);cx,cy=grid.world_to_cell(*cursor);label=labels[cy,cx]
    if label==0:raise RuntimeError('Measured pose is not in saved free space')
    connected=(labels==label).astype(np.uint8)
    distance=cv2.distanceTransform(connected,cv2.DIST_L2,cv2.DIST_MASK_PRECISE)*resolution
    yy,xx=np.nonzero(connected)
    lo=np.array([xx.min(), yy.min()],float)*resolution+[ox,oy]
    hi=np.array([xx.max()+1, yy.max()+1],float)*resolution+[ox,oy]
    route=[];groups=[];offset=.55;level=0
    while np.min(hi-lo)>2*offset+1.21:
        a=lo+offset;b=hi-offset
        world=np.array([[a[0],a[1]],[a[0],b[1]],[b[0],b[1]],[b[0],a[1]]])
        start=int(np.argmin(np.sum((world-cursor)**2,axis=1)))
        world=np.roll(world,-start,axis=0)
        lines=segments_for_ring(world,grid,cells)
        if lines:
            first=int(np.argmin([np.linalg.norm(np.asarray(a)-cursor) for a,b in lines]))
            lines=lines[first:]+lines[:first]
            groups.append(dict(level=level,offset_m=offset,first_swath=len(route)))
            route.extend(lines);cursor=np.array(route[-1][1])
        offset+=1.02;level+=1
    print('Rectangular inward loops: %d loops, %d directed lines'%(level,len(route)),flush=True)
    # Add residual boundary-following loops in their original order. They
    # address obstacle sides that rectangular rings cannot align with.
    def sweep_mask(line):
        a,b=np.asarray(line);delta=b-a;normal=np.array([-delta[1],delta[0]])/np.linalg.norm(delta)*.6
        mask=np.zeros(cells.shape,np.uint8)
        corners=np.array([a+normal,b+normal,b-normal,a-normal])
        cv2.fillConvexPoly(mask,np.round((corners-[ox,oy])/resolution-.5).astype(np.int32),1)
        return mask
    accounted=np.zeros(cells.shape,np.uint8)
    for line in route:accounted|=sweep_mask(line)
    rectangular_count=len(route);offset=.55;residual_groups=0
    while offset<float(distance.max()):
        contours,hierarchy=cv2.findContours((distance>=offset).astype(np.uint8),cv2.RETR_CCOMP,cv2.CHAIN_APPROX_SIMPLE)
        loops=[]
        for contour in contours:
            if cv2.arcLength(contour,True)*resolution<7.:continue
            approx=cv2.approxPolyDP(contour,.5/resolution,True).reshape(-1,2)
            if len(approx)<3:continue
            world=approx.astype(float)*resolution+[ox+resolution*.5,oy+resolution*.5]
            lines=segments_for_ring(world,grid,cells,reserve_corners=False)
            kept=[]
            for line in lines:
                mask=sweep_mask(line);new=mask&(cells==0)&(1-accounted)
                if new.sum()>=max(mask.sum()*.35,.6/resolution**2):
                    kept.append(line);accounted|=mask
            if kept:loops.append(kept)
        while loops:
            lines=min(loops,key=lambda ls:min(np.linalg.norm(np.asarray(a)-cursor) for a,b in ls))
            loops.remove(lines)
            first=int(np.argmin([np.linalg.norm(np.asarray(a)-cursor) for a,b in lines]))
            lines=lines[first:]+lines[:first];route.extend(lines);cursor=np.array(lines[-1][1]);residual_groups+=1
        offset+=1.02
    print('Residual contour sweeps: %d'%(len(route)-rectangular_count),flush=True)
    # Use the exact online collision evaluator on the final serialized samples.
    planner=CoveragePlanner(grid,footprint_front=1.32,footprint_rear=.62,footprint_half_width=.5)
    cache={};verified=[];rejected=0
    for a,b in route:
        pa,pb=Point(*a),Point(*b);yaw=math.atan2(b[1]-a[1],b[0]-a[0])
        if all(planner._pose_is_free(p,yaw,cache) for p in sample_path(pa,pb,resolution)):
            verified.append([a,b])
        else:rejected+=1
    route=verified
    covered=np.zeros((height,width),np.uint8)
    for a,b in route:
        delta=np.subtract(b,a);length=np.linalg.norm(delta);normal=np.array([-delta[1],delta[0]])/length*.6
        box=np.array([a+normal,b+normal,b-normal,a-normal])
        cv2.fillConvexPoly(covered,np.round((box-[ox,oy])/resolution-.5).astype(np.int32),1)
    region=[[ox,oy],[ox+width*config['resolution'],oy],
            [ox+width*config['resolution'],oy+height*config['resolution']],[ox,oy+height*config['resolution']]]
    region=[[p.x,p.y] for p in minimum_area_rectangle([Point(float(np.float32(x)),float(np.float32(y))) for x,y in region])]
    swaths=[Swath(Point(*a),Point(*b),0.,math.dist(a,b)) for a,b in route]
    q=pose['orientation'];yaw=math.atan2(2*(q[3]*q[2]+q[0]*q[1]),1-2*(q[1]*q[1]+q[2]*q[2]))
    est=estimate_route_time(swaths,Point(*pose['position'][:2]),yaw,1.35,CoverageTimeParameters())
    result=dict(pattern='inward_contours',grid_digest=grid.digest(),region=region,
        operation_width_m=1.2,overlap_ratio=.15,lane_spacing_m=1.02,minimum_turning_radius_m=1.35,
        footprint=dict(front_m=1.32,rear_m=.62,half_width_m=.5),swaths=route,
        contour_levels=level,contour_groups=len(groups),residual_contour_groups=residual_groups,swath_count=len(route),
        rejected_short_or_conflicting_final_lines=rejected,
        sweep_length_m=sum(s.length for s in swaths),estimated_seconds=est.total_time_sec,
        preview_swept_known_area_m2=float(np.count_nonzero(covered&(cells==0))*resolution**2),
        known_free_area_m2=float(np.count_nonzero(cells==0)*resolution**2),
        generated_seconds=time.monotonic()-began,
        transition_note='Ordered rectangular inward sweeps plus residual boundary contours; rectangular corners reserve turning space. Connections are not straight executable edges: existing Hybrid A*/TEB computes footprint-safe, radius-constrained forward/reverse transitions online.')
    (ROOT/'contour_plan.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    import matplotlib;matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
    fig,ax=plt.subplots(figsize=(12,14));ax.imshow(raster,cmap='gray',vmin=0,vmax=255,
        extent=[ox,ox+width*resolution,oy,oy+height*resolution])
    lc=LineCollection(route,cmap='turbo',array=np.arange(len(route)),linewidths=.9);ax.add_collection(lc)
    ax.plot(*pose['position'][:2],'r*',markersize=12,label='Current pose')
    ax.plot(*route[0][0],'go',markersize=6,label='First contour sweep')
    ax.set_aspect('equal');ax.set_xlabel('map X (m)');ax.set_ylabel('map Y (m)')
    ax.set_title('Inward contour coverage / vehicle constraints\n1.2m width, 1.02m spacing; footprint 1.94 x 1.00m; turning radius >= 1.35m')
    ax.legend();fig.tight_layout();fig.savefig(ROOT/'contour_route.png',dpi=160)
    print(json.dumps({k:v for k,v in result.items() if k!='swaths'},ensure_ascii=False),flush=True)


if __name__=='__main__':main()

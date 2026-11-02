"""Offline production Hybrid A* and independent footprint/curvature checks."""
import ctypes
import json
import math
from pathlib import Path
import time

import numpy as np
from scipy.ndimage import distance_transform_edt


class GarageMap:
    def __init__(self, root=Path('validation/garage_full_navigation1_20261101')):
        info=json.loads((root/'live_map_info.json').read_text())
        self.cells=np.load(root/'live_map_cells.npy')
        self.resolution=info['resolution'];self.origin=np.asarray(info['origin'])
        self.height,self.width=self.cells.shape;self.digest=info['digest']
        distance=distance_transform_edt(self.cells==0)*self.resolution
        self.costs=np.clip(252*np.exp(-3*np.maximum(distance-.5,0)),0,252).astype(np.uint8)
        self.costs[self.cells==-1]=255;self.costs[self.cells==100]=254
        # Interior and boundary lattice, half a map cell apart. This catches
        # occupied cells inside a footprint, not only its polygon perimeter.
        xx,yy=np.meshgrid(np.linspace(-.62,1.32,40),np.linspace(-.5,.5,21))
        self.local_x=xx.ravel();self.local_y=yy.ravel()

    def valid(self, poses):
        poses=np.asarray(poses,dtype=float).reshape(-1,3);result=[]
        for start in range(0,len(poses),512):
            p=poses[start:start+512];c=np.cos(p[:,2,None]);s=np.sin(p[:,2,None])
            x=p[:,0,None]+c*self.local_x-s*self.local_y
            y=p[:,1,None]+s*self.local_x+c*self.local_y
            ix=np.floor((x-self.origin[0])/self.resolution).astype(int)
            iy=np.floor((y-self.origin[1])/self.resolution).astype(int)
            inside=(ix>=0)&(iy>=0)&(ix<self.width)&(iy<self.height)
            free=self.cells[np.clip(iy,0,self.height-1),np.clip(ix,0,self.width-1)]==0
            result.extend(np.all(inside&free,axis=1))
        return np.asarray(result,dtype=bool)

    def verify_path(self,path):
        p=np.asarray(path);delta=np.diff(p[:,:2],axis=0);length=np.linalg.norm(delta,axis=1)
        dyaw=np.arctan2(np.sin(np.diff(p[:,2])),np.cos(np.diff(p[:,2])))
        nonzero=length>1e-6
        if np.any((~nonzero)&(np.abs(dyaw)>1e-5)):
            return False,{'reason':'rotation without translation'}
        curvature=np.zeros(len(length));curvature[nonzero]=2*np.sin(np.abs(dyaw[nonzero])/2)/length[nonzero]
        if np.any(curvature>1/1.35+1e-4):
            return False,{'reason':'turning radius violation','max_curvature':float(curvature.max())}
        # Reconstruct each short edge as a constant-curvature arc, including
        # reverse arcs. A polyline/chord is never treated as an executable turn.
        samples=[p[0]];gears=[];travel=0.;reverse=0.
        for i,d in enumerate(length):
            if d<1e-6:continue
            mid=p[i,2]+dyaw[i]/2
            along=delta[i,0]*math.cos(mid)+delta[i,1]*math.sin(mid)
            lateral=abs(-delta[i,0]*math.sin(mid)+delta[i,1]*math.cos(mid))
            if lateral>max(.002,d*.01):return False,{'reason':'lateral nonholonomic violation','lateral':float(lateral)}
            gear=1 if along>=0 else -1;gears.append(gear)
            angle=dyaw[i];arc=d if abs(angle)<1e-8 else d*abs(angle)/(2*math.sin(abs(angle)/2))
            travel+=arc
            if gear<0:reverse+=arc
            n=max(1,int(math.ceil(arc/.05)))
            for j in range(1,n+1):
                f=j/n;yaw=p[i,2]+f*angle
                if abs(angle)<1e-8:xy=p[i,:2]+delta[i]*f
                else:
                    radius=gear*arc/angle
                    xy=p[i,:2]+radius*np.array([math.sin(yaw)-math.sin(p[i,2]),-math.cos(yaw)+math.cos(p[i,2])])
                samples.append([*xy,yaw])
        valid=self.valid(samples)
        summary=dict(length_m=travel,reverse_m=reverse,pose_samples=len(samples),
            max_curvature=float(curvature.max(initial=0)),
            direction_changes=sum(a!=b for a,b in zip(gears,gears[1:])),
            invalid_footprints=int(np.count_nonzero(~valid)))
        if not np.all(valid):summary['reason']='full footprint intersects occupied/unknown space'
        return bool(np.all(valid)),summary


class Hybrid:
    def __init__(self, grid, library=Path('validation/garage_drivable_coverage_20261101/libgarage_hybrid_v2.so')):
        self.grid=grid;self.lib=ctypes.CDLL(str(library.resolve()))
        double=ctypes.POINTER(ctypes.c_double);byte=ctypes.POINTER(ctypes.c_ubyte)
        self.lib.garage_create.argtypes=[ctypes.c_uint,ctypes.c_uint,ctypes.c_double,ctypes.c_double,ctypes.c_double,byte]
        self.lib.garage_create.restype=ctypes.c_void_p
        self.lib.garage_plan.argtypes=[ctypes.c_void_p,double,double,ctypes.c_double,double,ctypes.c_int,double]
        self.lib.garage_plan.restype=ctypes.c_int
        self.lib.garage_reason.argtypes=[ctypes.c_void_p];self.lib.garage_reason.restype=ctypes.c_char_p
        self.lib.garage_navfn.argtypes=[ctypes.c_void_p,double,double,double,ctypes.c_int]
        self.lib.garage_navfn.restype=ctypes.c_int
        self.handle=self.lib.garage_create(grid.width,grid.height,grid.resolution,*grid.origin,grid.costs.ctypes.data_as(byte))
        self.out=np.empty((30000,3),dtype=np.float64);self.metrics=np.empty(4,dtype=np.float64);self.double=double

    def plan(self,start,goal,seconds=2.):
        a=np.ascontiguousarray(start,dtype=np.float64);b=np.ascontiguousarray(goal,dtype=np.float64)
        begin=time.monotonic()
        n=self.lib.garage_plan(self.handle,a.ctypes.data_as(self.double),b.ctypes.data_as(self.double),seconds,
            self.out.ctypes.data_as(self.double),len(self.out),self.metrics.ctypes.data_as(self.double))
        metrics=dict(success=n>0,reason=self.lib.garage_reason(self.handle).decode(),wall_s=time.monotonic()-begin,
            expansions=int(self.metrics[0]),estimated_s=self.metrics[1],reverse_m=self.metrics[2],direction_changes=int(self.metrics[3]))
        path=self.out[:n].copy() if n>0 else None
        if path is not None:
            ok,check=self.grid.verify_path(path);metrics['independent_validation']=check
            if not ok:metrics['success']=False;metrics['reason']='Independent check: '+check['reason'];path=None
        return path,metrics

    def topology(self,start,goal):
        a=np.ascontiguousarray(start,dtype=np.float64);b=np.ascontiguousarray(goal,dtype=np.float64)
        out=np.empty((30000,2),dtype=np.float64)
        n=self.lib.garage_navfn(self.handle,a.ctypes.data_as(self.double),b.ctypes.data_as(self.double),
                              out.ctypes.data_as(self.double),len(out))
        return out[:n].copy() if n>0 else None

    def retreat_connect(self, previous, goal, seconds=.15):
        previous=np.asarray(previous);length=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(previous[:,:2],axis=0),axis=1))]
        for retreat in [3.,6.,10.]:
            index=int(np.searchsorted(length,max(0.,length[-1]-retreat)))
            reverse=previous[index:][::-1]
            path,metrics=self.plan(reverse[-1],goal,seconds)
            if path is None:continue
            joined=np.vstack([reverse,path[1:]])
            ok,check=self.grid.verify_path(joined)
            if ok:return joined,dict(success=True,reason='Verified reverse retreat then Hybrid connector',independent_validation=check)
        return None,dict(success=False,reason='No verified connection after reverse retreat')

    def connect(self,start,goal,seconds=3.):
        deadline=time.monotonic()+max(2.,seconds*10.)
        path,metrics=self.plan(start,goal,seconds)
        if path is not None:return path,metrics
        original=metrics;current=np.array(start,dtype=float);pieces=[];parts=[];initial_length=None
        for turn in range(80):
            if time.monotonic()>=deadline:
                return None,dict(success=False,reason='Offline connector search budget exhausted',direct=original)
            course=self.topology(current,goal)
            if course is None:return None,dict(success=False,reason='Navfn found no known-space topology',direct=original)
            distance=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(course,axis=0),axis=1))]
            if initial_length is None:initial_length=float(distance[-1])
            if distance[-1]<8.:
                p,m=self.plan(current,goal,seconds)
                if p is not None:
                    pieces.append(p);parts.append(m);joined=np.vstack([p if i==0 else p[1:] for i,p in enumerate(pieces)])
                    ok,check=self.grid.verify_path(joined)
                    return (joined if ok else None),dict(success=ok,reason='Navfn topology joined by verified Hybrid A* segments',
                        topology_length_m=initial_length,parts=parts,independent_validation=check)
            proposals=[]
            for ahead in [6.,4.,2.,8.,10.,3.]:
                if ahead>=distance[-1]-.6:continue
                i=int(np.searchsorted(distance,ahead));lo=max(0,i-12);hi=min(len(course)-1,i+12)
                v=course[hi]-course[lo];yaw=math.atan2(v[1],v[0]);pose=np.array([*course[i],yaw])
                if self.grid.valid([pose])[0]:proposals.append(pose)
            for pose in proposals:
                if time.monotonic()>=deadline:break
                p,m=self.plan(current,pose,seconds)
                if p is not None:
                    pieces.append(p);parts.append(m);current=p[-1].copy();break
            else:return None,dict(success=False,reason='No footprint/curvature-valid connector through this topology',direct=original,completed_parts=len(parts))
        return None,dict(success=False,reason='Topology refinement exceeded offline search budget',direct=original)

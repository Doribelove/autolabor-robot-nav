"""Deterministic OFFLINE synthetic workload; no Gazebo/perception qualification."""
from pathlib import Path
import json,sys,time,hashlib,platform
import numpy as np
from shapely.geometry import box,Point
W=Path(__file__).resolve().parents[2]/'workspace';sys.path.insert(0,str(W/'scripts/astra_gazebo'))
from sensor_tracks_v1 import SensorTracks,TrackConfig
from route_sensor_buffer import ScanFrame
BASE=1_000_000_000
class Grid:
    image_sha256='a'*64;yaml_sha256='b'*64
    def clearance(self,p):return abs(p[1]-4.)
def pts(cx,cy,n=9):
    a=np.linspace(-.06,.06,n);return np.column_stack((cx+a,cy+.02*np.cos(a*10)))
def sample(case,i):
    t=i*.1
    wall=np.column_stack((np.linspace(-6,6,180),np.full(180,4.)))
    if case=='static_prior_720':return np.vstack([wall]*4)
    if case=='two_crossing':return np.vstack((wall,pts(2,t*.2-.5),pts(2,.5-t*.2)))
    if case=='occluded':return wall if 8<=i<20 else np.vstack((wall,pts(2+t*.1,0)))
    if case=='sparse':return np.vstack((wall,[[1.,0.],[2.,1.],[3.,-1.]]))
    if case=='ego_motion':return np.vstack((wall,pts(2,0),pts(3,1)))
    if case=='crowded_12':return np.vstack((wall,*[pts(1+(j%4)*1.,-2+(j//4)*1.,12) for j in range(12)]))
def frame(stamp,p):return ScanFrame(stamp,np.ones(72),p,(0.,0.,0.))
def odom(stamp):return dict(stamp_ns=stamp,pose_map=(0.,0.,0.),body_twist=(0.,0.,0.))
def summary(a):return dict(count=len(a),median_ms=float(np.median(a)),p95_ms=float(np.percentile(a,95)),max_ms=float(np.max(a)))
rows=[]
for case in ('static_prior_720','two_crossing','occluded','sparse','ego_motion','crowded_12'):
    tracker=SensorTracks(Grid());times=[];query=[];whole=[];last=None
    for i in range(30):
        stamp=BASE+i*100_000_000;points=sample(case,i);start=time.perf_counter_ns()
        f=frame(stamp,points);o=odom(stamp)
        if case=='ego_motion':
            f=ScanFrame(stamp,np.ones(72),points,(i*.03,0.,0.));o=dict(stamp_ns=stamp,pose_map=(i*.03,0.,0.),body_twist=(.3,0.,0.))
        last=tracker.accept(f,tf_stamp_ns=stamp,odom_state=o,now_ns=stamp)
        accepted=time.perf_counter_ns();env=tracker.occupancy(stamp)
        for action in range(8):
            for k in range(31):
                x=k*.07;y=(action-3)*.10
                env.distance_lower_bound(box(x-.5,y-.35,x+.5,y+.35),k*100_000_000)
        end=time.perf_counter_ns()
        times.append((accepted-start)/1e6);query.append((end-accepted)/1e6);whole.append((end-start)/1e6)
    rows.append(dict(workload=case,scan_count=30,query_states_per_scan=248,accept=summary(times),occupancy_queries=summary(query),whole=summary(whole),final_stats=dict(last.statistics)))
# Explicit supported-size adversarial clustering workload (all unknown points).
class Free(Grid):
    def clearance(self,p):return float('inf')
adversarial=[]
for size in (720,1440):
    a=np.linspace(-.1,.1,size);points=np.column_stack((a,a*.1));durations=[]
    for _ in range(5):
        tracker=SensorTracks(Free());start=time.perf_counter_ns();tracker.accept(frame(BASE,points),tf_stamp_ns=BASE,odom_state=odom(BASE),now_ns=BASE);durations.append((time.perf_counter_ns()-start)/1e6)
    adversarial.append(dict(unknown_returns=size,accept=summary(durations)))
# Worst configured sparse memory is intentionally reported, not hidden behind typical workloads.
tracker=SensorTracks(Free());points=np.asarray([(x*.4,y*.4) for x in range(64) for y in range(32)])
for i,chunk in enumerate((points[:1024],points[1024:])):
    stamp=BASE+i*100_000_000;last=tracker.accept(frame(stamp,chunk),tf_stamp_ns=stamp,odom_state=odom(stamp),now_ns=stamp)
env=tracker.occupancy(stamp);start=time.perf_counter_ns()
for k in range(248):env.distance_lower_bound(box(-.5,-.35,.5,.35),(k%31)*100_000_000)
sparse_stress=dict(retained_cells=len(last.sparse_supports),query_states=248,total_query_ms=(time.perf_counter_ns()-start)/1e6)
paths=[Path(__file__).resolve(),W/'scripts/astra_gazebo/sensor_tracks_v1.py',W/'scripts/astra_gazebo/route_sensor_buffer.py']
out=dict(schema='offline_sensor_track_timing_v1',scope='synthetic component timing, no live deadline or perception qualification',python=sys.version,platform=platform.platform(),numpy=np.__version__,cases=rows,dense_unknown_stress=adversarial,maximum_sparse_memory=sparse_stress,all_six_whole_max_ms=max(r['whole']['max_ms'] for r in rows),sources=[dict(path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in paths])
Path(__file__).with_name('timing.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out,indent=2))

"""Full M08 actual-geometry pure-core timing; synthetic sensing, no ROS evidence."""
from pathlib import Path
import argparse,hashlib,json,sys,time
import numpy as np
S=Path(__file__).resolve().parents[2];P=S/'workspace/scripts/astra_gazebo';sys.path.insert(0,str(P));sys.path.insert(0,str(P/'tests'))
from common_route import load_bundle,CommonRoute
from route_candidate_core_m08 import RouteTracksRuntime
from route_sensor_buffer import ScanFrame
from sensor_tracks_v1 import HardOccupancy
from test_common_route import make_grid
T=2_000_000_000
URDF=S/'runs/passive_sweep_q05_static/robot_description_runtime.xml'
def bind(p):return {'path':str(p),'sha256':hashlib.sha256(p.read_bytes()).hexdigest()}
def summary(x):return dict(count=len(x),median_ms=float(np.median(x)),p95_ms=float(np.percentile(x,95)),max_ms=float(max(x)))
def state(stamp,pose):return dict(stamp_ns=stamp,pose_map=tuple(pose),body_twist=(.8,0.,0.),roll_pitch=(0.,0.),joints={'front_left_steer_joint':0.,'front_right_steer_joint':0.})
def frame(stamp,pose,pts):return ScanFrame(stamp,np.full(72,8.),np.asarray(pts).reshape(-1,2),tuple(pose))
def accept(c,f,p):return c.accept_scan(f,f.stamp_ns,state(f.stamp_ns,p),f.stamp_ns)
def bench(grid,route,pose,points,trials,retained=()):
    times={'accept':[],'build':[],'select_and_recheck':[],'whole':[]};raw=[];parity=None
    for trial in range(trials):
        c=RouteTracksRuntime(grid,route,URDF.read_text())
        for index,chunk in enumerate(retained):accept(c,frame(T-200_000_000+index*100_000_000,pose,chunk),pose)
        f=frame(T,pose,points);a=state(T,pose);start=time.perf_counter_ns();accept(c,f,pose);middle=time.perf_counter_ns()
        built=c.build(a,f,np.full((6,72),8.),1.,T);built_at=time.perf_counter_ns();chosen,_,_,_=c.select(built);command=c.command(a,f,chosen,T);end=time.perf_counter_ns()
        parts=[(middle-start)/1e6,(built_at-middle)/1e6,(end-built_at)/1e6,(end-start)/1e6]
        for name,value in zip(times,parts):times[name].append(value)
        raw.append(parts)
        if parity is None:
            trajectories=built['previews']+built['backups'];states=[s for rows in trajectories for s in rows]
            old=HardOccupancy(c.tracker.snapshot,T);expected=np.array([old.distance_lower_bound(c.footprint.at(s['base']),int(np.ceil(s['t']*1e9))) for s in states])
            new=c.predict(f,T)['occupancy'].distance_lower_bounds(c.batch_footprint,np.array([s['base'] for s in states]),np.array([int(np.ceil(s['t']*1e9)) for s in states],dtype=np.int64))
            finite=np.isfinite(expected);assert np.all(new[finite]<=expected[finite]+1e-11)
            assert np.all(np.isinf(new[~finite]))
            parity=dict(query_count=len(states),preview_queries=sum(map(len,built['previews'])),backup_queries=sum(map(len,built['backups'])),max_batch_minus_v1_m=float(max(new[finite]-expected[finite])) if finite.any() else None,
                        max_absolute_difference_m=float(max(abs(new[finite]-expected[finite]))) if finite.any() else 0.,support_count=c.predict(f,T)['dynamic_count'],footprint_components=c.footprint.component_count,footprint_planes=len(c.footprint.vertices))
    return dict(parity=parity,timing={k:summary(v) for k,v in times.items()},raw_trials_ms=raw,all_whole_under50ms=max(times['whole'])<=50.)
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',required=True);ap.add_argument('--trials',type=int,default=10);args=ap.parse_args();cases=[];bindings=[bind(URDF)]
    for path in sorted((S/'audits/route_runtime_components_v3/new_guide_radius_1m').glob('*/common_route.json')):
        grid,route,data=load_bundle(path.parent);pose=route.sample(min(3.,route.length/2));pts=[(pose[0]+2.5,pose[1]+1.),(pose[0]+2.52,pose[1]+1.01),(pose[0]+2.54,pose[1]+1.)]
        cases.append(dict(name=path.parent.name,**bench(grid,route,pose,pts,args.trials)));bindings.append(bind(path))
    grid,_,_=make_grid(size=400,obstacles=[]);route=CommonRoute([(2.,7.),(12.,7.)]);pose=(3.,7.,0.)
    sparse=np.array([(x*.4+1,y*.4+1) for x in range(64) for y in range(32)])
    cases.append(dict(name='retained_2048_sparse_actual_candidate_geometry',**bench(grid,route,pose,[],args.trials,np.array_split(sparse,2))))
    theta=np.linspace(-np.pi,np.pi,1440,endpoint=False);dense=np.array([pose[0]+3*np.cos(theta),pose[1]+3*np.sin(theta)]).T
    cases.append(dict(name='maximum1440_dense_new_scan',**bench(grid,route,pose,dense,args.trials)))
    cases.append(dict(name='combined_maximum1440_new_and2048_retained',**bench(grid,route,pose,dense,args.trials,np.array_split(sparse,2))))
    cases.append(dict(name='combined1440_sparse_reobserved_and2048_retained',**bench(grid,route,pose,sparse[:1440],args.trials,np.array_split(sparse,2))))
    result=dict(schema='astra_route_tracks_full_core_benchmark_v1',scope='offline synthetic sensing; real nine-component URDF geometry and six stored development maps; not live or WCET',cases=cases,sources=[bind(P/n) for n in ('route_candidate_core.py','route_candidate_core_m08.py','sensor_tracks_v1.py','sensor_tracks_v2.py','sensor_tracks_ingestion_m08.py')]+bindings+[bind(Path(__file__))],
        full_core_all_measured_under50ms=all(c['all_whole_under50ms'] for c in cases),includes_scan_accept=True,includes_8previews_8backups_and_current_revalidation=True,includes_ros_tf_callback_scheduling=False,includes_actor=False,live_qualification=False)
    Path(args.output).write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({c['name']:c['timing']['whole'] for c in cases},indent=2))
if __name__=='__main__':main()
